#!/usr/bin/env python3
"""
导航 ↔ 机械臂 双向桥接

功能：
  1. 订阅 /patrol/arrived，默认只记录到位，不自动触发机械臂
  2. 监听 8189 端口，收到 /api/arm_done 后发布下一点
  3. 固定巡检顺序为 1 → 2 → 结束

用法：
  ros2 run jy_tk arm_nav_bridge.py

直接触发指定巡检点（不调用机械臂）：
  curl -X POST http://127.0.0.1:8189/api/trigger_nav \
    -H 'Content-Type: application/json' \
    -d '{"point_id":"2"}'

机械臂侧对接：
  机械臂完成动作后，调用：
    curl -X POST http://<jetson_ip>:8189/api/arm_done \
      -H 'Content-Type: application/json' \
      -d '{"arm":"BOTH","result":"ok","point_id":"1"}'
"""

import asyncio
import json
import os
import threading
from typing import Any, Dict

import httpx
import uvicorn
from fastapi import FastAPI, Request
from pydantic import BaseModel, Field

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ArmDoneRequest(BaseModel):
    arm: str = Field("BOTH")
    result: str = Field("ok")
    point_id: str = Field("")


class ArmNavBridge(Node):
    def __init__(self):
        super().__init__('arm_nav_bridge')

        # ---------- 参数 ----------
        self.declare_parameter('arm_api_base', 'http://127.0.0.1:8188')
        self.declare_parameter('arrived_topic', '/patrol/arrived')
        self.declare_parameter(
            'current_point_topic', '/patrol/current_point')
        self.declare_parameter('next_topic', '/patrol/next')
        self.declare_parameter('http_port', 8189)
        self.declare_parameter('arm_speed', 12.0)
        self.declare_parameter('trigger_arm_on_arrive', False)

        p = self.get_parameter
        self.arm_api_base = p('arm_api_base').value.rstrip('/')
        self.arrived_topic = p('arrived_topic').value
        self.current_point_topic = p('current_point_topic').value
        self.next_topic = p('next_topic').value
        self.http_port = int(p('http_port').value)
        self.arm_speed = float(p('arm_speed').value)
        self.trigger_arm_on_arrive = bool(p('trigger_arm_on_arrive').value)

        self.state_lock = threading.RLock()

        # 根据 map -> base_footprint 二维位置识别的当前点位。
        self.current_point_id = None
        # 已完成导航和对接，正在等待机械臂完成的点位。
        self.arm_pending_point_id = None

        # 固定巡检顺序：1 → 2 → 结束。
        self.next_point_map = {
            '1': '2',
            '2': None,
        }

        # ---------- ROS pub/sub ----------
        self.arrived_sub = self.create_subscription(
            String, self.arrived_topic, self.on_arrived, 10)

        current_point_qos = QoSProfile(depth=1)
        current_point_qos.reliability = ReliabilityPolicy.RELIABLE
        current_point_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.current_point_sub = self.create_subscription(
            String,
            self.current_point_topic,
            self.on_current_point,
            current_point_qos,
        )
        self.next_pub = self.create_publisher(String, self.next_topic, 10)

        # ---------- HTTP server ----------
        self._app = self._build_fastapi()
        self._start_http_server()

        self.get_logger().info('=' * 60)
        self.get_logger().info('arm_nav_bridge 已启动')
        self.get_logger().info(f'  机械臂 API: {self.arm_api_base}')
        self.get_logger().info(f'  订阅: {self.arrived_topic}')
        self.get_logger().info(f'  点位同步: {self.current_point_topic}')
        self.get_logger().info(f'  发布: {self.next_topic}')
        self.get_logger().info(f'  HTTP 监听: 0.0.0.0:{self.http_port}')
        self.get_logger().info(
            f'  机械臂完成后请调: POST http://<jetson_ip>:'
            f'{self.http_port}/api/arm_done')
        self.get_logger().info('=' * 60)

    # ================================================================
    # ROS → HTTP
    # ================================================================
    def on_current_point(self, msg: String):
        point_id = msg.data.strip() or None

        with self.state_lock:
            old_point_id = self.current_point_id
            self.current_point_id = point_id

            if (
                self.arm_pending_point_id is not None
                and self.arm_pending_point_id != point_id
            ):
                old_pending_id = self.arm_pending_point_id
                self.arm_pending_point_id = None
                self.get_logger().warn(
                    f'机器人已离开待处理点位 '
                    f'[{old_pending_id}]，清除机械臂待完成状态')

        if old_point_id != point_id:
            self.get_logger().info(
                f'当前点位更新: '
                f'{old_point_id or "None"} -> {point_id or "None"}')

    def on_arrived(self, msg: String):
        point_id = msg.data.strip()
        if not point_id:
            self.get_logger().error('收到空的导航到达点位，忽略')
            return

        with self.state_lock:
            self.current_point_id = point_id
            self.arm_pending_point_id = point_id
        self.get_logger().info(f'导航到达 [{point_id}]')

        if not self.trigger_arm_on_arrive:
            self.get_logger().info('  trigger_arm_on_arrive=false，跳过')
            return

        # 异步发 HTTP，不阻塞 ROS 回调
        threading.Thread(
            target=self._call_arm_startup,
            args=(point_id,),
            daemon=True,
        ).start()

    def _call_arm_startup(self, point_id: str):
        url = f'{self.arm_api_base}/api/startup'
        payload = {'confirm': True, 'speed': self.arm_speed}
        try:
            with httpx.Client(timeout=30.0) as client:
                resp = client.post(url, json=payload)
            if resp.status_code == 200:
                self.get_logger().info(
                    f'  ✅ 机械臂启动成功 [{point_id}]')
            else:
                self.get_logger().error(
                    f'  ❌ 机械臂启动失败: HTTP {resp.status_code} '
                    f'{resp.text[:200]}')
        except Exception as e:
            self.get_logger().error(f'  ❌ 调用机械臂 API 异常: {e}')

    # ================================================================
    # HTTP → ROS
    # ================================================================
    def _build_fastapi(self) -> FastAPI:
        app = FastAPI(title='arm_nav_bridge')

        bridge = self  # 闭包捕获

        @app.get('/health')
        async def health():
            with bridge.state_lock:
                current_point_id = bridge.current_point_id
                pending_point_id = bridge.arm_pending_point_id
            return {
                'status': 'healthy',
                'node': 'arm_nav_bridge',
                'current_point_id': current_point_id,
                'arm_pending_point_id': pending_point_id,
            }

        @app.post('/api/arm_done')
        async def arm_done(req: ArmDoneRequest):
            """机械臂完成 → 触发导航下一个。"""
            bridge.get_logger().info(
                f'收到机械臂完成信号: arm={req.arm} '
                f'result={req.result} point={req.point_id}')

            with bridge.state_lock:
                current_point_id = bridge.current_point_id
                pending_point_id = bridge.arm_pending_point_id

            next_point_id = bridge.next_point_map.get(current_point_id)

            if req.result.strip().lower() != 'ok':
                message = (
                    f'机械臂任务失败，未发布下一点；'
                    f'下一点位={next_point_id}'
                )
                bridge.get_logger().error(f'  ❌ {message}')
                return {
                    'status': 'failed',
                    'success': False,
                    'message': message,
                    'next_published': False,
                    'next_point_id': next_point_id,
                }

            if current_point_id is None:
                message = '机器人当前不在任何已配置点位附近；下一点位=None'
                bridge.get_logger().error(f'  ❌ {message}')
                return {
                    'status': 'failed',
                    'success': False,
                    'message': message,
                    'next_published': False,
                    'next_point_id': None,
                }

            request_point_id = req.point_id.strip()
            if not request_point_id:
                message = '机械臂完成消息必须提供 point_id'
                bridge.get_logger().error(f'  ❌ {message}')
                return {
                    'status': 'failed',
                    'success': False,
                    'message': message,
                    'next_published': False,
                    'next_point_id': next_point_id,
                }

            if request_point_id != current_point_id:
                message = (
                    f'上报点位 {request_point_id} 与当前点位 '
                    f'{current_point_id} 不一致；'
                    f'下一点位={next_point_id}'
                )
                bridge.get_logger().error(f'  ❌ {message}')
                return {
                    'status': 'failed',
                    'success': False,
                    'message': message,
                    'next_published': False,
                    'next_point_id': next_point_id,
                }

            if request_point_id != pending_point_id:
                message = (
                    f'点位 {request_point_id} 没有待完成的机械臂任务；'
                    f'当前待处理点位={pending_point_id or "None"}'
                )
                bridge.get_logger().error(f'  ❌ {message}')
                return {
                    'status': 'failed',
                    'success': False,
                    'message': message,
                    'next_published': False,
                    'next_point_id': next_point_id,
                }

            if next_point_id is None:
                with bridge.state_lock:
                    if (
                        bridge.current_point_id != request_point_id
                        or bridge.arm_pending_point_id != request_point_id
                    ):
                        message = '点位状态已变化，未确认机械臂完成'
                        bridge.get_logger().error(f'  ❌ {message}')
                        return {
                            'status': 'failed',
                            'success': False,
                            'message': message,
                            'next_published': False,
                            'next_point_id': None,
                        }
                    bridge.arm_pending_point_id = None
                bridge.get_logger().info(
                    f'  ✅ 点位 {current_point_id} 任务完成，'
                    '巡检结束；下一点位=None')
                return {
                    'status': 'finished',
                    'success': True,
                    'message': '巡检结束；下一点位=None',
                    'next_published': False,
                    'next_point_id': None,
                }

            msg = String()
            msg.data = next_point_id
            with bridge.state_lock:
                if (
                    bridge.current_point_id != request_point_id
                    or bridge.arm_pending_point_id != request_point_id
                ):
                    message = '点位状态已变化，未发布下一点'
                    bridge.get_logger().error(f'  ❌ {message}')
                    return {
                        'status': 'failed',
                        'success': False,
                        'message': message,
                        'next_published': False,
                        'next_point_id': next_point_id,
                    }
                try:
                    bridge.next_pub.publish(msg)
                except Exception as exception:
                    message = (
                        f'发布 {bridge.next_topic} 失败: {exception}；'
                        f'下一点位={next_point_id}'
                    )
                    bridge.get_logger().error(f'  ❌ {message}')
                    return {
                        'status': 'failed',
                        'success': False,
                        'message': message,
                        'next_published': False,
                        'next_point_id': next_point_id,
                    }
                bridge.arm_pending_point_id = None

            bridge.get_logger().info(
                f'  ✅ 已发 {bridge.next_topic}={next_point_id}，'
                f'导航将前往 {next_point_id} 号点；'
                f'下一点位={next_point_id}')
            return {
                'status': 'ok',
                'success': True,
                'message': f'发布成功；下一点位={next_point_id}',
                'next_published': True,
                'next_point_id': next_point_id,
            }

        @app.post('/api/trigger_nav')
        async def trigger_nav(req: ArmDoneRequest):
            """启动巡检；可通过 point_id 直接指定目标点位。"""
            next_point_id = req.point_id.strip() or '1'

            if next_point_id not in bridge.next_point_map:
                message = f'未知点位 {next_point_id}，未发布导航指令'
                bridge.get_logger().error(f'  ❌ {message}')
                return {
                    'status': 'failed',
                    'success': False,
                    'message': message,
                    'next_published': False,
                    'next_point_id': next_point_id,
                }

            msg = String()
            msg.data = next_point_id
            try:
                bridge.next_pub.publish(msg)
            except Exception as exception:
                message = (
                    f'发布 {bridge.next_topic} 失败: {exception}；'
                    f'下一点位={next_point_id}'
                )
                bridge.get_logger().error(f'  ❌ {message}')
                return {
                    'status': 'failed',
                    'success': False,
                    'message': message,
                    'next_published': False,
                    'next_point_id': next_point_id,
                }

            bridge.get_logger().info(
                f'  ✅ 已发 {bridge.next_topic}={next_point_id}，'
                f'导航将前往 {next_point_id} 号点；'
                f'下一点位={next_point_id}')
            return {
                'status': 'ok',
                'success': True,
                'message': f'发布成功；下一点位={next_point_id}',
                'next_published': True,
                'next_point_id': next_point_id,
            }

        return app

    def _start_http_server(self):
        app = self._app
        port = self.http_port

        def run():
            uvicorn.run(
                app,
                host='0.0.0.0',
                port=port,
                log_level='warning',
            )

        t = threading.Thread(target=run, daemon=True)
        t.start()
        self.get_logger().info(f'HTTP server 启动在 0.0.0.0:{port}')


def main():
    rclpy.init()
    node = ArmNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
