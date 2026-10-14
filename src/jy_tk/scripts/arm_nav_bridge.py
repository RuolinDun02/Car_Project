#!/usr/bin/env python3
"""
导航 ↔ 机械臂 双向桥接

功能：
  1. 订阅 /patrol/arrived，收到后 HTTP 调 8188 触发机械臂
  2. 监听 8189 端口，收到 /api/arm_done 后发布下一点
  3. 固定巡检顺序为 1 → 2 → 结束

用法：
  ros2 run jy_tk arm_nav_bridge.py

机械臂侧对接：
  机械臂完成动作后，调用：
    curl -X POST http://<jetson_ip>:8189/api/arm_done \
      -H 'Content-Type: application/json' \
      -d '{"arm":"BOTH","result":"ok"}'
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
        self.declare_parameter('next_topic', '/patrol/next')
        self.declare_parameter('http_port', 8189)
        self.declare_parameter('arm_speed', 12.0)
        self.declare_parameter('trigger_arm_on_arrive', True)

        p = self.get_parameter
        self.arm_api_base = p('arm_api_base').value.rstrip('/')
        self.arrived_topic = p('arrived_topic').value
        self.next_topic = p('next_topic').value
        self.http_port = int(p('http_port').value)
        self.arm_speed = float(p('arm_speed').value)
        self.trigger_arm_on_arrive = bool(p('trigger_arm_on_arrive').value)

        # 当前已到达并执行机械臂动作的点位。
        self.current_point_id = None

        # 固定巡检顺序：1 → 2 → 结束。
        self.next_point_map = {
            '1': '2',
            '2': None,
        }

        # ---------- ROS pub/sub ----------
        self.create_subscription(
            String, self.arrived_topic, self.on_arrived, 10)
        self.next_pub = self.create_publisher(String, self.next_topic, 10)

        # ---------- HTTP server ----------
        self._app = self._build_fastapi()
        self._start_http_server()

        self.get_logger().info('=' * 60)
        self.get_logger().info('arm_nav_bridge 已启动')
        self.get_logger().info(f'  机械臂 API: {self.arm_api_base}')
        self.get_logger().info(f'  订阅: {self.arrived_topic}')
        self.get_logger().info(f'  发布: {self.next_topic}')
        self.get_logger().info(f'  HTTP 监听: 0.0.0.0:{self.http_port}')
        self.get_logger().info(
            f'  机械臂完成后请调: POST http://<jetson_ip>:'
            f'{self.http_port}/api/arm_done')
        self.get_logger().info('=' * 60)

    # ================================================================
    # ROS → HTTP
    # ================================================================
    def on_arrived(self, msg: String):
        point_id = msg.data
        self.current_point_id = point_id
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
            return {'status': 'healthy', 'node': 'arm_nav_bridge'}

        @app.post('/api/arm_done')
        async def arm_done(req: ArmDoneRequest):
            """机械臂完成 → 触发导航下一个。"""
            bridge.get_logger().info(
                f'收到机械臂完成信号: arm={req.arm} '
                f'result={req.result} point={req.point_id}')

            next_point_id = bridge.next_point_map.get(
                bridge.current_point_id
            )

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

            if bridge.current_point_id is None:
                message = '尚未收到导航到达消息；下一点位=None'
                bridge.get_logger().error(f'  ❌ {message}')
                return {
                    'status': 'failed',
                    'success': False,
                    'message': message,
                    'next_published': False,
                    'next_point_id': None,
                }

            if req.point_id and req.point_id != bridge.current_point_id:
                message = (
                    f'上报点位 {req.point_id} 与当前点位 '
                    f'{bridge.current_point_id} 不一致；'
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

            if next_point_id is None:
                bridge.get_logger().info(
                    f'  ✅ 点位 {bridge.current_point_id} 任务完成，'
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

        @app.post('/api/trigger_nav')
        async def trigger_nav(req: ArmDoneRequest):
            """启动巡检，前往第一个点位。"""
            next_point_id = '1'
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
                f'  ✅ 已发 {bridge.next_topic}=1，'
                '导航将前往 1 号点；下一点位=1')
            return {
                'status': 'ok',
                'success': True,
                'message': '发布成功；下一点位=1',
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
