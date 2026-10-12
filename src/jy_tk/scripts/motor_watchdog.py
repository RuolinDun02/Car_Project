#!/usr/bin/env python3
"""
电机看门狗 (Motor Watchdog) — 带 4 重保护版

保护机制：
  1. 启动宽限期（默认 10s）：底盘刚上电时驱动器在初始化，不干预
  2. 运动状态检查：只有车在运动（|vx|>0.05 或 |vz|>5°/s）时才判断堵转
  3. 电流合理性检查：|电流| > 50A 视为数据错误，不触发恢复
  4. E3 致命故障锁定：进入 LOCKED 状态，需手动 /motor_watchdog/reset 清除
"""

import time

import rclpy
from rclpy.node import Node
from yhs_can_interfaces.msg import ChassisInfoFb, MotorCmd, IoCmd
from std_msgs.msg import Bool, String


ERROR_LEVEL_NAMES = {0x00: "无故障", 0xE1: "一般故障", 0xE2: "严重故障", 0xE3: "致命故障"}
ERROR_DEVICE_NAMES = {0x00: "无", 0xA0: "遥控器", 0xB0: "电池", 0xC0: "VCU",
                      0xD1: "驱动系统", 0xD2: "转向系统", 0xD3: "制动系统",
                      0xF0: "传感器", 0x80: "其他部件"}
ERROR_DEVICE_ID_NAMES = {0x00: "默认", 0x06: "左前", 0x07: "左后", 0x08: "右前", 0x09: "右后"}
ERROR_CODE_NAMES = {
    0x00: "复位/无异常",
    0x10: "一般异常", 0x11: "失速", 0x12: "堵转", 0x13: "速度超差", 0x14: "速度越限",
    0x15: "位置超差", 0x16: "位置越限", 0x17: "功率越限",
    0x20: "电流异常", 0x21: "过载", 0x22: "过流", 0x23: "短路", 0x24: "电流超差",
    0x25: "电流越限", 0x26: "I2T故障", 0x27: "充电过流",
    0x30: "电压异常", 0x31: "过压", 0x32: "欠压", 0x33: "电压越限",
    0x34: "充电过压", 0x35: "电源电压异常",
    0x40: "温度异常", 0x41: "驱动器过温", 0x42: "电机过温", 0x43: "功率模块过温",
    0x44: "电池过温", 0x45: "电池低温", 0x46: "设备过温", 0x47: "制动电阻过温",
    0x50: "硬件异常", 0x51: "编码器故障", 0x52: "霍尔故障", 0x53: "EEPROM故障",
    0x54: "IGBT故障", 0x55: "制动电阻故障", 0x56: "FLASH故障", 0x57: "FRAM故障",
    0x58: "ADC故障", 0x59: "电机缺相", 0x5A: "功率器件故障",
    0x60: "软件异常", 0x61: "初始化异常",
    0x70: "辅件异常",
    0x80: "通讯异常", 0x81: "通讯硬件错误", 0x82: "CAN通讯超载", 0x83: "节点守护错误",
    0x84: "协议错误", 0x85: "通讯超时", 0x86: "心跳故障", 0x87: "CRC故障",
    0x88: "掉线故障", 0x89: "掉线提示",
}


def decode_error(level, device, device_id, emergency, register):
    level_name = ERROR_LEVEL_NAMES.get(level, f"未知({level:#x})")
    device_name = ERROR_DEVICE_NAMES.get(device, f"未知({device:#x})")
    device_id_name = ERROR_DEVICE_ID_NAMES.get(device_id, f"未知({device_id:#x})")
    code_name = ERROR_CODE_NAMES.get(emergency, f"未知({emergency:#x})")
    return (f"等级={level_name} 部件={device_name} ID={device_id_name} "
            f"编码={code_name}(0x{emergency:02X}) 寄存器=0x{register:04X}")


class MotorWatchdog(Node):
    def __init__(self):
        super().__init__('motor_watchdog')

        # ---------- 参数 ----------
        self.declare_parameter('check_interval',    0.5)
        self.declare_parameter('retry_interval',    2.0)
        self.declare_parameter('max_retry',         10)
        self.declare_parameter('publish_times',     5)
        self.declare_parameter('current_warn_a',    5.0)
        self.declare_parameter('startup_grace',     10.0)   # ★ 新增
        self.declare_parameter('moving_vx_thresh',  0.05)   # ★ 新增
        self.declare_parameter('moving_vz_thresh',  5.0)    # ★ 新增
        self.declare_parameter('current_sanity_a',  50.0)   # ★ 新增
        self.declare_parameter('intervene_remote',  False)
        self.declare_parameter('intervene_charge',  False)

        p = self.get_parameter
        self.check_interval    = p('check_interval').value
        self.retry_interval    = p('retry_interval').value
        self.max_retry         = p('max_retry').value
        self.publish_times     = p('publish_times').value
        self.current_warn_a    = p('current_warn_a').value
        self.startup_grace     = p('startup_grace').value
        self.moving_vx_thresh  = p('moving_vx_thresh').value
        self.moving_vz_thresh  = p('moving_vz_thresh').value
        self.current_sanity_a  = p('current_sanity_a').value
        self.intervene_remote  = p('intervene_remote').value
        self.intervene_charge  = p('intervene_charge').value

        # ---------- 运行状态 ----------
        self.enabled          = True
        self.retry_count      = 0
        self.last_recover_time = 0.0
        self.last_log_time    = {}
        self.need_recover     = False
        self.recover_action   = 'enable'

        # ★ 新增：锁定状态
        self.locked           = False
        self.lock_reason      = ''

        # ★ 新增：记录启动时刻
        self.start_time       = self.get_clock().now().nanoseconds / 1e9

        # ---------- 订阅 ----------
        self.fb_sub = self.create_subscription(
            ChassisInfoFb, '/chassis_info_fb', self.on_chassis_fb, 10)
        self.enable_sub = self.create_subscription(
            Bool, '/motor_watchdog/enable', self.on_enable, 10)
        self.reset_sub = self.create_subscription(
            Bool, '/motor_watchdog/reset', self.on_reset, 10)   # ★ 新增

        # ---------- 发布 ----------
        self.motor_pub = self.create_publisher(MotorCmd, '/motor_cmd', 10)
        self.io_pub    = self.create_publisher(IoCmd, '/io_cmd', 10)
        self.state_pub = self.create_publisher(String, '/motor_watchdog/state', 10)

        # ---------- 定时器 ----------
        self.timer = self.create_timer(self.check_interval, self.tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('motor_watchdog 已启动（带 4 重保护）')
        self.get_logger().info(f'  check_interval={self.check_interval}s')
        self.get_logger().info(f'  startup_grace={self.startup_grace}s  ← 前 N 秒不干预')
        self.get_logger().info(f'  moving_threshold: vx>{self.moving_vx_thresh}m/s, '
                               f'vz>{self.moving_vz_thresh}°/s  ← 只有运动时才判断堵转')
        self.get_logger().info(f'  current_sanity_a={self.current_sanity_a}A  ← 超出视为数据错误')
        self.get_logger().info(f'  current_warn_a={self.current_warn_a}A  ← 预警阈值')
        self.get_logger().info('=' * 60)

    # ================================================================
    # 节流日志
    # ================================================================
    def log_throttled(self, key, level, msg, interval=5.0):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time.get(key, 0.0) < interval:
            return
        self.last_log_time[key] = now
        if level == 'warn':
            self.get_logger().warn(msg)
        elif level == 'error':
            self.get_logger().error(msg)
        elif level == 'info':
            self.get_logger().info(msg)

    # ================================================================
    # 底盘反馈
    # ================================================================
    def on_chassis_fb(self, msg: ChassisInfoFb):
        now = self.get_clock().now().nanoseconds / 1e9

        # ★ 保护 0：锁定状态，直接返回
        if self.locked:
            return

        # ★ 保护 1：启动宽限期
        if now - self.start_time < self.startup_grace:
            self.log_throttled(
                'startup', 'info',
                f'启动宽限期 {self.startup_grace}s 内，跳过检测'
                f'（剩余 {self.startup_grace - (now - self.start_time):.1f}s）',
                5.0)
            return

        # ---- 1. 急停 ----
        if msg.io_fb.io_fb_estop:
            self.log_throttled('estop', 'error',
                '急停开关已按下！请手动顺时针旋转急停旋钮复位', 10.0)
            return

        # ---- 2. 充电中 ----
        charging = msg.io_fb.io_fb_charge_state or msg.io_fb.io_fb_charger_sign
        if charging and not self.intervene_charge:
            self.log_throttled('charge', 'info', '底盘充电中，watchdog 暂停', 30.0)
            return

        # ---- 3. 遥控模式 ----
        if msg.io_fb.io_fb_joypad_ctrl and not self.intervene_remote:
            self.log_throttled('remote', 'info',
                '遥控器在线，watchdog 暂停（避免与遥控冲突）', 30.0)
            return

        # ---- 4. 故障信息 ----
        err = msg.error_fb
        if err.error_fb_level != 0x00:
            desc = decode_error(
                err.error_fb_level,
                err.error_fb_device_type,
                err.error_fb_devive_id,
                err.error_fb_emergency_code,
                err.error_fb_register_code)
            self.log_throttled(f'err_{err.error_fb_emergency_code}',
                'warn', f'底盘故障: {desc}', 5.0)

            # ★ 保护 4：E3 进入锁定状态
            if err.error_fb_level == 0xE3:
                if not self.locked:
                    self.locked = True
                    self.lock_reason = desc
                    self.need_recover = False
                    self.get_logger().error('=' * 60)
                    self.get_logger().error(f'❌ E3 致命故障，进入锁定状态')
                    self.get_logger().error(f'   原因: {desc}')
                    self.get_logger().error(f'   排查后手动清除:')
                    self.get_logger().error(f'   ros2 topic pub --once /motor_watchdog/reset '
                                            f'std_msgs/Bool "{{data: true}}"')
                    self.get_logger().error('=' * 60)
                return

        # ---- 5. 堵转检测 ----
        oc = msg.drive_motor_current_fb
        oc_drive = {
            '左前': oc.drive_motor_oc_flag_fb_lf,
            '左后': oc.drive_motor_oc_flag_fb_lr,
            '右前': oc.drive_motor_oc_flag_fb_rf,
            '右后': oc.drive_motor_oc_flag_fb_rr,
        }
        stalled = [k for k, v in oc_drive.items() if v]

        cur = {
            '左前': oc.drive_motor_current_fb_lf,
            '左后': oc.drive_motor_current_fb_lr,
            '右前': oc.drive_motor_current_fb_rf,
            '右后': oc.drive_motor_current_fb_rr,
        }

        if stalled:
            info = ', '.join(f'{k}={cur[k]:.1f}A' for k in stalled)

            # ★ 保护 2：只有"车在运动"时才判断堵转
            vx = abs(msg.ctrl_fb.ctrl_fb_x_linear)
            vz = abs(msg.ctrl_fb.ctrl_fb_z_angular)
            is_moving = (vx > self.moving_vx_thresh) or (vz > self.moving_vz_thresh)

            if not is_moving:
                self.log_throttled('stall_static', 'warn',
                    f'检测到堵转标志但车未运动 ({info})，'
                    f'疑似误报，跳过（vx={vx:.3f}m/s, vz={vz:.1f}°/s）', 10.0)
                return

            # ★ 保护 3：电流合理性检查
            bogus = [k for k in stalled if abs(cur[k]) > self.current_sanity_a]
            if bogus:
                bogus_info = ', '.join(f'{k}={cur[k]:.1f}A' for k in bogus)
                self.log_throttled('current_bogus', 'warn',
                    f'堵转电流异常: {bogus_info} '
                    f'(> {self.current_sanity_a}A，超物理合理范围，视为数据错误)',
                    10.0)
                return

            self.log_throttled('stall', 'warn',
                f'驱动电机堵转: {info}，准备执行电机总电源重启', 3.0)
            self.need_recover = True
            self.recover_action = 'power_restart'
            return

        # ---- 6. 电流预警（未堵转但偏高）----
        max_cur = max(abs(v) for v in cur.values())
        if max_cur > self.current_warn_a and max_cur <= self.current_sanity_a:
            self.log_throttled('cur_warn', 'warn',
                f'驱动电机电流偏高: {max_cur:.1f}A '
                f'(阈值 {self.current_warn_a}A)', 5.0)

        # ---- 7. 电机使能检测 ----
        mf = msg.motor_fb
        all_drive = (mf.motor_cmd_drive_enable_lf and
                     mf.motor_cmd_drive_enable_lr and
                     mf.motor_cmd_drive_enable_rf and
                     mf.motor_cmd_drive_enable_rr)
        all_steer = (mf.motor_cmd_steering_enable_lf and
                     mf.motor_cmd_steering_enable_lr and
                     mf.motor_cmd_steering_enable_rf and
                     mf.motor_cmd_steering_enable_rr)

        if not (all_drive and all_steer):
            if not self.need_recover:
                self.log_throttled('disabled', 'warn',
                    f'检测到电机失能: drive={all_drive}, steer={all_steer}', 3.0)
            self.need_recover = True
            self.recover_action = 'enable'
        else:
            if self.need_recover:
                self.get_logger().info('电机已恢复使能')
            self.need_recover = False
            self.retry_count = 0

        # ---- 8. 防撞条 ----
        if (msg.io_fb.io_fb_fl_impact_sensor or
            msg.io_fb.io_fb_fm_impact_sensor or
            msg.io_fb.io_fb_fr_impact_sensor or
            msg.io_fb.io_fb_rl_impact_sensor or
            msg.io_fb.io_fb_rm_impact_sensor or
            msg.io_fb.io_fb_rr_impact_sensor):
            self.log_throttled('impact', 'warn',
                '防撞条触发，尝试解锁安全停车', 5.0)
            self.need_recover = True
            self.recover_action = 'unlock'

    # ================================================================
    # 启用/禁用
    # ================================================================
    def on_enable(self, msg: Bool):
        if msg.data != self.enabled:
            self.enabled = msg.data
            self.get_logger().info(
                f'motor_watchdog {"启用" if msg.data else "禁用"}')

    # ★ 新增：清除锁定
    def on_reset(self, msg: Bool):
        if not msg.data:
            return
        if self.locked:
            self.locked = False
            self.lock_reason = ''
            self.retry_count = 0
            self.get_logger().info('✅ 锁定状态已清除，watchdog 恢复工作')

    # ================================================================
    # 主循环
    # ================================================================
    def tick(self):
        # 状态发布
        s = String()
        if self.locked:
            s.data = f'LOCKED: {self.lock_reason}'
        elif self.need_recover:
            s.data = f'NEED_RECOVER: {self.recover_action}'
        else:
            s.data = 'OK'
        self.state_pub.publish(s)

        # 锁定 / 未启用 / 不需要恢复 → 不动作
        if self.locked or not self.enabled or not self.need_recover:
            return

        if self.max_retry > 0 and self.retry_count >= self.max_retry:
            self.log_throttled('max_retry', 'error',
                f'已达最大重试次数 ({self.max_retry})，停止恢复，请人工检查', 60.0)
            self.need_recover = False
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_recover_time < self.retry_interval:
            return

        action = self.recover_action
        if action == 'power_restart':
            self._do_power_restart()
        elif action == 'unlock':
            self._do_unlock()
        else:
            self._do_enable()

        self.retry_count += 1
        self.last_recover_time = now

    # ================================================================
    # 恢复动作
    # ================================================================
    def _do_enable(self):
        cmd = MotorCmd()
        cmd.motor_cmd_drive_enable_lf = True
        cmd.motor_cmd_drive_enable_lr = True
        cmd.motor_cmd_drive_enable_rf = True
        cmd.motor_cmd_drive_enable_rr = True
        cmd.motor_cmd_steering_enable_lf = True
        cmd.motor_cmd_steering_enable_lr = True
        cmd.motor_cmd_steering_enable_rf = True
        cmd.motor_cmd_steering_enable_rr = True

        for _ in range(self.publish_times):
            self.motor_pub.publish(cmd)
            time.sleep(0.02)

        self.get_logger().info(
            f'[恢复] 发送电机使能 (第 {self.retry_count + 1}/{self.max_retry} 次)')

    def _do_power_restart(self):
        cmd_on = MotorCmd()
        cmd_on.motor_cmd_drive_enable_lf = True
        cmd_on.motor_cmd_drive_enable_lr = True
        cmd_on.motor_cmd_drive_enable_rf = True
        cmd_on.motor_cmd_drive_enable_rr = True
        cmd_on.motor_cmd_steering_enable_lf = True
        cmd_on.motor_cmd_steering_enable_lr = True
        cmd_on.motor_cmd_steering_enable_rf = True
        cmd_on.motor_cmd_steering_enable_rr = True
        cmd_on.motor_cmd_power_restart = True

        for _ in range(self.publish_times):
            self.motor_pub.publish(cmd_on)
            time.sleep(0.02)

        cmd_off = MotorCmd()
        cmd_off.motor_cmd_drive_enable_lf = True
        cmd_off.motor_cmd_drive_enable_lr = True
        cmd_off.motor_cmd_drive_enable_rf = True
        cmd_off.motor_cmd_drive_enable_rr = True
        cmd_off.motor_cmd_steering_enable_lf = True
        cmd_off.motor_cmd_steering_enable_lr = True
        cmd_off.motor_cmd_steering_enable_rf = True
        cmd_off.motor_cmd_steering_enable_rr = True
        cmd_off.motor_cmd_power_restart = False

        for _ in range(self.publish_times):
            self.motor_pub.publish(cmd_off)
            time.sleep(0.02)

        self.get_logger().info(
            f'[恢复] 电机总电源重启 + 使能 (第 {self.retry_count + 1}/{self.max_retry} 次)')

    def _do_unlock(self):
        cmd_on = IoCmd()
        cmd_on.io_cmd_unlock = True
        cmd_on.io_cmd_low_power_enable = True

        for _ in range(3):
            self.io_pub.publish(cmd_on)
            time.sleep(0.02)

        cmd_off = IoCmd()
        cmd_off.io_cmd_unlock = False
        cmd_off.io_cmd_low_power_enable = True

        for _ in range(3):
            self.io_pub.publish(cmd_off)
            time.sleep(0.02)

        self.get_logger().info(
            f'[恢复] 安全停车解锁脉冲 (第 {self.retry_count + 1}/{self.max_retry} 次)')

        self._do_enable()


def main():
    rclpy.init()
    node = MotorWatchdog()
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