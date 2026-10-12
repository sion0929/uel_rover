"""landing_controller 노드: 드론 착륙-포획-이륙 시험 상태기계.

드론은 6자유도 가상 조인트(drone_x/y/z/yaw/pitch/roll)로 구성되어 있고, 각 조인트에
ApplyJointForce 로 힘/토크를 준다. 조인트 힘이 모두 0 이면 자유 강체와 같다(= 모터 OFF).

closed_loop:
  INIT -> APPROACH(공중 정렬 후 descent_speed 로 하강) -> TOUCHDOWN(추력 0, 모든 제어 OFF)
  -> CAPTURED(captured_hold_time 유지 후 결과 기록) -> TAKEOFF(추력 ratio x W, 수평/요 OFF)
  -> CLIMB(climb_height 유지) -> DONE
sw_openloop:
  SolidWorks Motion 추력 스케줄(0.85 W -> 0 -> 1.10 W)을 시간 함수로 그대로 재현하고,
  프레임 위 takeoff_clear_height 를 지나면 CLIMB(폐루프)로 전환한다.
출력: /landing/state, /landing/result (std_msgs/String, result 는 JSON), CSV(선택)
"""

import csv
import json
import math
import os

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

DRONE_JOINTS = ['drone_x', 'drone_y', 'drone_z', 'drone_yaw', 'drone_pitch', 'drone_roll']


def wrap_pi(a):
    """각도를 [-pi, pi) 로 정규화한다."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_error_to_square_deg(yaw_rad):
    """정사각 프레임 기준 yaw 오차: 가장 가까운 90 deg 배수와의 차이[deg]."""
    d = math.degrees(yaw_rad)
    return (d + 45.0) % 90.0 - 45.0


def clamp(v, lo, hi):
    """값 v 를 [lo, hi] 로 제한한다."""
    return lo if v < lo else hi if v > hi else v


def _round_or_none(v, n):
    return None if v is None else round(v, n)


class LandingController(Node):

    def __init__(self):
        super().__init__('landing_controller')
        self._declare_all()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/landing/state', latched)
        self.result_pub = self.create_publisher(String, '/landing/result', latched)
        self.force_pubs = {}
        for j in DRONE_JOINTS:
            topic = self.force_prefix + j + '/cmd_force'
            self.force_pubs[j] = self.create_publisher(Float64, topic, 10)
        self.create_subscription(JointState, self.drone_topic, self._on_drone, 1)
        self.create_subscription(JointState, self.platform_topic, self._on_platform, 1)

        self.q = dict.fromkeys(DRONE_JOINTS, 0.0)
        self.qd = dict.fromkeys(DRONE_JOINTS, 0.0)
        self.floor_d = 0.0
        self.floor_v = 0.0
        self.plate_q = dict.fromkeys(self.plate_joints, 0.0)
        self.have_drone = False
        self.have_platform = False

        self.state = 'INIT'
        self.t0 = None
        self.t_state = 0.0
        self.t_last_ctrl = None
        self.t_last_log = None
        self.t_last_state_pub = None
        self.descending = False
        self.z_ref = self.spawn_z
        self.iz = 0.0
        self.td_accum = 0.0
        self.settle_accum = 0.0
        self.done_accum = 0.0
        self.last_cmd = dict.fromkeys(DRONE_JOINTS, 0.0)
        self.ramp_from = dict(self.last_cmd)
        self.hold = (0.0, 0.0, 0.0)
        self.z_target = 0.0
        self.t_touchdown = None
        self.t_floor_stop = None
        self.max_sink = 0.0
        self.capture = None
        self.takeoff_ok = None
        self.result_sent = False
        self.finished = False
        self.t_finish = None

        self.csv_file = None
        self.csv_writer = None
        self._open_csv()
        self.get_logger().info(
            'mode=%s scenario=%s target=(%.1f, %.1f) mm yaw=%.1f deg mass=%.3f kg'
            % (self.mode, self.scenario, self.target_x * 1e3, self.target_y * 1e3,
               math.degrees(self.target_yaw), self.mass))
        self._publish_state()

    # ------------------------------------------------------------------ 파라미터
    def _p(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        if isinstance(default, float) and isinstance(v, int) and not isinstance(v, bool):
            v = float(v)
        return v

    def _declare_all(self):
        p = self._p
        self.mode = p('mode', 'closed_loop')
        self.scenario = p('scenario', 'center')
        self.target_x = p('target_x', 0.0)
        self.target_y = p('target_y', 0.0)
        self.target_yaw = math.radians(p('target_yaw_deg', 0.0))
        self.spawn_x = p('spawn_x', 0.0)
        self.spawn_y = p('spawn_y', 0.0)
        self.spawn_z = p('spawn_z', 0.8084)
        self.mass = p('drone_mass', 1.9)
        self.chain_mass = p('chain_mass', 0.015)
        self.ixx = p('inertia_xx', 0.01203)
        self.iyy = p('inertia_yy', 0.01203)
        self.izz = p('inertia_zz', 0.01956)
        self.g = p('gravity', 9.80665)
        self.drone_topic = p('drone_joint_state_topic', '/drone/joint_states')
        self.platform_topic = p('platform_joint_state_topic', '/joint_states')
        self.force_prefix = p('force_topic_prefix', '/model/simple_drone/joint/')
        self.floor_joint = p('floor_joint', 'floor_joint')
        self.plate_joints = list(p('plate_joints', ['plate_px_joint', 'plate_mx_joint',
                                                    'plate_py_joint', 'plate_my_joint']))
        self.plate_zero_deg = p('plate_zero_angle_deg', 10.367)
        self.frame_top = p('frame_top_z', 0.144)
        self.floor_top0 = p('floor_top_standby_z', 0.1211)
        self.gear = p('gear_bottom_to_origin', 0.1644)
        self.hard_stop = p('floor_hard_stop', 0.0606)
        rate = p('control_rate_hz', 200.0)
        self.ctrl_period = 1.0 / rate if rate > 0.0 else 0.0
        self.fmax_xy = p('max_force_xy', 20.0)
        self.fmax_z = p('max_force_z', 60.0)
        self.tmax = p('max_torque', 2.0)
        self.kp_xy = p('kp_xy', 16.0)
        self.kd_xy = p('kd_xy', 8.0)
        self.kp_z = p('kp_z', 25.0)
        self.kd_z = p('kd_z', 10.0)
        self.ki_z = p('ki_z', 2.0)
        self.i_max_z = p('i_max_z', 2.0)
        self.kp_yaw = p('kp_yaw', 25.0)
        self.kd_yaw = p('kd_yaw', 10.0)
        self.kp_att = p('kp_att', 100.0)
        self.kd_att = p('kd_att', 20.0)
        self.approach_height = p('approach_height', 0.5)
        self.align_min_time = p('align_min_time', 2.0)
        self.align_timeout = p('align_timeout', 15.0)
        self.align_pos_tol = p('align_pos_tol', 0.002)
        self.align_yaw_tol = math.radians(p('align_yaw_tol_deg', 1.0))
        self.descent_speed = p('descent_speed', 0.3)
        self.td_gap_tol = p('touchdown_gap_tol', 0.012)
        self.td_vz_tol = p('touchdown_vz_tol', 0.08)
        self.td_confirm = p('touchdown_confirm_time', 0.15)
        self.cut_ramp = p('thrust_cut_ramp_time', 0.05)
        self.settle_vel_tol = p('settle_vel_tol', 0.005)
        self.settle_time = p('settle_time', 0.3)
        self.settle_timeout = p('settle_timeout', 3.0)
        self.captured_hold = p('captured_hold_time', 3.0)
        self.takeoff_ratio = p('takeoff_thrust_ratio', 1.5)
        self.takeoff_clear = p('takeoff_clear_height', 0.1)
        self.takeoff_timeout = p('takeoff_timeout', 5.0)
        self.takeoff_att = p('takeoff_attitude_hold', True)
        self.climb_height = p('climb_height', 0.6)
        self.climb_speed = p('climb_speed', 0.5)
        self.done_tol = p('done_tol', 0.02)
        self.done_hold = p('done_hold_time', 1.5)
        self.ol_r_desc = p('ol_descent_ratio', 0.85)
        self.ol_hold_until = p('ol_descent_hold_until', 0.30)
        self.ol_cut_end = p('ol_cut_end', 0.45)
        self.ol_to_start = p('ol_takeoff_start', 2.0)
        self.ol_to_full = p('ol_takeoff_full', 2.2)
        self.ol_r_to = p('ol_takeoff_ratio', 1.10)
        self.ol_capture_t = p('ol_capture_log_time', 1.95)
        self.ol_att = p('ol_attitude_hold', False)
        self.pos_tol = p('pos_tol', 0.02)
        self.yaw_tol_deg = p('yaw_tol_deg', 12.0)
        self.max_sim_time = p('max_sim_time', 60.0)
        self.csv_dir = p('csv_dir', '')
        log_rate = p('log_rate_hz', 50.0)
        self.log_period = 1.0 / log_rate if log_rate > 0.0 else 0.0
        self.shutdown_on_done = p('shutdown_on_done', False)
        self.linger = p('done_linger_time', 1.0)
        if self.mode not in ('closed_loop', 'sw_openloop'):
            raise ValueError('mode 는 closed_loop 또는 sw_openloop 여야 합니다: %s' % self.mode)

    # ------------------------------------------------------------------ 입력
    def _on_platform(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            if math.isnan(pos):
                continue
            if name == self.floor_joint:
                self.floor_d = pos
                self.floor_v = vel
            elif name in self.plate_q:
                self.plate_q[name] = pos
        self.have_platform = True

    def _on_drone(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        for i, name in enumerate(msg.name):
            if name in self.q:
                if i < len(msg.position):
                    self.q[name] = msg.position[i]
                if i < len(msg.velocity):
                    self.qd[name] = msg.velocity[i]
        self.have_drone = True
        if self.t_last_ctrl is not None and t < self.t_last_ctrl:
            self.t_last_ctrl = None          # 시뮬레이션 리셋
        if self.t_last_ctrl is not None and t - self.t_last_ctrl < self.ctrl_period - 1e-9:
            return
        dt = 0.0 if self.t_last_ctrl is None else t - self.t_last_ctrl
        self.t_last_ctrl = t
        if not self.finished:
            self._step(t, dt)

    # ------------------------------------------------------------------ 상태량
    def _pose(self):
        x = self.spawn_x + self.q['drone_x']
        y = self.spawn_y + self.q['drone_y']
        z = self.spawn_z + self.q['drone_z']
        return x, y, z, self.q['drone_yaw'], self.q['drone_pitch'], self.q['drone_roll']

    def _weight(self):
        return (self.mass + self.chain_mass) * self.g

    def _floor_top(self):
        return self.floor_top0 - self.floor_d

    def _plate_theta_deg(self):
        return {j: self.plate_zero_deg + math.degrees(v) for j, v in self.plate_q.items()}

    # ------------------------------------------------------------------ 제어 법칙
    def _f_xy(self, x_ref, y_ref):
        x, y = self._pose()[:2]
        fx = self.mass * (self.kp_xy * (x_ref - x) - self.kd_xy * self.qd['drone_x'])
        fy = self.mass * (self.kp_xy * (y_ref - y) - self.kd_xy * self.qd['drone_y'])
        return clamp(fx, -self.fmax_xy, self.fmax_xy), clamp(fy, -self.fmax_xy, self.fmax_xy)

    def _f_z(self, z_ref, vz_ref, dt):
        z = self._pose()[2]
        e = z_ref - z
        self.iz = clamp(self.iz + self.ki_z * self.mass * e * dt, -self.i_max_z, self.i_max_z)
        fz = (self._weight() + self.iz
              + self.mass * (self.kp_z * e + self.kd_z * (vz_ref - self.qd['drone_z'])))
        return clamp(fz, 0.0, self.fmax_z)

    def _t_yaw(self, yaw_ref):
        e = wrap_pi(yaw_ref - self.q['drone_yaw'])
        tau = self.izz * (self.kp_yaw * e - self.kd_yaw * self.qd['drone_yaw'])
        return clamp(tau, -self.tmax, self.tmax)

    def _t_level(self):
        """roll/pitch 를 0 으로 유지하는 토크 (pitch, roll)."""
        kp, kd = self.kp_att, self.kd_att
        tp = self.iyy * (-kp * self.q['drone_pitch'] - kd * self.qd['drone_pitch'])
        tr = self.ixx * (-kp * self.q['drone_roll'] - kd * self.qd['drone_roll'])
        return clamp(tp, -self.tmax, self.tmax), clamp(tr, -self.tmax, self.tmax)

    def _send(self, fx=0.0, fy=0.0, fz=0.0, tyaw=0.0, tpitch=0.0, troll=0.0):
        vals = {'drone_x': fx, 'drone_y': fy, 'drone_z': fz,
                'drone_yaw': tyaw, 'drone_pitch': tpitch, 'drone_roll': troll}
        for j, v in vals.items():
            m = Float64()
            m.data = float(v)
            self.force_pubs[j].publish(m)
        self.last_cmd = vals

    def _ol_ratio(self, tr):
        if tr < self.ol_hold_until:
            return self.ol_r_desc
        if tr < self.ol_cut_end:
            span = max(self.ol_cut_end - self.ol_hold_until, 1e-6)
            return self.ol_r_desc * (1.0 - (tr - self.ol_hold_until) / span)
        if tr < self.ol_to_start:
            return 0.0
        if tr < self.ol_to_full:
            span = max(self.ol_to_full - self.ol_to_start, 1e-6)
            return self.ol_r_to * (tr - self.ol_to_start) / span
        return self.ol_r_to

    # ------------------------------------------------------------------ 상태기계
    def _set_state(self, new, t):
        self.get_logger().info('[%.3f s] %s -> %s' % (t - self.t0, self.state, new))
        self.state = new
        self.t_state = t
        self.iz = 0.0
        self._publish_state()

    def _publish_state(self):
        m = String()
        m.data = self.state
        self.state_pub.publish(m)

    def _step(self, t, dt):
        if self.t0 is None:
            self.t0 = t
            self.t_state = t
            self.z_ref = self.spawn_z
            self._set_state('APPROACH', t)
        tr = t - self.t0
        x, y, z, yaw, pitch, roll = self._pose()
        gear_z = z - self.gear
        gap = gear_z - self._floor_top()
        vz_rel = self.qd['drone_z'] + self.floor_v
        ts = t - self.t_state
        self.max_sink = max(self.max_sink, self.floor_d)
        if self.t_floor_stop is None and self.floor_d >= self.hard_stop - 0.0005:
            self.t_floor_stop = tr
        if self.mode == 'closed_loop':
            self._step_closed(t, dt, ts, gap, vz_rel)
        else:
            self._step_openloop(t, dt, tr, ts, gap, vz_rel)
        if self.state == 'CLIMB':
            self._do_climb(t, dt, ts)
        elif self.state == 'DONE':
            self._do_hold(dt)
            if self.t_finish is not None and t - self.t_finish >= self.linger:
                if self.shutdown_on_done:
                    self.finished = True
        if tr > self.max_sim_time and self.state != 'DONE':
            self.get_logger().error('max_sim_time 초과 (state=%s)' % self.state)
            self.hold = (x, y, yaw)
            self.z_ref = z
            self._set_state('DONE', t)
            self._finish(t, timeout=True)
        self._log(t, tr)
        if self.t_last_state_pub is None or t - self.t_last_state_pub >= 1.0:
            self.t_last_state_pub = t
            self._publish_state()

    def _touchdown_check(self, dt, gap, vz_rel):
        if gap < self.td_gap_tol and abs(vz_rel) < self.td_vz_tol:
            self.td_accum += dt
        else:
            self.td_accum = 0.0
        return self.td_accum >= self.td_confirm

    def _settled(self, dt):
        if abs(self.floor_v) < self.settle_vel_tol:
            self.settle_accum += dt
        else:
            self.settle_accum = 0.0
        return self.settle_accum >= self.settle_time

    def _step_closed(self, t, dt, ts, gap, vz_rel):
        x, y, z, yaw = self._pose()[:4]
        if self.state == 'APPROACH':
            z_hold = self.frame_top + self.approach_height + self.gear
            if not self.descending:
                self.z_ref = z_hold
                vz_ref = 0.0
                slow = abs(self.qd['drone_x']) < 0.01 and abs(self.qd['drone_y']) < 0.01
                aligned = (slow
                           and abs(x - self.target_x) < self.align_pos_tol
                           and abs(y - self.target_y) < self.align_pos_tol
                           and abs(wrap_pi(self.target_yaw - yaw)) < self.align_yaw_tol)
                if (ts >= self.align_min_time and aligned) or ts >= self.align_timeout:
                    self.descending = True
                    self.get_logger().info('정렬 완료(%.2f s) -> 하강 시작' % ts)
            else:
                z_min = self._floor_top() + self.gear - 0.05
                self.z_ref = max(self.z_ref - self.descent_speed * dt, z_min)
                vz_ref = -self.descent_speed if self.z_ref > z_min else 0.0
            fx, fy = self._f_xy(self.target_x, self.target_y)
            fz = self._f_z(self.z_ref, vz_ref, dt)
            tp, tr_ = self._t_level()
            self._send(fx, fy, fz, self._t_yaw(self.target_yaw), tp, tr_)
            if self.descending and self._touchdown_check(dt, gap, vz_rel):
                self.t_touchdown = t - self.t0
                self.ramp_from = dict(self.last_cmd)
                self._set_state('TOUCHDOWN', t)
        elif self.state == 'TOUCHDOWN':
            self._motors_off_ramp(ts)
            if ts >= self.cut_ramp and (self._settled(dt) or ts >= self.settle_timeout):
                self._set_state('CAPTURED', t)
        elif self.state == 'CAPTURED':
            self._send()
            if ts >= self.captured_hold:
                self._record_capture(t)
                self._set_state('TAKEOFF', t)
        elif self.state == 'TAKEOFF':
            self._do_takeoff(t, ts, self.takeoff_ratio * self._weight(), self.takeoff_att)

    def _step_openloop(self, t, dt, tr, ts, gap, vz_rel):
        if self.state in ('APPROACH', 'TOUCHDOWN', 'CAPTURED'):
            fz = self._ol_ratio(tr) * self._weight()
            tp, tr_ = self._t_level() if self.ol_att else (0.0, 0.0)
            self._send(fz=fz, tpitch=tp, troll=tr_)
            if self.state == 'APPROACH' and self._touchdown_check(dt, gap, vz_rel):
                self.t_touchdown = tr
                self._set_state('TOUCHDOWN', t)
            if self.state == 'TOUCHDOWN' and tr >= self.ol_cut_end and self._settled(dt):
                self._set_state('CAPTURED', t)
            if tr >= self.ol_capture_t and self.capture is None:
                self._record_capture(t)
            if tr >= self.ol_to_start:
                if self.capture is None:
                    self._record_capture(t)
                self._set_state('TAKEOFF', t)
        elif self.state == 'TAKEOFF':
            self._do_takeoff(t, ts, self._ol_ratio(tr) * self._weight(), self.ol_att)

    def _motors_off_ramp(self, ts):
        k = 0.0 if self.cut_ramp <= 0.0 else clamp(1.0 - ts / self.cut_ramp, 0.0, 1.0)
        self._send(**{
            'fx': k * self.ramp_from['drone_x'], 'fy': k * self.ramp_from['drone_y'],
            'fz': k * self.ramp_from['drone_z'], 'tyaw': k * self.ramp_from['drone_yaw'],
            'tpitch': k * self.ramp_from['drone_pitch'],
            'troll': k * self.ramp_from['drone_roll']})

    def _do_takeoff(self, t, ts, fz, attitude_hold):
        tp, tr_ = self._t_level() if attitude_hold else (0.0, 0.0)
        self._send(fz=clamp(fz, 0.0, self.fmax_z), tpitch=tp, troll=tr_)
        x, y, z, yaw = self._pose()[:4]
        cleared = (z - self.gear) > self.frame_top + self.takeoff_clear
        if cleared or ts >= self.takeoff_timeout:
            self.takeoff_ok = bool(cleared)
            if not cleared:
                msg = '이륙 시간 초과: 프레임 위 %.2f m 에 도달하지 못함' % self.takeoff_clear
                self.get_logger().warning(msg)
            self.hold = (x, y, yaw)
            self.z_ref = z
            self.z_target = self.frame_top + self.climb_height + self.gear
            self._set_state('CLIMB', t)

    def _do_climb(self, t, dt, ts):
        if self.z_ref < self.z_target:
            self.z_ref = min(self.z_ref + self.climb_speed * dt, self.z_target)
            vz_ref = self.climb_speed if self.z_ref < self.z_target else 0.0
        else:
            vz_ref = 0.0
        self._do_hold(dt, vz_ref)
        z = self._pose()[2]
        if abs(z - self.z_target) < self.done_tol and abs(self.qd['drone_z']) < 0.05:
            self.done_accum += dt
        else:
            self.done_accum = 0.0
        if self.done_accum >= self.done_hold:
            self._set_state('DONE', t)
            self._finish(t, timeout=False)

    def _do_hold(self, dt, vz_ref=0.0):
        fx, fy = self._f_xy(self.hold[0], self.hold[1])
        fz = self._f_z(self.z_ref, vz_ref, dt)
        tp, tr_ = self._t_level()
        self._send(fx, fy, fz, self._t_yaw(self.hold[2]), tp, tr_)

    # ------------------------------------------------------------------ 결과/기록
    def _record_capture(self, t):
        x, y, z, yaw = self._pose()[:4]
        yaw_err = yaw_error_to_square_deg(yaw)
        ok = (abs(x) <= self.pos_tol and abs(y) <= self.pos_tol
              and abs(yaw_err) <= self.yaw_tol_deg)
        self.capture = {
            'time_s': round(t - self.t0, 4),
            'x_mm': round(x * 1e3, 2), 'y_mm': round(y * 1e3, 2),
            'yaw_deg': round(math.degrees(wrap_pi(yaw)), 2),
            'yaw_err_to_90_deg': round(yaw_err, 2),
            'floor_sink_mm': round(self.floor_d * 1e3, 2),
            'plate_theta_deg': {j: round(v, 2) for j, v in self._plate_theta_deg().items()},
            'ok': bool(ok),
        }
        self.get_logger().info('포획 결과: %s' % json.dumps(self.capture, ensure_ascii=False))
        self._publish_result(final=False)

    def _finish(self, t, timeout):
        if self.capture is None:
            self._record_capture(t)
        if self.takeoff_ok is None:
            self.takeoff_ok = False
        self.t_finish = t
        self._publish_result(final=True, timeout=timeout)

    def _publish_result(self, final, timeout=False):
        res = {
            'scenario': self.scenario, 'mode': self.mode, 'final': final, 'timeout': timeout,
            'target': {'x_mm': round(self.target_x * 1e3, 2),
                       'y_mm': round(self.target_y * 1e3, 2),
                       'yaw_deg': round(math.degrees(self.target_yaw), 2)},
            'touchdown_time_s': _round_or_none(self.t_touchdown, 4),
            'floor_stop_time_s': _round_or_none(self.t_floor_stop, 4),
            'max_floor_sink_mm': round(self.max_sink * 1e3, 2),
            'floor_sink_now_mm': round(self.floor_d * 1e3, 2),
            'capture': self.capture,
            'takeoff_ok': self.takeoff_ok,
            'tolerance': {'pos_mm': self.pos_tol * 1e3, 'yaw_deg': self.yaw_tol_deg},
        }
        res['success'] = bool(final and not timeout and self.capture is not None
                              and self.capture['ok'] and self.takeoff_ok)
        m = String()
        m.data = json.dumps(res, ensure_ascii=False)
        self.result_pub.publish(m)
        if final:
            self.get_logger().info('최종 결과: %s' % m.data)
            self._write_result_file(res)
            self.result_sent = True

    def _base_name(self):
        suffix = '' if self.mode == 'closed_loop' else '_' + self.mode
        return self.scenario + suffix

    def _open_csv(self):
        if not self.csv_dir:
            return
        d = os.path.expanduser(self.csv_dir)
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, self._base_name() + '.csv')
        self.csv_file = open(path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ['t_s', 'state', 'x_mm', 'y_mm', 'z_mm', 'gear_z_mm', 'yaw_deg', 'pitch_deg',
             'roll_deg', 'floor_sink_mm', 'theta_px_deg', 'theta_mx_deg', 'theta_py_deg',
             'theta_my_deg', 'fz_cmd_n'])
        self.get_logger().info('CSV 기록: %s' % path)

    def _log(self, t, tr):
        if self.csv_writer is None:
            return
        if self.t_last_log is not None and tr - self.t_last_log < self.log_period - 1e-9:
            return
        self.t_last_log = tr
        x, y, z, yaw, pitch, roll = self._pose()
        th = self._plate_theta_deg()
        row = [round(tr, 4), self.state, round(x * 1e3, 3), round(y * 1e3, 3),
               round(z * 1e3, 3), round((z - self.gear) * 1e3, 3),
               round(math.degrees(yaw), 3), round(math.degrees(pitch), 3),
               round(math.degrees(roll), 3), round(self.floor_d * 1e3, 3)]
        row += [round(th.get(j, float('nan')), 3) for j in self.plate_joints]
        row.append(round(self.last_cmd['drone_z'], 3))
        self.csv_writer.writerow(row)

    def _write_result_file(self, res):
        if not self.csv_dir:
            return
        path = os.path.join(os.path.expanduser(self.csv_dir), self._base_name() + '_result.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(res, f, ensure_ascii=False, indent=2)
        if self.csv_file is not None:
            self.csv_file.flush()

    def close(self):
        """CSV 파일을 닫는다."""
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None


def main(args=None):
    rclpy.init(args=args)
    node = LandingController()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
