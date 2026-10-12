"""drive_test 노드: 로버 자동 주행 시험 (settle -> forward -> spin -> arc -> stop).

/rover/cmd_vel 로 단계별 속도 명령을 보내고, 실제 위치(/rover/odom_gt)로 단계마다 판정한다.
출력: /rover/drive_test/result (std_msgs/String, JSON),
      <csv_dir>/drive_test.csv, <csv_dir>/drive_test_result.json
"""

import csv
import json
import math
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from spacek_rover.drive_eval import evaluate_phase, make_phases, quat_to_rpy, summarize
from std_msgs.msg import String

FLOAT_PARAMS = {
    'cmd_rate_hz': 50.0, 'start_timeout_s': 30.0,
    'settle_duration_s': 2.0, 'forward_speed_mps': 0.2, 'forward_duration_s': 5.0,
    'spin_rate_radps': 0.8, 'spin_duration_s': 4.0, 'arc_speed_mps': 0.15,
    'arc_rate_radps': 0.3, 'arc_duration_s': 4.0, 'stop_duration_s': 2.0,
    'ride_height_m': 0.0877, 'settle_z_tol_m': 0.005, 'settle_tilt_tol_deg': 1.0,
    'settle_wheel_speed_tol': 0.05, 'forward_distance_tol': 0.10,
    'forward_heading_tol_deg': 2.0, 'forward_lateral_tol_m': 0.03, 'spin_drift_tol_m': 0.10,
    'stop_speed_tol_mps': 0.01, 'stop_window_s': 0.5, 'linear_accel_mps2': 0.5,
    'angular_accel_radps2': 2.0, 'min_motion_fraction': 0.3,
}
STR_PARAMS = {
    'cmd_topic': '/rover/cmd_vel', 'odom_gt_topic': '/rover/odom_gt',
    'odom_topic': '/rover/odom', 'joint_state_topic': '/rover/joint_states',
    'csv_dir': '~/rover_results',
}
CSV_HEADER = ['t_s', 'phase', 'x_m', 'y_m', 'z_m', 'roll_deg', 'pitch_deg', 'yaw_deg',
              'odom_yaw_deg']


def _odom_pose(msg):
    q = msg.pose.pose.orientation
    roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
    pos = msg.pose.pose.position
    return pos.x, pos.y, pos.z, roll, pitch, yaw


class DriveTest(Node):

    def __init__(self):
        super().__init__('drive_test')
        self.p = {}
        for name, default in FLOAT_PARAMS.items():
            self.p[name] = float(self._param(name, default))
        for name, default in STR_PARAMS.items():
            self.p[name] = str(self._param(name, default))
        self.shutdown_on_done = bool(self._param('shutdown_on_done', False))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, self.p['cmd_topic'], 10)
        self.result_pub = self.create_publisher(String, '/rover/drive_test/result', latched)
        self.create_subscription(Odometry, self.p['odom_gt_topic'], self._on_gt, 10)
        self.create_subscription(Odometry, self.p['odom_topic'], self._on_odom, 10)
        self.create_subscription(JointState, self.p['joint_state_topic'], self._on_joints, 10)
        self.phases = make_phases(self.p)
        self.gt = None
        self.odom_yaw = None
        self.wheel_speeds = None
        self.idx = -1
        self.t_phase = None
        self.t_wait = None
        self.samples = []
        self.odom_yaws = []
        self.results = []
        self.finished = False
        self.csv_file = None
        self.csv_writer = None
        self._open_csv()
        self.create_timer(1.0 / self.p['cmd_rate_hz'], self._tick)
        self.get_logger().info('drive_test 시작: %s'
                               % ' -> '.join(ph['name'] for ph in self.phases))

    def _param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return self.get_parameter(name).value

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_gt(self, msg):
        self.gt = _odom_pose(msg)

    def _on_odom(self, msg):
        self.odom_yaw = _odom_pose(msg)[5]

    def _on_joints(self, msg):
        self.wheel_speeds = [v for n, v in zip(msg.name, msg.velocity)
                             if n.endswith('_wheel_joint')]

    def _send(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.cmd_pub.publish(m)

    def _tick(self):
        if self.finished:
            return
        t = self._now()
        if self.gt is None:
            if self.t_wait is None:
                self.t_wait = t
            elif t - self.t_wait > self.p['start_timeout_s']:
                self.get_logger().error('%s 를 받지 못했습니다' % self.p['odom_gt_topic'])
                self._finish(timeout=True)
            return
        if self.idx < 0:
            self._start_phase(0, t)
        if t - self.t_phase >= self.phases[self.idx]['duration']:
            self._end_phase(t)
            if self.finished:
                return
        phase = self.phases[self.idx]
        self._send(phase['v'], phase['w'])
        self.samples.append((t,) + tuple(self.gt))
        if self.odom_yaw is not None:
            self.odom_yaws.append(self.odom_yaw)
        self._log(t, phase['name'])

    def _start_phase(self, idx, t):
        self.idx = idx
        self.t_phase = t
        self.samples = []
        self.odom_yaws = []
        self.get_logger().info('단계 시작: %s' % self.phases[idx]['name'])

    def _end_phase(self, t):
        phase = self.phases[self.idx]
        wheels = self.wheel_speeds if phase['name'] == 'settle' else None
        res = evaluate_phase(phase, self.samples, self.p, self.odom_yaws, wheels)
        self.results.append(res)
        self.get_logger().info('단계 %s: %s %s' % (phase['name'],
                                                 '통과' if res['passed'] else '실패',
                                                 json.dumps(res['measured'])))
        if self.idx + 1 < len(self.phases):
            self._start_phase(self.idx + 1, t)
        else:
            self._finish(timeout=False)

    def _finish(self, timeout):
        self._send(0.0, 0.0)
        res = summarize(self.results)
        res['timeout'] = bool(timeout)
        if timeout:
            res['success'] = False
        m = String()
        m.data = json.dumps(res, ensure_ascii=False)
        self.result_pub.publish(m)
        self.get_logger().info('최종 결과: %s' % m.data)
        self._write_result(res)
        self.finished = True

    def _open_csv(self):
        if not self.p['csv_dir']:
            return
        d = os.path.expanduser(self.p['csv_dir'])
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, 'drive_test.csv')
        self.csv_file = open(path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.get_logger().info('CSV 기록: %s' % path)

    def _log(self, t, phase_name):
        if self.csv_writer is None:
            return
        x, y, z, roll, pitch, yaw = self.gt
        odom = '' if self.odom_yaw is None else round(math.degrees(self.odom_yaw), 3)
        self.csv_writer.writerow([round(t, 4), phase_name, round(x, 5), round(y, 5),
                                  round(z, 5), round(math.degrees(roll), 3),
                                  round(math.degrees(pitch), 3), round(math.degrees(yaw), 3),
                                  odom])

    def _write_result(self, res):
        if not self.p['csv_dir']:
            return
        path = os.path.join(os.path.expanduser(self.p['csv_dir']), 'drive_test_result.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(res, f, ensure_ascii=False, indent=2)
        self.get_logger().info('결과 파일: %s' % path)
        if self.csv_file is not None:
            self.csv_file.flush()

    def close(self):
        """CSV 파일을 닫는다."""
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None


def main(args=None):
    rclpy.init(args=args)
    node = DriveTest()
    try:
        while rclpy.ok() and not (node.finished and node.shutdown_on_done):
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
