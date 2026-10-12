"""plate_coupler 노드: 바닥 침하량 d 를 읽어 4개 플레이트 위치 명령을 발행한다.

입력 : /joint_states (sensor_msgs/JointState, floor_joint 위치 = d [m], 아래로 +)
출력 : /platform/<plate_joint>/cmd_pos (std_msgs/Float64, theta - theta0 [rad])
       -> ros_gz_bridge -> /model/landing_platform/joint/<joint>/0/cmd_pos (gz.msgs.Double)
바닥은 구동하지 않는다 (드론 무게 vs 스프링, 물리로만 움직임).
"""

import math

from drone_landing_platform.coupling import make_coupling, relative_command_rad
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class PlateCoupler(Node):

    def __init__(self):
        super().__init__('plate_coupler')
        self.topic = self._p('joint_state_topic', '/joint_states')
        self.floor_joint = self._p('floor_joint', 'floor_joint')
        self.plate_joints = list(self._p(
            'plate_joints',
            ['plate_px_joint', 'plate_mx_joint', 'plate_py_joint', 'plate_my_joint']))
        prefix = self._p('cmd_topic_prefix', '/platform/')
        rate = float(self._p('publish_rate_hz', 200.0))
        self.period = 1.0 / rate if rate > 0.0 else 0.0
        mode = self._p('mode', 'law')
        law = {
            'length_mm': float(self._p('law.length_mm', 107.25)),
            'h0_mm': float(self._p('law.h0_mm', 19.3)),
            'max_sink_mm': float(self._p('law.max_sink_mm', 84.0)),
        }
        tab_d = [float(v) for v in self._p('table.d_mm', [0.0, 60.6])]
        tab_t = [float(v) for v in self._p('table.theta_deg', [10.37, 48.16])]
        self.zero_deg = float(self._p('plate_zero_angle_deg', 10.367))
        self.min_deg = float(self._p('theta_min_deg', 0.0))
        self.max_deg = float(self._p('theta_max_deg', 95.0))
        self.coupling = make_coupling(mode, law, tab_d, tab_t)

        self.pubs = {}
        for j in self.plate_joints:
            self.pubs[j] = self.create_publisher(Float64, prefix + j + '/cmd_pos', 10)
        self.sub = self.create_subscription(JointState, self.topic, self._on_state, 1)
        self.last_pub_t = None
        self.last_log_t = None
        self.warned = False
        th0 = self.coupling.theta_deg(0.0)
        self.get_logger().info(
            '커플링 mode=%s, theta(0)=%.3f deg, SDF 0점=%.3f deg, 명령 토픽=%s<joint>/cmd_pos'
            % (mode, th0, self.zero_deg, prefix))

    def _p(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return self.get_parameter(name).value

    def _on_state(self, msg):
        if self.floor_joint not in msg.name:
            if not self.warned:
                self.get_logger().warning('%s 가 %s 에 없습니다' % (self.floor_joint, self.topic))
                self.warned = True
            return
        idx = msg.name.index(self.floor_joint)
        if idx >= len(msg.position) or math.isnan(msg.position[idx]):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pub_t is not None and t >= self.last_pub_t:
            if t - self.last_pub_t < self.period - 1e-9:
                return
        self.last_pub_t = t
        d_mm = msg.position[idx] * 1000.0
        theta = self.coupling.theta_deg(d_mm)
        cmd = relative_command_rad(theta, self.zero_deg, self.min_deg, self.max_deg)
        out = Float64()
        out.data = cmd
        for j in self.plate_joints:
            self.pubs[j].publish(out)
        if self.last_log_t is None or t - self.last_log_t >= 1.0 or t < self.last_log_t:
            self.last_log_t = t
            self.get_logger().debug('d=%.1f mm theta=%.2f deg cmd=%.4f rad' % (d_mm, theta, cmd))


def main(args=None):
    rclpy.init(args=args)
    node = PlateCoupler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
