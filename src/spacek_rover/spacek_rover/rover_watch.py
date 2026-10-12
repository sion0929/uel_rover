"""rover_watch 노드: 착륙 시뮬레이션 중 로버가 밀리거나 기울었는지 기록한다.

/rover/odom_gt 를 WatchTracker 로 추적해 1 Hz 로 /rover/watch/result (std_msgs/String, JSON) 를
발행하고 <csv_dir>/<name>_rover_watch.json 에 같은 내용을 덮어쓴다 (종료 시 한 번 더).
start_topic (sensor_msgs/JointState) 을 주면 그 토픽의 첫 메시지에서 기준 자세를 잡는다
(settle_s 를 기다리지 않음): 드론 스폰 직후 착지 충격을 놓치지 않기 위함.
"""

import json
import os

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from spacek_rover.drive_eval import quat_to_rpy
from spacek_rover.watch_eval import WatchTracker
from std_msgs.msg import String


class RoverWatch(Node):

    def __init__(self):
        super().__init__('rover_watch')
        topic = str(self._param('odom_gt_topic', '/rover/odom_gt'))
        self.limits = (float(self._param('max_shift_m', 0.005)),
                       float(self._param('max_tilt_deg', 1.0)),
                       float(self._param('max_yaw_deg', 0.5)))
        self.csv_dir = str(self._param('csv_dir', '~/landing_results'))
        self.name = str(self._param('name', 'rover'))
        self.tracker = WatchTracker(float(self._param('settle_s', 2.0)))
        start_topic = str(self._param('start_topic', ''))
        self.pub = self.create_publisher(String, '/rover/watch/result', 10)
        self.create_subscription(Odometry, topic, self._on_odom, 10)
        if start_topic:
            self.create_subscription(JointState, start_topic, self._on_start, 10)
        self.create_timer(1.0, self._report)

    def _param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return self.get_parameter(name).value

    def _on_start(self, _msg):
        if not self.tracker.started:
            self.tracker.start()
            self.get_logger().info('start 토픽 수신: 지금 로버 자세를 기준으로 잡음')

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tracker.update(t, (p.x, p.y, p.z) + quat_to_rpy(q.x, q.y, q.z, q.w))

    def _report(self):
        res = self.write_result()
        m = String()
        m.data = json.dumps(res, ensure_ascii=False)
        self.pub.publish(m)

    def write_result(self):
        """현재 판정을 파일에 쓰고 돌려준다 (csv_dir 가 비어 있으면 파일 없음)."""
        res = self.tracker.result(*self.limits)
        if self.csv_dir:
            d = os.path.expanduser(self.csv_dir)
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, self.name + '_rover_watch.json')
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(res, f, ensure_ascii=False, indent=2)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = RoverWatch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_result()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
