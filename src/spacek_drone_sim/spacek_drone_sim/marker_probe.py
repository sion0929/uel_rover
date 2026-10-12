"""marker_probe: 카메라 영상에서 마커 번호를 1 Hz 로 출력한다 (마커가 검출되는지 확인용).

검출이 안 되면 config/markers.yaml 의 flip_v 를 true 로 바꿔 다시 실행해 본다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from spacek_drone_sim.markers import detect_ids, image_to_gray, load_markers_config

DEFAULT_TOPICS = ['/drone/down_camera/image_raw', '/drone/front_camera/image_raw']


class MarkerProbe(Node):

    def __init__(self):
        super().__init__('marker_probe')
        default_cfg = os.path.join(get_package_share_directory('spacek_drone_sim'), 'config',
                                   'markers.yaml')
        cfg_path = str(self._param('markers_config', default_cfg)) or default_cfg
        self.family = load_markers_config(cfg_path)['family']
        self.latest = {}
        for topic in list(self._param('image_topics', DEFAULT_TOPICS)):
            self.create_subscription(Image, topic, self._keep(topic), qos_profile_sensor_data)
        self.create_timer(1.0, self._report)

    def _param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return self.get_parameter(name).value

    def _keep(self, topic):
        def callback(msg):
            self.latest[topic] = msg
        return callback

    def _report(self):
        for topic, msg in sorted(self.latest.items()):
            gray = image_to_gray(msg.encoding, msg.height, msg.width, msg.step, msg.data)
            ids = detect_ids(gray, self.family)
            self.get_logger().info('%s: %s 마커 %s' % (topic, self.family, ids))


def main(args=None):
    rclpy.init(args=args)
    node = MarkerProbe()
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
