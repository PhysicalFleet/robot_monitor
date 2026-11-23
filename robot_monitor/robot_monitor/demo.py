# Copyright 2026 The Robot Monitor Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
A small robot with typical real-world bugs, to try Robot Monitor in 10 seconds.

    ros2 launch robot_monitor demo.launch.py

Each role runs in its own process, like a real robot. The bugs are the ones
people lose hours on: QoS mismatch, a mistyped topic, duplicate node names, a
CPU-bound node, a slow memory leak, a flaky sensor, a lifecycle node nobody
activated, a node that logs errors, a /diagnostics warning and a TF frame with two
parents.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def _stamp(node, header, frame):
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = frame


def _have_tf():
    try:
        # tf2_msgs is not part of ros-core, so the TF parts of the demo are optional.
        import tf2_msgs.msg  # noqa: F401
        return True
    except ImportError:
        return False


def _tf_publisher(node, static=False):
    from tf2_msgs.msg import TFMessage
    if static:
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        return node.create_publisher(TFMessage, '/tf_static', qos)
    return node.create_publisher(TFMessage, '/tf', 100)


def _transform(node, parent, child, x=0.0, y=0.0, z=0.0):
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    _stamp(node, t.header, parent)
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
    t.transform.rotation.w = 1.0
    return t


class Camera(Node):

    def __init__(self):
        super().__init__('camera', namespace='/camera')
        from sensor_msgs.msg import CameraInfo, Image
        self.img = Image(height=240, width=320, encoding='rgb8', step=320 * 3)
        self.img.data = bytes(320 * 240 * 3)
        self.pub = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
        self.info = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.ci = CameraInfo(height=240, width=320)
        self.create_timer(1 / 15.0, self.tick)

    def tick(self):
        _stamp(self, self.img.header, 'camera')
        self.ci.header = self.img.header
        self.pub.publish(self.img)
        self.info.publish(self.ci)


class Lidar(Node):

    def __init__(self):
        super().__init__('lidar')
        from sensor_msgs.msg import LaserScan
        self.msg = LaserScan(angle_min=-math.pi, angle_max=math.pi,
                             angle_increment=2 * math.pi / 360, range_min=0.1, range_max=12.0)
        self.msg.ranges = [3.0] * 360
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)

    def tick(self):
        _stamp(self, self.msg.header, 'laser')
        self.pub.publish(self.msg)


class Slam(Node):
    """BUG: subscribes RELIABLE to a BEST_EFFORT lidar, so it never gets a scan."""

    def __init__(self):
        super().__init__('slam')
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        self.create_subscription(LaserScan, '/scan', lambda m: None,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        m = OccupancyGrid()
        m.info.width = m.info.height = 50
        m.data = [0] * 2500
        _stamp(self, m.header, 'map')
        self.map_pub.publish(m)
        self.create_timer(20.0, lambda: self.get_logger().warning('no scans received yet'))
        # BUG: publishes map -> base_link instead of map -> odom, so base_link gets two parents.
        if _have_tf():
            from tf2_msgs.msg import TFMessage
            tf = _tf_publisher(self)
            self.create_timer(0.1, lambda: tf.publish(TFMessage(
                transforms=[_transform(self, 'map', 'base_link')])))


class Detector(Node):
    """BUG: subscribes to /camera/image_rect, but the camera publishes /camera/image_raw."""

    def __init__(self):
        super().__init__('detector')
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        self.create_subscription(Image, '/camera/image_rect', lambda m: None,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(String, '/detections', 10)
        self.create_timer(0.2, lambda: self.pub.publish(String(data='[]')))


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')
        from nav_msgs.msg import Odometry as Odom
        self.msg = Odom()
        self.msg.child_frame_id = 'base_link'
        self.pub = self.create_publisher(Odom, '/odom', 10)
        self.tf = None
        if _have_tf():
            from tf2_msgs.msg import TFMessage
            self.tf, self.tfmsg = _tf_publisher(self), TFMessage
            self._static = _tf_publisher(self, static=True)
            self._static.publish(TFMessage(transforms=[
                _transform(self, 'base_link', 'laser', 0.1, 0.0, 0.2),
                _transform(self, 'base_link', 'camera_link', 0.15, 0.0, 0.3),
                _transform(self, 'camera_link', 'camera_optical_frame'),
                _transform(self, 'base_link', 'imu_link')]))
        self.create_timer(0.02, self.tick)

    def tick(self):
        _stamp(self, self.msg.header, 'odom')
        self.pub.publish(self.msg)
        if self.tf is not None:
            self.tf.publish(self.tfmsg(transforms=[_transform(self, 'odom', 'base_link')]))


class Planner(Node):
    """BUG: a busy loop in a single-threaded executor eats a whole core."""

    def __init__(self):
        super().__init__('planner')
        from nav_msgs.msg import Odometry as Odom, Path
        self.create_subscription(Odom, '/odom', lambda m: None, 10)
        self.pub = self.create_publisher(Path, '/plan', 10)
        self.create_timer(1.0, self.tick)

    def tick(self):
        end = time.monotonic() + 0.95
        x = 0.0
        while time.monotonic() < end:
            x += math.sqrt(12345.678)
        from nav_msgs.msg import Path
        p = Path()
        _stamp(self, p.header, 'map')
        self.pub.publish(p)


class Mapper(Node):
    """BUG: keeps every message it ever received (memory leak, capped at ~270 MB)."""

    def __init__(self):
        super().__init__('mapper')
        from nav_msgs.msg import Odometry as Odom
        self.msgs, self.blobs = [], []
        self.create_subscription(Odom, '/odom', self.on_odom, 10)
        self.create_timer(1.0, self.grow)

    def on_odom(self, m):
        if len(self.msgs) < 30000:   # the cap keeps a forgotten demo from hurting the machine
            self.msgs.append(m)

    def grow(self):
        if len(self.blobs) < 420:    # ~36 MB/min up to ~250 MB
            self.blobs.append(bytearray(600 * 1024))


class Imu(Node):
    """BUG: a flaky driver; every ~40 s the sensor stalls for 12 s."""

    def __init__(self):
        super().__init__('imu_driver')
        from sensor_msgs.msg import Imu as ImuMsg
        self.msg = ImuMsg()
        self.pub = self.create_publisher(ImuMsg, '/imu/data', qos_profile_sensor_data)
        self.t0 = time.monotonic()
        self.create_timer(0.01, self.tick)
        self.create_timer(5.0, self.check)

    def stalled(self):
        return (time.monotonic() - self.t0) % 40 > 28

    def tick(self):
        if not self.stalled():
            _stamp(self, self.msg.header, 'imu')
            self.pub.publish(self.msg)

    def check(self):
        if self.stalled():
            self.get_logger().error('IMU read timeout on /dev/ttyUSB0 (no data for 5 s)')


class Driver(Node):
    """BUG: two processes start a node with the same name."""

    def __init__(self):
        super().__init__('motor_driver')
        from std_msgs.msg import Float32
        self.pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.create_timer(1.0, lambda: self.pub.publish(Float32(data=11.1)))
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        self.dpub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        def diag():
            a = DiagnosticArray()
            a.header.stamp = self.get_clock().now().to_msg()
            a.status = [DiagnosticStatus(level=DiagnosticStatus.WARN, name='motors/battery',
                                         hardware_id='bms0', message='Battery low: 11.1 V',
                                         values=[KeyValue(key='voltage', value='11.1')])]
            self.dpub.publish(a)
        self.create_timer(1.0, diag)


def _lifecycle_node():
    from rclpy.lifecycle import Node as LNode

    class Arm(LNode):
        """A lifecycle node that nobody configures or activates."""

        def __init__(self):
            super().__init__('arm_controller')
    return Arm()


# One process per entry, like a real robot ('driver' twice on purpose: duplicate node names).
DEMO_PROCESSES = ['camera', 'lidar', 'slam', 'detector', 'odometry', 'planner', 'mapper', 'imu',
                  'driver', 'driver', 'arm']

ROLES = {
    'camera': Camera, 'lidar': Lidar, 'slam': Slam, 'detector': Detector,
    'odometry': Odometry, 'planner': Planner, 'mapper': Mapper, 'imu': Imu,
    'driver': Driver, 'arm': _lifecycle_node,
}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--role', choices=sorted(ROLES), required=True)
    args = parser.parse_args(remove_ros_args(argv)[0:] if argv else [])
    rclpy.init(args=argv)
    node = ROLES[args.role]()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
