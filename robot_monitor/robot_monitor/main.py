# Copyright 2026 PhysicalFleet
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
"""`ros2 run robot_monitor robot_monitor`: the monitor node plus its web dashboard."""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException

from .monitor import make_executor, Monitor
from .server import serve, urls


def main(args=None):
    rclpy.init(args=args)
    node = Monitor()
    host = node.declare_parameter('host', '0.0.0.0').value
    port = int(node.declare_parameter('port', 9191).value)
    try:
        _, bound = serve(node, host, port)
    except OSError as e:
        node.get_logger().error('cannot open the web dashboard on %s:%d: %s' % (host, port, e))
        bound = None
    if bound:
        node.get_logger().info('Robot Monitor dashboard: ' + '  '.join(urls(host, bound)))
    try:
        ex = make_executor()
        ex.add_node(node)
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
