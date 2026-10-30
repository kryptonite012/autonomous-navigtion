"""Exposes the sorting state machine as ROS services.

Services (std_srvs/Trigger): /sorting/start, /sorting/stop, /sorting/pause, /sorting/resume
Topic: /sorting/status (std_msgs/String, JSON), /sorting/records (std_msgs/String, JSON per record)

Chassis motion for alignment is published on /cmd_vel and therefore passes
through the safety controller in nexus_base.
"""

import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.environ.get("NEXUS_ROOT", os.path.expanduser("~/PROJECT-NEXUS")))

from robot.config import load_config  # noqa: E402
from robot.hardware.factory import build_hardware  # noqa: E402
from robot.sorting.arm import SortingArm  # noqa: E402
from robot.sorting.sorter import Sorter  # noqa: E402
from robot.vision.pipeline import VisionPipeline  # noqa: E402


class SortingNode(Node):
    def __init__(self):
        super().__init__("nexus_sorting")
        self.declare_parameter("config", os.environ.get("NEXUS_CONFIG", "config/nexus.yaml"))
        cfg = load_config(self.get_parameter("config").value)
        keep = {"servos", "bin_sensor"}
        cfg["hardware"] = {k: {**v, "enabled": v.get("enabled", False) and k in keep}
                           for k, v in cfg["hardware"].items()}
        hw = build_hardware(cfg)
        self.vision = VisionPipeline(hw.camera, cfg)
        if hw.camera:
            self.vision.start()
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "sorting/status", 10)
        self.rec_pub = self.create_publisher(String, "sorting/records", 10)
        self.pose = None
        self.create_subscription(Odometry, "odom", self._odom, 10)
        arm = SortingArm(hw.servos, cfg["sorting"]) if hw.servos else None
        self.sorter = Sorter(cfg, arm, hw.bin_sensor, self.vision.latest, drive=self._drive,
                             stop_drive=lambda: self._drive(0.0, 0.0), pose=lambda: self.pose,
                             on_record=lambda r: self.rec_pub.publish(String(data=json.dumps(r.as_dict()))),
                             on_event=lambda t, s, d: self.get_logger().info(f"[{t}] {d}"))
        for name, fn in (("start", self.sorter.start), ("stop", self.sorter.stop),
                         ("pause", self.sorter.pause), ("resume", self.sorter.resume)):
            self.create_service(Trigger, f"sorting/{name}", self._wrap(fn))
        self.create_timer(1.0, lambda: self.status_pub.publish(String(data=json.dumps(self.sorter.status()))))

    def _odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _drive(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)
        return True

    def _wrap(self, fn):
        def handler(_req, resp):
            try:
                fn()
                resp.success, resp.message = True, self.sorter.state.value
            except Exception as exc:
                resp.success, resp.message = False, str(exc)
            return resp
        return handler


def main(args=None):
    rclpy.init(args=args)
    node = SortingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sorter.stop()
        node.vision.stop()
        node.destroy_node()
        rclpy.try_shutdown()
