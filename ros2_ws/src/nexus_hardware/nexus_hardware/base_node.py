"""ROS 2 base driver node.

Wraps the NEXUS hardware abstraction (``robot`` package) and exposes it with
standard ROS interfaces so Nav2 and SLAM Toolbox can use it:

  subscribes  /cmd_vel (geometry_msgs/Twist)  -> SafetyController (never direct to motors)
  publishes   /odom (nav_msgs/Odometry) + TF odom->base_footprint
              /range/<sensor> (sensor_msgs/Range)
              /scan (sensor_msgs/LaserScan) when a LiDAR is configured and the
                  rplidar_ros node is not used
              /imu/data_raw (sensor_msgs/Imu)
              /battery (sensor_msgs/BatteryState)
              /safety/status (std_msgs/String, JSON)
  services    /emergency_stop, /emergency_stop/reset (std_srvs/Trigger)

The robot package must be importable: set NEXUS_ROOT to the repository root
(the launch file does this).
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, LaserScan, Range
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.environ.get("NEXUS_ROOT", os.path.expanduser("~/PROJECT-NEXUS")))

from robot.config import load_config  # noqa: E402
from robot.hardware.factory import build_hardware  # noqa: E402
from robot.motors.kinematics import DiffDriveKinematics, WheelOdometry  # noqa: E402
from robot.safety.controller import SafetyConfig, SafetyController  # noqa: E402


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class NexusBase(Node):
    def __init__(self):
        super().__init__("nexus_base")
        self.declare_parameter("config", os.environ.get("NEXUS_CONFIG", "config/nexus.yaml"))
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("publish_scan", True)
        cfg = load_config(self.get_parameter("config").value)
        self.cfg = cfg
        self.hw = build_hardware(cfg)
        if self.hw.world is not None:
            from robot.simulation.world import SimClock
            self._clock = SimClock(self.hw.world)
            self._clock.start()
        ch = cfg["chassis"]
        self.kin = DiffDriveKinematics(ch["wheel_radius_m"], ch["wheel_base_m"], ch["max_linear_mps"] * 1.6)
        self.odom = WheelOdometry(self.kin, ch["encoder_ticks_per_rev"], gyro_weight=0.85 if self.hw.imu else 0.0)
        self.safety = SafetyController(self.hw.motors, self.kin, SafetyConfig.from_config(cfg), self.hw.estop,
                                       on_event=self._on_event,
                                       expected_sensors=[r.name for r in self.hw.ranges])
        for h in self.hw.health():
            if h.state.value in ("missing", "fault"):
                self.get_logger().warn(f"{h.name}: {h.state.value} ({h.message})")

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", qos_profile_sensor_data)
        self.bat_pub = self.create_publisher(BatteryState, "battery", 10)
        self.safety_pub = self.create_publisher(String, "safety/status", 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data) \
            if self.hw.lidar and self.get_parameter("publish_scan").value else None
        self.range_pubs = {s.name: self.create_publisher(Range, f"range/{s.name}", qos_profile_sensor_data)
                           for s in self.hw.ranges}
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel, 10)
        self.create_service(Trigger, "emergency_stop", self._estop)
        self.create_service(Trigger, "emergency_stop/reset", self._estop_reset)
        self.timer = self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._step)
        self._last_bat = 0.0
        self.get_logger().info(f"nexus_base running in {self.hw.mode} mode")

    def _on_event(self, type_: str, severity: str, text: str) -> None:
        log = self.get_logger()
        (log.error if severity in ("error", "critical") else log.warn if severity == "warning" else log.info)(
            f"[{type_}] {text}")

    def _cmd_vel(self, msg: Twist) -> None:
        self.safety.command(msg.linear.x, msg.angular.z, "nav2")

    def _estop(self, _req, resp):
        self.safety.emergency_stop("ROS service /emergency_stop")
        resp.success, resp.message = True, "emergency stop engaged"
        return resp

    def _estop_reset(self, _req, resp):
        resp.success, resp.message = self.safety.reset_emergency_stop()
        return resp

    def _step(self) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        t = time.time()
        gyro_z = None
        if self.hw.imu:
            try:
                imu = self.hw.imu.read()
                gyro_z = imu.gyro[2]
                m = Imu()
                m.header.stamp, m.header.frame_id = stamp, "imu_link"
                m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = imu.accel
                m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = imu.gyro
                m.orientation_covariance[0] = -1.0     # no orientation estimate
                self.imu_pub.publish(m)
            except Exception as exc:
                self.get_logger().warn(f"imu read failed: {exc}", throttle_duration_sec=5.0)
        if self.hw.encoders:
            ticks = self.hw.encoders.ticks()
            pose = self.odom.update(ticks, t, gyro_z)
            self.safety.update_encoders(ticks)
            q = yaw_to_quat(pose.theta)
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_footprint"
            o.pose.pose.position.x, o.pose.pose.position.y = pose.x, pose.y
            (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
             o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
            o.twist.twist.linear.x, o.twist.twist.angular.z = self.odom.v, self.odom.w
            o.pose.covariance[0] = o.pose.covariance[7] = 0.01
            o.pose.covariance[35] = 0.05
            self.odom_pub.publish(o)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_footprint"
            tf.transform.translation.x, tf.transform.translation.y = pose.x, pose.y
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = q
            self.tf.sendTransform(tf)
        readings = []
        for s in self.hw.ranges:
            r = s.read()
            readings.append(r)
            m = Range()
            m.header.stamp, m.header.frame_id = stamp, s.name
            m.radiation_type = Range.ULTRASOUND if s.kind == "ultrasonic" else Range.INFRARED
            m.field_of_view = math.radians(15 if s.kind == "ultrasonic" else 25)
            m.min_range, m.max_range = 0.02, float(s.max_range_m)
            m.range = float(r.distance_m) if r.distance_m is not None else math.inf
            if r.valid:
                self.range_pubs[s.name].publish(m)
        self.safety.update_ranges(readings)
        if self.scan_pub is not None:
            scan = self.hw.lidar.scan()
            if scan is not None:
                m = LaserScan()
                m.header.stamp, m.header.frame_id = stamp, "laser"
                m.angle_min, m.angle_increment = scan.angle_min, scan.angle_increment
                m.angle_max = scan.angle_min + scan.angle_increment * (len(scan.ranges) - 1)
                m.range_min, m.range_max = 0.12, float(scan.range_max)
                m.ranges = [float(x) for x in scan.ranges]
                self.scan_pub.publish(m)
        if self.hw.battery and t - self._last_bat > 1.0:
            self._last_bat = t
            v = self.hw.battery.voltage()
            self.safety.update_battery(v)
            b = BatteryState()
            b.header.stamp = stamp
            b.voltage = float(v) if v is not None else math.nan
            b.present = v is not None
            self.bat_pub.publish(b)
        self.safety.tick()
        self.safety_pub.publish(String(data=json.dumps(self.safety.status.as_dict())))

    def destroy_node(self):
        self.safety.stop("shutdown")
        self.hw.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NexusBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
