"""Publishes colour detections as vision_msgs/Detection2DArray and the annotated image.

Topics:
  /vision/detections        vision_msgs/Detection2DArray (class_id = colour name)
  /vision/image_annotated   sensor_msgs/CompressedImage (JPEG)
"""

import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

sys.path.insert(0, os.environ.get("NEXUS_ROOT", os.path.expanduser("~/PROJECT-NEXUS")))

from robot.config import load_config  # noqa: E402
from robot.hardware.factory import build_hardware  # noqa: E402
from robot.vision.pipeline import VisionPipeline  # noqa: E402


class VisionNode(Node):
    def __init__(self):
        super().__init__("nexus_vision")
        self.declare_parameter("config", os.environ.get("NEXUS_CONFIG", "config/nexus.yaml"))
        cfg = load_config(self.get_parameter("config").value)
        # Only the camera is needed here; the base node owns the other devices.
        cfg["hardware"] = {k: {**v, "enabled": False} for k, v in cfg["hardware"].items()}
        hw = build_hardware(cfg)
        if hw.camera is None:
            self.get_logger().error("no camera available, vision node idle")
        self.pipeline = VisionPipeline(hw.camera, cfg)
        self.det_pub = self.create_publisher(Detection2DArray, "vision/detections", 10)
        self.img_pub = self.create_publisher(CompressedImage, "vision/image_annotated", 2)
        period = 1.0 / max(1, cfg["camera"].get("fps", 15))
        self.timer = self.create_timer(period, self._step)

    def _step(self):
        if self.pipeline.camera is None:
            return
        dets = self.pipeline.process_once()
        stamp = self.get_clock().now().to_msg()
        arr = Detection2DArray()
        arr.header.stamp, arr.header.frame_id = stamp, "camera_link"
        for d in dets:
            m = Detection2D()
            m.header = arr.header
            m.id = str(d.object_id)
            x, y, w, h = d.bbox
            m.bbox.center.position.x, m.bbox.center.position.y = float(d.centroid[0]), float(d.centroid[1])
            m.bbox.size_x, m.bbox.size_y = float(w), float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.color
            hyp.hypothesis.score = float(d.confidence)
            m.results.append(hyp)
            arr.detections.append(m)
        self.det_pub.publish(arr)
        jpg = self.pipeline.jpeg("annotated")
        if jpg:
            img = CompressedImage()
            img.header = arr.header
            img.format = "jpeg"
            img.data = jpg
            self.img_pub.publish(img)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
