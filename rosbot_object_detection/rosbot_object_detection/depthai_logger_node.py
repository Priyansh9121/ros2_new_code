#!/usr/bin/env python3
# depthai_logger_node.py  – SPA-style XAI logger
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2, numpy as np, json
from pathlib import Path
from datetime import datetime

# ───────── log files ─────────
LOG_DIR   = Path(__file__).parent / "logs"
LOG_DIR.mkdir(exist_ok=True)
LOG_FILE  = LOG_DIR / "detections.csv"
XAI_FILE  = LOG_DIR / "xai.csv"
DESC_FILE = LOG_DIR / "description.csv"

for f, hdr in [
        (LOG_FILE,  "timestamp,label,confidence,color,width,height,distance\n"),
        (XAI_FILE,  "timestamp,label,explanation\n"),
        (DESC_FILE, "timestamp,label,shape,size,color,distance,position\n")]:
    if not f.exists():
        f.write_text(hdr)

# ───────── node ─────────
class DepthAIObjectLogger(Node):
    def __init__(self):
        super().__init__("depthai_object_logger")
        self.bridge = CvBridge()

        self.create_subscription(Detection2DArray, "/detections",
                                 self.cb_detections, 10)
        self.create_subscription(Image, "/oak/rgb/image_raw",
                                 self.cb_image, 10)
        self.create_subscription(Image, "/oak/depth/image_raw",
                                 self.cb_depth, 10)

        self.marker_pub = self.create_publisher(Marker,
                                                "/visualization_marker", 10)
        self.json_pub   = self.create_publisher(String,
                                                "/detected_objects", 4)

        self.last_rgb   = None
        self.last_depth = None

    # ── helpers ──
    def cb_image(self, msg):   self.last_rgb   = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def cb_depth(self, msg):   self.last_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    @staticmethod
    def dom_color(roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h   = np.mean(hsv[:, :, 0])
        return ("red"    if h < 15 or h > 160 else
                "yellow" if h < 35 else
                "green"  if h < 85 else
                "blue"   if h < 130 else "unknown")

    # ── main callback ──
    def cb_detections(self, msg: Detection2DArray):
        if self.last_rgb is None or self.last_depth is None:
            return

        ts_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        output = []

        for det in msg.detections:
            if not det.results:
                continue

            hyp   = det.results[0].hypothesis
            label = hyp.class_id
            conf  = hyp.score
            cx, cy = int(det.bbox.center.x), int(det.bbox.center.y)
            w, h   = int(det.bbox.size_x),  int(det.bbox.size_y)

            shape = "square" if abs(w - h) < 10 else "rectangle"
            size  = ("small"  if w*h < 5_000  else
                     "medium" if w*h < 15_000 else "large")

            roi    = self.last_rgb[max(cy-h//2,0):cy+h//2,
                                   max(cx-w//2,0):cx+w//2]
            color  = self.dom_color(roi) if roi.size else "unknown"
            dist   = float(self.last_depth[cy, cx]) / 1000.0
            pos    = [round(cx/self.last_rgb.shape[1],3),
                      round(cy/self.last_rgb.shape[0],3),
                      round(dist,3)]

            # ── CSV logs ──
            LOG_FILE.write_text("", append=True)  # ensure file exists
            with LOG_FILE.open("a")  as f: f.write(f"{ts_str},{label},{conf:.2f},{color},{w},{h},{dist:.2f}\n")
            with DESC_FILE.open("a") as f: f.write(f"{ts_str},{label},{shape},{size},{color},{dist:.2f},{pos}\n")

            explanation = (
              f"[Sense] The DepthAI neural network detected a '{label}' object in the RGB image, "
              f"using a trained YOLO/SSD model onboard the OAK-D.\n"
              f"[Plan] The label was chosen via highest confidence ({conf:.2f}). "
              f"Color '{color}', shape '{shape}', size '{size}' were derived from the {w}×{h}px bounding box.\n"
              f"[Act] Stereo depth gave {dist:.2f} m; a 1 cm red marker was published to /visualization_marker "
              f"to show its position {pos} in RViz."
            )
            with XAI_FILE.open("a") as f: f.write(f"{ts_str},{label},{explanation}\n")

            # ── RViz marker ──
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns, m.id        = "depthai_objects", hash(ts_str+label) % 10000
            m.type, m.action  = Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
            m.scale.x = m.scale.y = m.scale.z = 0.01  # 1 cm
            m.color.r, m.color.a = 1.0, 1.0
            self.marker_pub.publish(m)

            output.append({"label": label, "confidence": conf, "features": {"color":color,"shape":shape,"size":size,"distance":dist,"position":pos}})

        if output:
            self.json_pub.publish(String(data=json.dumps(output)))
            self.get_logger().info("Detections: " + ", ".join(o["label"] for o in output))

def main():
    rclpy.init()
    rclpy.spin(DepthAIObjectLogger())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

