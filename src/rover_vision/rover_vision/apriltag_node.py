"""
apriltag_node.py - Gator Rover marker detection node.
 
Flow:
  /camera/image_raw -> cv_bridge -> grayscale -> pupil_apriltags (tag36h11)
  -> cv2.solvePnP (X cm tag; something small and A LOT OF tags) -> distance / horizontal offset
  -> text-only JSON on /apriltag/detections (no images over the link)
 
Optional (local only, off by default):
  - /apriltag/debug_image : annotated image with green box + ID + distance
  - cv2.imshow window     : only works if the container has a display
"""

import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String


class AprilTagNode(Node):
    def __init__(self):
        super().__init__("apriltag_node")

        # Parameters
        self.declare_parameter("image_topic", "/camera/image_raw") 
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("tag_size_m", 0.16) # 16cm tag but we can change 
        self.declare_parameter("tag_family", "tag36h11")

        # fx/fy/cx/cy = 0.0 means "estimate from horizontal FOV"; used ONLY until a real CameraInfo message is received.
        self.declare_parameter("fx", 0.0)
        self.declare_parameter("fy", 0.0)
        self.declare_parameter("cx", 0.0)
        self.declare_parameter("cy", 0.0)

        self.declare_parameter("hfov_deg", 70.0) # Horizontal field of view in degrees; used ONLY until a real CameraInfo message is received.
        self.declare_parameter("valid_ids", [-1]) # List of valid tag IDs; -1 means "accept all IDs"
        self.declare_parameter("publish_debug_image", False) # Whether to publish a debug image with detected tags drawn on it
        self.declare_parameter("show_window", False) # Whether to show a window with the debug image (requires a display)
        
        p = self.get_parameter
        self.tag_size = p("tag_size_m").value
        self.valid_ids = list(p("valid_ids").value)
        self.publish_debug = p("publish_debug_image").value
        self.show_window = p("show_window").value
        
        # geometry for solvePnP
        # order matches pupil_apriltags corners (bottom-left, bottom-right, top-right, top-left)
        h = self.tag_size / 2.0
        self.object_points = np.array([
            [-h, -h, 0.0],  # bottom-left
            [ h, -h, 0.0],  # bottom-right
            [ h,  h, 0.0],  # top-right
            [-h,  h, 0.0]   # top-left
        ], dtype=np.float64)
        
        # detector, bridge, intrinsics
        self.detector = Detector(families=p("tag_family").value)
        self.bridge = CvBridge()
        self.camera_matrix = None # 3x3, filled in from CameraInfo or estimated from HFOV
        self.dist_coeffs = np.zeros((5, 1)) # assume no lens distortion
        self.have_real_intrinsics = False # set to True when we get a real CameraInfo message
        
        # ros2 interfaces
        self.create_subscription(
            Image, p("image_topic").value, self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, p("camera_info_topic").value, self.camera_info_callback, 10
        )
        self.det_pub = self.create_publisher(String, "/apriltag/detections", 10)
        self.debug_pub = (
            self.create_publisher(Image, "/apriltag/debug_image", 1) 
            if self.publish_debug else None
        )
        self.get_logger().info(
            f'AprilTag node ready | tag = {self.tag_size * 100:.0f} cm |'
            f'waiting on {p("image_topic").value} and {p("camera_info_topic").value}...'
        )
        
    def camera_info_callback(self, msg: CameraInfo):
        """
        real calibration wins over any fallback. 
        runs once, then ignores later msgs.
        """
        if self.have_real_intrinsics:
            return
        
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if k[0, 0] <= 0.0: # driver published an uncalibrated (all-zero) matrix
            return
        self.camera_matrix = k
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1) \
            if len(msg.d) > 0 else np.zeros((5, 1))
        self.have_real_intrinsics = True
        self.get_logger().info(
            'Using intrinsics from CameraInfo (calibrated).'
        )
        
    def fallback_intrinsics(self, width: int, height: int):
        """
        Estimate intrinsics from horizontal FOV and image size.
        Rough pinhole guess so the node runs before calibration exists.
        """
        p = self.get_parameter
        fx, fy = p('fx').value, p('fy').value
        cx, cy = p('cx').value, p('cy').value
        if fx <= 0.0:
            fx = (width / 2.0) / math.tan(math.radians(p('hfov_deg').value) / 2.0)
        if fy <= 0.0:
            fy = fx # assume square pixels if not specified
        if cx <= 0.0:
            cx = width / 2.0
        if cy <= 0.0:
            cy = height / 2.0
        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.get_logger().warn(
            f'No CameraInfo received yet; using fallback intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}' 
            'Dinstances will be approximate until the camera is calibrated.'  
        )
        
    def image_callback(self, msg: Image):
        """
        Process each incoming image, detect AprilTags, and publish results.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        if self.camera_matrix is None:
            self.fallback_intrinsics(frame.shape[1], frame.shape[0])
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        
        detections = []
        for r in results:
            if self.valid_ids != [-1] and r.tag_id not in self.valid_ids:
                continue
            
            # Solve for pose
            image_points = np.array(r.corners, dtype=np.float64)
            success, rvec, tvec = cv2.solvePnP(
                self.object_points, image_points, 
                self.camera_matrix, self.dist_coeffs,
                flags = cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not success:
                self.get_logger().warn(f"solvePnP failed for tag ID {r.tag_id}")
                continue
            
            x, y ,z = tvec.flatten()                    # meters, camera frame: x right, y down, z forward
            distance = float(np.linalg.norm(tvec))      # straight-line distance
            bearing = math.degrees(math.atan2(x, z))    # + = tag is to the right, - = tag is to the left

            detections.append({
                "id": int(r.tag_id),
                "distance": round(distance, 3),
                "offset_x": round(float(x), 3), # right is +, left is -
                "bearing_deg": round(bearing, 2),
                "tvec": [round(float(v), 3) for v in (x, y, z)],
                "rvec": [round(float(v), 3) for v in rvec.flatten()],
                "calibrated": self.have_real_intrinsics
            })
            
            self.get_logger().info(
                f'Tag {r.tag_id}: {distance:.2f} m, offset {x:+.2f} m, bearing {bearing:+.1f} deg',
                throttle_duration_sec=1.0
            )

            if self.publish_debug or self.show_window:
                self._draw(frame, r, distance)
                
        if detections:
            out = String()
            out.data = json.dumps({
                "stamp": {
                    'sec': msg.header.stamp.sec, 
                    'nanosec': msg.header.stamp.nanosec
                },
                "detections": detections
            })
            self.det_pub.publish(out)
            
        if self.debug_pub is not None:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        
        if self.show_window:
            cv2.imshow("AprilTag Detections", frame)
            cv2.waitKey(1)
            
        @staticmethod
        def _draw(frame, r, distance):
            pts = [(int(c[0]), int(c[1])) for c in r.corners]
            for i in range(4):
                cv2.line(frame, pts[i], pts[(i + 1) % 4], (0, 255, 0), 2) # BGR green
            cv2.putText(
                frame, f"ID: {r.tag_id} {distance:.2f} m", (pts[3][0], pts[3][1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                    )

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        
        
if __name__ == "__main__":
    main()