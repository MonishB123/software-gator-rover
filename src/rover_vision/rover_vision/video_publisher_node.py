"""
video_publisher_node.py - stand-in "camera" for testing with no hardware.
 
Reads a video file and publishes it on /camera/image_raw, so apriltag_node
sees exactly what it will see from the real camera driver later.
 
Run:
  ros2 run rover_vision video_publisher --ros-args -p video_path:=/path/to/video.mov
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__("video_publisher")
        self.declare_parameter("video_path", "")
        self.declare_parameter("topic", "/camera/image_raw")
        self.declare_parameter("fps", 30.0)  # Default to 30 FPS if not specified
        self.declare_parameter("loop", True)  # Whether to loop the video when it ends
        
        path = self.get_parameter("video_path").value
        self.loop = self.get_parameter("loop").value
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video file: {path}")
            raise SystemExit(1)
        
        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Image, self.get_parameter("topic").value, qos_profile_sensor_data)
        self.create_timer(1.0 / self.get_parameter("fps").value, self.tick)
        self.get_logger().info(f"Publishing video from {path} on {self.get_parameter('topic').value}")
        
    def tick(self):
        success, frame = self.cap.read()
        if not success:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart video
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self.pub.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == "__main__":
    main()