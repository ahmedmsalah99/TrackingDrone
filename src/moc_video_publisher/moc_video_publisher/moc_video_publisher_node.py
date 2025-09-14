import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import time
import os
class MocVideoPublisher(Node):
    def __init__(self):
        super().__init__('moc_video_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.video_path = os.path.join(get_package_share_directory('moc_video_publisher'), 'videos', 'cars1.mp4')
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video file: {self.video_path}")
            return
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_time = 1.0 / self.fps if self.fps > 0 else 0.033  # default 30fps
        self.get_logger().info(f"Video FPS: {self.fps}, Frame time: {self.frame_time}")

    def publish_video(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                # Loop the video
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("Failed to read frame after reset")
                    break
            # Convert BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Convert to ROS Image
            img_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera"
            self.publisher_.publish(img_msg)
            self.get_logger().debug("Published image")
            time.sleep(self.frame_time)

def main(args=None):
    rclpy.init(args=args)
    node = MocVideoPublisher()
    try:
        node.publish_video()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
