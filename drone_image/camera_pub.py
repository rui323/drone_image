import av
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os

class CameraPub(Node):
    def __init__(self, device="/dev/video2"):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        self.device = device

        self.container = None
        self.packet_iterator = None
        self.frame_iterator = iter([])

        if not os.path.exists(device):
            self.get_logger().error(f"Device {device} not found. Publishing black images.")
            self.timer = self.create_timer(1/30, self.publish_black_image)
        else:
            try:
                self.container = av.open(device, options={"video_size": "1280x720", "input_format": "yuyv422"})
                self.packet_iterator = self.container.demux(video=0)
                self.timer = self.create_timer(1/30, self.timer_callback)
            except Exception as e:
                self.get_logger().error(f"Failed to open device {device}: {e}")
                self.timer = self.create_timer(1/30, self.publish_black_image)

    def timer_callback(self):
        try:
            frame = next(self.frame_iterator)
        except StopIteration:
            try:
                packet = next(self.packet_iterator)
                self.frame_iterator = iter(packet.decode())
                frame = next(self.frame_iterator)
            except StopIteration:
                self.get_logger().warn("No frame available")
                return
            except Exception as e:
                self.get_logger().error(f"Error decoding packet: {e}")
                return

        img_bgr = frame.to_ndarray(format="bgr24")
        msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info("Published image")

    def publish_black_image(self):
        # 1280x720の黒画像を作成
        black_img = np.zeros((720, 1280, 3), dtype=np.uint8)
        msg = self.bridge.cv2_to_imgmsg(black_img, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info("Published black image due to missing device")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
