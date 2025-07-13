import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.subscription = self.create_subscription(
            Image,
            '/send_img',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # display_img = cv2.resize(cv_image, (cv_image.shape[1]*2, cv_image.shape[0]*2))

            cv2.imshow("Detected Image", cv_image)
            
            if cv2.waitKey(1) == ord('q'):
                print("Shutting down...")
                cv2.destroyAllWindows()
                exit(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
