import rclpy
from rclpy.node import Node
from custom_interfaces.srv import LogMessage, ImgLoad  # LogMessage and ImgLoad service types
from cv_bridge import CvBridge
import cv2
from flask import Flask, send_file
import threading
import io

# Flask server setup
app = Flask(__name__)

# A simple path for the image that will be served
image_path = "/home/rokey/choi_ws/src/fitomi/resource/shorts.jpg"

# Initialize CvBridge
bridge = CvBridge()

@app.route('/image')
def get_image():
    """Flask server serves the image"""
    return send_file(image_path, mimetype='image/jpeg')


class ContextManagerServer(Node):
    def __init__(self):
        super().__init__('context_manager_server')
        self.get_logger().info("ContextManagerServer initialized!")
        
        # ROS2 services
        self.create_service(LogMessage, '/log_message', self.handle_log_message)
        self.create_service(ImgLoad, '/image_load', self.handle_image_request)
        
        self.get_logger().info("LogMessage and ImgLoad services are ready.")
    
    def handle_log_message(self, request, response):
        """Handle log message requests"""
        self.get_logger().info(f"Received log message: {request.log}")
        response.answer = True
        response.message = "Log message successfully received."
        return response

    def handle_image_request(self, request, response):
        """Handle image requests"""
        self.get_logger().info("Image request received")
        
        # Read the image from the file path
        cv_image = cv2.imread(image_path)
        
        if cv_image is not None:
            # Convert OpenCV image to ROS2 image message
            response.img = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.get_logger().info("Image successfully sent")
        else:
            self.get_logger().error("Failed to read the image")
            response.img = None
        return response


def run_flask():
    """Run the Flask server in a separate thread"""
    app.run(host="0.0.0.0", port=5000)


def main(args=None):
    """ROS2 server and Flask server execution"""
    rclpy.init(args=args)

    # Start Flask server in a background thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()

    # Start ROS2 server
    context_manager_server = ContextManagerServer()
    rclpy.spin(context_manager_server)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
