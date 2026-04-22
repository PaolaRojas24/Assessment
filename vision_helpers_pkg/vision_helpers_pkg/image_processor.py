import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        # 1. Declare the parameter with a default value
        self.declare_parameter('subscribe_topic', '/camera/image_raw')
        
        # 2. Get the parameter value
        topic_name = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.get_logger().info('Image Processor Node has been started.')

    def listener_callback(self, data):
        # self.get_logger().info('Receiving video frame') # Uncomment for debugging
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        
        if current_frame is not None:
            try: 
                # --- Image Processing Logic ---
                gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
                
                # Display the processed frame
                cv2.imshow("Grayscale Feed", gray_frame)
                cv2.waitKey(1)
            except:
                return

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()