import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ImageDecompressor(Node):

    def __init__(self):
        super().__init__('image_decompressor')
        self.bridge = CvBridge()
        self.compressed_subscription = self.create_subscription(
            CompressedImage,
            '/qbot/camera/compressed_image',
            self.compressed_image_callback,
            10
        )
        self.uncompressed_publisher = self.create_publisher(
            Image,
            '/qbot/camera/image',
            10
        )

    def compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')  # Or your desired encoding
            self.uncompressed_publisher.publish(ros_image)
            self.get_logger().info('Published uncompressed image')
        except Exception as e:
            self.get_logger().error(f'Error during decompression: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_decompressor = ImageDecompressor()
    rclpy.spin(image_decompressor)
    image_decompressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
