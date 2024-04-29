import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class ObjectFollower(Node):
    def __init__(self):
        super().__init__('_object_follower')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.image_subscriber
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_publisher
        self.cv_window_name = "USB Camera Image"
        cv2.namedWindow(self.cv_window_name)

        self.screen_center_x = 320
        self.screen_center_y = 240

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        bbox = self.detect__object(cv_image)
        if bbox:
            cv2.rectangle(cv_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            self.adjust_robot_movement(bbox)
        
        cv2.imshow(self.cv_window_name, cv_image)
        cv2.waitKey(10)
        
    def detect__object(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_ = np.array([0, 255, 120])
        upper_ = np.array([10, 255, 255])
        _mask = cv2.inRange(hsv_image, lower_, upper_)
        contours, _ = cv2.findContours(_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            x, y, w, h = cv2.boundingRect(contours[0])
            return (x, y, x+w, y+h)
        else:
            return None

    def adjust_robot_movement(self, bbox):
        obj_center_x = (bbox[0] + bbox[2]) // 2
        obj_center_y = (bbox[1] + bbox[3]) // 2
        offset_x = obj_center_x - self.screen_center_x

        if offset_x != 0:
            move_cmd = Twist()
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = -0.01 * offset_x
            self.cmd_publisher.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    _object_follower = ObjectFollower()
    rclpy.spin(_object_follower)
    _object_follower.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
