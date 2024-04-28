import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class GreenObjectFollower(Node):
    def __init__(self):
        super().__init__('green_object_follower')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.image_subscriber
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_publisher
        self.cv_window_name = "USB Camera Image"
        cv2.namedWindow(self.cv_window_name)

        # 螢幕中心座標
        self.screen_center_x = 320
        self.screen_center_y = 240

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 檢測綠色物體
        bbox = self.detect_green_object(cv_image)
        if bbox:
            # 如果檢測到物體，則畫出矩形框
            cv2.rectangle(cv_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            # 調整車子移動
            self.adjust_robot_movement(bbox)
        
        cv2.imshow(self.cv_window_name, cv_image)
        cv2.waitKey(10)
        
    def detect_green_object(self, cv_image):
        # 轉換圖片到HSV色彩空間
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # 定義綠色物體的HSV範圍
        lower_green = np.array([0, 255, 120])
        upper_green = np.array([10, 255, 255])
        # 在HSV圖片中找到綠色物體的mask
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        # 找到綠色物體的輪廓
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # 計算物體的bounding box
            x, y, w, h = cv2.boundingRect(contours[0])
            return (x, y, x+w, y+h)
        else:
            return None

    def adjust_robot_movement(self, bbox):
        # 計算物體的中心座標
        obj_center_x = (bbox[0] + bbox[2]) // 2
        obj_center_y = (bbox[1] + bbox[3]) // 2
        
        # 計算螢幕中心與物體中心的水平偏移量
        offset_x = obj_center_x - self.screen_center_x

        # 如果物體不在螢幕中心，調整車子移動
        if offset_x != 0:
            move_cmd = Twist()
            move_cmd.linear.x = 0.1  # 前進速度
            move_cmd.angular.z = -0.01 * offset_x  # 根據偏移量調整角速度
            self.cmd_publisher.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    green_object_follower = GreenObjectFollower()
    rclpy.spin(green_object_follower)
    green_object_follower.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
