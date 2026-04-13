import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class RedBoxDetector(Node):

    def __init__(self):
        super().__init__('box_detector')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.pub = self.create_publisher(Float32MultiArray, '/detected_object', 10)

        self.get_logger().info("Box Detector Node Started")

        self.valid_count = 0
        self.required_frames = 2
        self.min_valid_area = 1000  

        self.no_detection_count = 0

        self.last_cx = 0.0
        self.last_cy = 0.0
        self.last_area = 0.0

        self.memory_frames = 5
        self.memory_count = 0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cx , cy, area = self.detect_box(frame)

        data = Float32MultiArray()
        data.data = [float(cx), float(cy) , float(area)]

        self.pub.publish(data)

        cv2.imshow('Detection', frame)
        cv2.waitKey(1)

    def detect_box(self, frame):
        h , w, _ = frame.shape

        centre_x = w // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red_1 = np.array([0 , 150, 100])
        upper_red_1 = np.array([8, 255, 255])

        lower_red_2 = np.array([172, 150, 100])
        upper_red_2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

        mask = mask1 | mask2

        mask = cv2.GaussianBlur(mask,(5,5), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
         
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.no_detection_count += 1

            if self.no_detection_count > 3:
                self.valid_count = max(0, self.valid_count - 1)
                self.no_detection_count = 0


            if self.memory_count > 0:
                self.memory_count -= 1
                return self.last_cx, self.last_cy, self.last_area
            else:
                return 0.0, 0.0, 0.0
        
        self.no_detection_count = 0
        
        best = None
        max_area = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < self.min_valid_area:
                continue

            x, y, w_box, h_box = cv2.boundingRect(cnt)
            aspect_ratio = w_box / float(h_box)

            if 0.6 < aspect_ratio < 1.2:
                if area > max_area:
                    max_area = area
                    best = cnt

        if best is None:
            if self.memory_count > 0:
                self.memory_count -= 1
                return self.last_cx, self.last_cy, self.last_area
            else:
                return 0.0, 0.0, 0.0
        

        if max_area < self.min_valid_area:
            self.valid_count = max(0, self.valid_count - 1)

            if self.memory_count > 0:
                self.memory_count -= 1
                return self.last_cx, self.last_cy, self.last_area
            else:
                return 0.0, 0.0, 0.0

        self.valid_count += 1

        if self.valid_count < self.required_frames:
            return 0, 0, 0
        
        x, y, w_box, h_box = cv2.boundingRect(best)
        cx = x + w_box // 2
        cy = y + h_box // 2

        self.last_cx = cx
        self.last_cy = cy
        self.last_area = max_area
        self.memory_count = self.memory_frames

        cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
        cv2.line(frame, (centre_x, 0), (centre_x, frame.shape[0]), (0, 255, 255), 1)

        return cx, cy, max_area


def main():
    rclpy.init()
    node = RedBoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()