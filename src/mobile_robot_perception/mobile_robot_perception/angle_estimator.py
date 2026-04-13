import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class AngleEstimator(Node):
    def __init__(self):
        super().__init__('angle_estimator')

        self.sub = self.create_subscription(Float32MultiArray, '/detected_object', self.callback, 10)

        self.pub = self.create_publisher(Float32MultiArray, '/object_angle', 10)

        self.image_width = 640
        self.fov = 1.57

        self.get_logger().info("Angle Estimator Node Started")

    def callback(self, msg):
        cx, cy, area = msg.data

        if area == 0:
            self.publish_angle(0.0, 0.0)
            return
        
        error = cx - (self.image_width // 2)

        angle_scale = 1.0
        angle = angle_scale * (error / (self.image_width / 2)) * (self.fov / 2)

        self.publish_angle(angle, area)


    def publish_angle(self, angle, area):
        out = Float32MultiArray()
        out.data = [float(angle), float(area)]
        self.pub.publish(out)

def main():
    rclpy.init()
    node = AngleEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
