import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math


class ObjectLocalizer(Node):
    def __init__(self):
        super().__init__('object_localizer')

        self.angle_sub = self.create_subscription(
            Float32MultiArray, '/object_angle', self.angle_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_clean', self.odom_callback, 10)

        self.pub = self.create_publisher(Float32MultiArray, '/object_position', 10)

        self.current_angle = None
        self.current_area  = 0.0
        self.scan_data     = None

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.last_valid_angle = None
        self.last_valid_time = self.get_clock().now()   

        self.get_logger().info('Object Localizer Node Started')

    def angle_callback(self, msg):
        self.current_angle = msg.data[0]
        self.current_area  = msg.data[1]
        self.compute_position()

    def scan_callback(self, msg):
        self.scan_data = msg
        self.compute_position()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def compute_position(self):
        if self.scan_data is None or self.current_angle is None:
            return

        if self.current_area > 0.0:
            self.last_valid_angle = self.current_angle
            self.last_valid_time = self.get_clock().now()
        else:
            time_diff = (self.get_clock().now() - self.last_valid_time).nanoseconds / 1e9
            if time_diff < 0.5 and self.last_valid_angle is not None:
                self.current_angle = self.last_valid_angle
            else:
                return

        angle_min       = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment

        index = int((self.current_angle - angle_min) / angle_increment)

        if index < 0 or index >= len(self.scan_data.ranges):
            return

        window    = 2
        distances = []
        for i in range(index - window, index + window + 1):
            if 0 <= i < len(self.scan_data.ranges):
                d = self.scan_data.ranges[i]
                if not math.isinf(d) and not math.isnan(d) and d > 0.0:
                    distances.append(d)

        if len(distances) == 0:
            return

        distance = min(distances)

        if distance > 4.0:
            return

        total_angle = self.yaw + self.current_angle

        x_obj = self.x + distance * math.cos(total_angle)
        y_obj = self.y + distance * math.sin(total_angle)

        self.get_logger().info(f"Computing object position | angle: {self.current_angle:.3f}")

        point = PointStamped()
        point.header.frame_id = "odom_clean"
        point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x = x_obj
        point.point.y = y_obj
        point.point.z = 0.0

        try:
            transformed = self.tf_buffer.transform(point, "map")

            msg = Float32MultiArray()
            msg.data = [
                float(transformed.point.x),
                float(transformed.point.y)
            ]
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")


def main():
    rclpy.init()
    node = ObjectLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
