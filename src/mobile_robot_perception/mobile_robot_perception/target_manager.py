import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math


class TargetManager(Node):
    def __init__(self):
        super().__init__('target_manager')

        self.sub = self.create_subscription(
            Float32MultiArray, '/object_position', self.callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_clean', self.odom_callback, 10)

        self.pub = self.create_publisher(Float32MultiArray, '/targets', 10)

        self.targets = []

        self.cluster_threshold    = 1.5
        self.dist_diff_threshold  = 0.5

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.min_publish_count = 50

        self.get_logger().info('Target Manager Started')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def callback(self, msg):
        x, y = msg.data
        self.update_targets(x, y)
        self.merge_close_targets()
        self.publish_targets()
        self.print_targets()

    def update_targets(self, x, y):
        new_dist = math.sqrt((x - self.robot_x)**2 + (y - self.robot_y)**2)

        for i, (tx, ty, count) in enumerate(self.targets):

            dist     = math.sqrt((x - tx)**2 + (y - ty)**2)
            old_dist = math.sqrt((tx - self.robot_x)**2 + (ty - self.robot_y)**2)

            if dist < self.cluster_threshold and \
               abs(new_dist - old_dist) < self.dist_diff_threshold:

                new_x = (tx * count + x) / (count + 1)
                new_y = (ty * count + y) / (count + 1)
                self.targets[i] = (new_x, new_y, count + 1)

                self.get_logger().info(
                    f"Updated target {i} → x:{new_x:.2f}, y:{new_y:.2f} "
                    f"(count:{count + 1})")
                return
            
        self.targets.append((x, y, 1))
        self.get_logger().info(f"New target added → x:{x:.2f}, y:{y:.2f}")

    def publish_targets(self):
        """
        Publish ALL targets immediately from first detection.
        Position improves with each new reading via running average.
        """
        msg = Float32MultiArray()
        flat_list = []

        for (x, y, count) in self.targets:
            if count >= self.min_publish_count:
                flat_list.extend([x, y])
        msg.data = flat_list
        self.pub.publish(msg)

    def merge_close_targets(self):
        merged = []

        for tx, ty, count in self.targets:
            merged_flag = False

            for i, (mx, my, mcount) in enumerate(merged):
                dist = math.sqrt((tx - mx)**2 + (ty - my)**2)

                if dist < self.cluster_threshold:
                    # merge clusters
                    new_x = (mx * mcount + tx * count) / (mcount + count)
                    new_y = (my * mcount + ty * count) / (mcount + count)

                    merged[i] = (new_x, new_y, mcount + count)
                    merged_flag = True
                    break

            if not merged_flag:
                merged.append((tx, ty, count))

        self.targets = merged

    def print_targets(self):
        self.get_logger().info("=== Current Targets ===")
        for i, (x, y, count) in enumerate(self.targets):
            self.get_logger().info(
                f"  [{i}] x:{x:.2f}, y:{y:.2f}  (readings: {count})")


def main():
    rclpy.init()
    node = TargetManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
