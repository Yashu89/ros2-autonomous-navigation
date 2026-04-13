import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math
import random


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.object_sub = self.create_subscription(
            Float32MultiArray, '/detected_object', self.object_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # distances
        self.front_dist = float('inf')
        self.left_dist  = float('inf')
        self.right_dist = float('inf')

        # parameters
        self.safe_distance     = 0.6
        self.wall_distance     = 0.5
        self.critical_distance = 0.3

        self.linear_speed = 0.2
        self.turn_speed   = 0.5

        # states
        self.state          = "FORWARD"
        self.turn_direction = None
        self.state_timer    = 0
        self.random_timer   = 0

        # object tracking
        self.object_angle = None
        self.object_area  = 0.0
        self.object_reached = False
        self.visiting_object = False

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Navigator (Autonomous + Object Aware) Started")

    def scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)

        def safe_min(data):
            valid = [d for d in data if not math.isinf(d) and not math.isnan(d) and d > 0.0]
            return min(valid) if valid else float('inf')

        self.front_dist = safe_min(ranges[n//2 - 10 : n//2 + 10])
        self.left_dist  = safe_min(ranges[int(0.75*n) - 10 : int(0.75*n) + 10])
        self.right_dist = safe_min(ranges[int(0.25*n) - 10 : int(0.25*n) + 10])

    def object_callback(self, msg):
        cx, cy, area = msg.data

        if area > 0:
            frame_center = 320  # adjust if needed
            error = cx - frame_center
            self.object_angle = error / frame_center
            self.object_area = area
        else:
            self.object_angle = None
            self.object_area = 0.0

    def control_loop(self):
        twist = Twist()
        self.random_timer += 1

        if self.state == "FORWARD":

            if self.front_dist > self.safe_distance:

                twist.linear.x = self.linear_speed

                if self.object_angle is not None and not self.object_reached:

                    if self.object_area > 5000:
                        self.object_reached = True
                        self.get_logger().info("Object reached → resume exploration")
                    else:
                        twist.angular.z = 0.6 * self.object_angle
                        self.visiting_object = True

                else:
                    twist.angular.z = 0.0

                if self.random_timer >= 150:
                    self.random_timer = 0
                    if random.random() < 0.35:
                        self.turn_direction = random.choice(["left", "right"])
                        self.state_timer = random.randint(6, 18)
                        self.state = "RANDOM_TURN"

                if self.object_reached:
                    if self.random_timer > 100:
                        self.object_reached = False
                        self.visiting_object = False
                        self.random_timer = 0

            else:
                if self.left_dist > self.right_dist:
                    self.turn_direction = "left"
                else:
                    self.turn_direction = "right"

                self.state_timer = 8
                self.state = "TURN"

        elif self.state == "TURN":
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed if self.turn_direction == "left" else -self.turn_speed

            self.state_timer -= 1

            if self.state_timer <= 0:
                self.state = "FOLLOW"

        elif self.state == "RANDOM_TURN":
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed if self.turn_direction == "left" else -self.turn_speed

            self.state_timer -= 1

            if self.state_timer <= 0:
                self.state = "FORWARD"
                self.random_timer = 0

        elif self.state == "FOLLOW":

            twist.linear.x = 0.15

            if self.turn_direction == "left":
                error = self.left_dist - self.wall_distance
                twist.angular.z = -0.8 * error
            else:
                error = self.right_dist - self.wall_distance
                twist.angular.z = 0.8 * error

            if self.front_dist > self.safe_distance:
                self.state = "FORWARD"

            if self.front_dist < self.critical_distance:
                self.state = "TURN"
                self.state_timer = 8

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()