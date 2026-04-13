import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTF(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            Odometry,
            '/odom_clean',
            10
        )

        self.br = TransformBroadcaster(self)

    def cb(self, msg):

        msg.header.frame_id = "odom_clean"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.y = -msg.pose.pose.position.y
        msg.pose.pose.orientation.z = -msg.pose.pose.orientation.z

        self.pub.publish(msg)

        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom_clean"
        t.child_frame_id = "base_link"

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()