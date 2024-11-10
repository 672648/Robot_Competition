from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.create_subscription(LaserScan, "/scan", self.clbk_laser, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, "my_namespace", 10)

    def timer_callback(self):
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)

        timer_period = 0.3
        self.timer = self.create_timer(timer_period, self.timer_callback) 


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()