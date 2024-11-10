import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf_transformations import euler_from_quaternion
from bug2_interfaces.srv import GotoService
import math


class GoToPointController(Node):
    def __init__(self):
        
        super().__init__("GoToPointController")

        self.namespace = self.get_namespace()

        self.create_subscription(Odometry, f"{self.namespace}/odom", self.clbk_odom, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, f"{self.namespace}/cmd_vel", 10)
        
        self.srv = self.create_service(GotoService, f"{self.namespace}/go_to_point", self.go_to_point_callback)
        self.isAlreadyActive = False
        
        self.timer = None  


        self.position = Point() 
        self.yaw = 0.0  
        self.target_position = Point()

    def go_to_point_callback(self, request, response):
        if request.move_switch:
            if self.isAlreadyActive:
                response.success = False
            else:
                self.isAlreadyActive = True
                timer_period = 0.1
                self.target_position = request.target_position
                self.timer = self.create_timer(timer_period, self.timer_callback)
                response.success = True
        else:
            if self.isAlreadyActive:
                self.isAlreadyActive = False
                self.timer.cancel()
                response.success = True
            else:
                response.success = False
        return response
    

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def timer_callback(self):
        vel_msg = Twist()


        delta_x = self.target_position.x - self.position.x
        delta_y = self.target_position.y - self.position.y
        theta_goal = math.atan2(delta_y, delta_x)
        distance_to_point = math.sqrt(delta_x**2 + delta_y**2)
        
        point_closeness = 0.1

        angle_diff = theta_goal - self.yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        if distance_to_point <= point_closeness:
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.0
        else:  

            if(abs(angle_diff) > 0.1):
                if(angle_diff < 0):
                    vel_msg.angular.z = -0.3
                else:
                    vel_msg._angular.z = 0.3
            else:
                vel_msg.angular.z = 0.0
            
            if(abs(angle_diff) < 0.1):
                vel_msg.linear.x = 0.3
            else:
                vel_msg.linear.x = 0.0

        self.cmd_vel_pub.publish(vel_msg)
    

def main(args=None):
    rclpy.init(args=args)

    controller = GoToPointController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
