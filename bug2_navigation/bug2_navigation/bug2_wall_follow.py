import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

class WallFollowerController(Node):
    def __init__(self):
        
        super().__init__("wall_follower")

        namespace = self.get_namespace()

        self.create_service(SetBool, f"{namespace}/set_bool", self.set_bool_callback)

        self.isAlreadyActive = False

        self.create_subscription(LaserScan, f"{namespace}/scan", self.clbk_laser, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, f"{namespace}/cmd_vel", 10)
        #self.cmd_vel_pub = self.create_publisher(Twist, "wall_follower/cmd_vel", 10)

        self.lidar_left_front = 100
        self.lidar_right_front = 100
        self.lidar_front = 100
        self.lidar_left = 100
        self.lidar_right = 100

        self.timer = None 
    
    def set_bool_callback(self, request, response):
        if request.data:
            if self.isAlreadyActive:
                response.success = False
            else:
                self.isAlreadyActive = True
                timer_period = 0.1
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
            


    def clbk_laser(self, msg):
        self.lidar_left_front = min(msg.ranges[12:30])
        self.lidar_right_front = msg.ranges[348]
        self.lidar_front = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))
        self.lidar_left = msg.ranges[90]

    def timer_callback(self):
        vel_msg = Twist()
    
        wall_distance = 1.6
        max_front_distance = 0.7

        print(str(self.lidar_left_front))

        if self.lidar_right_front > max_front_distance and self.lidar_left_front < wall_distance:
            vel_msg = self.follow_wall()
        elif self.lidar_front < wall_distance:
            vel_msg = self.go_right()
        else: 
            vel_msg = self.find_wall()

        
        self.cmd_vel_pub.publish(vel_msg)

    def find_wall(self):
        print("Finding wall \n")
        msg = Twist()
        msg.linear.x = 0.4
        msg.angular.z = 0.6
        return msg

    def go_right(self):
        print("Going right \n")
        msg = Twist()
        msg.angular.z = -0.4
        return msg

    def follow_wall(self):
        print("following wall \n")
        msg = Twist()
        too_far = 1.5
        too_close = 1.0

        if self.lidar_left_front > too_far:
            msg.angular.z = 0.4
        elif self.lidar_left_front < too_close:
            msg.angular.z = -0.4
        else:
            msg.angular.z = 0.0
            msg.linear.x = 0.4
        

        return msg

def main(args=None):
    rclpy.init(args=args)

    controller = WallFollowerController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()