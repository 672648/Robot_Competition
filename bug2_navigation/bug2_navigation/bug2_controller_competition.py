import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist
from std_srvs.srv import SetBool
from bug2_interfaces.srv import GotoService
from nav_msgs.msg import Odometry
from math import sqrt
from std_msgs.msg import Int8
from ros2_aruco_interfaces.msg import ArucoMarkers
from rclpy.action import ActionServer
from scoring_interfaces.srv import SetMarkerPosition
from bug2_interfaces.action import Targetnavigation


class BugController(Node):
    def __init__(self):
        super().__init__("BugController")

        namespace = self.get_namespace()
        if namespace == '/': namespace = ''

        self.client_go_to_point = self.create_client(GotoService, f'{namespace}/go_to_point')
        self.client_wall_follower = self.create_client(SetBool, f'{namespace}/set_bool')
        self.client_scoring = self.create_client(SetMarkerPosition, 'set_marker_position')
        self.cmd_vel_pub = self.create_publisher(Twist, f"{namespace}/cmd_vel", 10)


        self.action_point = ActionServer(self, Targetnavigation, f'{namespace}/target_navigation', self.target_navigation_clbk)

        self.startPoint = None
        self.startSub = self.create_subscription(Odometry, f"{namespace}/odom", self.clbk_odom_new, 10)

        self.create_subscription(LaserScan, f"{namespace}/scan", self.clbk_laser, 10)

        self.create_subscription(Odometry, f"{namespace}/odom", self.clbk_odom, 10)

        self.lidar_front = 100

        self.endPointX = 100.0 #The end point might fail if its too close too a wall
        self.endPointY = 100.0 # x cannot be 0

        self.xPos = None
        self.yPos = None

        self.hit_point = None

        self.leave_point = None

        self.is_wall_follower = False

        self.is_big_flame = False

        self.timer = self.create_timer(0.1, self.timer_callback) 

    #uses feedback or goal to know when to exit big flame mode in robot_controller     doesn't work
    def target_navigation_clbk(self, goal_handle):
        self.endPointX = goal_handle.request.target_position.x
        self.endPointY = goal_handle.request.target_position.y
        self.is_big_flame = goal_handle.request.is_big_flame
    
        feedback_msg = Targetnavigation.Feedback()
        if not self.is_big_flame and self.is_close_to_big_fire():
            feedback_msg.current_position.x = self.xPos
            feedback_msg.current_position.y = self.yPos
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()    
        result = Targetnavigation.Result()
        result.base_position.x = self.xPos
        result.base_position.y = self.yPos
        result.base_position.z = 0.0
        return result
        
    

    def clbk_laser(self, msg):
        self.lidar_front = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))
        self.lidar_left_front = min(msg.ranges[12:30])
    
    def clbk_odom_new(self, msg):
        self.startPoint = msg.pose.pose.position
        if self.startPoint is not None:
            self.destroy_subscription(self.startSub)

    def clbk_odom(self, msg):
        self.xPos = msg.pose.pose.position.x
        self.yPos = msg.pose.pose.position.y

    def timer_callback(self):

        if(self.xPos is None or self.yPos is None): #vent på odom
            return

        if self.is_big_flame:
            self.is_wall_follower = False
            self.call_go_to_point(False, None)
            self.call_wall_follower(False)
            vel_msg = Twist()
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(vel_msg)

        if(self.endPointX == 100.0 or self.endPointY == 100.0):
            self.is_wall_follower = True
            self.call_go_to_point(False, None)
            self.call_wall_follower(True)
            return
        
        max_distance = 0.4
        target_position = Point()
        target_position.x = self.endPointX+0.0
        target_position.y = self.endPointY+0.0
        target_position.z = 0.0



        delta_x = self.endPointX - self.xPos
        delta_y = self.endPointY - self.yPos

        gode_gamle_pytagoras = sqrt(delta_x**2 + delta_y**2)
        threshold = 0.1


        if(gode_gamle_pytagoras <= threshold):
            self.endPointX = None
            self.endPointY = None
            self.call_go_to_point(False, None)
            self.call_wall_follower(True)
            self.timer.cancel()


        elif(self.lidar_front < max_distance and not self.is_wall_follower):
            if(not self.is_wall_follower):
                self.hit_point = Point(x=self.xPos, y=self.yPos)
                self.is_wall_follower = True
                self.call_go_to_point(False, None)
                self.call_wall_follower(True)
            
        elif(self.calcOnLine(self.xPos, self.yPos)):
            if(self.is_wall_follower or self.hit_point is None):
                self.leave_point = Point(x=self.xPos, y=self.yPos)

                if(self.isDistanceGreater()):
                    self.is_wall_follower = False
                    self.call_wall_follower(False)
                    self.call_go_to_point(True, target_position)

        

    def calcOnLine(self, xPos, yPos):
        if(self.startPoint is None):
            return
            
        slope = (self.endPointY - self.startPoint.y) / (self.endPointX - self.startPoint.x) #hvor fort y basert på x
        intersection = self.startPoint.y - slope*self.startPoint.x   #skjæring
        
        y_after_formula = slope*xPos+intersection
        point_closeness = 0.1

        if(y_after_formula - point_closeness <= yPos and yPos <= y_after_formula + point_closeness):
            return True

        return False
    
    def isDistanceGreater(self):
        if(self.hit_point is None or self.leave_point is None):
            return True

        hit_distance_from_goal = sqrt((self.endPointX - self.hit_point.x)**2 + (self.endPointY - self.hit_point.y)**2)
        leave_distance_from_goal = sqrt((self.endPointX - self.leave_point.x)**2 + (self.endPointY - self.leave_point.y)**2)
        self.get_logger().info(str(leave_distance_from_goal) + " " + str(hit_distance_from_goal))

        buffer = 0.4

        if(hit_distance_from_goal < leave_distance_from_goal + buffer):
            return False
        
        return True
    
    #sees if is close to the big fire after one robot has spotted it
    def is_close_to_big_fire(self):
        distance = ((self.endPointX - self.xPos) ** 2 + (self.endPointY - self.yPos) ** 2) ** 0.5
        if distance <= 1.5:
            return True
        return False
    def call_go_to_point(self, turn_on, point):
        go_to_point_request = GotoService.Request()
        go_to_point_request.move_switch = turn_on
        if point is not None:
            go_to_point_request.target_position = point
        self.client_go_to_point.call_async(go_to_point_request)

    def call_wall_follower(self, turn_on):
        wall_follower_request = SetBool.Request()
        wall_follower_request.data = turn_on
        self.client_wall_follower.call_async(wall_follower_request)


    


def main(args=None):
    rclpy.init(args=args)

    controller = BugController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()