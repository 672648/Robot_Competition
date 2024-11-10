import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from bug2_interfaces.srv import GotoService
from nav_msgs.msg import Odometry
from math import sqrt
from std_msgs.msg import Int8
from ros2_aruco_interfaces.msg import ArucoMarkers
from bug2_interfaces.srv import SetMarkerPosition


class BugController(Node):
    def __init__(self):
        super().__init__("BugController")

        namespace = self.get_namespace()

        self.client_go_to_point = self.create_client(GotoService, f'{namespace}/go_to_point')
        self.client_wall_follower = self.create_client(SetBool, f'{namespace}/set_bool')
        self.client_scoring = self.create_client(SetMarkerPosition, 'set_marker_position')

        self.sub_aruco = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.marker_callback,
            10
        )

        self.startPoint = None
        self.startSub = self.create_subscription(Odometry, f"{namespace}/odom", self.clbk_odom_new, 10)

        self.create_subscription(LaserScan, f"{namespace}/scan", self.clbk_laser, 10)

        self.create_subscription(Odometry, f"{namespace}/odom", self.clbk_odom, 10)

        self.lidar_front = 100

        self.endPoint = Point() #The end point might fail if its too close too a wall
        self.endPoint.x = -7.0 # x cannot be 0
        self.endPoint.y = 6.0

        self.xPos = None
        self.yPos = None

        self.hit_point = None

        self.leave_point = None

        self.is_wall_follower = False

        self.timer = self.create_timer(0.1, self.timer_callback) 

    
    def marker_callback(self,msg):
        if msg.marker_ids:
            marker_id = msg.marker_ids[0]
            marker_pose = msg.poses[0]

            self.get_logger().info(f'Marker ID: {marker_id}, Pose: {marker_pose}')

            if marker_id < -128 or marker_id > 127:
                return
        
            request = SetMarkerPosition.Request()
            request.marker_id = marker_id
            request.marker_position = marker_pose.position 
            self.client_scoring.call_async(request)
            
            


    def clbk_laser(self, msg):
        self.lidar_front = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))
        #self.lidar_front = msg.ranges[0]
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

        max_distance = 0.4

        go_to_point_request = GotoService.Request()
        go_to_point_request.target_position.x = self.endPoint.x
        go_to_point_request.target_position.y = self.endPoint.y

        wall_follower_request = SetBool.Request()

        delta_x = self.endPoint.x - self.xPos
        delta_y = self.endPoint.y - self.yPos

        gode_gamle_pytagoras = sqrt(delta_x**2 + delta_y**2)
        threshold = 0.1

        if(gode_gamle_pytagoras <= threshold):
            go_to_point_request.move_switch = False
            wall_follower_request.data = False
            self.client_go_to_point.call_async(go_to_point_request)
            self.client_wall_follower.call_async(wall_follower_request)
            self.get_logger().info('gollaaaassssssssssssoooooooo!')
            self.timer.cancel()


        elif(self.lidar_front < max_distance):
            if(not self.is_wall_follower):
                self.hit_point = Point(x=self.xPos, y=self.yPos)
                self.get_logger().info('bytt til wall follower')
                self.get_logger().info('endret hit point til: ' + str(self.hit_point))

            self.is_wall_follower = True
            go_to_point_request.move_switch = False
            wall_follower_request.data = True
            self.client_go_to_point.call_async(go_to_point_request)
            self.client_wall_follower.call_async(wall_follower_request)
            
        elif(self.calcOnLine(self.xPos, self.yPos)):
            if(self.is_wall_follower or self.hit_point is None):
                self.leave_point = Point(x=self.xPos, y=self.yPos)

                if(self.isDistanceGreater()):
                    self.get_logger().info('bytt til go to point')
                    self.get_logger().info('endret leave point til: ' + str(self.leave_point))
                    self.is_wall_follower = False
                    wall_follower_request.data = False
                    go_to_point_request.move_switch = True
                    self.client_wall_follower.call_async(wall_follower_request)
                    self.client_go_to_point.call_async(go_to_point_request)

        

    def calcOnLine(self, xPos, yPos):
        if(self.startPoint is None):
            return
            
        slope = (self.endPoint.y - self.startPoint.y) / (self.endPoint.x - self.startPoint.x) #hvor fort y basert på x
        intersection = self.startPoint.y - slope*self.startPoint.x   #skjæring
        
        y_after_formula = slope*xPos+intersection
        point_closeness = 0.1

        if(y_after_formula - point_closeness <= yPos and yPos <= y_after_formula + point_closeness):
            return True

        return False
    
    def isDistanceGreater(self):
        if(self.hit_point is None or self.leave_point is None):
            return True

        hit_distance_from_goal = sqrt((self.endPoint.x - self.hit_point.x)**2 + (self.endPoint.y - self.hit_point.y)**2)
        leave_distance_from_goal = sqrt((self.endPoint.x - self.leave_point.x)**2 + (self.endPoint.y - self.leave_point.y)**2)
        self.get_logger().info(str(leave_distance_from_goal) + " " + str(hit_distance_from_goal))

        buffer = 0.4

        if(hit_distance_from_goal < leave_distance_from_goal + buffer):
            return False
        
        return True

    


def main(args=None):
    rclpy.init(args=args)

    controller = BugController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()