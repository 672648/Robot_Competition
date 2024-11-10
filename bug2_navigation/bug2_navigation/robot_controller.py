import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose,Point
from std_msgs.msg import Int64, Int8MultiArray
from bug2_interfaces.action import Targetnavigation
from scoring_interfaces.srv import SetMarkerPosition


class robot_controller(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.namespace = self.get_namespace()
        self._action_client = ActionClient(self, Targetnavigation, f'{self.namespace}/target_navigation')
        self.client_scoring = self.create_client(SetMarkerPosition, '/set_marker_position')
        self.sub_marker_pose = self.create_subscription(Pose, f'{self.namespace}/marker_map_pose', self.aruco_pose_callback, 10)
        self.sub_marker_id = self.create_subscription(Int64, f'{self.namespace}/marker_id', self.marker_callback,10)
        self.pub_aruco_found = self.create_publisher(Int8MultiArray, '/marker_list', 10)
        self.sub_aruco_found = self.create_subscription(Int8MultiArray, '/marker_list', self.update_array_clbk ,10)

        #get the name of the other robot so it can send it big flame position
        name = self.namespace[-1]
        if name == "0":
            self.send_to_other_robot = ActionClient(self, Targetnavigation, 'tb3_1/target_navigation')
        else:
            self.send_to_other_robot = ActionClient(self, Targetnavigation, 'tb3_0/target_navigation')
    
        self.lidar_front = 100
        self.aruco_pose = None

        self.found_markers = Int8MultiArray()
        self.found_markers.data = [0, 0, 0, 0, 0]

        self.other_xPos = None
        self.other_yPos = None

        self.marker_id = None


        self.timer = self.create_timer(0.2, self.timer_callback) 

    #array used for keeping track of which markers are already used
    def update_array_clbk(self, msg):
        self.found_markers.data = list(msg.data)

    def aruco_pose_callback(self,msg):
        self.aruco_pose = msg

    def clbk_odom_other(self, msg):
        if msg.pose.pose.position.x is None or msg.pose.pose.position is None:
            return
        self.other_xPos = msg.pose.pose.position.x
        self.other_yPos = msg.pose.pose.position.y

    def marker_callback(self,msg):
        if msg.data < 0 or msg.data > 5:
            return
        if self.found_markers.data[msg.data] == 0:  
            self.marker_id = msg.data


    def timer_callback(self):
        if self.marker_id is None or self.aruco_pose is None:
            return
        #return if already seen the marker
        elif self.found_markers.data[self.marker_id] == 1:
            return

        
        #call the scoring
        scoring_request = SetMarkerPosition.Request()

        scoring_request.marker_id = self.marker_id
        scoring_request.marker_position = self.aruco_pose.position
        self.client_scoring.call_async(scoring_request)

    
    def handle_response(self, future):
        #try to get response from scoring
        try:
            response = future.result()
            accepted = response.accepted

            aruco_pos = self.aruco_pose.position

            #if their is a big flame and the scoring has accepted the id
            if self.marker_id == 4 and accepted:


                self._action_client.send_goal_async(self.make_action_request(True, aruco_pos))

                self.send_to_other_robot(self.make_action_request(False, aruco_pos))

                self.publish_found_list()

                self.is_in_big_flame = True

                return

            if accepted:
                self.publish_found_list()
            else:
                self._action_client.send_goal_async(self,False,aruco_pos)

        except Exception as e:
            self.get_logger().info(f'response svarer ikke {e}]')

    def publish_found_list(self):
        self.found_markers.data[self.marker_id] = 1
        self.pub_aruco_found.publish(self.found_markers)

    def make_action_request(self, big_flame, point):
        request = Targetnavigation.Goal()
        request.is_big_flame = big_flame
        request.target_position = point
        return request
    
    #there should have been a callback for action result here but we couldn't get it to work

def main(args=None):
    rclpy.init(args=args)

    robot_controller_node = robot_controller()

    while rclpy.ok():
        rclpy.spin_once(robot_controller_node)  
        
    robot_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()