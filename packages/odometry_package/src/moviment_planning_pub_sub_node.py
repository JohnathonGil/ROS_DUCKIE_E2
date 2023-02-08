#!/usr/bin/env python3
import numpy as np
import os
import rospy
import rosbag
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, WheelsCmdStamped
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import Header, Float32, String
from tf2_ros import TransformBroadcaster

class MovimentPlanningNode(DTROS):

    def __init__(self, node_name):

    # Initialize the DTROS parent class
        super(MovimentPlanningNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")
   
    # Intialize required parameters
        self.dist = 0
        self.move_dist = 0
        self.starting_distance = 0
        self.starting_angle = 0
        self.distance_reached = 0
        self.angle_reached = 0
        self.angle = 0
        self.move_num = 0

        self.set_start_dist = True
        self.set_start_angle = True
        self.run_led = 0
        self.goal_reached = [True, False, False]
        # Setup Service

        led_emitter = "/%s" % os.environ['VEHICLE_NAME'] + "/led_emitter_node/set_pattern"
        rospy.wait_for_service(led_emitter)
        self.change_pattern = rospy.ServiceProxy(led_emitter, ChangePattern)

        # self.current_pattern_name = "LIGHT_OFF"
        # self.changePattern(self.current_pattern_name)

    #Subscriber
        robot_pose = "/%s" % os.environ['VEHICLE_NAME'] + "/wheel_odometry/pose"
        self.sub_location = rospy.Subscriber(robot_pose, Pose2DStamped, self.cb_location_data, queue_size=1)
        
    # Publisher
        wheels_cmd = "/%s" % os.environ['VEHICLE_NAME'] + "/wheels_driver_node/wheels_cmd"
        self.pub_wheel_command = rospy.Publisher(wheels_cmd, WheelsCmdStamped, queue_size=1)

        #os.system("dts duckiebot demo --demo_name led_emitter_node --duckiebot_name $BOT --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8")

        self.log("Initialized")

    def cb_location_data(self, data):
        
        self.dist = data.x
        self.angle = data.theta

        self.distance_reached = self.dist - self.starting_distance
        self.angle_reached = self.angle - self.starting_angle
        rospy.loginfo("The encoder angle measurement: " + str(self.angle))
        rospy.loginfo("The starting angle measurement: " + str(self.starting_angle))
        rospy.loginfo("The current angle measurement: " + str(self.angle_reached))
        rospy.loginfo("Action running: " + str(self.goal_reached))
        #self.execution_time = data.header.stamp
        #rospy.loginfo("Execution Time: " + str(self.execution_time))
        
    # def led_server(self):
    #     led_color_setter = "/%s" % os.environ['VEHICLE_NAME'] + "/led_emitter_node/set_pattern"
    #     rospy.init_node(led_color_setter)
    #     self.srv_set_pattern_ = rospy.Service(led_color_setter, ChangePattern, self.srvSetPattern)

    # def srvSetPattern(self, msg):
    #     """Changes the current pattern according to the pattern name sent in the message.
    #     Args:
    #         msg (String): requested pattern name
    #     """
    #     self.changePattern(str(msg.pattern_name.data))
    #     return ChangePatternResponse()
    
    def LED_emitor_client (self, pattern_name):
        color = String()
        color.data = pattern_name
        self.change_pattern(color)
      
    def set_velocity(self, v_left, v_right):

        msg = WheelsCmdStamped()
        msg.vel_left = v_left
        msg.vel_right = v_right
        self.pub_wheel_command.publish(msg)

    def set_initial_distance(self):
        
        if self.set_start_dist:
            self.starting_distance = self.dist
            self.set_start_dist = False

    def set_initial_angle(self):
        if self.set_start_angle:
            self.starting_angle = self.angle
            self.set_start_angle = False

    def LED_state(self, Color1, Color2, switch):
        if switch == 0:
            time = 5
            self.LED_emitor_client(Color1)
            rospy.sleep(time)
            self.LED_emitor_client(Color2)
            self.run_led = 1


    def turn(self, direction):
        self.set_initial_angle()
        if direction == "right":
            if self.angle_reached == 0:
                self.set_velocity(0.4, -0.4)
            elif self.angle_reached < -(np.pi/2)+0.4:
                self.set_velocity(0.0, 0.0)

        elif direction == "left":
            if self.angle_reached == 0:
                self.set_velocity(-0.4, 0.4)
            elif self.angle_reached < (np.pi/2)-0.4:
                self.set_velocity(0.0, 0.0)
            

    def drive_straight(self):
        if self.distance_reached == 0:
                self.set_velocity(0.4, 0.4)
        elif self.distance_reached > 1.22:
                self.set_velocity(0.0, 0.0)

               
    def run(self):
        while not rospy.is_shutdown():
            
            self.LED_state("RED","GREEN",self.run_led)
            self.set_initial_angle()
            if self.angle_reached == 0:
                self.set_velocity(0.4, -0.4)
            elif self.angle_reached < -(np.pi/2)+0.4:
                self.set_velocity(0.0, 0.0)

           

if __name__ == '__main__':
    # create the node
    node = MovimentPlanningNode(node_name='moviment_planning_node')
    node.run()
    # keep spinning
    rospy.spin()    