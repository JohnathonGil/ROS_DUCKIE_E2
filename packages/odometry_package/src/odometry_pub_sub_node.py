#!/usr/bin/env python3
import numpy as np
import os
import rospy
import rosbag
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32
from tf2_ros import TransformBroadcaster

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        #This is to keep the records of the previous encoder values
        self.starting_ticks_l = 0
        self.starting_ticks_r = 0
        self.previous_tick_count_set = [False, False]
        self.current_left_wheel_ticks = 0
        self.current_right_wheel_ticks = 0
        self.left_ticks = 0
        self.right_ticks = 0
        self.total_dist = 0
        self.dist = 0

        # Get static parameters
        #self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self.gain = 1.0
        self.trim = 0.0
        self._baseline = 0.1
        self._radius = 0.0318
        self._k = 27.0
        self._limit = 1.0
        self._limit_max = 1.0
        self._limit_min = 0.0

        # Subscribing to the wheel encoders
        left_wheel_topic = "/%s" % os.environ['VEHICLE_NAME'] + "/left_wheel_encoder_node/tick"
        self.sub_encoder_ticks_left = rospy.Subscriber(left_wheel_topic, WheelEncoderStamped, self.cb_encoder_data, callback_args="left", queue_size=1)
        right_wheel_topic = "/%s" % os.environ['VEHICLE_NAME'] + "/right_wheel_encoder_node/tick"
        self.sub_encoder_ticks_right = rospy.Subscriber(right_wheel_topic, WheelEncoderStamped, self.cb_encoder_data, callback_args="right", queue_size=1)
        # wheels_cmd_velocity = "/%s" % os.environ['VEHICLE_NAME'] + "/wheel_kinematics_node/velocity"
        # self.sub_executed_commands = rospy.Subscriber(wheels_cmd_velocity, Twist2DStamped)
        

        # Publishers
        wheels_cmd = "/%s" % os.environ['VEHICLE_NAME'] + "/wheels_driver_node/wheels_cmd"
        self.pub_wheel_command = rospy.Publisher(wheels_cmd, WheelsCmdStamped, queue_size=1)
        
        self.log("Initialized")

        
    def cb_encoder_data(self, msg, wheel):
        """ Update encoder distance information from ticks.
        """

        if wheel == "left":
            self.current_left_wheel_ticks = msg.data
            if not self.previous_tick_count_set[0]:
                self.starting_ticks_l = self.current_left_wheel_ticks
                self.previous_tick_count_set[0] = True
            else:
                self.left_ticks = np.abs(self.current_left_wheel_ticks-self.starting_ticks_l)

        elif wheel == "right":
            self.current_right_wheel_ticks = msg.data
            if not self.previous_tick_count_set[1]:
                self.starting_ticks_r = self.current_right_wheel_ticks
                self.previous_tick_count_set[1] = True
            else:
                self.right_ticks = np.abs(self.current_right_wheel_ticks-self.starting_ticks_r)

            self.distance_set(self.right_ticks, self.lef_ticks)
    

    def distance_set(self, right_ticks, left_ticks):

        distance_left = (2*np.pi*self._radius*left_ticks)/135
        distance_right = (2*np.pi*self._radius*right_ticks)/135

        self.dist = ((distance_left)+(distance_right))/2
        self.dist = self.r_value(self.dist)
        
        rospy.loginfo("The distance: " + str(self.dist))

    # def error_filter(value):

    #     epsilon = 0.02

    #     if (value - epsilon) 

    def r_value(self, value):

        r_val = np.round(value, decimals=2)

        return r_val

    def set_velocity(self, v_left, v_right):

        msg = WheelsCmdStamped()
        msg.vel_left = v_left
        msg.vel_right = v_right
        self.pub_wheel_command.publish(msg)
    
    def set_rotation(self, v_left, v_right) :

        

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    # while not rospy.is_shutdown():
    #         if node.dist == 0:
    #             node.set_velocity(0.2, 0.2)
    #         elif node.dist > 1.2:
    #             node.set_velocity(-0.2,-0.2)
    #         elif node.dist > 2.4:
    #             node.set_velocity(0.0,0.0)
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
