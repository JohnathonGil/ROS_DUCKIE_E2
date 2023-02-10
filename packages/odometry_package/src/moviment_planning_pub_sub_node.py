#!/usr/bin/env python3
import numpy as np
import os
import rospy
import rosbag
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, WheelsCmdStamped
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import Header, Float32, String, Bool
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
        
        self.state = 1
        self.stop = False
        self.running = True
        self.cmd_start_dist = 0
        self.cmd_start_angle = 0
        self.cmd_start = 0
        self.vel = [0, 0]

        self.forward_vel = [0.44, 0.4]
        self.left = [-0.445, 0.4]
        self.right = [0.445, -0.4]
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

        self.exit_pub = rospy.Publisher("/exit", Bool, queue_size=1)

        #os.system("dts duckiebot demo --demo_name led_emitter_node --duckiebot_name $BOT --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8")

        self.log("Initialized")

    def cb_location_data(self, data):
        self.dist = data.x
        self.angle = data.theta

        self.distance_reached = self.dist - self.starting_distance
        self.angle_reached = self.angle - self.starting_angle
        # rospy.loginfo("The encoder angle measurement: " + str(self.angle))
        # rospy.loginfo("The starting angle measurement: " + str(self.starting_angle))
        # rospy.loginfo("The current angle measurement: " + str(self.angle_reached))
        #rospy.loginfo("Distance: " + str(self.distance_reached))
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
    
    def LED_emitor_client(self, pattern_name):
        color = String()
        color.data = pattern_name
        self.change_pattern(color)
      
    def set_velocity(self):

        msg = WheelsCmdStamped()
        msg.vel_left = self.vel[0]
        msg.vel_right = self.vel[1]
        self.pub_wheel_command.publish(msg)

        if self.stop:
            self.running = False
            rospy.signal_shutdown("ROBO MOVED")

    def publish_exit(self):
        msg  = Bool()
        msg.data = self.stop
        self.exit_pub.publish(msg)

        

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
        
    def is_near(self, dist):
        epsilon = 0.3
        print(f"NEAR: {self.dist}, GOAL: {dist}, DIFF: {abs(self.dist - dist)}")
        if abs(self.dist - dist) < epsilon or self.dist > dist:
            print("#################CHANGING DIR")
            return True
        return False

    def angle_is_near(self, theta):
        epsilon = 0.5
        rospy.loginfo(f"theta: {self.angle}, GOAL: {theta}, DIFF: {abs(self.angle - theta)}")
        if abs(self.angle - theta) < epsilon :#or abs(self.angle) > abs(theta):
            print("#################CHANGING SPEED")
            return True
        return False

    def run_cmd(self, cmd, state):
        print(f"diff dist: {cmd['dist'] + self.cmd_start_dist}, diff angle: {cmd['angle'] + self.cmd_start_angle}")
        diff_angle = True
        diff_dist = True

        if cmd["angle"] != 0:
            diff_angle = self.angle_is_near(cmd["angle"] + self.cmd_start_angle)
        if cmd["dist"] != 0:
            diff_dist = self.is_near(cmd["dist"] + self.cmd_start_dist)
        return diff_dist and diff_angle and self.state == state

    def pause(self):
        self.vel = [0, 0]
        self.set_velocity()
        rospy.sleep(0.5)

               
    def run(self):
        #self.LED_state("RED","GREEN",self.run_led)

        #self.set_initial_distance()
        # if self.angle_reached == 0:
        #     self.set_velocity(0.4, -0.4)
        # elif self.angle_reached < -(np.pi/2)+0.4:
        #     self.set_velocity(0.0, 0.0)

        cmd = [
        # once these conditions are met |  do this  

                    
            {"dist": 0, "angle": 0, "vel": [0, 0], "col": "RED", "wait": 5},    # wait 5 secs
            {"dist": 0, "angle": 0, "vel": self.right, "col": "BLUE"},         # turn right
            {"dist": 0, "angle": -np.pi/2, "vel": self.forward_vel, "col": "BLUE"},   # go straight 
            {"dist": 1.25, "angle": 0, "vel": self.left, "col": "BLUE"},      # turn left
            {"dist": 0, "angle": np.pi / 2, "vel": self.forward_vel, "col": "BLUE"},    # go straight
            {"dist": 1.25, "angle": 0,  "vel": self.left, "col": "BLUE"},     #turn left
            {"dist": 0, "angle": np.pi/2, "vel": self.forward_vel, "col": "BLUE"},    # go straight
            {"dist": 1.25, "angle": 0, "vel": [0, 0], "col": "RED", "wait": 5},    # wait 5 secs
            {"dist": 0, "angle": 0,  "vel": self.left, "col": "GREEN"},     #turn left
            {"dist": 0, "angle": np.pi, "vel": self.forward_vel, "col": "GREEN"},    # go straight
            {"dist": 1.25, "angle": 0, "vel": self.right, "col": "GREEN"},         # turn right
            {"dist": 0, "angle": -np.pi/2, "vel": self.forward_vel, "col": "GREEN"},    # go straight
            {"dist": 1.25, "angle": 0, "vel": self.right, "col": "GREEN"},         # turn right
            {"dist": 0, "angle": -np.pi/2, "vel": self.forward_vel, "col": "GREEN"},    # go straight
            {"dist": 1.25, "angle": 0, "vel": [0, 0], "col": "RED", "wait": 5},    # wait 5 secs
            "circle",
            "end",    
            {"dist": 0, "angle": 0, "vel": [0, 0], "col": "LIGHT_OFF", "wait": 0},
        ]


        #self.set_velocity(*self.vel)
        curr_cmd = cmd[self.state-1]

        if curr_cmd == "circle":
            print("CIRCLE_START", self.dist)
            self.vel = [0.3, 0.6]
            self.cmd_start_dist = self.dist
            self.state += 1
            self.LED_emitor_client("WHITE")
            print(curr_cmd)
        elif curr_cmd == "end" and self.is_near((1.25*np.pi) + self.cmd_start_dist):
            print("CIRCEL_END", self.dist)
            print(curr_cmd)
            self.vel = [0, 0]
            self.LED_emitor_client("LIGHT_OFF")
            self.stop = True

        elif type(curr_cmd) != str and self.run_cmd(curr_cmd, self.state) and not self.stop:
            self.pause()
            self.cmd_start_dist = self.dist
            self.cmd_start_angle = self.angle
            self.cmd_start = rospy.Time.now().to_sec()
            self.state += 1
            print(curr_cmd)
            self.vel = curr_cmd["vel"]
            self.LED_emitor_client(curr_cmd["col"])
            self.set_velocity()
            if self.vel == [0, 0]:
                rospy.sleep(curr_cmd["wait"])
            print(f"EXECUTING STATE: {self.state-1}")
            if self.state > len(cmd):
                self.stop = True


        



            # if self.is_near(0) and self.state == 1:
            #     self.cmd_start_dist = self.dist
            #     self.state += 1
            #     self.set_velocity(0.6, 0.6)
            #     self.LED_emitor_client("RED")
            #     print("GOING STTROIGH")
            # elif self.is_near(1.25) and self.state == 2:
            #     self.state += 1
            #     self.set_velocity(-0.6, 0.6)
            #     self.LED_emitor_client("BLUE")
            #     print("TURNING")
            # elif self.angle_is_near(np.pi/2) and self.state == 3:
            #     self.state += 1
            #     self.set_velocity(0.0, 0.0)
            #     self.stop = True
            #     self.LED_emitor_client("GREEN")
            #     print("STOPPING")

           

if __name__ == '__main__':
    # create the node
    node = MovimentPlanningNode(node_name='moviment_planning_node')
    while not rospy.is_shutdown() and node.running:
        node.run()
        node.publish_exit()
        node.set_velocity()
        rospy.sleep(0.1)

