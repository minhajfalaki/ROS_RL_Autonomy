import sys
import rospy
import random
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError 
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Twist
from dbw_mkz_msgs.msg import BrakeCmd
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringCmd
from dbw_mkz_msgs.msg import GearCmd, Gear
from dbw_mkz_msgs.msg import SteeringReport, BrakeReport, ThrottleReport
from sensor_msgs.msg import PointCloud2, PointField, Image
from sample_vector_data.msg import localization
from std_msgs.msg import Empty


class Play():

    def __init__(self,vehicle_name):
		self.posex=0.0
		self.posey=0.0
		self.posez=0.0
		self.speed=0.0
		self.pub_steering_cmd = rospy.Publisher('/%s/steering_cmd'%(vehicle_name), SteeringCmd, queue_size=1)
		self.pub_velocity_cmd = rospy.Publisher('/%s/cmd_velocity'%(vehicle_name), Twist, queue_size=1)

    def steer_callback(self,data):
        self.steering_wheel_angle = data.steering_wheel_angle
        self.steering_wheel_angle_cmd = data.steering_wheel_angle_cmd
        self.speed = data.speed

    def cmd_publisher(self,action):
        kp_steer = 1
        ki_steer = 1
        kd_steer = 1
        kp_throt = 0.16
        ki_throt = 0
        kd_throt = 0
        kp_brake = -0.05
        ki_brake = -0
        kd_brake = -0
        # rate = rospy.Rate(5)
        cmd_speed = 10
        cmd_y = 0
        prev_t = 1
        e_speed_prev = 0
        e_lat_prev = 0
        e_sum_speed = 0
        e_sum_lat = 0

        
        if action==[1,0,0,0,0,0,0,0,0,0]:
            throttle = 5
            e_speed = 0
            cmd_y = 0.5
        
        elif action==[0,1,0,0,0,0,0,0,0,0]:
            throttle = 0
            e_speed = 0
            cmd_y = 0.0
        
        cur_t = rospy.get_time()
        delta_t = cur_t-prev_t
        prev_t = cur_t
        e_lat = cmd_y-self.posey
        e_delta_lat = e_lat-e_lat_prev
        e_sum_lat += e_lat*delta_t
        e_lat_prev = e_lat
        steer = (kp_steer*e_lat)+ (kd_steer*e_delta_lat/delta_t) + (ki_steer*e_sum_lat)

        if(steer<-8.2):
            steer = -8.2
        elif(steer>8.2):
            steer = 8.2
        steer = 0.0

        self.steering_cmd_publisher(angle_rad=steer)
        self.throttle_cmd_publisher(throttle_value=throttle)

    def throttle_cmd_publisher(self, throttle_value=0.0):
        throttle_command_object = Twist()
        throttle_command_object.linear.x = throttle_value
        self.pub_velocity_cmd.publish(throttle_command_object)


    def steering_cmd_publisher(self, angle_rad=0.0):
        steering_command_object = SteeringCmd()
        steering_command_object.steering_wheel_angle_cmd = angle_rad
        steering_command_object.steering_wheel_angle_velocity = random.uniform(0.0, 8.7)
        steering_command_object.enable = True
        steering_command_object.ignore = False
        steering_command_object.quiet = False
        self.pub_steering_cmd.publish(steering_command_object)


class Reward:

    def __init__(self,vehicle_name,action,spawn,l):

        self.r = rospy.Rate(5)
        self.play = Play(vehicle_name)
        self.action = action
        self.game = Game(vehicle_name,False)
        self.spawn = spawn
        self.vehicle_name = vehicle_name
        self.l=l
        self.bridge = CvBridge()
        self.loc = rospy.Subscriber("%s/localization_data"%(vehicle_name), localization, self.loc_callback)
        self.top_view = rospy.Subscriber("%s/vector_map_image"%(vehicle_name), Image, self.image_callback)


    def loc_callback(self,data):

        data = data
        self.left_d = data.left_d
        self.right_d = data.right_d
        self.angle = data.angle
        self.v_act = data.velocity.linear.x
        self.play.cmd_publisher(self.action)
        self.game.respawn(self.spawn)
        self.r.sleep()


    def image_callback(self,image):

        # print time.time(),"image taken"
        state_input = self.bridge.imgmsg_to_cv2(image, "bgr8")
        state_input = cv2.cvtColor(state_input, cv2.COLOR_BGR2GRAY)
        self.state_input = cv2.resize(state_input, (150,200))
        cv2.imwrite("img_%s/%s.jpg"%(self.vehicle_name,self.l), self.state_input)
        # cv2.imshow("image",self.state_input)
        # cv2.waitKey(150)
        self.r.sleep()
        print time.time(), "image returned", "iteration", self.l, self.vehicle_name 


    def reward_func(self):
        left = self.left_d
        right = self.right_d
        ang = self.angle

        a = 2.5-left
        b = 7.5-right
        c = ang 

        if (-0.2 < a < 0.2):
            r1 = 5
        elif (-3< a <= -0.2):
            r1 = 2*a
        elif (0.2 <=a< 2.5):
            r1 = -2*a
        else:
            r1 = -20

        nS = self.v_act - 5.56

        if (-2< nS < 2):
            r2=5

        elif (-5< nS <=-2):
            r2=2*nS

        elif (2 <= nS <5):
            r2=-2*nS

        else:
            r2=-25

        reward = r2+r2

        return reward

    def returns(self):
        state = self.state_input
        reward = self.reward_func()
        returns = [state,reward]
        return returns
        # print returns


class Game:

    def __init__(self,vehicle_name,spawn):

        # self.play = Play([1,0,0,0,0,0,0,0,0,0])
        self.r = rospy.Rate(5)
        self.spawn=spawn
        self.odom_data = [[0,0,0],[0,0,0,0]]
        self.twist_data = [[0,0,0],[0,0,0]]
        self.linear_velocity = [0,0,0]
        self.angular_velocity = [0,0,0]
        self.vehicle_name = vehicle_name
        self.odom = rospy.Subscriber("/gazebo/model_states", ModelStates, self.odom_callback)
        self.cmd_vel = rospy.Subscriber("/%s/cmd_vel"%(vehicle_name), Twist, self.vel_callback )
        if vehicle_name == 'fusion':
        	# print vehicle_name
        	v2 = "mkz"
        	self.nn=27
        else:
        	# print vehicle_name
        	v2 = "fusion"
        	self.nn=28


    def vel_callback(self,data):

        self.linear_velocity[0] = data.linear.x
        self.linear_velocity[1] = data.linear.y
        self.linear_velocity[2] = data.linear.z

        self.angular_velocity[0] = data.angular.x
        self.angular_velocity[1] = data.angular.y
        self.angular_velocity[2] = data.angular.z
        self.r.sleep()


    def odom_callback(self,data):

        self.odom_data[0][0] = data.pose[self.nn].position.x
        self.odom_data[0][1] = data.pose[self.nn].position.y
        self.odom_data[0][2] = data.pose[self.nn].position.z
        
        self.odom_data[1][0] = data.pose[self.nn].orientation.x
        self.odom_data[1][1] = data.pose[self.nn].orientation.y
        self.odom_data[1][2] = data.pose[self.nn].orientation.z
        self.odom_data[1][3] = data.pose[self.nn].orientation.w
        
        self.twist_data[0][0] = data.twist[self.nn].linear.x
        self.twist_data[0][1] = data.twist[self.nn].linear.y
        self.twist_data[0][2] = data.twist[self.nn].linear.z
        
        self.twist_data[1][0] = data.twist[self.nn].angular.x
        self.twist_data[1][1] = data.twist[self.nn].angular.y
        self.twist_data[1][2] = data.twist[self.nn].angular.z
        self.r.sleep

    def is_episode_finished(self):

        ep = False
        if self.counter < 20000:
            ep = False
        else:
            ep = True
        return ep

    def new_episode(self):
        self.counter = 10000


    def respawn(self,spawning):



		if self.spawn == True or spawning == True:
			self.counter=20000
			self.counter+=1

		else:
			self.counter = 0

		# while not rospy.is_shutdown():
		self.spawn_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
		# self.counter+=1
		# print self.is_episode_finished()
		# print self.counter

		if self.vehicle_name == "fusion":
			xx = 55.2
			yy = -100
			zz = 0.3

		else:
			xx = 44.2
			yy = -100
			zz = 0.3        	
		# xx = 44.2
		# yy = -100
		# zz = 0.3

		if self.counter==0:

		    posex = self.odom_data[0][0]
		    posey = self.odom_data[0][1]
		    posez = self.odom_data[0][2]
		    
		    poseox = self.odom_data[1][0]
		    poseoy = self.odom_data[1][1]
		    poseoz = self.odom_data[1][2]
		    poseow = self.odom_data[1][3]

		    twistx = self.linear_velocity[0]
		    twisty = self.linear_velocity[1]
		    twistz = self.linear_velocity[2]
		    
		    twistax = self.angular_velocity[0]
		    twistay = self.angular_velocity[1]
		    twistaz = self.angular_velocity[2]

		    return

		while self.counter>20000:

		    if 20000<=self.counter<20002:

		        posex = self.odom_data[0][0]
		        posey = self.odom_data[0][1]
		        posez = self.odom_data[0][2]
		        
		        poseox = self.odom_data[1][0]
		        poseoy = self.odom_data[1][1]
		        poseoz = self.odom_data[1][2]
		        poseow = self.odom_data[1][3]
		        
		        twistx = self.twist_data[0][0]
		        twisty = self.twist_data[0][1]
		        twistz = self.twist_data[0][2]
		        
		        twistax = self.twist_data[1][0]
		        twistay = self.twist_data[1][1]
		        twistaz = self.twist_data[1][2]
		        # self.new_episode()
		        self.counter+=1
		        print self.counter,'in'

		        # print self.state
		        
		    elif 20002<=self.counter<20004:

		        posex = xx
		        posey = yy
		        posez = zz
		        
		        poseox = 0
		        poseoy = 0
		        poseoz = 0
		        poseow = 1
		        
		        twistx = 0
		        twisty = 0
		        twistz = 0
		        
		        twistax = 0
		        twistay = 0
		        twistaz = 0
		        self.counter+=1
		        print self.counter,"inn"
		        
		        # rospy.loginfo("########### Respawning the car... ###########")
		        
		    if self.counter >= 20004:
		        self.counter = 0
		        # self.new_episode()

		    states = ModelState()
		    print self.vehicle_name
		    states.model_name = "%s"%(self.vehicle_name)
		    states.twist.linear.x = twistx
		    states.twist.linear.y = twisty
		    states.twist.linear.z = twistz

		    states.twist.angular.x = twistax
		    states.twist.angular.y = twistay
		    states.twist.angular.z = twistaz

		    states.pose.position.x = posex 
		    states.pose.position.y = posey
		    states.pose.position.z = posez

		    states.pose.orientation.x = poseox
		    states.pose.orientation.y = poseoy
		    states.pose.orientation.z = poseoz
		    states.pose.orientation.w = poseow
		    self.spawn_publisher.publish(states)
		    self.r.sleep()




