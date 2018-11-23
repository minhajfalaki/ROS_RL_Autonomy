import sys
import rospy
import random
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError 
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Twist,TwistStamped
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

        self.throttle_command_object = Twist()
        self.brake_command_object = BrakeCmd()
        self.steering_command_object = SteeringCmd()
        self.pub_steering_cmd = rospy.Publisher('/%s/steering_cmd'%(vehicle_name), SteeringCmd, queue_size=1)
        self.pub_velocity_cmd = rospy.Publisher('/%s/cmd_velocity'%(vehicle_name), Twist, queue_size=1)
        self.pub_brake_cmd = rospy.Publisher('/%s/cmd_velocity'%(vehicle_name), BrakeCmd, queue_size=1)

    def steer_callback(self,data):
        self.steering_wheel_angle = data.steering_wheel_angle
        self.steering_wheel_angle_cmd = data.steering_wheel_angle_cmd
        self.speed = data.speed

    def cmd_publisher(self,action):

        if action==[1,0,0,0,0,0,0,0,0,0]:
            throttle = 0
            steer = 0
            # brake = 2

        elif action==[0,1,0,0,0,0,0,0,0,0]:
            throttle = 3
            steer = -6
            # brake = 0

        elif action==[0,0,1,0,0,0,0,0,0,0]:
            throttle = 4
            steer = -3
            # brake = 0

        elif action==[0,0,0,1,0,0,0,0,0,0]:
            throttle = 5
            steer = -1.5
            # brake = 0

        elif action==[0,0,0,0,1,0,0,0,0,0]:
            throttle = 6
            steer = 0.0
            # brake = 0

        elif action==[0,0,0,0,0,1,0,0,0,0]:
            throttle = 6
            steer = 0.0
            # brake = 0

        elif action==[0,0,0,0,0,0,1,0,0,0]:
            throttle = 5
            steer = 1.5
            # brake = 0

        elif action==[0,0,0,0,0,0,0,1,0,0]:
            throttle = 4
            steer = 3
            # brake = 0

        elif action==[0,0,0,0,0,0,0,0,1,0]:
            throttle = 3
            steer = 6
            # brake = 0

        elif action==[0,0,0,0,0,0,0,0,0,1]:
            throttle = 0
            steer = 0.0
            # brake = 2

        if(steer<-8.2):
            steer = -8.2
        elif(steer>8.2):
            steer = 8.2

        self.steering_cmd_publisher(angle_rad=steer)
        self.throttle_cmd_publisher(throttle_value=throttle)
        # self.brake_cmd_publisher(brake_value=brake)

    def throttle_cmd_publisher(self, throttle_value=0.0):
        self.throttle_command_object.linear.x = throttle_value
        self.pub_velocity_cmd.publish(self.throttle_command_object)


    def steering_cmd_publisher(self, angle_rad=0.0):
        self.steering_command_object.steering_wheel_angle_cmd = angle_rad
        self.steering_command_object.steering_wheel_angle_velocity = random.uniform(0.0, 8.7)
        self.pub_steering_cmd.publish(self.steering_command_object)

    # def brake_cmd_publisher(self, brake_value=0.0):
    #     self.brake_command_object.pedal_cmd = brake_value
    #     self.brake_command_object.pedal_cmd_type = 1

class Reward:

    def __init__(self,vehicle_name,spawn):

        self.r = rospy.Rate(30)
        self.play = Play(vehicle_name)
        self.game = Game(vehicle_name,False)
        self.spawn = spawn
        self.vehicle_name = vehicle_name
        self.bridge = CvBridge()
        
        self.loc = rospy.Subscriber("%s/localization_data"%(vehicle_name), localization, self.loc_callback)


    def loc_callback(self,data):

        data = data
        self.left_d = data.left_d
        self.right_d = data.right_d
        self.angle = data.angle
        self.v_act = data.velocity.linear.x
        print self.spawn, self.left_d,self.right_d,self.vehicle_name
        if self.left_d >= 12.5 or self.right_d >=12.5:
            self.spawn = True
        else:
            self.spawn = self.spawn

        self.spawn = self.game.respawn(self.spawn)

    def image_callback(self,image):
        try:
            self.seq=image.header.seq
            self.image_time = time.time()
            state_input = self.bridge.imgmsg_to_cv2(image, "bgr8")
            state_input = cv2.cvtColor(state_input, cv2.COLOR_BGR2GRAY)
            # print "image no",self.seq,"made at",time.time(),"==="
            self.state_input = cv2.resize(state_input, (150,200))
        except CvBridgeError as e:
            print(e)
            last_time = time.time()
        self.top_view.unregister()


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

        reward = r1+r2
        # print " left:",left," right: ",right, " ang: ",ang," r1: ",r1, " r2: ",r2, 

        return reward

    def returns(self,action):
        self.play.cmd_publisher(action)
        self.top_view = rospy.Subscriber("%s/vector_map_image"%(self.vehicle_name), Image, self.image_callback,  queue_size=1)
        # print "cmd given", time.time()
        self.r.sleep()
        self.r.sleep()
        time.sleep(0.1)
        self.ret_time = time.time()
        state = self.state_input
        seq = self.seq
        reward = self.reward_func()
        returns = [seq ,state,reward,self.spawn]
        # print self.spawn, "is ep",self.vehicle_name
        return returns

class Game:

    def __init__(self,vehicle_name,spawn):

        rospy.init_node('Game', anonymous=True)
        self.r = rospy.Rate(30)
        self.spawn=spawn
        self.spawning = False
        self.odom_data = [[0,0,0],[0,0,0,0]]
        self.twist_data = [[0,0,0],[0,0,0]]
        self.linear_velocity = [0,0,0]
        self.angular_velocity = [0,0,0]
        self.vehicle_name = vehicle_name
        self.odom = rospy.Subscriber("/gazebo/model_states", ModelStates, self.odom_callback)
        self.cmd_vel = rospy.Subscriber("/%s/twist"%(vehicle_name), TwistStamped, self.vel_callback )
        if vehicle_name == 'fusion':
        	v2 = "mkz"
        	self.nn=27
        else:
        	v2 = "fusion"
        	self.nn=28


    def vel_callback(self,data):

        self.linear_velocity[0] = data.twist.linear.x
        self.linear_velocity[1] = data.twist.linear.y
        self.linear_velocity[2] = data.twist.linear.z

        self.angular_velocity[0] = data.twist.angular.x
        self.angular_velocity[1] = data.twist.angular.y
        self.angular_velocity[2] = data.twist.angular.z


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
        # self.r.sleep

    def is_episode_finished(self):

        ep = False
        # print self.spawning,"is ep"
        if self.spawning != True:
            ep = False
        else:
            ep = True
        # print ep
        return ep

    def new_episode(self):
        self.counter = 10000


    def respawn(self,spawning):
        self.spawning = spawning

        if self.spawn == True or self.spawning == True:
            self.counter=20000
            self.counter+=1
            # print self.counter

        else:
        	self.counter = 0


        self.spawn_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)

        if self.vehicle_name == "fusion":
            xx = 55.2
            yy = -100
            zz = 0.3
            yaw = 0

        else:
            xx = -87.7943398547
            yy = 394.797117511
            zz = 0.3 
            yaw = -1  	
        states = ModelState()
        # print self.spawning, "is the actual value"

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

            return False

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
                self.counter+=1
                time.sleep(0.2)
                
            elif 20002<=self.counter<20004:

                posex = xx
                posey = yy
                posez = zz
                
                poseox = 0
                poseoy = 0
                poseoz = yaw
                poseow = 0
                
                twistx = 0
                twisty = 0
                twistz = 0
                
                twistax = 0
                twistay = 0
                twistaz = 0
                self.counter+=1
                time.sleep(0.2)
                
            if self.counter >= 20004:
                # print self.counter
                self.counter = 0

                # print self.counter

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

        return False




 



