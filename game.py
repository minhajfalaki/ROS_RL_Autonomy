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

    def __init__(self,action):
        self.action = action
        self.posex=0.0
        self.posey=0.0
        self.posez=0.0
        self.speed=0.0
        self.brake_pedal_cmd=0.0
        self.brake_pedal_input=0.0
        self.brake_pedal_output=0.0
        self.throttle_pedal_cmd=0.0
        self.throttle_pedal_input=0.0
        self.throttle_pedal_output=0.0
        self.steering_wheel_angle=0.0
        self.steering_wheel_angle_cmd=0.0
        self.i=0
        self.pub_brake_cmd = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)
        self.pub_throttle_cmd = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.pub_steering_cmd = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)

        sub_steer = rospy.Subscriber('/vehicle/steering_report', SteeringReport, self.steer_callback)
        sub_brake = rospy.Subscriber('/vehicle/brake_report', BrakeReport, self.brake_callback)
        sub = rospy.Subscriber("/fusion/enable", Empty, self.enable_callback)
        sub_state = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        sub_throttle = rospy.Subscriber('/vehicle/throttle_report', ThrottleReport, self.throttle_callback)
        rospy.wait_for_service('/gazebo/set_physics_properties')
        rospy.loginfo("Gazebo Ready...")

    def enable_callback(self,data):
        self._enable_dbw = True

    def state_callback(self,data):
        self.posex = data.pose[1].position.x
        self.posey = data.pose[1].position.y
        self.posez = data.pose[1].position.z
        self.quat = data.pose[1].orientation

    def brake_callback(self,data):
        self.brake_pedal_cmd = data.pedal_cmd
        self.brake_pedal_input = data.pedal_input
        self.brake_pedal_output = data.pedal_output

    def throttle_callback(self,data):
        self.throttle_pedal_cmd = data.pedal_cmd
        self.throttle_pedal_input = data.pedal_input
        self.throttle_pedal_output = data.pedal_output
        # print self.throttle_pedal_output,"ll"

    def steer_callback(self,data):
        self.steering_wheel_angle = data.steering_wheel_angle
        self.steering_wheel_angle_cmd = data.steering_wheel_angle_cmd
        self.speed = data.speed
        # print self.speed


    def cmd_publisher(self):
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

        # while not rospy.is_shutdown():
        
        if self.action==[1,0,0,0,0,0,0,0,0,0]:
            cmd_speed = 10
            e_speed = 0
            cmd_y = 0.5
        
        elif self.action==[0,1,0,0,0,0,0,0,0,0]:
            cmd_speed = 5
            e_speed = 0
            cmd_y = 0.0
        
        # elif self.action=[1,0,0,0,0,0,0,0,0,0]:

        #   cmd_speed = 0.75
        #   e_speed = 0
        #   cmd_y = 0.5
        
        # elif self.action=[1,0,0,0,0,0,0,0,0,0]:

        #   cmd_speed = 0.75
        #   e_speed = 0
        #   cmd_y = 0.5
        
        # elif self.action=[1,0,0,0,0,0,0,0,0,0]:

        #   cmd_speed = 0.75
        #   e_speed = 0
        #   cmd_y = 0.5
        
        # elif self.action=[1,0,0,0,0,0,0,0,0,0]:

        #   cmd_speed = 0.75
        #   e_speed = 0
        #   cmd_y = 0.5
        
        # elif self.action=[1,0,0,0,0,0,0,0,0,0]:

        #   cmd_speed = 0.75
        #   e_speed = 0
        #   cmd_y = 0.5
        
        # elif self.action=[1,0,0,0,0,0,0,0,0,0]:

        #   cmd_speed = 0.75
        #   e_speed = 0
        #   cmd_y = 0.5
        
        # elif self.action=[1,0,0,0,0,0,0,0,0,0]:

        #   cmd_speed = 0.75
        #   e_speed = 0
        #   cmd_y = 0.5
        
        # elif self.action=[1,0,0,0,0,0,0,0,0,0]:

        #   cmd_speed = 0.75
        #   e_speed = 0
        #   cmd_y = 0.5
        
        # elif self.action=[1,0,0,0,0,0,0,0,0,0]:

        #   cmd_speed = 0.75
        #   e_speed = 0
        #   cmd_y = 0.5


        # rospy.loginfo("########### Starting CarControl Cmd Publish Round... ###########")
        e_speed = cmd_speed - self.speed
        gear = 4
        cur_t = rospy.get_time()
        delta_t = cur_t-prev_t
        prev_t = cur_t
        e_delta_speed = e_speed-e_speed_prev
        e_speed_prev = e_speed
        e_sum_speed += e_speed*delta_t
        throttle = (kp_throt*e_speed)+ (kd_throt*e_delta_speed/delta_t) + (ki_throt*e_sum_speed)
        brake = (kp_brake*e_speed)+ (kd_brake*e_delta_speed/delta_t) + (ki_brake*e_sum_speed)
        e_lat = cmd_y-self.posey
        e_delta_lat = e_lat-e_lat_prev
        e_sum_lat += e_lat*delta_t
        e_lat_prev = e_lat
        steer = (kp_steer*e_lat)+ (kd_steer*e_delta_lat/delta_t) + (ki_steer*e_sum_lat)

        if(throttle<0.15):
            throttle = 0.15
        elif(throttle>0.8):
            throttle = 0.8
        if(brake<0.15):
            brake = 0.15
        elif(brake>0.5):
            brake = 0.5
        if(steer<-8.2):
            steer = -8.2
        elif(steer>8.2):
            steer = 8.2
        steer = 0.0

        self.brake_cmd_publisher(brake_value=brake)
        self.steering_cmd_publisher(angle_rad=steer)
        # self.gear_cmd_publisher(gear_value=gear)
        self.throttle_cmd_publisher(throttle_value=throttle)
        self.i+=1
        # rate.sleep()

    def throttle_cmd_publisher(self, throttle_value=0.0):
        throttle_command_object = ThrottleCmd()
        throttle_command_object.pedal_cmd = throttle_value
        throttle_command_object.pedal_cmd_type = 1
        throttle_command_object.enable = True
        throttle_command_object.ignore = False
        throttle_command_object.count = 0
        self.pub_throttle_cmd.publish(throttle_command_object)


    def brake_cmd_publisher(self, brake_value=0.0):
        rospy.get_time()
        brake_command_object = BrakeCmd()
        brake_command_object.pedal_cmd = brake_value
        brake_command_object.pedal_cmd_type = 1
        brake_command_object.boo_cmd = True
        brake_command_object.enable = True
        brake_command_object.ignore = False
        brake_command_object.count = 0
        self.pub_brake_cmd.publish(brake_command_object)

    def steering_cmd_publisher(self, angle_rad=0.0):
        steering_command_object = SteeringCmd()
        steering_command_object.steering_wheel_angle_cmd = angle_rad
        steering_command_object.steering_wheel_angle_velocity = random.uniform(0.0, 8.7)
        steering_command_object.enable = True
        steering_command_object.ignore = False
        steering_command_object.quiet = False
        self.pub_steering_cmd.publish(steering_command_object)


class Reward:

	def __init__(self,action):

		self.play = Play(action)
		self.bridge = CvBridge()
		self.loc = rospy.Subscriber("/localization_data", localization, self.loc_callback)
		self.top_view = rospy.Subscriber("/vector_map_image", Image, self.image_callback)
		self.v_need = 5


	def loc_callback(self,data):
		
		self.play.cmd_publisher()
		data = data
		self.left_d = data.left_d
		self.right_d = data.right_d
		self.angle = data.angle


	def image_callback(self,image):
	    
	    try:
	        state_input = self.bridge.imgmsg_to_cv2(image, "bgr8")
	        state_input = cv2.cvtColor(state_input, cv2.COLOR_BGR2GRAY)
	        self.state_input = cv2.resize(state_input, (150,200))
	        cv2.imshow("image",self.state_input)
	        cv2.waitKey(150)
	        self.reward_func()
	        # cv2.destroyAllWindows()
	    except CvBridgeError as e:
	        print(e)
	        last_time = time.time()

	def reward_func(self):
		left = self.left_d
		right = self.right_d
		ang = self.angle
		nS = self.v_need

		a = 2.5-left
		b = 7.5-right
		c = ang 

		if (-0.2 < a < 0.2):
			r1 = 5
			print "r1",r1,a
		elif (-2.5< a <= -0.2) or (0.2 <=a< 2):
			r1 = -2*a
			print "r1",r1,a
		else:
			r1 = -20
			print "r1",r1,a




class Game:

    def __init__(self):

        # self.play = Play([1,0,0,0,0,0,0,0,0,0])
        self.odom_data = [[0,0,0],[0,0,0,0]]
        self.twist_data = [[0,0,0],[0,0,0]]
        self.linear_velocity = [0,0,0]
        self.angular_velocity = [0,0,0]
        self.odom = rospy.Subscriber("/gazebo/model_states", ModelStates, self.odom_callback)
        self.cmd_vel = rospy.Subscriber("/vehicle/cmd_vel", Twist, self.vel_callback )



    def vel_callback(self,data):

        self.linear_velocity[0] = data.linear.x
        self.linear_velocity[1] = data.linear.y
        self.linear_velocity[2] = data.linear.z

        self.angular_velocity[0] = data.angular.x
        self.angular_velocity[1] = data.angular.y
        self.angular_velocity[2] = data.angular.z


    def odom_callback(self,data):

        self.odom_data[0][0] = data.pose[27].position.x
        self.odom_data[0][1] = data.pose[27].position.y
        self.odom_data[0][2] = data.pose[27].position.z
        
        self.odom_data[1][0] = data.pose[27].orientation.x
        self.odom_data[1][1] = data.pose[27].orientation.y
        self.odom_data[1][2] = data.pose[27].orientation.z
        self.odom_data[1][3] = data.pose[27].orientation.w
        
        self.twist_data[0][0] = data.twist[27].linear.x
        self.twist_data[0][1] = data.twist[27].linear.y
        self.twist_data[0][2] = data.twist[27].linear.z
        
        self.twist_data[1][0] = data.twist[27].angular.x
        self.twist_data[1][1] = data.twist[27].angular.y
        self.twist_data[1][2] = data.twist[27].angular.z

    def is_episode_finished(self):

        ep = False
        if self.counter < 20000:
            ep = False
        else:
            ep = True
        return ep

    def new_episode(self):
        self.counter = 0


    def respawn(self):

        self.counter=0
        
        while not rospy.is_shutdown():
            self.spawn_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
            self.counter+=1
            rospy.sleep(0.0002)
            # print self.is_episode_finished()
            # print self.linear_velocity,self.counter
            # print self.angular_velocity
            # print counter

            if self.counter<20000:

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

            if self.counter>20000:
                if 20000<=self.counter<24000:

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

                    # print self.state
                    
                elif 24000<=self.counter<25500:

                    posex = 44.2
                    posey = -100
                    posez = 0.3
                    
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
                    
                    # rospy.loginfo("########### Respawning the car... ###########")
                    
                if self.counter >= 25500:
                    # self.counter = 0
                    self.new_episode()

                states = ModelState()
                states.model_name = "vehicle"
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


def main(args):
    rospy.init_node('Game', anonymous=True)
    game = Game()
    returns = Reward([0,1,0,0,0,0,0,0,0,0])
    # returns.reward_func()
    try:         
        game.respawn()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)


