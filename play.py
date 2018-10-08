import rospy
import numpy as np
import random
import time
import sys

from dbw_mkz_msgs.msg import BrakeCmd
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringCmd
from dbw_mkz_msgs.msg import GearCmd, Gear
from gazebo_msgs.msg import ModelStates
from dbw_mkz_msgs.msg import SteeringReport, BrakeReport, ThrottleReport
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
		rate = rospy.Rate(5)
		cmd_speed = 10
		cmd_y = 0
		prev_t = 1
		e_speed_prev = 0
		e_lat_prev = 0
		e_sum_speed = 0
		e_sum_lat = 0

		while not rospy.is_shutdown():
			
			if self.action==[1,0,0,0,0,0,0,0,0,0]:
				cmd_speed = 0.5
				e_speed = 0
				cmd_y = 0.5
			
			elif self.action==[0,1,0,0,0,0,0,0,0,0]:
				cmd_speed = 0.75
				e_speed = 0
				cmd_y = 0.0
			
			# elif self.action=[1,0,0,0,0,0,0,0,0,0]:

			# 	cmd_speed = 0.75
			# 	e_speed = 0
			# 	cmd_y = 0.5
			
			# elif self.action=[1,0,0,0,0,0,0,0,0,0]:

			# 	cmd_speed = 0.75
			# 	e_speed = 0
			# 	cmd_y = 0.5
			
			# elif self.action=[1,0,0,0,0,0,0,0,0,0]:

			# 	cmd_speed = 0.75
			# 	e_speed = 0
			# 	cmd_y = 0.5
			
			# elif self.action=[1,0,0,0,0,0,0,0,0,0]:

			# 	cmd_speed = 0.75
			# 	e_speed = 0
			# 	cmd_y = 0.5
			
			# elif self.action=[1,0,0,0,0,0,0,0,0,0]:

			# 	cmd_speed = 0.75
			# 	e_speed = 0
			# 	cmd_y = 0.5
			
			# elif self.action=[1,0,0,0,0,0,0,0,0,0]:

			# 	cmd_speed = 0.75
			# 	e_speed = 0
			# 	cmd_y = 0.5
			
			# elif self.action=[1,0,0,0,0,0,0,0,0,0]:

			# 	cmd_speed = 0.75
			# 	e_speed = 0
			# 	cmd_y = 0.5
			
			# elif self.action=[1,0,0,0,0,0,0,0,0,0]:

			# 	cmd_speed = 0.75
			# 	e_speed = 0
			# 	cmd_y = 0.5
			
			# elif self.action=[1,0,0,0,0,0,0,0,0,0]:

			# 	cmd_speed = 0.75
			# 	e_speed = 0
			# 	cmd_y = 0.5


			rospy.loginfo("########### Starting CarControl Cmd Publish Round... ###########")
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
			rate.sleep()

	def throttle_cmd_publisher(self, throttle_value=0.0):
		throttle_command_object = ThrottleCmd()
		throttle_command_object.pedal_cmd = throttle_value
		throttle_command_object.pedal_cmd_type = 1
		throttle_command_object.enable = True
		throttle_command_object.ignore = False
		throttle_command_object.count = 0
		rospy.loginfo("Throttle Publish ::>"+str(throttle_command_object.pedal_cmd))
		rospy.loginfo("Throttle Pedal_cmd Report ::>"+str(self.throttle_pedal_cmd))
		rospy.loginfo("Throttle Pedal_input Report ::>"+str(self.throttle_pedal_input))
		rospy.loginfo("Throttle Pedal_output Report ::>"+str(self.throttle_pedal_output))
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
		rospy.loginfo("Brake Publish ::>"+str(brake_command_object.pedal_cmd))
		rospy.loginfo("Brake Pedal_cmd Report ::>"+str(self.brake_pedal_cmd))
		rospy.loginfo("Brake Pedal_input Report ::>"+str(self.brake_pedal_input))
		rospy.loginfo("Brake Pedal_output Report ::>"+str(self.brake_pedal_output))
		self.pub_brake_cmd.publish(brake_command_object)

	def steering_cmd_publisher(self, angle_rad=0.0):
		steering_command_object = SteeringCmd()
		steering_command_object.steering_wheel_angle_cmd = angle_rad
		steering_command_object.steering_wheel_angle_velocity = random.uniform(0.0, 8.7)
		steering_command_object.enable = True
		steering_command_object.ignore = False
		steering_command_object.quiet = False
		steering_command_object.count = 0
		rospy.loginfo("Steering Publish::>"+str(steering_command_object.steering_wheel_angle_cmd))
		rospy.loginfo("Steering wheel_angle Report::>"+str(self.steering_wheel_angle))
		rospy.loginfo("Steering wheel_cmd Report::>"+str(self.steering_wheel_angle))
		rospy.loginfo("Speed ::>"+str(self.speed))
		self.pub_steering_cmd.publish(steering_command_object)

	# def gear_cmd_publisher(self,gear_value=0):
	# 	gear_command_object = GearCmd()
	# 	gear_object = Gear()
	# 	gear_object.gear = gear_value
	# 	gear_command_object.cmd = gear_object
	# 	rospy.loginfo("Gear Publish ::>"+str(gear_object.gear))
	# 	self.pub_gear_cmd.publish(gear_command_object)
	

def main(args):
    rospy.init_node('Game', anonymous=True)
    play = Play([1,0,0,0,0,0,0,0,0,0])
    try:
        play.cmd_publisher()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
