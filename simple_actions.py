import sys
import rospy
import random
import time
import numpy as np
import cv2
import threading
from game import Game, Reward


class SummingThread(threading.Thread):
    def __init__(self,vehicle_name,i):
        super(SummingThread, self).__init__()
        rospy.init_node('Game', anonymous=True)
        self.vehicle_name = vehicle_name
        self.i = i


    def run(self):
		l=0
		r = rospy.Rate(10)
		# while not rospy.is_shutdown():
		print "in rospy,at",time.time()
		reward = Reward(self.vehicle_name,[1,0,0,0,0,0,0,0,0,0],True,l)
		r.sleep()
		returns = reward.returns()
		print returns,i


for i in range(1000):

	thread1 = SummingThread("fusion",i)
	thread2 = SummingThread("mkz",i)
	thread1.start() # This actually causes the thread to run
	thread2.start()
	thread1.join()  # This waits until the thread has completed
	thread2.join()  
