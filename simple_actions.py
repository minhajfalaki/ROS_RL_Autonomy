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
		r = rospy.Rate(30)
		reward = Reward(self.vehicle_name,False)
		game = Game(self.vehicle_name,False)
		r.sleep()
		returns = reward.returns([1,0,0,0,0,0,0,0,0,0])
		if self.i>=100:
			game.respawn(True)
			self.i=0

		# print returns,i

for n in range(10):
	for i in range(130):
		print i

		thread1 = SummingThread("fusion",i)
		thread2 = SummingThread("mkz",i)
		thread1.start() # This actually causes the thread to run
		thread2.start()
		thread1.join()  # This waits until the thread has completed
		thread2.join()  
