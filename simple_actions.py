import sys
import rospy
import random
import time
import numpy as np
import cv2
from game import Game, Reward

def main(args):
    rospy.init_node('Game', anonymous=True)
    # game = Game(True)
    l=0
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        # time.sleep(1)
        if l<=20:
            reward = Reward("fusion",[1,0,0,0,0,0,0,0,0,0],False,l)
            # time.sleep(0.5)
            reward = Reward("mkz",[0,1,0,0,0,0,0,0,0,0],False,l)
            # S = reward.returns()
            # print S
        elif l<=22:
            reward = Reward("fusion",[0,1,0,0,0,0,0,0,0,0],True,l)
            reward = Reward("mkz",[0,1,0,0,0,0,0,0,0,0],True,l)
            l=0
            # S = reward.reward_func()
            # print S
    	else:
            l=0
        l+=1
        print time.time()
        r.sleep()
        # print l,"ll"

 

                                                # def main(args):
                                                #     rospy.init_node('Game', anonymous=True)
                                                #     l=0
                                                #     try:
                                                #         if l<=2000:
                                                #             reward = Reward("fusion",[1,0,0,0,0,0,0,0,0,0],False,l)
                                                #             # time.sleep(0.5)
                                                #             reward = Reward("mkz",[0,1,0,0,0,0,0,0,0,0],False,l)
                                                #             # S = reward.returns()
                                                #             # print S
                                                #         elif l<=2001:
                                                #             reward = Reward("fusion",[0,1,0,0,0,0,0,0,0,0],True,l)
                                                #             reward = Reward("mkz",[0,1,0,0,0,0,0,0,0,0],True,l)
                                                #             time.sleep(0.2)
                                                #             l=0
                                                #             # S = reward.reward_func()
                                                #             # print S
                                                #         else:
                                                #             l=0
                                                #         l+=1
                                                #         # print l,"ll"
                                                #     except rospy.ROSInterruptException:
                                                #         print "Exception occured"
                                                #         pass
if __name__ == '__main__':
    main(sys.argv)