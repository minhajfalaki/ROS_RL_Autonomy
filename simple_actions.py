import sys
import rospy
import random
import time
import numpy as np
import cv2
from game import Game, Reward

def main(args):
    rospy.init_node('Game', anonymous=True)
    game = Game()
    l=0
    try:
    	if l<=10:
    		returns = Reward([0,1,0,0,0,0,0,0,0,0])
    	elif l<=20:
    		returns = Reward([1,0,0,0,0,0,0,0,0,0])
    	l+=1
    	game.respawn()
    except KeyboardInterrupt:
    	print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)