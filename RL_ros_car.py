import sys
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Twist
import time

class Game:

    def __init__(self):
    	
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
            print self.is_episode_finished()
            print self.linear_velocity,self.counter
            print self.angular_velocity
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
    game = Game()
    rospy.init_node('Game', anonymous=True)
    try:
        game.respawn()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)


