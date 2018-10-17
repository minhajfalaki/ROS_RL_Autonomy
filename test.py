import rospy
from gazebo_msgs.msg import ModelStates, ModelState

l=['ground_plane', '(DS) Straight Road - 200m_8', '(DS) Straight Road - 100m_0', '(DS) Curved Road - 100m_3', '(DS) Straight Road - 50m', '(DS) Curved Road - 100m_7', '(DS) Straight Road - 50m_2', '(DS) Curved Road - 50m_1', '(DS) Curved Road - 50m_2', '(DS) Curved Road - 50m_3', '(DS) Straight Road - 100m_5', '(DS) Curved Road - 50m_7', '(DS) Curved Road - 50m_8',
 '(DS) Curved Road - 50m_9', '(DS) Straight Road - 100m_6', '(DS) Curved Road - 100m_8', 
 '(DS) Curved Road - 100m_9', '(DS) Curved Road - 100m_10', '(DS) Curved Road - 100m_11',
  '(DS) Straight Road - 200m_9', '(DS) Straight Road - 200m_11', '(DS) Curved Road - 50m_10',
   '(DS) Curved Road - 50m_11', '(DS) Curved Road - 50m_12', 
   '(DS) Curved Road - 50m_13', '(DS) Straight Road - 100m_7', '(DS) Straight Road - 100m_8',
  'fusion', 'mkz']


# for i in range(len(l)):
# 	print i,l[i]
rospy.init_node('Game', anonymous=True)

r = rospy.Rate(1) # 10hz
while not rospy.is_shutdown():
    print "sfg"
    r.sleep()



