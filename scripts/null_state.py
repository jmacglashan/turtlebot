#!/usr/bin/python

# license removed for brevity
import rospy
from std_msgs.msg import String
from burlap_msgs.msg import burlap_state
from burlap_msgs.msg import burlap_object
from burlap_msgs.msg import burlap_value

def talker():
    pub = rospy.Publisher('burlap_state', burlap_state, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	s = burlap_state()
	s.objects = []
	
	pub.publish(s)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


