#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('test_topic', Float64, queue_size=10)
    rospy.init_node('sweep', anonymous=False)
    rate = rospy.Rate(10)

    i = 0.5
    step = 0.01

    while not rospy.is_shutdown():
        rospy.loginfo(f'Sent: {i}')
        pub.publish(i)

        i = i+step
        if i >= 1.0:
            i = 1.0
            step = -step
        
        elif i <= 0.0:
            i = 0.0
            step = -step
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

