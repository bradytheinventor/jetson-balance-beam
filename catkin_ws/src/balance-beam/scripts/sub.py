#!/usr/bin/python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + f'heard {data.data}')

def listener():
    rospy.init_node('sub', anonymous=True)
    rospy.Subscriber('test_topic', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
