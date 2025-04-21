#!/usr/bin/python3
import smbus
import rospy
from std_msgs.msg import Float64

# Arduino is 0x29 on Bus 1 (pins 3,5)
# i2cdetect -y -r 1
bus = smbus.SMBus(1)
addr = 0x29

# for the standalone servo with the clip attached
#mid_us = 1590

# balance beam servo
mid_us = 1450

min_us = mid_us-200
max_us = mid_us+200



def write(msg: str) -> None:
    for c in list(msg):
        bus.write_byte(addr, ord(c))

def write_uint16(i: int) -> None:
    bytes = [(i&0xFF00) >> 8, i&0x00FF]
    bus.write_i2c_block_data(addr, bytes[0], bytes[1:])

def callback(data):
    # 0.0 : 1.0 -> min_us : max_us
    data_us = (data.data*(max_us-min_us)) + min_us
    data_us = int(data_us)
    write_uint16(data_us)

    rospy.loginfo(rospy.get_caller_id() + f' Got: {data.data} Wrote: {data_us}')

def on_shutdown():
    write_uint16(0)
    bus.close()

def listener():
    write_uint16(mid_us)
    rospy.init_node('single_servo_i2c', anonymous=True)
    rospy.on_shutdown(on_shutdown)
    rospy.Subscriber('test_topic', Float64, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

