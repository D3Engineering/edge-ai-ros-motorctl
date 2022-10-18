#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import can
import os
import time

global estop

def update_motor_speed(canbus, speed):
    hexstr = "0x{:04x}".format(speed & 0xffff)[-4:]
    hexmsb = int("0x"+hexstr[0:2], 16)
    hexlsb = int("0x"+hexstr[2:], 16)
    drivemotorb = 0
    if not speed == 0:
        drivemotorb = 1
    msg = can.Message(arbitration_id=0x01, data=[drivemotorb, 0, hexmsb, hexlsb, 0, 0, 0, 0], is_extended_id=False)
    canbus.send(msg)


def callback(data):
    global last_left_speed
    global last_right_speed
    global estop

    ###### MOTOR DRIVE COEFFICIENTS
    # Should add to <= 4096
    linear_coeff = 192
    angular_coeff = 144
    #linear_coeff = 512
    #angular_coeff = 256
    left_linear_offset = 0
    right_linear_offset = 0
    ###############################


    lbus = can.Bus(channel='can0', interface='socketcan')
    rbus = can.Bus(channel='can2', interface='socketcan')
    rospy.loginfo(rospy.get_caller_id() + "\n%s", data)
    
    base_left_speed = int((linear_coeff + left_linear_offset) * data.linear.x) - int(angular_coeff * data.angular.z)
    base_right_speed = int((linear_coeff + right_linear_offset) * data.linear.x) + int(angular_coeff * data.angular.z)
    
    if data.linear.y == 1:
        estop = True
        rospy.loginfo("EStop Triggered")
    elif data.linear.z == 1:
        estop = False
        rospy.loginfo("EStop Released")

    triggered_zero = False
    if (base_left_speed < 0 and last_left_speed > 0) or (base_left_speed > 0 and last_left_speed < 0):
        update_motor_speed(lbus, 0)
        triggered_zero = True

    if (base_right_speed < 0 and last_right_speed > 0) or (base_right_speed > 0 and last_right_speed < 0):
        update_motor_speed(rbus, 0)
        triggered_zero = True
        
    if triggered_zero:
        time.sleep(0.1)
    
    try:
        if not estop:
            update_motor_speed(lbus, -base_left_speed)
            update_motor_speed(rbus, base_right_speed)
            rospy.loginfo("Left Motor: " + str(base_left_speed) + ", Right Motor: " + str(base_right_speed))
            last_left_speed = base_left_speed
            last_right_speed = base_right_speed
        else:
            update_motor_speed(lbus, 0)
            update_motor_speed(rbus, 0)
            rospy.loginfo("EStop Active - Motors Disabled")
            last_left_speed = 0
            last_right_speed = 0
    except Exception as e:
        rospy.loginfo(e)
        update_motor_speed(lbus, 0)
        update_motor_speed(rbus, 0)
        rospy.loginfo("Caught Exception in callback, killing motors")


def listener():
    global last_left_speed
    global last_right_speed
    global estop

    last_left_speed = 0
    last_right_speed = 0
    estop = False
    os.system("ip link set can0 type can bitrate 500000")
    os.system("ip link set can0 up")
    os.system("ip link set can2 type can bitrate 500000")
    os.system("ip link set can2 up")
    rospy.init_node('motor_listener', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()
    rospy.loginfo("End of Script, killing motors...")
    lbus = can.Bus(channel='can0', interface='socketcan')
    rbus = can.Bus(channel='can2', interface='socketcan')
    update_motor_speed(lbus, 0)
    update_motor_speed(rbus, 0)


if __name__ == '__main__':
    listener()
