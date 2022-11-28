#!/usr/bin/env python3
import math
import can
import asyncio
from typing import List
from can.notifier import MessageRecipient
import math
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, PoseWithCovariance, TwistWithCovariance, Pose, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from d3_motorctl.motor_ctl import bytes_to_velocity
import tf

# Define Constants
wheel_gear_teeth = 30
motor_gear_teeth = 18
wheel_speed_correction = 1.25
gear_ratio = wheel_gear_teeth/motor_gear_teeth
wheel_diameter = 0.083 # meters
wheel_radius = wheel_diameter/2
#wheel_base = 0.38 # meters
wheel_base = 0.37 # meters # HACK
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

# Define CAN Buses and Variables
left_can_bus = can.Bus(channel='can0', interface='socketcan')
right_can_bus = can.Bus(channel='can2', interface='socketcan')
can_global_lmsg = None

# Define other Variables
last_proc_time = None
total_x = 0.0
total_y = 0.0
total_theta = 0.0

def setup_listeners():
    global last_proc_time
    rospy.init_node('d3_odometry', anonymous=True)
    last_proc_time = rospy.Time.now()
    left_listeners: List[MessageRecipient] = [
        left_motor_callback
    ]
    right_listeners: List[MessageRecipient] = [
        right_motor_callback
    ]
    loop = asyncio.get_event_loop()
    left_notifier = can.Notifier(left_can_bus, left_listeners, loop=loop)
    right_notifier = can.Notifier(right_can_bus, right_listeners, loop=loop)
    return loop


def left_motor_callback(lmsg: can.Message):
    global can_global_lmsg
    if lmsg.is_rx:
        can_global_lmsg = lmsg


def right_motor_callback(rmsg: can.Message):
    global can_global_lmsg
    if rmsg.is_rx and can_global_lmsg is not None:
        process_can_messages(can_global_lmsg, rmsg)
        can_global_lmsg = None


def compute_wheel_linear_velocity(can_msg, wheel_name):
    global gear_ratio, wheel_radius
    speed_bytes = can_msg.data[4:6]
    speed_raw_int = int.from_bytes(speed_bytes, 'big')
    if ((speed_raw_int >> 15) == 1):
        speed_raw_int -= 65535
    wheel_linear_velocity = bytes_to_velocity(speed_raw_int)
    return wheel_linear_velocity


def compute_x_y_theta(left_linear_velocity, right_linear_velocity, time_elapsed):
    theta = ((left_linear_velocity-right_linear_velocity)/wheel_base)*time_elapsed
    x = (((right_linear_velocity / 2) * math.cos(theta)) +
         ((left_linear_velocity / 2) * math.cos(theta))) * time_elapsed
    y = (((right_linear_velocity / 2) * math.sin(theta)) +
         ((left_linear_velocity / 2) * math.sin(theta))) * time_elapsed
    return x, y, theta


# Processing of CAN Messages into Odometry Data
def process_can_messages(lmsg, rmsg):
    global last_proc_time, wheel_base, total_x, total_y, total_theta
    current_time = rospy.Time.now()
    left_linear_velocity = -compute_wheel_linear_velocity(lmsg, "Left")
    print("Resolved Left LinVel: " + str(left_linear_velocity))
    right_linear_velocity = compute_wheel_linear_velocity(rmsg, "Right")
    print("Resolved Right LinVel: " + str(right_linear_velocity))
    time_elapsed_since_last_proc = current_time.to_sec() - last_proc_time.to_sec()

    print("Time Elapsed: " + str(time_elapsed_since_last_proc))
    dx, dy, dt = compute_x_y_theta(left_linear_velocity, right_linear_velocity,
                                                      time_elapsed_since_last_proc)
    print("Local [X: " + str(dx) + ", Y: " + str(dy) + ", Theta: " + str(dt) + "]")
    if dx != float("Inf") and not math.isnan(dx) and dy != float("Inf") and not math.isnan(dy):
        H = np.linalg.norm((dx,dy))
        if dx < 0 and dy < 0:
            H = -H
        total_x += H * math.cos(total_theta)
        total_y += H * math.sin(total_theta)
    total_theta += dt
    if total_theta >= 2 * math.pi:
        total_theta -= 2 * math.pi
    elif total_theta < 0:
        total_theta += 2*math.pi
    print("Total [X: " + str(total_x) + ", Y: " + str(total_y) + ", Theta: " + str(total_theta) + "]")
    # odom_quat = tf.transformations.quaternion_from_euler(0, 0, dt)
    odom_tf_quat = tf.transformations.quaternion_from_euler(0, 0, total_theta)
    odom_broadcaster.sendTransform(
        (total_x, total_y, 0.),
        odom_tf_quat,
        current_time,
        "base_link",
        "odom"
    )
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(total_x, total_y, 0.), Quaternion(*odom_tf_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(dx, dy, 0), Vector3(0, 0, dt))
    odom_pub.publish(odom)
    last_proc_time = current_time


if __name__ == '__main__':
    try:
        loop = setup_listeners()
        loop.run_forever()
    except KeyboardInterrupt:
        print("Keyboard Interrupt, exiting")
        exit()
