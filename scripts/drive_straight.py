#!/usr/bin/env python3
"""
Helper file - issues a cmd_vel to drive at a
fixed velocity for a specified duration, then stop.

Useful for odometry testing / calibration
"""
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

duration=5.0
forward_vel=0.5
angle_vel=0.0

def drive():
    global forward_vel, angle_vel

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('drive_forward', anonymous=True)
    rate = rospy.Rate(25)
    start_time = rospy.Time.now()
    elapsed_time = 0.0
    rospy.loginfo("Driving. . .")
    while not rospy.is_shutdown() and elapsed_time < duration:

        vel_msg = Twist()
        vel_msg.linear.x = forward_vel
        vel_msg.angular.z = angle_vel

        pub.publish(vel_msg)
        rate.sleep()
        current_time=rospy.Time.now()
        elapsed_time = current_time.to_sec() - start_time.to_sec()
    rospy.loginfo("Done.")
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)


if __name__ == '__main__':
    try:
        drive()
    except rospy.ROSInterruptException:
        pass
