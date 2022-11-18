#!/usr/bin/env python3

import rospy
import math

# Define Constants
WHEEL_GEAR_TEETH = 30
MOTOR_GEAR_TEETH = 18

MOTOR_BYTES_PER_HZ = 10
MOTOR_POLES = 4

WHEEL_SPEED_CORRECTION = 1.25

GEAR_RATIO = WHEEL_GEAR_TEETH/MOTOR_GEAR_TEETH
WHEEL_DIAMETER = 0.083 # meters
WHEEL_RADIUS = WHEEL_DIAMETER/2
#wheel_base = 0.38 # meters
WHEEL_BASE = 0.37 # meters # HACK

def bytes_to_velocity(speed_raw):
    print(wheel_name + " Raw Speed: " + str(speed_raw_int))
    speed_hz = speed_raw_int / MOTOR_BYTES_PER_HZ / MOTOR_POLES
    print(wheel_name + " Hz: " + str(speed_hz))
    speed_rad_per_sec = (speed_hz * math.pi * 2) / GEAR_RATIO
    print(wheel_name + " Rad per Sec: " + str(speed_rad_per_sec))
    wheel_linear_velocity = speed_rad_per_sec * WHEEL_RADIUS
    return wheel_linear_velocity

def velocity_to_bytes(linear_vel, angular_vel):
    # Compute wheel velocity in radians / sec

    left_wheel = linear_vel / (WHEEL_RADIUS * math.cos(angular_vel*0.05)) + (angular_vel * WHEEL_BASE) / (2 * WHEEL_RADIUS)
    right_wheel = linear_vel / (WHEEL_RADIUS * math.cos(angular_vel*0.05)) - (angular_vel * WHEEL_BASE) / (2 * WHEEL_RADIUS)

    # Translate into correct motor drive (in hz)
    left_motor = (left_wheel * GEAR_RATIO) / (2 * math.pi)
    right_motor = (right_wheel * GEAR_RATIO) / (2 * math.pi)

    l_motor_raw = int(left_motor * MOTOR_BYTES_PER_HZ * MOTOR_POLES)
    r_motor_raw = int(right_motor * MOTOR_BYTES_PER_HZ * MOTOR_POLES)

    return (l_motor_raw, r_motor_raw)
