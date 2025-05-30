#!/usr/bin/env python3

import rospy
import moveit_commander
import math
import time

def main():
    rospy.init_node("wave_hi_node", anonymous=True)
    moveit_commander.roscpp_initialize([])

    rospy.sleep(2)
    arm = moveit_commander.MoveGroupCommander("arm")

    # Set max speed
    arm.set_max_velocity_scaling_factor(1.0)
    arm.set_max_acceleration_scaling_factor(1.0)

    rospy.loginfo("Starting fast wave")

    # Joint angles
    arm_up = [0, math.radians(-50), 0, 0]
    wave_left = [math.radians(10), math.radians(-50), 0, 0]
    wave_right = [math.radians(-10), math.radians(-50), 0, 0]

    # Move to starting pose
    rospy.loginfo("Arm up...")
    arm.go(arm_up, wait=True)
    arm.stop()
    rospy.sleep(0.5)

    # Wave motion
    for i in range(3):
        rospy.loginfo(f"Wave left {i+1}")
        arm.go(wave_left, wait=True)
        arm.stop()
        rospy.sleep(0.05)

        rospy.loginfo(f"Wave right {i+1}")
        arm.go(wave_right, wait=True)
        arm.stop()
        rospy.sleep(0.05)

    # Return to up position
    rospy.loginfo("Arm back to up...")
    arm.go(arm_up, wait=True)
    arm.stop()

    rospy.loginfo("Fast wave complete!")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()