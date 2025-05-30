#!/usr/bin/env python3

import rospy
import moveit_commander
import math
import time

def main():
    rospy.init_node("nod_hi_node", anonymous=True)
    moveit_commander.roscpp_initialize([])
    rospy.sleep(2)

    arm = moveit_commander.MoveGroupCommander("arm")

    # Set max speed
    arm.set_max_velocity_scaling_factor(1.0)
    arm.set_max_acceleration_scaling_factor(1.0)

    rospy.loginfo("Starting nod motion")

    # Define nodding poses
    arm_up = [0, math.radians(-50), 0, 0]     # neutral up
    nod_down = [0, math.radians(-30), 0, 0]   # nod up (slightly raised)
    nod_up = [0, math.radians(-60), 0, 0]     # nod down (deeper bend)

    # Move to arm-up position
    rospy.loginfo("Arm to neutral pose...")
    arm.go(arm_up, wait=True)
    arm.stop()
    rospy.sleep(0.5)

    # Perform nodding motion
    for i in range(12):
        rospy.loginfo(f"Nod down {i+1}")
        arm.go(nod_up, wait=True)
        arm.stop()
        rospy.sleep(0.2)

        rospy.loginfo(f"Nod up {i+1}")
        arm.go(nod_down, wait=True)
        arm.stop()
        rospy.sleep(0.2)

    # Return to neutral position
    rospy.loginfo("Back to neutral pose...")
    arm.go(arm_up, wait=True)
    arm.stop()

    rospy.loginfo("Nodding sequence complete.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()