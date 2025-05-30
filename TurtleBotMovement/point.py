#!/usr/bin/env python3

import rospy
import moveit_commander
import math
import time

def main():
    rospy.init_node("point_left_right_node", anonymous=True)
    moveit_commander.roscpp_initialize([])
    rospy.sleep(2)

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(1.0)
    arm.set_max_acceleration_scaling_factor(1.0)

    rospy.loginfo("Starting point gestures")

    default_pose = [0.0, math.radians(5.0), math.radians(20.0), math.radians(-20.0)]
    # Joint angles for different directions
    point_center = [0, math.radians(-30), math.radians(45), math.radians(-15)]
    point_left = [math.radians(-30), math.radians(-60), math.radians(45), math.radians(-15)]
    point_right = [math.radians(30), math.radians(-30), math.radians(45), math.radians(-15)]
    arm.go(default_pose, wait=True)
    arm.stop()
    rospy.sleep(1)
    # Move to center first
    rospy.loginfo("Pointing forward...")
    arm.go(point_center, wait=True)
    arm.stop()
    rospy.sleep(1)

    # Point left
    #rospy.loginfo("Pointing left...")
    #arm.go(point_left, wait=True)
    #arm.stop()
    #rospy.sleep(2)

    # Point right
    rospy.loginfo("Pointing right...")
    rospy.loginfo(f"Joint state before move {arm.get_current_joint_values()}")
    arm.go(point_right, wait=True)
    rospy.loginfo(f"Joint state after move {arm.get_current_joint_values()}")
    arm.go(point_right, wait=True)
    arm.stop()
    rospy.sleep(2)

    # Back to center
    rospy.loginfo("Returning to center...")
    arm.go(point_center, wait=True)
    arm.stop()

    rospy.loginfo("Point gesture sequence complete.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()