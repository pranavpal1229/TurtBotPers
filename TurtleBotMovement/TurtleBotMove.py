#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

def play_confirm_sound(sound_pub):
    """
    Play the TurtleBot3 confirm sound (sound ID 4).
    """
    sound_pub.publish(UInt8(data=4))
    rospy.sleep(1)  # Give it time to play

def move(pub, linear_x=0.0, angular_z=0.0, duration=2.0):
    """
    Send a Twist command to move the robot.
    """
    cmd = Twist()
    cmd.linear.x = linear_x
    cmd.angular.z = angular_z

    rate = rospy.Rate(10)
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(duration):
        pub.publish(cmd)
        rate.sleep()

    # Stop
    pub.publish(Twist())
    rospy.sleep(1)

def motion_sequence():
    rospy.init_node('motion_sequence_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sound_pub = rospy.Publisher('/sound', UInt8, queue_size=10)
    rospy.sleep(1)  # Let publishers connect

    rospy.loginfo("Spinning...")
    play_confirm_sound(sound_pub)
    move(pub, linear_x=0.0, angular_z=0.8, duration=2.0)

    rospy.loginfo("Moving forward...")
    play_confirm_sound(sound_pub)
    move(pub, linear_x=0.2, angular_z=0.0, duration=2.0)

    rospy.loginfo("Bobbing...")
    play_confirm_sound(sound_pub)
    for _ in range(3):
        move(pub, linear_x=0.2, angular_z=0.0, duration=0.3)
        move(pub, linear_x=-0.2, angular_z=0.0, duration=0.3)

    rospy.loginfo("Done.")

if __name__ == '__main__':
    try:
        motion_sequence()
    except rospy.ROSInterruptException:
        pass
