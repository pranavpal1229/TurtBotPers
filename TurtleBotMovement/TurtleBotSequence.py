#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

def move(pub, linear_x=0.0, angular_z=0.0, duration=2.0):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(duration):
        pub.publish(twist)
        rate.sleep()

    # Stop motion
    pub.publish(Twist())
    rospy.sleep(1)

def play_confirm_sound(pub):
    pub.publish(UInt8(data=4))  # Confirm sound
    rospy.sleep(1)

def main():
    rospy.init_node('simple_turtlebot_sequence')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sound_pub = rospy.Publisher('/sound', UInt8, queue_size=10)
    rospy.sleep(1)  # Wait for publishers to connect

    rospy.loginfo("Moving forward...")
    move(cmd_pub, linear_x=0.2, angular_z=0.0, duration=2.0)

    rospy.loginfo("Turning 90 degrees...")
    move(cmd_pub, linear_x=0.0, angular_z=0.785, duration=2.0)

    # 3. Move forward again for 2 seconds
    rospy.loginfo("Moving forward again...")
    move(cmd_pub, linear_x=0.2, angular_z=0.0, duration=2.0)

    # 4. Play confirm sound
    rospy.loginfo("Playing confirm sound.")
    play_confirm_sound(sound_pub)

    rospy.loginfo("Sequence complete.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
