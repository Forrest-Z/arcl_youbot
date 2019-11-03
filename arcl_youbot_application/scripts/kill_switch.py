#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty

if __name__ == "__main__":
    rospy.init_node('kill_switch')
    robot_list = ["youbot_0", "youbot_1", "youbot_2"]

    kill_pub_0 = rospy.Publisher(robot_list[0] + '/yobot/stop', Empty, queue_size=1)
    kill_pub_1 = rospy.Publisher(robot_list[1] + '/yobot/stop', Empty, queue_size=1)
    kill_pub_2 = rospy.Publisher(robot_list[2] + '/yobot/stop', Empty, queue_size=1)
    msg = Empty()
    kill_pub_0.publish(msg)
    kill_pub_1.publish(msg)
    kill_pub_2.publish(msg)
    rospy.spin()
