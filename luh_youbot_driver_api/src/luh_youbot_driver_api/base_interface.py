import rospy
import math
import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
from tf.transformations import euler_from_quaternion

class baseInterface():
    velocity_command_ = geometry_msgs.msg.Twist()
    
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.init()
        self.odom_msg_ = nav_msgs.msg.Odometry()
        self.velocity_command_ = geometry_msgs.msg.Twist()
        self.velocity_command_.linear.x = 0 
        self.velocity_command_.linear.y = 0       
        self.velocity_command_.linear.z = 0       
        self.velocity_command_.angular.x = 0
        self.velocity_command_.angular.y = 0
        self.velocity_command_.angular.z = 0
    
    def odom_callback(data):
        self.odom_msg_ = data

    def init(): 
        self.cmd_vel_pub_ = rospy.Publisher("robot/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
        rospy.Subscriber("odom_" + str(self.robot_id), nav_msgs.msg.Odometry, odom_callback)

    def read_state():
        self.pose2d = geometry_msgs.msg.Pose2D()
        self.pose2d.x = self.odom_msg_.pose.pose.position.x
        self.pose2d.y = self.odom_msg_.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion(self.odom_msg_.pose.pose.orientation)
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < - math.pi:
            yaw += 2 * math.pi
        self.pose2d.theta = yaw
        
    def write_command():
        self.cmd_vel_pub_.publish(velocity_command_)

    def set_velocity(vel):
        velocity_command_ = vel

