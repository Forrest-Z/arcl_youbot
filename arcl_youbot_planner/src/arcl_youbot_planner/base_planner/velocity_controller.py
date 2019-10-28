import rospy
from pid import PID
from geometry_msgs.msg import Twist
from math import cos, sin, pi
from base_util import get_youbot_base_pose2d
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

class VelocityController(object):
    """ Send velocity command to follow a path based on current youbot position. """

    def __init__(self, youbot_name):
        # name of youbot
        self.youbot_name = youbot_name
        # path to follow
        self.path = []
        # current step of path
        self.step = 0
        # current position of youbot: (x, y, theta)
        self.current_pos = (0, 0, 0)
        # x: linear.x; y: linear.y; theta: angular.z
        self.velocity = Twist()

        # PID
        velocity_p_factor_x = rospy.get_param("module_base_controller/velocity_p_factor_x", 2)
        velocity_p_factor_y = rospy.get_param("module_base_controller/velocity_p_factor_y", 2)
        velocity_p_factor_theta = rospy.get_param("module_base_controller/velocity_p_factor_theta", 1.5)
        velocity_i_factor_x = rospy.get_param("module_base_controller/velocity_i_factor_x", 0.0)
        velocity_i_factor_y = rospy.get_param("module_base_controller/velocity_i_factor_y", 0.0)
        velocity_i_factor_theta = rospy.get_param("module_base_controller/velocity_i_factor_theta", 0.0)
        velocity_d_factor_x = rospy.get_param("module_base_controller/velocity_d_factor_x", 0.1)
        velocity_d_factor_y = rospy.get_param("module_base_controller/velocity_d_factor_y", 0.1)
        velocity_d_factor_theta = rospy.get_param("module_base_controller/velocity_d_factor_theta", 0.01)
        max_velocity_x = rospy.get_param("module_base_controller/max_velocity_x", 0.8)
        max_velocity_y = rospy.get_param("module_base_controller/max_velocity_y", 0.8)
        max_velocity_theta = rospy.get_param("module_base_controller/max_velocity_theta", 1.2)
        self.x_pid = PID(Kp=velocity_p_factor_x, Ki=velocity_i_factor_x, Kd=velocity_d_factor_x, output_limits=(-max_velocity_x, max_velocity_x))
        self.y_pid = PID(Kp=velocity_p_factor_y, Ki=velocity_i_factor_y, Kd=velocity_d_factor_y, output_limits=(-max_velocity_y, max_velocity_y))
        self.theta_pid = PID(Kp=velocity_p_factor_theta, Ki=velocity_i_factor_theta, Kd=velocity_d_factor_theta, output_limits=(-max_velocity_theta, max_velocity_theta))

        # Parameters for smoothing velocity
        step_reached_threshold_x = rospy.get_param("module_base_controller/step_reached_threshold_x", 0.15)
        step_reached_threshold_y = rospy.get_param("module_base_controller/step_reached_threshold_y", 0.15)
        step_reached_threshold_theta = rospy.get_param("module_base_controller/step_reached_threshold_theta", 0.174533)
        self.step_reached_threshold = (step_reached_threshold_x, step_reached_threshold_y, step_reached_threshold_theta)
        goal_reached_threshold_x = rospy.get_param("module_base_controller/goal_reached_threshold_x", 0.01)
        goal_reached_threshold_y = rospy.get_param("module_base_controller/goal_reached_threshold_y", 0.01)
        goal_reached_threshold_theta = rospy.get_param("module_base_controller/goal_reached_threshold_theta", 0.01)
        self.goal_reached_threshold = (goal_reached_threshold_x, goal_reached_threshold_y, goal_reached_threshold_theta)
        self.use_goal_reached_threshold = False

    def set_path(self, path):
        """ Have a new path to follow. Reset everything """
        self.path = path
        self.step = 0
        self.x_pid.reset()
        self.y_pid.reset()
        self.theta_pid.reset()
        self.velocity.linear.x = 0
        self.velocity.linear.y = 0
        self.velocity.angular.z = 0
        self.use_goal_reached_threshold = False

    def get_velocity(self):
        """ Output velocity based the current position and next target position from the path """
        if self.step < len(self.path):
            self.current_pos = get_youbot_base_pose2d(self.youbot_name)
            current_step_pos = self.path[self.step]
            diff_pos = self.compute_difference(current_step_pos, self.current_pos)
            current_step_reached = self.is_current_step_reached(diff_pos)
            # if current position is close enough to next target position
            if current_step_reached:   
                self.step += 1
                if self.step < (len(self.path) - 1):             
                    current_step_pos = self.path[self.step]
                    diff_pos = self.compute_difference(current_step_pos, self.current_pos)
                else:
                    self.use_goal_reached_threshold = True

            velocity = self.compute_velocity(diff_pos)
            self.velocity.linear.x = velocity[0]
            self.velocity.linear.y = velocity[1]
            self.velocity.angular.z = velocity[2]

            print("-----" + str(self.step) + "-----")
            print(self.current_pos)
            print(current_step_pos)
            print(velocity)
        else:
            self.velocity.linear.x = 0
            self.velocity.linear.y = 0
            self.velocity.angular.z = 0

        return self.velocity
    
    def is_current_step_reached(self, diff_pos):
        if self.use_goal_reached_threshold:
            if (abs(diff_pos[0]) < self.goal_reached_threshold[0] 
                    and abs(diff_pos[1]) < self.goal_reached_threshold[1] 
                    and abs(diff_pos[2]) < self.goal_reached_threshold[2]):
                return True
            else:
                return False
        else:    
            if (abs(diff_pos[0]) < self.step_reached_threshold[0] 
                    and abs(diff_pos[1]) < self.step_reached_threshold[1] 
                    and abs(diff_pos[2]) < self.step_reached_threshold[2]):
                return True
            else:
                return False

    def compute_difference(self, current_step_pos, current_pos):
        """ get differences of x, y, and theta between two positions """
        diff_x = ((current_step_pos[0] - current_pos[0]) * cos(current_pos[2]) 
                + (current_step_pos[1] - current_pos[1]) * sin(current_pos[2]))
        diff_y = (-(current_step_pos[0] - current_pos[0]) * sin(current_pos[2]) 
				+ (current_step_pos[1] - current_pos[1]) * cos(current_pos[2]))
        diff_theta = current_step_pos[2] - current_pos[2]
        diff_theta = self.adjust_diff_theta(diff_theta)
        return (diff_x, diff_y, diff_theta)

    def adjust_diff_theta(self, theta):
        """ theta should be in range [-pi, pi]"""
        if theta > pi:
            theta -= 2*pi
        elif theta < -pi:
            theta += 2*pi
        return theta

    def compute_velocity(self, diff_pos):
        return (self.x_pid(diff_pos[0]), self.y_pid(diff_pos[1]), self.theta_pid(diff_pos[2]))
