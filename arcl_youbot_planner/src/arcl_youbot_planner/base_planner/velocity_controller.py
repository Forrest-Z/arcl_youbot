import rospy
from pid import PID
from geometry_msgs.msg import Twist
from math import cos, sin, pi, sqrt
# from base_util import get_youbot_base_pose2d
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

ALPHA = 0.2

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
        # Exponential Moving Average
        # self.smooth_velocity = [0, 0, 0]

        # PID
        velocity_p_factor_x = rospy.get_param("/module_base_controller/velocity_p_factor_x", 2)
        velocity_p_factor_y = rospy.get_param("/module_base_controller/velocity_p_factor_y", 2)
        velocity_p_factor_theta = rospy.get_param("/module_base_controller/velocity_p_factor_theta", 1.5)
        velocity_i_factor_x = rospy.get_param("/module_base_controller/velocity_i_factor_x", 0.0)
        velocity_i_factor_y = rospy.get_param("/module_base_controller/velocity_i_factor_y", 0.0)
        velocity_i_factor_theta = rospy.get_param("/module_base_controller/velocity_i_factor_theta", 0.0)
        velocity_d_factor_x = rospy.get_param("/module_base_controller/velocity_d_factor_x", 0.1)
        velocity_d_factor_y = rospy.get_param("/module_base_controller/velocity_d_factor_y", 0.1)
        velocity_d_factor_theta = rospy.get_param("/module_base_controller/velocity_d_factor_theta", 0.01)
        velocity_p_factor_x_near = rospy.get_param("/module_base_controller/velocity_p_factor_x_near", 1)
        velocity_p_factor_y_near = rospy.get_param("/module_base_controller/velocity_p_factor_y_near", 1)
        velocity_p_factor_theta_near = rospy.get_param("/module_base_controller/velocity_p_factor_theta_near", 1.5)
        velocity_i_factor_x_near = rospy.get_param("/module_base_controller/velocity_i_factor_x_near", 0.0)
        velocity_i_factor_y_near = rospy.get_param("/module_base_controller/velocity_i_factor_y_near", 0.0)
        velocity_i_factor_theta_near = rospy.get_param("/module_base_controller/velocity_i_factor_theta_near", 0.0)
        velocity_d_factor_x_near = rospy.get_param("/module_base_controller/velocity_d_factor_x_near", 0.1)
        velocity_d_factor_y_near = rospy.get_param("/module_base_controller/velocity_d_factor_y_near", 0.1)
        velocity_d_factor_theta_near = rospy.get_param("/module_base_controller/velocity_d_factor_theta_near", 0.01)
        max_velocity_x = rospy.get_param("/module_base_controller/max_velocity_x", 0.3)
        max_velocity_y = rospy.get_param("/module_base_controller/max_velocity_y", 0.3)
        max_velocity_theta = rospy.get_param("/module_base_controller/max_velocity_theta", 0.4)
        max_velocity_x_approach = rospy.get_param("/module_base_controller/max_velocity_x_approach", 0.3)
        max_velocity_y_approach = rospy.get_param("/module_base_controller/max_velocity_y_approach", 0.3)
        max_velocity_theta_approach = rospy.get_param("/module_base_controller/max_velocity_theta_approach", 0.4)
        self.x_pid = PID(Kp=velocity_p_factor_x, Ki=velocity_i_factor_x, Kd=velocity_d_factor_x, output_limits=(-max_velocity_x, max_velocity_x))
        self.y_pid = PID(Kp=velocity_p_factor_y, Ki=velocity_i_factor_y, Kd=velocity_d_factor_y, output_limits=(-max_velocity_y, max_velocity_y))
        self.theta_pid = PID(Kp=velocity_p_factor_theta, Ki=velocity_i_factor_theta, Kd=velocity_d_factor_theta, output_limits=(-max_velocity_theta, max_velocity_theta))
        self.x_pid_near = PID(Kp=velocity_p_factor_x_near, Ki=velocity_i_factor_x_near, Kd=velocity_d_factor_x_near, output_limits=(-max_velocity_x_approach, max_velocity_x_approach))
        self.y_pid_near = PID(Kp=velocity_p_factor_y_near, Ki=velocity_i_factor_y_near, Kd=velocity_d_factor_y_near, output_limits=(-max_velocity_y_approach, max_velocity_y_approach))
        self.theta_pid_near = PID(Kp=velocity_p_factor_theta_near, Ki=velocity_i_factor_theta_near, Kd=velocity_d_factor_theta_near, output_limits=(-max_velocity_theta_approach, max_velocity_theta_approach))
   
        # Parameters for smoothing velocity
        step_reached_threshold_x = rospy.get_param("/module_base_controller/step_reached_threshold_x", 0.15)
        step_reached_threshold_y = rospy.get_param("/module_base_controller/step_reached_threshold_y", 0.15)
        step_reached_threshold_theta = rospy.get_param("/module_base_controller/step_reached_threshold_theta", 0.174533)
        self.step_reached_threshold = (step_reached_threshold_x, step_reached_threshold_y, step_reached_threshold_theta)
        goal_reached_threshold_x = rospy.get_param("/module_base_controller/goal_reached_threshold_x", 0.01)
        goal_reached_threshold_y = rospy.get_param("/module_base_controller/goal_reached_threshold_y", 0.01)
        goal_reached_threshold_theta = rospy.get_param("/module_base_controller/goal_reached_threshold_theta", 0.01)
        self.goal_reached_threshold = (goal_reached_threshold_x, goal_reached_threshold_y, goal_reached_threshold_theta)
        goal_near_threshold_x = rospy.get_param("/module_base_controller/goal_near_threshold_x", 0.01)
        goal_near_threshold_y = rospy.get_param("/module_base_controller/goal_near_threshold_y", 0.01)
        goal_near_threshold_theta = rospy.get_param("/module_base_controller/goal_near_threshold_theta", 0.01)
        self.goal_near_threshold = (goal_near_threshold_x, goal_near_threshold_y, goal_near_threshold_theta)
        self.use_goal_reached_threshold = False

    def set_path(self, path):
        """ Have a new path to follow. Reset everything """
        self.path = path
        self.step = 0
        self.x_pid.reset()
        self.y_pid.reset()
        self.theta_pid.reset()
        self.x_pid_near.reset()
        self.y_pid_near.reset()
        self.theta_pid_near.reset()

        self.velocity = Twist()
        if len(self.path) == 2:
            self.use_goal_reached_threshold = True
        else:
            self.use_goal_reached_threshold = False
        # self.smooth_velocity = [0, 0, 0]

    def get_velocity(self, current_pos):
        """ Output velocity based the current position and next target position from the path """
        if self.step < len(self.path):
            self.current_pos = current_pos
            current_step_pos = self.path[self.step]
            diff_pos = self.compute_difference(current_step_pos, self.current_pos)
            current_step_reached = self.is_current_step_reached(diff_pos)
            goal_near = self.is_goal_near(diff_pos)
            # if current position is close enough to next target position
            if current_step_reached:   
                self.step += 1
                if self.step < (len(self.path) - 1):             
                    current_step_pos = self.path[self.step]
                    diff_pos = self.compute_difference(current_step_pos, self.current_pos)
                else:
                    self.use_goal_reached_threshold = True

            if self.use_goal_reached_threshold and goal_near:
                velocity = self.compute_near_velocity(diff_pos)
            else:
                velocity = self.compute_velocity(diff_pos)

            # # smoothing
            # self.smooth_velocity[0] = (1.0 - ALPHA) * self.smooth_velocity[0] + ALPHA * velocity[0]
            # self.smooth_velocity[1] = (1.0 - ALPHA) * self.smooth_velocity[1] + ALPHA * velocity[1]
            # self.smooth_velocity[2] = (1.0 - ALPHA) * self.smooth_velocity[2] + ALPHA * velocity[2]
            # velocity = list(self.smooth_velocity)
            
            
            # normalization, because we limited velocity
            self.normalization(velocity, diff_pos, self.x_pid.output_limits[1], self.y_pid.output_limits[1])
            # if velocity[0] != 0 and velocity[1] != 0 and diff_pos[0] != 0 and diff_pos[1] != 0:
            #     if abs(velocity[0] / velocity[1] - diff_pos[0] / diff_pos[1]) > 0.01:
            #         if abs(velocity[0]) == self.x_pid.output_limits[1] and abs(velocity[1]) == self.y_pid.output_limits[1]:
            #             if abs(diff_pos[0]) > abs(diff_pos[1]):
            #                 velocity[1] = velocity[0] / diff_pos[0] * diff_pos[1]
            #             else:
            #                 velocity[0] = diff_pos[0] / diff_pos[1] * velocity[1]
            #         elif abs(velocity[0]) == self.x_pid.output_limits[1]:
            #             velocity[1] = velocity[0] / diff_pos[0] * diff_pos[1]
            #         elif abs(velocity[1]) == self.y_pid.output_limits[1]:
            #             velocity[0] = diff_pos[0] / diff_pos[1] * velocity[1]
            # if abs(velocity[0]) > self.x_pid.output_limits[1]:
            #     if velocity[0] > 0:
            #         velocity[1] = velocity[1] * self.x_pid.output_limits[1] / velocity[0]
            #         velocity[0] = self.x_pid.output_limits[1]
            #     else:
            #         velocity[1] = velocity[1] * self.x_pid.output_limits[0] / velocity[0]
            #         velocity[0] = self.x_pid.output_limits[0]
            # if abs(velocity[1]) > self.y_pid.output_limits[1]:
            #     if velocity[1] > 0:
            #         velocity[0] = velocity[0] * self.y_pid.output_limits[1] / velocity[1]
            #         velocity[1] = self.y_pid.output_limits[1]
            #     else:
            #         velocity[0] = velocity[0] * self.y_pid.output_limits[0] / velocity[1]
            #         velocity[1] = self.y_pid.output_limits[0]



            # velocity[0] = 0
            # velocity[2] = 0
            # print("-----global vel:" + str(self.step) + "-----")
            # print([velocity[0], velocity[1], velocity[2]])
            # print("global yaw:" + str(self.current_pos[2]))
            # if mode == 1:
            #     xt = velocity[0] * cos(self.current_pos[2]) - velocity[1] * sin(self.current_pos[2])
            #     yt = velocity[1] * cos(self.current_pos[2]) + velocity[0] * sin(self.current_pos[2])
            # if mode == 1:    
            #     xt = velocity[0]
            #     yt = velocity[1]
            #     velocity[0] = yt
            #     velocity[1] = 0-xt

            self.velocity.linear.x = velocity[0]
            self.velocity.linear.y = velocity[1]
            self.velocity.angular.z = velocity[2]
            
            # print("-----local vel:" + str(self.step) + " / " + str(len(self.path)) + "-----")
            # print(self.current_pos)
            # print(current_step_pos)
            # print([self.velocity.linear.x, self.velocity.linear.y, self.velocity.angular.z])
        else:
            self.velocity.linear.x = 0
            self.velocity.linear.y = 0
            self.velocity.angular.z = 0

        return self.velocity

    def normalization(self, velocity, diff_pos, x_max_velocity, y_max_velocity):
        if velocity[0] != 0 and velocity[1] != 0 and diff_pos[0] != 0 and diff_pos[1] != 0:
            if abs(velocity[0] / velocity[1] - diff_pos[0] / diff_pos[1]) > 0.01:
                if abs(velocity[0]) == x_max_velocity and abs(velocity[1]) == y_max_velocity:
                    if abs(diff_pos[0]) > abs(diff_pos[1]):
                        velocity[1] = velocity[0] / diff_pos[0] * diff_pos[1]
                    else:
                        velocity[0] = diff_pos[0] / diff_pos[1] * velocity[1]
                elif abs(velocity[0]) == x_max_velocity:
                    velocity[1] = velocity[0] / diff_pos[0] * diff_pos[1]
                elif abs(velocity[1]) == y_max_velocity:
                    velocity[0] = diff_pos[0] / diff_pos[1] * velocity[1]
        if abs(velocity[0]) > x_max_velocity:
            velocity[1] = velocity[1] * x_max_velocity / velocity[0]
            velocity[0] = x_max_velocity if velocity[0] > 0 else -x_max_velocity
        if abs(velocity[1]) > y_max_velocity:
            velocity[0] = velocity[0] * y_max_velocity / velocity[1]
            velocity[1] = y_max_velocity if velocity[1] > 0 else -y_max_velocity 
    
    def is_goal_near(self, diff_pos):
        if (abs(diff_pos[0]) < self.goal_near_threshold[0] 
                and abs(diff_pos[1]) < self.goal_near_threshold[1] 
                and abs(diff_pos[2]) < self.goal_near_threshold[2]):
            return True
        else:
            return False

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
        return [self.x_pid(diff_pos[0]), self.y_pid(diff_pos[1]), self.theta_pid(diff_pos[2])]

    def compute_near_velocity(self, diff_pos):
        return [self.x_pid_near(diff_pos[0]), self.y_pid_near(diff_pos[1]), self.theta_pid_near(diff_pos[2])]

