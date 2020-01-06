import rospy
from geometry_msgs.msg import TwistStamped, PointStamped, Twist
from arcl_youbot_msgs.msg import PolygonArray
from arcl_youbot_planner.base_planner.base_util import BaseController, YOUBOT_LONG_RADIUS
import rvo2
import os
import sys
import signal
import math
from collections import defaultdict


TIMEOUT = 1

class MultiBaseController():
    """ 
    This is used to publish velocity for Multi-YouBot system
    """

    def __init__(self, youbot_names, max_velocity):
        self.youbot_names = youbot_names
        self.max_velocity = max_velocity
        self.vel_pubs = {}
        self.vels = defaultdict(TwistStamped)
        self.poses = defaultdict(PointStamped)
        self.sim = rvo2.PyRVOSimulator(1/30., 3 * YOUBOT_LONG_RADIUS, 5, 5.0, 5.0, YOUBOT_LONG_RADIUS, self.max_velocity)
        self.agents = {}
        for name in self.youbot_names:
            self.agents[name] = self.sim.addAgent((0, 0))
            
    def publish_vels(self):
        # If we didn't receive velocity or position of youbot over TIMEOUT seconds, then, we stop it
        current_time = rospy.Time.now().secs
        for name in self.youbot_names:
            if current_time - self.vels[name].header.stamp.secs > TIMEOUT or current_time - self.poses[name].header.stamp.secs > TIMEOUT:
                self.sim.setAgentPrefVelocity(self.agents[name], (0, 0))
        
        # step
        self.sim.doStep()

        # publish velocity
        for name in self.youbot_names:
            vel = self.sim.getAgentVelocity(self.agents[name])
            theta = self.poses[name].point.z
            # global to local frame
            msg = Twist()
            msg = self.vels[name].twist
            # if not the last step
            if self.vels[name].header.frame_id != 'final':
                msg.linear.x = vel[0] * math.cos(theta) + vel[1] * math.sin(theta)
                msg.linear.y = vel[0] * -math.sin(theta) + vel[1] * math.cos(theta)
            self.vel_pubs[name].publish(msg)

    def obstacle_callback(self, data):
        for obs in data.polygons:
            poly = [[point.x, point.y] for point in obs.points]
            self.sim.addObstacle(poly)
        self.sim.processObstacles()
        print('Simulation has %i agents and %i obstacle vertices in it.' % (self.sim.getNumAgents(), self.sim.getNumObstacleVertices()))

    def velocity_callback(self, data, name):
        self.vels[name] = data
        theta = self.poses[name].point.z
        # local to global frame
        global_vel = (data.twist.linear.x * math.cos(theta) + data.twist.linear.y * -math.sin(theta), data.twist.linear.x * math.sin(theta) + data.twist.linear.y * math.cos(theta))
        self.sim.setAgentPrefVelocity(self.agents[name], global_vel)   

    def position_callback(self, data, name):
        self.poses[name] = data
        self.sim.setAgentPosition(self.agents[name], (data.point.x, data.point.y))
        
def ctrl_c_handler(signal, frame):
    """Gracefully quit the infrared_pub node"""
    print("\nCaught ctrl-c! Stopping node.")
    sys.exit()

def main():
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    youbot_names = ['youbot_0', 'youbot_1', 'youbot_2']
    max_velocity = rospy.get_param("/module_base_controller/max_velocity_x", 0.3)

    mbc = MultiBaseController(youbot_names, max_velocity)
    for name in youbot_names:
        mbc.vel_pubs[name] = rospy.Publisher('/' + name + '/robot/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/' + name + '/robot/temp_vel', TwistStamped, mbc.velocity_callback, name)
        rospy.Subscriber('/' + name + '/robot/pose', PointStamped, mbc.position_callback, name)
    rospy.Subscriber('/rvo2/obstacles', PolygonArray, mbc.obstacle_callback)
        
    signal.signal(signal.SIGINT, ctrl_c_handler)
    loop_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        mbc.publish_vels()
        loop_rate.sleep()

if __name__ == "__main__":
    main()
