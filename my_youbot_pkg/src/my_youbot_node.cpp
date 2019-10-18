#include <ros/ros.h>
#include <luh_youbot_controller_api/controller_api.h>

using namespace youbot_api;

int main(int argc, char** argv)
{
    // initialise ROS
    ros::init(argc, argv, "move_arm");
    ros::NodeHandle node_handle;

    // Initialise arm
    YoubotArm arm;
    arm.init(node_handle);

    // Move arm to ARM_UP pose
    arm.moveToPose("ARM_UP");
    arm.waitForCurrentAction();

    // Move arm back to HOME pose
    arm.moveToPose("DUMP_4_ABOVE");
    arm.waitForCurrentAction();

    return 0;
}