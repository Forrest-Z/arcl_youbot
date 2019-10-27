#ifndef MODULE_EXAMPLE_H
#define MODULE_EXAMPLE_H

#include "luh_youbot_controller/module_base_class/controller_module.h"
#include "arcl_youbot_msgs/CartesianVector.h"
#include "arcl_youbot_msgs/JointVector.h"

class ModuleExample : public ControllerModule
{
public:
    ModuleExample();

protected:

    void init();
    void activate();
    void deactivate();
    void update();

    ros::Subscriber cartesian_pose_subscriber_;
    ros::Publisher joint_pose_publisher_;

    bool activated_;

    arcl_youbot_kinematics::CartesianPosition cartesian_pose_;

    void cartesianPoseCallback(const arcl_youbot_msgs::CartesianVector::ConstPtr &cartesian_pose_msg);

};

#endif // MODULE_EXAMPLE_H
