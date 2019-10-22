#include "arcl_youbot_application/utilities.hpp"

#include <boost/geometry/io/dsv/write.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>

namespace arc
{

/**
 * Rotate the given polygon and then translate it 
 */
void movePolygon(const polygon_2& poly, polygon_2& outPoly, 
    double x, double y, double theta)
{
    // std::cout << boost::geometry::dsv(poly) << std::endl;
    // std::cout << "Outer has " << poly.outer().size() << " points." <<  std::endl;
    for(auto it = (poly.outer()).begin();
        it != poly.outer().end(); it++)
    {
        double px = bg::get<0>(*it);
        double py = bg::get<1>(*it);

        // Rotate
        double rpx = px*cos(theta) - py*sin(theta);
        double rpy = px*sin(theta) + py*cos(theta);

        // Translate and add to polygon
        outPoly.outer().push_back(point_2(rpx + x, rpy + y));
    }
    // bg::correct(outPoly);
}


tf::Quaternion getChangedPoseFromYaw(double yaw){
    double x_axis_x, x_axis_y, y_axis_x, y_axis_y;
    x_axis_x = cos(yaw);
    x_axis_y = sin(yaw);
    y_axis_x = -sin(yaw); 
    y_axis_y = cos(yaw);
    tf::Matrix3x3 m;
    //m.setValue(x_axis_x, y_axis_x, 0, x_axis_y, y_axis_y, 0, 0, 0, 1);
    m.setValue(0, y_axis_x, -x_axis_x, 0 , y_axis_y, -x_axis_y, 1, 0, 0);
    
    tf::Quaternion q;
    m.getRotation(q);
    if(std::abs(yaw- 0.0)<0.01){
        tf::Quaternion q(0, 0, 0, 1);
        return q;
    }else if(std::abs(yaw- 1.57)<0.01){
        tf::Quaternion q(0, 0, 0.7071068, 0.7071068 );
        return q;
    }else if(std::abs(yaw+ 1.57)<0.01){
        tf::Quaternion q(0, 0, -0.7071068, 0.7071068 );
        return q;
    }
    if(std::abs(q.x()+0.5)<0.01){
        tf::Quaternion q(-0.5, -0.5, -0.5, 0.5);
        return q;
    }

    return q;
}

tf::Quaternion getPoseFromPolygon(polygon_2 poly){
    if(poly.outer().size() == 5){ 
        double long_x, long_y, short_x, short_y;
        double long_seg, short_seg;
        double temp_seg = 0;
        double temp_x, temp_y;
        double yaw = 0;
        long_x = poly.outer()[0].get<0>() - poly.outer()[3].get<0>();
        long_y = poly.outer()[0].get<1>() - poly.outer()[3].get<1>();
        long_seg = std::sqrt(long_x*long_x + long_y*long_y);

        short_x = poly.outer()[0].get<0>() - poly.outer()[1].get<0>();
        short_y = poly.outer()[0].get<1>() - poly.outer()[1].get<1>();
        short_seg = std::sqrt(short_x*short_x + short_y*short_y);

        if(short_seg > long_seg){
            temp_seg = short_seg;
            temp_x = short_x;
            temp_y = short_y;
            short_seg = long_seg;
            short_x = long_x;
            short_y = long_y;
            long_seg = temp_seg;
            long_x = temp_x;
            long_y = temp_y;

        }
        //if(std::abs(long_x - 0.0)< 0.000001){
        //    return 3.1415926/2.0;
        //}else{ 
        ROS_WARN_STREAM("yaw:"<<atan(long_y/long_x));
        yaw = atan(long_y/long_x);
        if(std::abs(yaw- 0.0)<0.01){
        tf::Quaternion q(0, 0, 0, 1);
        return q;
        }else if(std::abs(yaw- 1.57)<0.01){
            tf::Quaternion q(0, 0, 0.7071068, 0.7071068 );
            return q;
        }else if(std::abs(yaw+ 1.57)<0.01){
            tf::Quaternion q(0, 0, -0.7071068, 0.7071068 );
            return q;
        }
        //return atan(long_y/long_x);
        //}
    }else if(poly.outer().size() == 9){
        double long_x, long_y, short_x, short_y, center_x, center_y;
        double long_seg, short_seg;
        double temp_seg = 0;
        double temp_x, temp_y;
        double yaw = 0;

        center_x = (poly.outer()[0].get<0>() + poly.outer()[5].get<0>()) / 2.0;
        center_y = (poly.outer()[0].get<1>() + poly.outer()[5].get<1>()) / 2.0;
        long_x = (poly.outer()[2].get<0>() + poly.outer()[3].get<0>()) / 2.0;
        long_y = (poly.outer()[2].get<1>() + poly.outer()[3].get<1>())  /2.0;
        long_seg = std::sqrt(long_x*long_x + long_y*long_y);

        short_x = long_x - center_x;
        short_y = long_y - center_y;
        short_seg = std::sqrt(short_x*short_x + short_y*short_y);
        yaw = atan(short_y/short_x);

        if(std::abs(short_x - 0.0) < 0.01){
            tf::Quaternion q(0, 0, 0.7071068, 0.7071068 );
            return q;
        }
        if(std::abs(yaw- 0.0)<0.01){
        tf::Quaternion q(0, 0, 0, 1);
        return q;
        }else if(std::abs(yaw- 1.57)<0.01){
            tf::Quaternion q(0, 0, 0.7071068, 0.7071068 );
            return q;
        }else if(std::abs(yaw+ 1.57)<0.01){
            tf::Quaternion q(0, 0, -0.7071068, 0.7071068 );
            return q;
        }
        
    }else if(poly.outer().size() == 7){
        double long_x, long_y, short_x, short_y, center_x, center_y;
        double long_seg, short_seg;
        double temp_seg = 0;
        double temp_x, temp_y;
        double yaw = 0;

        center_x = poly.outer()[5].get<0>() - poly.outer()[0].get<0>();
        center_y = poly.outer()[5].get<1>() - poly.outer()[0].get<1>();

        if(std::abs(center_y - 0.0) < 0.01){
            if(center_x < 0){
                tf::Quaternion q(0, 1, 0, 0);
        return q;
            }else{
                tf::Quaternion q(0, 0, 0, 1);
        return q;
            }
        }else if(std::abs(center_x - 0.0) < 0.01){
tf::Quaternion q(0.7071068, -0.7071068, 0, 0 );
            return q;
        }
     
    }
    

    tf::Quaternion q(0, 0, 0, 1);
        return q;


}


double getYawFromPolygon(polygon_2 poly){
    if(poly.outer().size() == 5){ 
        double long_x, long_y, short_x, short_y;
        double long_seg, short_seg;
        double temp_seg = 0;
        double temp_x, temp_y;
        double yaw = 0;
        long_x = poly.outer()[0].get<0>() - poly.outer()[3].get<0>();
        long_y = poly.outer()[0].get<1>() - poly.outer()[3].get<1>();
        long_seg = std::sqrt(long_x*long_x + long_y*long_y);

        short_x = poly.outer()[0].get<0>() - poly.outer()[1].get<0>();
        short_y = poly.outer()[0].get<1>() - poly.outer()[1].get<1>();
        short_seg = std::sqrt(short_x*short_x + short_y*short_y);

        if(short_seg > long_seg){
            temp_seg = short_seg;
            temp_x = short_x;
            temp_y = short_y;
            short_seg = long_seg;
            short_x = long_x;
            short_y = long_y;
            long_seg = temp_seg;
            long_x = temp_x;
            long_y = temp_y;

        }
        //if(std::abs(long_x - 0.0)< 0.000001){
        //    return 3.1415926/2.0;
        //}else{ 
        ROS_WARN_STREAM("yaw:"<<atan(long_y/long_x));
        return atan(long_y/long_x);
        //}
    }else if(poly.outer().size() == 9){
        double long_x, long_y, short_x, short_y, center_x, center_y;
        double long_seg, short_seg;
        double temp_seg = 0;
        double temp_x, temp_y;
        double yaw = 0;

        center_x = (poly.outer()[0].get<0>() + poly.outer()[5].get<0>()) / 2.0;
        center_y = (poly.outer()[0].get<1>() + poly.outer()[5].get<1>()) / 2.0;
        long_x = (poly.outer()[2].get<0>() + poly.outer()[3].get<0>()) / 2.0;
        long_y = (poly.outer()[2].get<1>() + poly.outer()[3].get<1>())  /2.0;
        long_seg = std::sqrt(long_x*long_x + long_y*long_y);

        short_x = long_x - center_x;
        short_y = long_y - center_y;
        short_seg = std::sqrt(short_x*short_x + short_y*short_y);

        return atan(short_y/short_x);
    }else if(poly.outer().size() == 7){
        double long_x, long_y, short_x, short_y, center_x, center_y;
        double long_seg, short_seg;
        double temp_seg = 0;
        double temp_x, temp_y;
        double yaw = 0;

        center_x = poly.outer()[5].get<0>() - poly.outer()[0].get<0>();
        center_y = poly.outer()[5].get<1>() - poly.outer()[0].get<1>();

        return atan(center_y/center_x);
    }
    

    return 0;


}

geometry_msgs::Pose assignGeometryPose(double x, double y, double z, double q_x, double q_y, double q_z, double q_w)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = q_x;
    pose.orientation.y = q_y;
    pose.orientation.z = q_z;
    pose.orientation.w = q_w;
    return pose;
}

/**
 * Compute SE2 distance as if Euclidean
 */
double computeSE2Distance(double x_1, double y_1, double t_1, double x_2, double y_2, double t_2)
{
    return sqrt((x_1-x_2)*(x_1-x_2) + (y_1-y_2)*(y_1-y_2) + (t_1-t_2)*(t_1-t_2));
}

double computeSE2Distance(point_se2& p1, point_se2 p2)
{
    return computeSE2Distance(p1.get<0>(), p1.get<1>(), p1.get<2>(), 
        p2.get<0>(), p2.get<1>(), p2.get<2>());
}

geometry_msgs::Pose SE2ToGpose(SE2Pose se2_pose)
{
    geometry_msgs::Pose g_pose;
    tf::Quaternion q = getPoseFromYaw(se2_pose.yaw);
    g_pose.position.x = se2_pose.x;
    g_pose.position.y = se2_pose.y;
    g_pose.position.z = 0;
    g_pose.orientation.x = q.x();
    g_pose.orientation.y = q.y();
    g_pose.orientation.z = q.z();
    g_pose.orientation.w = q.getW();
    
    
    return g_pose;
}

SE2Pose GPoseToSE2(geometry_msgs::Pose pose){
    double target_x = pose.position.x;
    double target_y = pose.position.y;
    tf::Quaternion target_q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double target_yaw = getYawFromPose(target_q);
    SE2Pose se;
    se.x = target_x;
    se.y = target_y;
    se.yaw = target_yaw;
    return se;
}

double getYawFromPose(tf::Matrix3x3 m){
    tf::Vector3 x_axis = m.getColumn(0);
    double base_xx = x_axis.getX() / sqrt( pow(x_axis.getX(), 2) + pow(x_axis.getY(), 2) );
    double base_xy = x_axis.getY() / sqrt( pow(x_axis.getX(), 2) + pow(x_axis.getY(), 2) );

    double yaw;
    yaw = atan2(base_xy, base_xx);

    return yaw;    
}

tf::Quaternion getPoseFromYaw(double yaw){
    double x_axis_x, x_axis_y, y_axis_x, y_axis_y;
    x_axis_x = cos(yaw);
    x_axis_y = sin(yaw);
    y_axis_x = -sin(yaw); 
    y_axis_y = cos(yaw);
    tf::Matrix3x3 m;
    m.setValue(x_axis_x, y_axis_x, 0, x_axis_y, y_axis_y, 0, 0, 0, 1);
    tf::Quaternion q;
    m.getRotation(q);
    return q;
}

double getYawFromPose(tf::Quaternion q){
    tf::Matrix3x3 m(q);
    tf::Vector3 x_axis = m.getColumn(0);
    double base_xx = x_axis.getX() / sqrt( pow(x_axis.getX(), 2) + pow(x_axis.getY(), 2) );
    double base_xy = x_axis.getY() / sqrt( pow(x_axis.getX(), 2) + pow(x_axis.getY(), 2) );

    double yaw;
    yaw = atan2(base_xy, base_xx);

    return yaw;
}

double angleToRotateToTarget(tf::Matrix3x3 target_matrix_, tf::Matrix3x3 current_matrix)
{
    
    tf::Vector3 base_x_axis = current_matrix.getColumn(0);
    tf::Vector3 base_y_axis = current_matrix.getColumn(1);  

    tf::Vector3 target_x_axis = target_matrix_.getColumn(0);
    tf::Vector3 target_y_axis = target_matrix_.getColumn(1);

    double target_matrix_xx = target_x_axis.getX();
    double target_matrix_xy = target_y_axis.getX();
    double target_matrix_yx = target_x_axis.getY();
    double target_matrix_yy = target_y_axis.getY();

    target_matrix_xx = target_matrix_xx / sqrt(pow(target_matrix_xx,2) + pow(target_matrix_yx,2));
    target_matrix_yx = target_matrix_yx / sqrt(pow(target_matrix_xx,2) + pow(target_matrix_yx,2));
    target_matrix_xy = target_matrix_xy / sqrt(pow(target_matrix_xy,2) + pow(target_matrix_yy,2));
    target_matrix_yy = target_matrix_yy / sqrt(pow(target_matrix_xy,2) + pow(target_matrix_yy,2));

    double base_matrix_xx = base_x_axis.getX();
    double base_matrix_xy = base_y_axis.getX();
    double base_matrix_yx = base_x_axis.getY();
    double base_matrix_yy = base_y_axis.getY();
//    ROS_INFO_STREAM("base_x_axis:"<<base_matrix_xx<<","<<base_matrix_yx);
 //   ROS_INFO_STREAM("base_y_axis:"<<base_matrix_xy<<","<<base_matrix_yy);

    base_matrix_xx = base_matrix_xx / sqrt(pow(base_matrix_xx,2) + pow(base_matrix_yx,2));
    base_matrix_yx = base_matrix_yx / sqrt(pow(base_matrix_xx,2) + pow(base_matrix_yx,2));
    base_matrix_xy = base_matrix_xy / sqrt(pow(base_matrix_xy,2) + pow(base_matrix_yy,2));
    base_matrix_yy = base_matrix_yy / sqrt(pow(base_matrix_xy,2) + pow(base_matrix_yy,2));


    double base_to_target_dot, base_to_target_cross, base_to_target_angle;
    base_to_target_dot = base_matrix_xx*target_matrix_xx + base_matrix_yx*target_matrix_yx;
    base_to_target_cross = base_matrix_xx*target_matrix_yx - base_matrix_yx*target_matrix_xx;
    base_to_target_angle = atan2(base_to_target_cross, base_to_target_dot);
    return base_to_target_angle;
}

double angleToRotateToTarget(geometry_msgs::Pose target, tf::Matrix3x3 current_matrix){
    
    tf::Vector3 base_x_axis = current_matrix.getColumn(0);
    tf::Vector3 base_y_axis = current_matrix.getColumn(1);
    tf::Quaternion target_quaternion_;
                
    target_quaternion_.setX(target.orientation.x);  
    target_quaternion_.setY(target.orientation.y);  
    target_quaternion_.setZ(target.orientation.z);  
    target_quaternion_.setW(target.orientation.w);  

    tf::Matrix3x3 target_matrix_(target_quaternion_);
    tf::Vector3 target_x_axis = target_matrix_.getColumn(0);
    tf::Vector3 target_y_axis = target_matrix_.getColumn(1);

    double target_matrix_xx = target_x_axis.getX();
    double target_matrix_xy = target_y_axis.getX();
    double target_matrix_yx = target_x_axis.getY();
    double target_matrix_yy = target_y_axis.getY();

    target_matrix_xx = target_matrix_xx / sqrt(pow(target_matrix_xx,2) + pow(target_matrix_yx,2));
    target_matrix_yx = target_matrix_yx / sqrt(pow(target_matrix_xx,2) + pow(target_matrix_yx,2));
    target_matrix_xy = target_matrix_xy / sqrt(pow(target_matrix_xy,2) + pow(target_matrix_yy,2));
    target_matrix_yy = target_matrix_yy / sqrt(pow(target_matrix_xy,2) + pow(target_matrix_yy,2));

    double base_matrix_xx = base_x_axis.getX();
    double base_matrix_xy = base_y_axis.getX();
    double base_matrix_yx = base_x_axis.getY();
    double base_matrix_yy = base_y_axis.getY();
//    ROS_INFO_STREAM("base_x_axis:"<<base_matrix_xx<<","<<base_matrix_yx);
 //   ROS_INFO_STREAM("base_y_axis:"<<base_matrix_xy<<","<<base_matrix_yy);

    base_matrix_xx = base_matrix_xx / sqrt(pow(base_matrix_xx,2) + pow(base_matrix_yx,2));
    base_matrix_yx = base_matrix_yx / sqrt(pow(base_matrix_xx,2) + pow(base_matrix_yx,2));
    base_matrix_xy = base_matrix_xy / sqrt(pow(base_matrix_xy,2) + pow(base_matrix_yy,2));
    base_matrix_yy = base_matrix_yy / sqrt(pow(base_matrix_xy,2) + pow(base_matrix_yy,2));


    double base_to_target_dot, base_to_target_cross, base_to_target_angle;
    base_to_target_dot = base_matrix_xx*target_matrix_xx + base_matrix_yx*target_matrix_yx;
    base_to_target_cross = base_matrix_xx*target_matrix_yx - base_matrix_yx*target_matrix_xx;
    base_to_target_angle = atan2(base_to_target_cross, base_to_target_dot);
    return base_to_target_angle;
}

void convertGlobalMoveToLocalMove(tf::Matrix3x3 current_matrix, double &global_move_x, double &global_move_y, double &local_move_x, double &local_move_y){
    tf::Vector3 base_x_axis = current_matrix.getColumn(0);
    tf::Vector3 base_y_axis = current_matrix.getColumn(1);

    double base_matrix_xx = base_x_axis.getX();
    double base_matrix_xy = base_y_axis.getX();
    double base_matrix_yx = base_x_axis.getY();
    double base_matrix_yy = base_y_axis.getY();

    base_matrix_xx = base_matrix_xx / sqrt(pow(base_matrix_xx,2) + pow(base_matrix_yx,2));
    base_matrix_yx = base_matrix_yx / sqrt(pow(base_matrix_xx,2) + pow(base_matrix_yx,2));
    base_matrix_xy = base_matrix_xy / sqrt(pow(base_matrix_xy,2) + pow(base_matrix_yy,2));
    base_matrix_yy = base_matrix_yy / sqrt(pow(base_matrix_xy,2) + pow(base_matrix_yy,2));

    local_move_x = base_matrix_xx * global_move_x + base_matrix_yx * global_move_y;
    local_move_y = base_matrix_xy * global_move_x + base_matrix_yy * global_move_y;
  //  ROS_INFO_STREAM("global move_x:"<<x_move<<", global move_y:"<<y_move);

//    ROS_INFO_STREAM("local move_x:"<<base_x_move<<", local move_y:"<<base_y_move);
}


tf::Matrix3x3 GetMatrixFromPose(geometry_msgs::Pose pose)
{ 
    tf::Quaternion current_quaternion_;
    current_quaternion_.setX(pose.orientation.x);  
    current_quaternion_.setY(pose.orientation.y);
    current_quaternion_.setZ(pose.orientation.z);
    current_quaternion_.setW(pose.orientation.w);
    tf::Matrix3x3 * current_matrix_ = new tf::Matrix3x3(current_quaternion_);
    return *current_matrix_;
}

// ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
// {
//     return boost::make_shared<ob::PathLengthOptimizationObjective>(si);
// }

double getDistance(std::tuple<double, double, double> pt_0, std::tuple<double, double, double> pt_1){
    return std::sqrt((std::get<0>(pt_0) - std::get<0>(pt_1))*(std::get<0>(pt_0) - std::get<0>(pt_1)) - (std::get<1>(pt_0) - std::get<1>(pt_1))*(std::get<1>(pt_0) - std::get<1>(pt_1)));
}

double getDistance(std::pair<double, double> pt_0, std::pair<double, double> pt_1){
    return std::sqrt((std::get<0>(pt_0) - std::get<0>(pt_1))*(std::get<0>(pt_0) - std::get<0>(pt_1)) - (std::get<1>(pt_0) - std::get<1>(pt_1))*(std::get<1>(pt_0) - std::get<1>(pt_1)));    
}

bool isPointVeryClose(double limit, std::tuple<double, double, double> pt_0, std::tuple<double, double, double> pt_1){
    double dist = std::sqrt((std::get<0>(pt_0) - std::get<0>(pt_1))*(std::get<0>(pt_0) - std::get<0>(pt_1)) - (std::get<1>(pt_0) - std::get<1>(pt_1))*(std::get<1>(pt_0) - std::get<1>(pt_1)));
    if(dist < limit){
        return true;
    }else{
        return false;
    }
}

bool isTwoSegmentClose(arc::segment_2 seg0, arc::segment_2 seg1, double dist){
    double x0 = bg::get<0, 0>(seg0); 
    double y0 = bg::get<0, 1>(seg0);
    double x1 = bg::get<1, 0>(seg0);
    double y1 = bg::get<1, 1>(seg0);
    
    
    std::pair<double, double> seg0_center, seg1_center;
    seg0_center = std::make_pair((x0 + x1) / 2.0, (y0 + y1) / 2.0);
    
    x0 = bg::get<0, 0>(seg1); 
    y0 = bg::get<0, 1>(seg1);
    x1 = bg::get<1, 0>(seg1);
    y1 = bg::get<1, 1>(seg1);
    seg1_center = std::make_pair((x0 + x1) / 2.0, (y0 + y1) / 2.0);
    if(getDistance(seg0_center, seg1_center) < dist){
        return true;
    }else{
        return false;
    }
}

}

