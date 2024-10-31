#include "my_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner 
{
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {}

    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("该我上场表演了！");
    }
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        return true;
    }
    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return false;
    }
} // namespace my_planner
