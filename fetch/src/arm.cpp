// Standard
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <vector>
// ROS Stuff
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <fetch/ros_goalAction.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// Move base
#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
using namespace std;


class FetchActionServer
{
  protected:
    ros::NodeHandle nh_;
    ros::AsyncSpinner *spinner;
    ros::Publisher display_publisher;


    // Moveit!
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    geometry_msgs::Pose target_pose;
    moveit::planning_interface::MoveGroup *group;
    // Move communication: base and arm
    bool success;
  

  public:
    FetchActionServer()
    {
        spinner = new ros::AsyncSpinner(1);
        group = new moveit::planning_interface::MoveGroup("arm_with_torso");
        group->setPoseReferenceFrame("odom");
       
        ROS_INFO("server is beginning");
    }
    ~FetchActionServer(void)
    {
    }
    //初始化数据
    void init_data()
    {
        //设置目标点
        this->target_pose.position.x = 0.5;
        this->target_pose.position.y = 0;
        this->target_pose.position.z = 0.8;
        this->target_pose.orientation.w = 1;
        this->target_pose.orientation.x = 0;
        this->target_pose.orientation.y = 0;
        this->target_pose.orientation.z = 0; //0.8

    }
    void init()
    {
        this->spinner->start();
        this->display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        ROS_INFO(" Connected");
    }

    bool move_arm_to_goal()
    {
        group->setPoseTarget(this->target_pose);
        bool success = group->move();
        return success;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fetch");

    FetchActionServer FetchServer; // ros::this_node::getName()
    FetchServer.init();
    FetchServer.init_data();
    FetchServer.move_arm_to_goal();

    return 0;
}
