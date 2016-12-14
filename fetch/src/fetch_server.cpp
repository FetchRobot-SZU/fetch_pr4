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
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class FetchActionServer
{
protected:
  ros::NodeHandle nh_;
  ros::AsyncSpinner *spinner;
  ros::Publisher display_publisher;
  // Actionlib server
  actionlib::SimpleActionServer<fetch::ros_goalAction> as_;
  std::string action_name_;
  fetch::ros_goalFeedback feedback_;
  fetch::ros_goalResult result_;
  // Move base
  MoveBaseClient base_ac;
  move_base_msgs::MoveBaseGoal base_goal;
  vector<move_base_msgs::MoveBaseGoal> goals;
  // Moveit!
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  geometry_msgs::Pose target_pose;
  moveit::planning_interface::MoveGroup *group;
  // Move communication: base and arm
  bool success;
  bool needMoveBase;

public:
  FetchActionServer(std::string name) : as_(nh_, name, boost::bind(&FetchActionServer::executeCB, this, _1), false),
                                        action_name_(name), base_ac("move_base", true)
  { //fetch_rotate1();
    as_.start();
    spinner = new ros::AsyncSpinner(1);
    group = new moveit::planning_interface::MoveGroup("arm_with_torso");
    group->setPoseReferenceFrame("map");
    success = true;
    needMoveBase = false;
    ROS_INFO("server is beginning");
  }
  ~FetchActionServer(void)
  {
  }
  void init()
  {
    this->spinner->start();
    this->display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    while (!base_ac.waitForServer(ros::Duration()))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO(" Connected");
  }
  void executeCB(const fetch::ros_goalGoalConstPtr &goal)
  {
    init();
    // set_base_goal(goal);
    //this->target_pose=goal->poseStamped.pose;
    //ROS_INFO("%s: Executing, get reach point %f", action_name_.c_str(), goal.position[1]);
    bool isok = true;
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      isok = false;
    }
    if (isok)
    {
      //设置目标点
      this->target_pose.position.x = goal->poseStamped.pose.position.x - 0.4;
      this->target_pose.position.y = goal->poseStamped.pose.position.y;
      this->target_pose.position.z = goal->poseStamped.pose.position.z;
      this->target_pose.orientation.w = goal->poseStamped.pose.orientation.w;
      this->target_pose.position.x = goal->poseStamped.pose.position.x;
      this->target_pose.position.y = goal->poseStamped.pose.position.y;
      this->target_pose.position.z = goal->poseStamped.pose.position.z; //0.8
      //寻找目标点
      this->success = go_to_pose();
    }
    if (this->success)
    {
      //need to do compare the completeness
      feedback_.completeness = computeCompleteness();
      result_.success = true;
      as_.publishFeedback(feedback_);
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
    }
    else
    {
      result_.success = false;
      ROS_INFO("%s: failed: ", action_name_.c_str());
    }
  }

  float computeCompleteness()
  {
    return 0.6;
  }

  void setBaseGoalFromTargetPose()
  {
    this->base_goal.target_pose.header.frame_id = "odom"; // base_link
    this->base_goal.target_pose.header.stamp = ros::Time::now();
    this->base_goal.target_pose.pose.position.x = this->target_pose.position.x - 0.5;
    this->base_goal.target_pose.pose.position.y = this->target_pose.position.y - 0.2;
    this->base_goal.target_pose.pose.orientation.w = 1.0;
    // this->base_goal.target_pose.pose.orientatoin.w = this->target_pose.position.w;
  }
  bool move_arm_to_goal()
  {
    group->setPoseTarget(this->target_pose);
    bool success = group->move();
    return success;
  }
  bool move_base_to_goal()
  {
    this->base_goal.target_pose.pose.position.x = this->target_pose.position.x - 0.5;
    this->base_goal.target_pose.pose.position.y = this->target_pose.position.y - 0.2;
    base_ac.sendGoal(this->base_goal);
    base_ac.waitForResult();
    bool success;
    if (base_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      success = true;
    else
      success = false;
  }
  bool go_to_pose()
  {

    bool success = move_arm_to_goal();
    if (success)
      needMoveBase = false;
    else
      needMoveBase = true;
    if (needMoveBase)
    {
      setBaseGoalFromTargetPose();
      bool move_base_success = move_base_to_goal();
      if (move_base_success)
        success = move_arm_to_goal();
      else
        ROS_INFO("The base failed to move for some reason");
    }
    return success;
  }
  //发布多个点
  bool send_goals()
  {
    int count = 0;
    for (vector<move_base_msgs::MoveBaseGoal>::iterator ite = goals.begin(); ite != goals.end(); ite++)
    {
      ROS_INFO("Sending point");
      base_ac.sendGoal(*ite);
      base_ac.waitForResult();
      ros::Duration(5.0).sleep();

      if (base_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Hooray");
        count++;
      }
      else
        ROS_INFO("failed for some reason");
    }
    if (count == goals.size())
      return true;
    else
      return false;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fetch");

  FetchActionServer FetchServer("fetch_server"); // ros::this_node::getName()
  ros::spin();

  return 0;
}