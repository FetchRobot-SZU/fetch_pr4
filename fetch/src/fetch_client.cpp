#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// #include <fetch/fetchAction.h>
#include <fetch/ros_goalAction.h>
#include <vector>
using namespace std;

class FetchActionClient
{
protected:
  ros::NodeHandle nh_;
  
  std::string action_name_;
  fetch::ros_goalGoal goal;
  vector<fetch::ros_goalGoal> goals;
  bool success;

public:
  actionlib::SimpleActionClient<fetch::ros_goalAction> ac;

public:
  FetchActionClient(std::string name) : ac("fetch_server", true) // name
  {
  }

  ~FetchActionClient()
  {
  }

  void init()
  {
    while (!ac.waitForServer(ros::Duration(10.0)))
    {
      ROS_INFO("Init: Waiting for the move_base action server to come up");
    }
    ROS_INFO(" Connected");
  }

  void execute()
  {
    
    // get all the points
    getPoints();

    ROS_INFO("Action server started, sending goals.");
    send_goals();
   // bool s = send_goals();
    // if (s)
    // {
    //   ROS_INFO("Succeeded!!");
    // }
    // else
    // {
    //   ROS_INFO("Failed for some reason!!");
    // }

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration());

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
  }

  void input_data_pose(fetch::ros_goalGoal goal_now)
  {
    goal.poseStamped.header.frame_id = "map";
    goal.poseStamped.header.stamp = ros::Time::now();
    goal.poseStamped.pose.position.x = goal_now.poseStamped.pose.position.x;
    goal.poseStamped.pose.position.y = goal_now.poseStamped.pose.position.y;
    goal.poseStamped.pose.position.z = goal_now.poseStamped.pose.position.z;
    goal.poseStamped.pose.orientation.x = goal_now.poseStamped.pose.orientation.x;
    goal.poseStamped.pose.orientation.y = goal_now.poseStamped.pose.orientation.y;
    goal.poseStamped.pose.orientation.z = goal_now.poseStamped.pose.orientation.z;
    goal.poseStamped.pose.orientation.w = goal_now.poseStamped.pose.orientation.w;
  }

  //发布多个点
  bool send_goals()
  {
    int count = 0;
    for (vector<fetch::ros_goalGoal>::iterator ite = goals.begin(); ite != goals.end(); ite++)
    {
      ROS_INFO("Sending point");
      ac.sendGoal(*ite);
      ac.waitForResult();
      ros::Duration(5.0).sleep();

      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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

  // get all the points
  void getPoints()
  {
    // define poses
    // point1
    fetch::ros_goalGoal point1;
    point1.poseStamped.header.frame_id = "map";
    point1.poseStamped.header.stamp = ros::Time::now();
    point1.poseStamped.pose.position.x = 1.0;
    point1.poseStamped.pose.position.y = 0.0;
    point1.poseStamped.pose.position.z = 0.5;
    point1.poseStamped.pose.orientation.z = 1.0;
    goals.push_back(point1);

    // point2
    fetch::ros_goalGoal point2;
    point2.poseStamped.header.frame_id = "map";
    point2.poseStamped.header.stamp = ros::Time::now();
    point2.poseStamped.pose.position.x = 2.0;
    point2.poseStamped.pose.position.y = 1.0;
    point2.poseStamped.pose.position.z = 1.0;
    point2.poseStamped.pose.orientation.z = 1.0;
    goals.push_back(point2);
  }

  // 由欧拉角创建四元数
  void angle2quat(vector<float> &angle, vector<float> &quaternion)
  {
    cout << endl
         << "start transfering" << endl;
    float cx = cos(angle[0] / 2);
    float sx = sin(angle[0] / 2);
    float cy = cos(angle[1] / 2);
    float sy = sin(angle[1] / 2);
    float cz = cos(angle[2] / 2);
    float sz = sin(angle[2] / 2);
    // std::cout << cx << " " << sx << " " << cy << " " << sy << " " << cz << " " << sz << std::endl;

    quaternion[0] = sx * cy * cz - cx * sy * sz; // x
    quaternion[1] = cx * sy * cz + sx * cy * sz; // y
    quaternion[2] = cx * cy * sz - sx * sy * cz; // z
    quaternion[3] = cx * cy * cz + sx * sy * sz; // w
    // std::cout << quaternion[0] << " " << quaternion[1] << " " << quaternion[2] << " " << quaternion[3] << std::endl << "end of transfering" << std::endl;
  }
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "test_fetch");

  // create the action client
  // true causes the client to spin its own thread
  FetchActionClient FetchClient("fetch_client");
  FetchClient.init();
  FetchClient.execute();
  
  return 0;
}


/*
// ---------------------------------------------------------
fetch::ros_goalGoal goal;
void input_data_pose()
{
  // double n[] = {0, 0, 1.57079};
  // vector<float> angle1(n, n+3);
  // vector<float> quat1(4);
  // angle2quat(angle1, quat1);
   goal.poseStamped.header.frame_id="odom";
   goal.poseStamped.header.stamp=ros::Time::now();
   goal.poseStamped.pose.position.x=2.0;
   goal.poseStamped.pose.position.y=2;
   goal.poseStamped.pose.position.z=0.8;
   goal.poseStamped.pose.orientation.w = 1.0;

}
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<fetch::ros_goalAction> ac("fetch", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  input_data_pose();
  //ROS_INFO("%f Action finished: %s",goal.position[0]);
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration());

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
*/