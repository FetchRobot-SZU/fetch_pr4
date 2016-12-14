
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
//#include <test_pure_pursuit/misc.hpp>
nav_msgs::Path path;

void addWayPoint(const geometry_msgs::PoseStamped& pose)
{
    path.poses.push_back(pose);
}
void clearWayPoints(const std_msgs::Empty& clear)
{
    path.poses.clear();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "multi_goal_navigation");
    ros::NodeHandle nh;
    path.header.frame_id="/odom";
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x=1.0;
    pose.pose.position.y=0;
    pose.header.frame_id="/odom";
    pose.pose.position.z=0;
    pose.pose.orientation.x=0;
    pose.pose.orientation.y=0;
    pose.pose.orientation.z=0;
    pose.pose.orientation.w=1;
    addWayPoint(pose);

    geometry_msgs::PoseStamped pose1;
    pose1.header.stamp = ros::Time::now();
    pose1.pose.position.x=2.0;
    pose1.pose.position.y=1;
    pose1.header.frame_id="/odom";
    pose1.pose.position.z=0;
    pose1.pose.orientation.x=0;
    pose1.pose.orientation.y=0;
    pose1.pose.orientation.z=0;
    pose1.pose.orientation.w=1;
    addWayPoint(pose1);

    geometry_msgs::PoseStamped pose2;
    pose2.header.stamp = ros::Time::now();
    pose2.pose.position.x=4.0;
    pose2.pose.position.y=2;
    pose2.header.frame_id="/odom";
    pose2.pose.position.z=0;
    pose2.pose.orientation.x=0;
    pose2.pose.orientation.y=0;
    pose2.pose.orientation.z=0;
    pose2.pose.orientation.w=1;
    addWayPoint(pose2);


    ros::Publisher pub_plan=nh.advertise<nav_msgs::Path>("/reference_path",1);
    while(nh.ok())
    {

      //  ROS_INFO("publish path: ");
        pub_plan.publish(path);
        ros::spinOnce();
    }
    // pub_plan.publish(path);

}
