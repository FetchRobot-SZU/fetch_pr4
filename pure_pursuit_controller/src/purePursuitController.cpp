/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "pure_pursuit_controller/purePursuitController.h"

#include <cmath>

#include <geometry_msgs/Twist.h>

#include <visualization_msgs/Marker.h>

namespace starleth
{

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

PurePursuitControllerNode::PurePursuitControllerNode(const ros::NodeHandle &nh) : _nodeHandle(nh),
                                                                                  ac("move_base", true), _nextWayPoint(-1)
{
  getParameters();
  _pathSubscriber = _nodeHandle.subscribe(_pathTopicName, _queueDepth, &PurePursuitControllerNode::pathCallback, this);
  _odometrySubscriber = _nodeHandle.subscribe(_odometryTopicName, _queueDepth, &PurePursuitControllerNode::odometryCallback, this);
  _cancelSubcriber=_nodeHandle.subscribe("/cancel_path", 10, &PurePursuitControllerNode::cancelCallback,this);

  _cmdVelocityPublisher = _nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  _cmdTrajectoryPublisher = _nodeHandle.advertise<visualization_msgs::Marker>(_cmdTrajectoryTopicName, _queueDepth);

  _timer = nh.createTimer(ros::Duration(1.0 / _frequency), &PurePursuitControllerNode::timerCallback, this);

  isGetPath = false;
  isFinish = false;
  isFirstPoint = false;
  isCancelPath=false;
  flag = 0;
  count = 0;
}

PurePursuitControllerNode::~PurePursuitControllerNode()
{
}

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/
// 由欧拉角创建四元数
void PurePursuitControllerNode::angle2quat(vector<float> &angle, vector<float> &quaternion)
{
  float cx = cos(angle[0] / 2);
  float sx = sin(angle[0] / 2);
  float cy = cos(angle[1] / 2);
  float sy = sin(angle[1] / 2);
  float cz = cos(angle[2] / 2);
  float sz = sin(angle[2] / 2);

  quaternion[0] = sx * cy * cz - cx * sy * sz; // x
  quaternion[1] = cx * sy * cz + sx * cy * sz; // y
  quaternion[2] = cx * cy * sz - sx * sy * cz; // z
  quaternion[3] = cx * cy * cz + sx * sy * sz; // w
}
//
void PurePursuitControllerNode::pathCallback(const nav_msgs::Path &msg)
{
  if (!isGetPath)
  {
    _currentReferencePath = msg;
    ROS_INFO("receive path msg");
    _nextWayPoint = -1;
    isGetPath = true;
  }
}

void PurePursuitControllerNode::odometryCallback(const nav_msgs::Odometry &
                                                     msg)
{
  //  ROS_INFO("heihei");
  _currentVelocity = msg.twist.twist;
}

void PurePursuitControllerNode::cancelCallback(const std_msgs::String::ConstPtr &msg){
    isCancelPath=true;
    isGetPath=false;
    _currentReferencePath.poses.clear();

}
void PurePursuitControllerNode::spin()
{
  ros::spin();
}

void PurePursuitControllerNode::timerCallback(const ros::TimerEvent &
                                                  event)
{
  if (!isFinish)
  {
    geometry_msgs::Twist cmdVelocity;
    if (step(cmdVelocity))
    {
      const size_t numPoints = 20;

      double lookAheadThreshold = getLookAheadThreshold();
      visualization_msgs::Marker cmdTrajectory;

      cmdTrajectory.header.frame_id = _poseFrameId;
      cmdTrajectory.header.stamp = ros::Time::now();
      cmdTrajectory.ns = "solution_trajectory";
      cmdTrajectory.type = 4;
      cmdTrajectory.action = 0;
      cmdTrajectory.scale.x = 0.12;
      cmdTrajectory.color.r = 0.0;
      cmdTrajectory.color.g = 0.0;
      cmdTrajectory.color.b = 1.0;
      cmdTrajectory.color.a = 1.0;
      cmdTrajectory.lifetime = ros::Duration(0);
      cmdTrajectory.frame_locked = true;
      cmdTrajectory.pose = geometry_msgs::Pose();
      cmdTrajectory.points.resize(numPoints);

      for (int i = 0; i < numPoints; ++i)
      {
        geometry_msgs::Pose pose;
        double dt = lookAheadThreshold * (double)i / (double)numPoints;

        pose.orientation.z = cmdVelocity.angular.x * dt;
        pose.position.x = cmdVelocity.linear.x * std::cos(
                                                     pose.orientation.z) *
                          dt;
        pose.position.y = cmdVelocity.linear.x * std::sin(
                                                     pose.orientation.z) *
                          dt;

        cmdTrajectory.points[i] = pose.position;
      }
      _cmdVelocityPublisher.publish(cmdVelocity);
      _cmdTrajectoryPublisher.publish(cmdTrajectory);
    }
  }
}

bool PurePursuitControllerNode::step(geometry_msgs::Twist &twist)
{
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  _nextWayPoint = getNextWayPoint(_nextWayPoint);

  if (_nextWayPoint >= 0)
  {
    geometry_msgs::PoseStamped pose;

    if (getInterpolatedPose(_nextWayPoint, pose))
    {
      double lookAheadDistance = getLookAheadDistance(pose);
      double lookAheadAngle = getLookAheadAngle(pose);

      double angularVelocity = 0.0;
      if (std::abs(std::sin(lookAheadAngle)) >= _epsilon)
        {
          // double radius = (lookAheadDistance/std::sin(lookAheadAngle));
          double linearVelocity = _velocity;
          //计算角速度v=rw
          geometry_msgs::PoseStamped origin = getCurrentPose();
          //四元数转欧拉角，这里只考虑z
          double x = origin.pose.orientation.x;
          double y = origin.pose.orientation.y;
          double z = origin.pose.orientation.z;
          double w = origin.pose.orientation.w;
          double robot_angel = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
          // ROS_INFO("robot_angel is %f", robot_angel);
          geometry_msgs::PoseStamped next = _currentReferencePath.poses[_nextWayPoint];
          double x1 = next.pose.position.x;
          double y1 = next.pose.position.y;
          double x0 = origin.pose.position.x;
          double y0 = origin.pose.position.y;
          double d = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);
          double distance = std::sqrt(d);
          // double r = distance / (2*std::sin(angle));

          //计算直线与x轴的夹
          double line_angel = asin((y1 - y0) / distance);
          //确实在坐标系中line_angel的取值
          if (line_angel > 0)
          {
            if (x1 < x0)
            {
              line_angel =M_PI-line_angel;
            }
          }
          else
          {
            if (x1 < x0)
            {
              line_angel=-M_PI-line_angel;
            }
            if(line_angel==-M_PI)
            {
              line_angel=M_PI;
            }
          }
          ROS_INFO("robot_angel is %f  line_angel is %f", robot_angel, line_angel);

          double angel = robot_angel - line_angel;

          double r = distance / (2 * std::sin(angel));
          angularVelocity = -linearVelocity / r;

          twist.linear.x = linearVelocity;
          twist.angular.z = angularVelocity;
        //ROS_INFO("angel is : %f  line is ", angularVelocity, linearVelocity);
        return true;
      }
    }
  }
  //double r = distance / (2 * std::sin(angel));
  //double angularVelocity = -linearVelocity / r;
  return false;
}
geometry_msgs::PoseStamped PurePursuitControllerNode::getCurrentPose()
    const
{
  geometry_msgs::PoseStamped pose, transformedPose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;
  try
  {
    _tfListener.transformPose(_currentReferencePath.header.frame_id,
                              pose, transformedPose);
  }
  catch (tf::TransformException &exception)
  {
    ROS_ERROR("_poseFrameId: %s ", _poseFrameId.c_str());
  }

  return transformedPose;
}

double PurePursuitControllerNode::getLookAheadDistance(const geometry_msgs::PoseStamped &pose) const
{
  geometry_msgs::PoseStamped origin = getCurrentPose();
  geometry_msgs::PoseStamped transformedPose;

  try
  {
    _tfListener.transformPose(_currentReferencePath.header.frame_id,
                              pose, transformedPose);
  }
  catch (tf::TransformException &exception)
  {
    ROS_ERROR_STREAM("PurePursuitControllerNode::getLookAheadDistance: " << exception.what());
    ROS_ERROR("currentReferencePath.header.frame_id: %s", _currentReferencePath.header.frame_id.c_str());
    return -1.0;
  }

  tf::Vector3 v1(origin.pose.position.x,
                 origin.pose.position.y,
                 0);
  tf::Vector3 v2(transformedPose.pose.position.x,
                 transformedPose.pose.position.y,
                 0);

  return tf::tfDistance(v1, v2);
}

double PurePursuitControllerNode::getLookAheadAngle(const geometry_msgs::PoseStamped &pose) const
{
  geometry_msgs::PoseStamped origin = getCurrentPose();
  geometry_msgs::PoseStamped transformedPose;

  try
  {
    _tfListener.transformPose(_currentReferencePath.header.frame_id,
                              pose, transformedPose);
  }
  catch (tf::TransformException &exception)
  {
    ROS_ERROR_STREAM("PurePursuitControllerNode::getLookAheadDistance: " << exception.what());

    ROS_ERROR("currentReferencePath.header.frame_id: %s", _currentReferencePath.header.frame_id.c_str());
    return -1.0;
  }

  tf::Vector3 v1(origin.pose.position.x,
                 origin.pose.position.y,
                 0);
  tf::Vector3 v2(transformedPose.pose.position.x,
                 transformedPose.pose.position.y,
                 0);
  double v1_theta = atan(origin.pose.position.y / origin.pose.position.x);
  if (v1_theta < 0)
    v1_theta = M_PI - v1_theta;
  double v2_theta = atan(transformedPose.pose.position.y / transformedPose.pose.position.x);
  if (v2_theta < 0)
    v2_theta = M_PI - v2_theta;
  //  ROS_INFO("v2_theta: %f, v1_theta:  %f", v2_theta, v1_theta);
  if (v2_theta > v1_theta)
  {
    if (v2_theta - v1_theta < M_PI)
      return tf::tfAngle(v1, v2);
    else
      return -tf::tfAngle(v1, v2);
  }
  else
  {
    if (v1_theta - v2_theta < M_PI)
      return -tf::tfAngle(v1, v2);
    else
      return tf::tfAngle(v1, v2);
  }
}

double PurePursuitControllerNode::getLookAheadThreshold() const
{
  return _lookAheadRatio * 0.1;
}

double PurePursuitControllerNode::getArcDistance(const geometry_msgs::PoseStamped &pose) const
{
  double lookAheadDistance = getLookAheadDistance(pose);
  double lookAheadAngle = getLookAheadAngle(pose);

  if (std::abs(std::sin(lookAheadAngle)) >= _epsilon)
    return lookAheadDistance / sin(lookAheadAngle) * lookAheadAngle;
  else
    return lookAheadDistance;
}

int PurePursuitControllerNode::getNextWayPoint(int wayPoint)
{
  if (!_currentReferencePath.poses.empty())
  {
    if (_nextWayPoint >= 0)
    {
      geometry_msgs::PoseStamped origin = getCurrentPose();
      tf::Vector3 v_1(origin.pose.position.x,
                      origin.pose.position.y,
                      0);
      double lookAheadThreshold = getLookAheadThreshold();

      for (int i = _nextWayPoint; i < _currentReferencePath.poses.size();
           ++i)
      {
        tf::Vector3 v_2(_currentReferencePath.poses[i].pose.position.x,
                        _currentReferencePath.poses[i].pose.position.y,
                        0);

        if (tf::tfDistance(v_1, v_2) > lookAheadThreshold)
        {
          ROS_INFO("wayPoints: %d", i);
          // ROS_INFO("lookAheadThreshold: %f", lookAheadThreshold);
          // ROS_INFO("origin.pose.position.x: %f,    origin.pose.position.y: %f", origin.pose.position.x, origin.pose.position.y);
          //ROS_INFO("currentReferencePath.pose.position.x: %f,    currentReferencePath.pose.position.y: %f", _currentReferencePath.poses[i].pose.position.x, _currentReferencePath.poses[i].pose.position.y);
          return i;
        }
      }
      if (_nextWayPoint == _currentReferencePath.poses.size() - 1)
        isFinish = true;
      return _nextWayPoint;
    }
    else
      return 0;
  }

  return -1;
}

int PurePursuitControllerNode::getClosestWayPoint() const
{
  if (!_currentReferencePath.poses.empty())
  {
    int closestWaypoint = -1;
    double minDistance = -1.0;

    for (int i = 0; i < _currentReferencePath.poses.size(); ++i)
    {
      double distance = getArcDistance(_currentReferencePath.poses[i]);

      if ((minDistance < 0.0) || (distance < minDistance))
      {
        closestWaypoint = i;
        minDistance = distance;
      }
    }

    return closestWaypoint;
  }

  return -1;
}

bool PurePursuitControllerNode::getInterpolatedPose(int wayPoint,
                                                    geometry_msgs::PoseStamped &interpolatedPose) const
{
  if (!_currentReferencePath.poses.empty())
  {
    if (wayPoint > 0)
    {
      double l_t = getLookAheadThreshold();
      double p_t = getLookAheadDistance(
          _currentReferencePath.poses[_nextWayPoint - 1]);

      if (p_t < l_t)
      {
        geometry_msgs::PoseStamped p_0 = getCurrentPose();
        geometry_msgs::PoseStamped p_1 =
            _currentReferencePath.poses[wayPoint - 1];
        geometry_msgs::PoseStamped p_2 =
            _currentReferencePath.poses[wayPoint];

        tf::Vector3 v_1(p_2.pose.position.x - p_0.pose.position.x,
                        p_2.pose.position.y - p_0.pose.position.y,
                        p_2.pose.position.z - p_0.pose.position.z);
        tf::Vector3 v_2(p_1.pose.position.x - p_0.pose.position.x,
                        p_1.pose.position.y - p_0.pose.position.y,
                        p_1.pose.position.z - p_0.pose.position.z);
        tf::Vector3 v_0(p_2.pose.position.x - p_1.pose.position.x,
                        p_2.pose.position.y - p_1.pose.position.y,
                        p_2.pose.position.z - p_1.pose.position.z);

        double l_0 = v_0.length();
        double l_1 = v_1.length();
        double l_2 = v_2.length();

        v_0.normalize();
        v_2.normalize();

        double alpha_1 = M_PI - tf::tfAngle(v_0, v_2);
        double beta_2 = asin(l_2 * sin(alpha_1) / l_t);
        double beta_0 = M_PI - alpha_1 - beta_2;
        double l_s = l_2 * sin(beta_0) / sin(beta_2);
        tf::Vector3 p_s(p_1.pose.position.x + v_0[0] * l_s,
                        p_1.pose.position.x + v_0[1] * l_s,
                        p_1.pose.position.x + v_0[2] * l_s);

        interpolatedPose.pose.position.x = p_s[0];
        interpolatedPose.pose.position.y = p_s[1];
        interpolatedPose.pose.position.z = p_s[2];

        return true;
      }
    }

    interpolatedPose = _currentReferencePath.poses[wayPoint];
    return true;
  }

  return false;
}

void PurePursuitControllerNode::getParameters()
{
  _nodeHandle.param<int>("pure_pursuit_controller/ros/queue_depth", _queueDepth, 100);
  _nodeHandle.param<std::string>("pure_pursuit_controller/ros/path_topic_name", _pathTopicName, "/reference_path");
  _nodeHandle.param<std::string>("pure_pursuit_controller/ros/odometry_topic_name", _odometryTopicName, "/odom");
  _nodeHandle.param<std::string>("pure_pursuit_controller/ros/cmd_velocity_topic_name", _cmdVelocityTopicName, "/cmd_vel");
  _nodeHandle.param<std::string>("pure_pursuit_controller/ros/cmd_trajectory_topic_name", _cmdTrajectoryTopicName, "/local_planner_solution_trajectory");
  _nodeHandle.param<std::string>("pure_pursuit_controller/ros/pose_frame_id", _poseFrameId, "base_link");

  _nodeHandle.param<double>("pure_pursuit_controller/controller/frequency", _frequency, 20.0);
  _nodeHandle.param<int>("pure_pursuit_controller/controller/initial_waypoint", _initialWayPoint, -1);
  _nodeHandle.param<double>("pure_pursuit_controller/controller/velocity", _velocity, 0.2);
  _nodeHandle.param<double>("pure_pursuit_controller/controller/look_ahead_ratio", _lookAheadRatio, 4.0);
  _nodeHandle.param<double>("pure_pursuit_controller/controller/epsilon", _epsilon, 1e-6);
}
}
