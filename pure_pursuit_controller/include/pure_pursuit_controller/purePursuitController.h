#ifndef PURE_PURSUIT_CONTROLLER_NODE_H
#define PURE_PURSUIT_CONTROLLER_NODE_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
using namespace std;
namespace starleth
{

/** The class PurePursuitControllerNode implements a pure pursuit controller.
      \brief Pure pursuit controller
    */
class PurePursuitControllerNode
{
public:
  /** \name Constructors/destructor
      @{
      */
  /// Constructor
  PurePursuitControllerNode(const ros::NodeHandle &nh);

  /// Destructor
  virtual ~PurePursuitControllerNode();
  /** @}
      */

  /** \name Methods
      @{
      */
  /// Spin once
  void spin();
  /// Step once
  bool step(geometry_msgs::Twist &twist);
  /// Returns the current pose of the robot
  geometry_msgs::PoseStamped getCurrentPose() const;
  /// Returns the lookahead distance for the given pose
  double getLookAheadDistance(const geometry_msgs::PoseStamped &pose) const;
  /// Returns the lookahead angle for the given pose in [rad]
  double getLookAheadAngle(const geometry_msgs::PoseStamped &pose) const;
  /// Returns the current lookahead distance threshold
  double getLookAheadThreshold() const;
  /// Returns the lookahead distance for the given pose
  double getArcDistance(const geometry_msgs::PoseStamped &pose) const;
  /// Returns the next way point by linear search from the current waypoint
  int getNextWayPoint(int wayPoint);
  /// Returns the current closest waypoint
  int getClosestWayPoint() const;
  /// Returns the interpolated pose based on the given way point
  bool getInterpolatedPose(int wayPoint, geometry_msgs::PoseStamped &
                                             interpolatedPose) const;
  /** @}
      */
  void angle2quat(vector<float> &angle, vector<float> &quaternion);

protected:
  /** \name Protected methods
      @{
      */
  /// Retrieves parameters
  void getParameters();
  /// Path message callback
  void pathCallback(const nav_msgs::Path &msg);
  /// Odometry message callback
  void odometryCallback(const nav_msgs::Odometry &msg);
  /// Timer callback
  void timerCallback(const ros::TimerEvent &event);

  void cancelCallback(const std_msgs::String::ConstPtr &msg);
  /** @}
      */

  /** \name Protected members
      @{
      */
  /// ROS node handle
  ros::NodeHandle _nodeHandle;
  /// Path message subscriber
  ros::Subscriber _pathSubscriber;
  /// Path message topic name
  std::string _pathTopicName;
  /// Odometry message subscriber
  ros::Subscriber _odometrySubscriber;
  /// Odometry message topic name
  std::string _odometryTopicName;
  /// Frame id of pose estimates
  std::string _poseFrameId;
  /// Queue size for receiving messages
  ///
  ros::Subscriber _cancelSubcriber;
  int _queueDepth;
  bool isGetPath;
  bool isFinish;
  bool isFirstPoint;
  bool isCancelPath;
  double flag;
  int count;
  /// Current reference path
  nav_msgs::Path _currentReferencePath;
  /// Current velocity
  geometry_msgs::Twist _currentVelocity;
  /// Controller frequency
  double _frequency;
  /// Next way point
  int _nextWayPoint;
  /// Commanded velocity publisher
  ros::Publisher _cmdVelocityPublisher;
  /// Commanded velocity topic name
  std::string _cmdVelocityTopicName;
  /// Commanded trajectory publisher
  ros::Publisher _cmdTrajectoryPublisher;
  /// Commanded trajectory topic name
  std::string _cmdTrajectoryTopicName;
  /// Initial way point
  int _initialWayPoint;
  /// Velocity
  double _velocity;
  /// Lookahead ratio
  double _lookAheadRatio;
  /// Epsilon
  double _epsilon;
  /// Transform listener for robot's pose w.r.t. map
  tf::TransformListener _tfListener;
  /// Timer
  ros::Timer _timer;
  /** @}
      */
public:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
};
}

#endif // PURE_PURSUIT_CONTROLLER_NODE_H
