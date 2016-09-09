// crbarker@google.com
//
// The JoystickNavigator takes input in the form of a normalized [-1.0, 1.0]
// right-hand-rule  joystick twist and poses from the camera. It computes new
// nudged poses based on the previous camera pose, then sends the results to the
// supplied CameraBuffer.

#ifndef EXPERIMENTAL_ACME_SRC_PORTAL_CATKIN_SRC_PORTAL_NAV_SRC_JOYSTICK_NAVIGATOR_H_
#define EXPERIMENTAL_ACME_SRC_PORTAL_CATKIN_SRC_PORTAL_NAV_SRC_JOYSTICK_NAVIGATOR_H_

#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include "portal_nav/PortalPose.h"

class CameraBuffer;

class JoystickNavigator {
 public:
  JoystickNavigator();
  void Init(
      ros::Publisher *kiosk_pub,
      ros::Publisher *display_pub,
      double joystick_sensitivity);

  // Updates the nav context with the latest pose from the camera.
  void ProcessCameraMoved(const portal_nav::PortalPose& pose);

  // Takes a normalized joystick twist and nudges the camera pose.
  void ProcessJoy(const geometry_msgs::Twist& normalized_joy);

  // Publish a pose.
  void PublishPose(
      ros::Publisher *pub, const geometry_msgs::Pose& pose_msg);

 private:
  // Math utilities:
  static double Quadratic(double v);
  static double Clamp(double v, double min, double max);
  static timespec DiffTimespec(timespec start, timespec end);
  static double DivTimespec(timespec num, timespec denom);
  static bool EqualPoses(
      const geometry_msgs::Pose& left, const geometry_msgs::Pose& right);

  // These compare or scale the 'normal' twist per the class settings.
  bool IsWithinGutter(const geometry_msgs::Twist& normal);
  void Scale(const geometry_msgs::Twist& normal,
             double interval_scale,
             geometry_msgs::Twist* result);

  // These are settings for the scaling and guttering of the joystick.
  geometry_msgs::Twist linear_sensitivity_;
  geometry_msgs::Twist quadratic_sensitivity_;
  geometry_msgs::Twist gutter_;

  // When the camera is moving by some other means (touch, key, mouse),
  // the navigator remembers the most recent pose so we can move relative to it.
  geometry_msgs::Pose last_camera_pose_;

  // The kiosk also tells us clamps for the pose. In some modes, the clamps
  // change to accomodate photospheres, etc.
  geometry_msgs::Pose pose_minimums_;
  geometry_msgs::Pose pose_maximums_;

  // Once under joystick navigation (out of the gutter), all nudges are relative
  // to the most recent requested pose. This prevents buffer stutter.
  geometry_msgs::Pose last_requested_pose_;

  // This flag indicates whether we're using the last_camera_pose_ or the
  // last_requested_pose_ as the basis for motion.
  bool under_joy_control_;

  // A coefficient for joystick sensitivity via parameter.
  double joystick_sensitivity_;

  // The joystick inputs can come at any frequency, and get scaled based on the
  // delta time between them.
  timespec last_joy_time_;

  // Publish pose to here.
  ros::Publisher *kiosk_pub_;
  ros::Publisher *display_pub_;
};

#endif  // EXPERIMENTAL_ACME_SRC_PORTAL_CATKIN_SRC_PORTAL_NAV_SRC_JOYSTICK_NAVIGATOR_H_
