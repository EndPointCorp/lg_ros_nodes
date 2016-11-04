// crbarker@google.com
//
// This is a ROS program to handle joystick-based navigation for google maps globe

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "joystick_navigator.h"
#include "lg_spacenav_globe/PortalPose.h"

// #define DEBUG

class PortalNavigatorNode {
 public:
  PortalNavigatorNode(void): joystick_sensitivity_(1.0) {}

  // Starts the run loop and does not return until killed.
  void Run(void);

  // Callbacks for ROS topic subscriptions:
  void HandleSpaceNav(const geometry_msgs::Twist::ConstPtr& twist);
  void HandleKioskPose(
      const lg_spacenav_globe::PortalPose::ConstPtr& portal_pose);

 private:
  ros::NodeHandle n_;
  ros::Subscriber spacenav_sub_;
  ros::Subscriber kiosk_pose_sub_;
  ros::Publisher joystick_pub_;
  ros::Publisher kiosk_pub_;
  ros::Publisher display_pub_;

  double joystick_sensitivity_;

  JoystickNavigator kiosk_joystick_navigator_;
};

void PortalNavigatorNode::Run(void) {
  kiosk_pub_ = n_.advertise<geometry_msgs::PoseStamped>(
      "/lg_spacenav_globe/kiosk_goto_pose", 1);
  display_pub_ = n_.advertise<geometry_msgs::PoseStamped>(
      "/lg_spacenav_globe/display_goto_pose", 1);

  ros::param::param<double>(
      "~joystick_sensitivity",
      joystick_sensitivity_,
      1.0);

  kiosk_joystick_navigator_.Init(
      &kiosk_pub_, &display_pub_, joystick_sensitivity_);

  // This subscriber takes commands from the SpaceNav.
  spacenav_sub_ = n_.subscribe("/spacenav/twist", 2,
      &PortalNavigatorNode::HandleSpaceNav, this);

  // This subscriber gets camera poses back from the kiosk.
  // We have a small Rx queue to make sure we don't lose any.
  kiosk_pose_sub_ = n_.subscribe("/portal_kiosk/current_pose", 1,
      &PortalNavigatorNode::HandleKioskPose, this);

  // This publishes a normalized version of the SpaceNav inputs.
  joystick_pub_ = n_.advertise<geometry_msgs::Twist>("/joystick/twist", 2);

  // Does not return until killed:
  ros::spin();
}

void PortalNavigatorNode::HandleSpaceNav(
    const geometry_msgs::Twist::ConstPtr& twist) {
  // The SpaceNav twist values range [-350, 350], so it must be
  // normalized for the joystick code, which expects [-1.0, 1.0].
  static const double kSpaceNavScale = 350.0;

  // Our JoystickNavigator expects a right-hand-rule twist, where
  // X is right/left, Y is forward/backward, and Z is up/down.
  // The SpaceNav ROS driver swaps both linear and angular x/y.
  // It also sign-flips the y for both linear and angular.
  // We correct that here:
  geometry_msgs::Twist normalized_joy;
  normalized_joy.linear.x = twist->linear.y / kSpaceNavScale * -1.0;
  normalized_joy.linear.y = twist->linear.x / kSpaceNavScale;
  normalized_joy.linear.z = twist->linear.z / kSpaceNavScale;
  normalized_joy.angular.x = twist->angular.y / kSpaceNavScale * -1.0;
  normalized_joy.angular.y = twist->angular.x / kSpaceNavScale;
  normalized_joy.angular.z = twist->angular.z / kSpaceNavScale;

  kiosk_joystick_navigator_.ProcessJoy(normalized_joy);
  joystick_pub_.publish(normalized_joy);
}

void PortalNavigatorNode::HandleKioskPose(
    const lg_spacenav_globe::PortalPose::ConstPtr& portal_pose) {
#ifdef DEBUG
  ROS_INFO("HandleKioskPose curr lat:%lf, lon:%lf, alt:%lf, hdg:%lf, tlt:%lf",
           portal_pose->current_pose.position.y,
           portal_pose->current_pose.position.x,
           portal_pose->current_pose.position.z,
           portal_pose->current_pose.orientation.z,
           portal_pose->current_pose.orientation.x);
  ROS_INFO("HandleKioskPose min lat:%lf, lon:%lf, alt:%lf, hdg:%lf, tlt:%lf",
           portal_pose->pose_minimums.position.y,
           portal_pose->pose_minimums.position.x,
           portal_pose->pose_minimums.position.z,
           portal_pose->pose_minimums.orientation.z,
           portal_pose->pose_minimums.orientation.x);
  ROS_INFO("HandleKioskPose max lat:%lf, lon:%lf, alt:%lf, hdg:%lf, tlt:%lf",
           portal_pose->pose_maximums.position.y,
           portal_pose->pose_maximums.position.x,
           portal_pose->pose_maximums.position.z,
           portal_pose->pose_maximums.orientation.z,
           portal_pose->pose_maximums.orientation.x);
#endif
  kiosk_joystick_navigator_.ProcessCameraMoved(*portal_pose);
}

/*
 * main()
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "lg_spacenav_globe");
  PortalNavigatorNode node;
  node.Run();
  return 0;
}
