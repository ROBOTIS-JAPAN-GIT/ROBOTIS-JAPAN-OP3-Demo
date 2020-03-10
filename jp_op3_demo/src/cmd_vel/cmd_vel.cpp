/* Author : Yoshimaru Tanaka */

#include "jp_op3_demo/cmd_vel.h"

namespace robotis_op {
  CmdVel::CmdVel() :
    BallTracker(),
    on_cmd_vel_(false),
    current_linear_x_(0),
    current_linear_y_(0),
    current_angular_yaw_(0),
    cmd_vel_status_(Off),
    count_off_(0) {

    ROS_INFO_STREAM("Initializing command velocity subscriber...");
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &CmdVel::cmdVelCallback, this);
  }

  CmdVel::~CmdVel() {}

  void CmdVel::cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    cmd_vel_ = msg->twist;
    current_linear_x = cmd_vel.linear.x;
    current_linear_y = cmd_vel.linear.y;
    current_angular_yaw = cmd_vel.angular.z;
  }

  void CmdVel::cmdVelCommandCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "start") {
        startCmdVel();
      }
    else if (msg->data == "stop"){
        stopCmdVel();
      }
    else if (msg->data == "toggle_start") {
        if (on_cmd_vel_ == false)
          startCmdVel();
        else
          stopCmdVel();
      }
  }

  void CmdVel::startCmdVel() {
    on_cmd_vel_ = true;
    ROS_INFO_COND(DEBUG_PRINT, "Start Command Velocity processing");
  }

  void CmdVel::stopCmdVel() {
    on_cmd_vel_ = false;
    ROS_INFO_COND(DEBUG_PRINT, "Stop Command Velocity processing");

    current_linear_x_ = 0;
    current_linear_y_ = 0;
    current_angular_yaw_ = 0;
    // x_error_sum_ = 0;
    // y_error_sum_ = 0;
  }

  int CmdVel::processCmdVel() {
    // int cmd_vel_status = On;
    // if (on_cmd_vel_ == false) {
    //   return Off;
    // }
    // cmd_vel_status_ = Off;
    // count_off_ = 0;
  }

} // namespace
