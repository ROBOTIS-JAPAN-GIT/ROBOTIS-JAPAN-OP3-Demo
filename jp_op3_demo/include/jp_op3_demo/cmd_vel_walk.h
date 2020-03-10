/* Author : Yoshimaru Tanaka */

#ifndef CMD_VEL_WALK_H
#define CMD_VEL_WALK_H

#include <ros/ros.h>
#include <ros/package.h>
// #include <yaml-cpp/yaml.h>
#include <boost/thread.hpp>

#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include "op3_demo/op_demo.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"

namespace robotis_op {

  // class CmdVelWalk : public OPDemo {
  class CmdVelWalk {
  public:
    CmdVelWalk();
    ~CmdVelWalk();

  protected:

    void process();
    void processThread();
    void callbackThread();

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void goInitPose();
    bool getWalkingParam();
    void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);
    void setWalkingCommand(const std::string &command);

    void walkingCommandCallback(const std_msgs::String::ConstPtr& msg);

    const bool DEBUG_PRINT_;
    const int SPIN_RATE_;
    const double CMD_VEL_TIMEOUT_;
    const double MAX_X_MOVE_;
    const double MAX_Y_MOVE_;
    const double MAX_R_ANGLE_;
    const double IN_PLACE_FB_STEP_;
    const double SPOT_FB_OFFSET_;
    const double SPOT_RL_OFFSET_;
    const double SPOT_ANGLE_OFFSET_;

    bool on_start_;
    double curr_period_time_;
    double curr_x_move_, curr_y_move_, curr_r_angle_;

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber walking_command_sub_;

    ros::Publisher init_pose_pub_;
    ros::Publisher set_walking_command_pub_;
    ros::Publisher set_walking_param_pub_;

    ros::ServiceClient get_walking_param_client_;

    ros::Time prev_time_;
    op3_walking_module_msgs::WalkingParam curr_walking_param_;

  }; // class
} // namespace

#endif // CMD_VEL_WALK_H_
