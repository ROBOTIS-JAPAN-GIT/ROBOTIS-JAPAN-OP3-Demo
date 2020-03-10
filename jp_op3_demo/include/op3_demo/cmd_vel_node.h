/* Author : Yoshimaru Tanaka */

#ifdef CMD_VEL_NODE_H
#define CMD_VEL_NODE_H

// #include "op3_demo/op_demo.h"
// #include "op3_demo/ball_follower.h"
#include "op3_demo/soccer_demo.h"
#include <geometry_msgs/TwistStamped.h>

namespace robotis_op {

  // class CmdVelNode : public SoccerDemo, public BallFollower {
  class CmdVelNode : public SoccerDemo {
  public:
    CmdVelNode();
    ~CmdVelNode();

  protected:

    void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void process();
    void processThread();
    void callbackThread();

    // public?
    void startCmdVelMode();
    void stopCmdVelMode();

    void currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void setWalkingCommand(const std::string &command);
    void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);
    bool getWalkingParam();

    const double CMD_VEL_TIMEOUT_; // time out period until cmd_vel to stop
    const double MAX_X_MOVE_;
    const double MAX_Y_MOVE_;
    const double MAX_YAW_MOVE_;

    const double IN_PLACE_FB_STEP;
    const double SPOT_FB_OFFSET;
    const double SPOT_RL_OFFSET;
    const double SPOT_ANGLE_OFFSET;

    ros::Subscriber cmd_vel_sub_;
    ros::Publisher set_walking_command_pub_;
    ros::Publisher set_walking_param_pub_;
    ros::ServiceClient get_walking_param_client_;
    ros::Time prev_time_;
    op3_walking_module_msgs::WalkingParam current_walking_param_;

    // double hip_pitch_offset_;
    double curr_period_time_;
    double current_x_move_, current_y_move, current_r_angle_;
    bool on_cmd_vel_;

  }; // class
} // namespace

#endif // CMD_VEL_NODE_H
