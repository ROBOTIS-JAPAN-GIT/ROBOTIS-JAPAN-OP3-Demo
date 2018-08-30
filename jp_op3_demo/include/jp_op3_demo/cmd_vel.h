/* Author : Yoshimaru Tanaka */

#ifdef CMD_VEL_H_
#define CMD_VEL_H_

#include "op3_demo/ball_tracker.h"

namespace robotis_op {
  class CmdVel {
  public:
    enum CmdVelStatus {
      Off = -1,
      Waiting = 0,
      On = 1,
    };

    CmdVel();
    ~CmdVel();

    int processCmdVel();
    void startCmdVel();
    void stopCmdVle();

    void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void cmdVelCommandCallback(const std_msgs::String::ConstPtr &msg);

    double getLinearX() {
      return current_linear_x;
    }

    double getLinearY() {
      return current_linear_y;
    }

    double getAngularYaw() {
      return current_angular_yaw;
    }
  protected:
    ros::Subscriber cmd_vel_sub_;
    geometry::Twist cmd_vel_;

    int cmd_vel_status_;
    double current_linear_x_, current_linear_y_;
    double current_angular_yaw_;
    int cound_off_;
  }; // class
} // namespace

#endif /* CMD_VEL_H_ */
