/* Author : Yoshimaru Tanaka */

#ifdef WALK_COM_H_
#define WALK_COM_H_

#include "op3_demo/ball_follower.h"

namespace robotis_op {
  class WalkCom {
  public:
    WalkCom();
    ~WalkCom();

    bool processWalkCom(double linear_x, double linear_y, double angular_yaw);
    void waitWalkCom();
    void startWalkCom();
    void stopWalkCom();

  protected:
    // override
    void setWalkingCommand(const std::string &command);
    void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);
    bool getWalkingParam();
    void calcFootstep(double target_distance, double target_angle, double delta_time,
                    double& fb_move, double& rl_angle);

    // geometry_msgs::Twist velocity;

    bool on_walk_com_;
    double current_x_move_, current_y_move_, current_yaw_move;

  }; // class
} // namespace

#endif // macro
