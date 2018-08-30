/* Author: Yoshimaru Tanaka */

#ifndef CMD_VEL_TO_WALK_DEMO_H
#define CMD_VEL_TO_WALK_DEMO_H

#include "op3_demo/soccer_demo.h"
#include "jp_op3_demo/cmd_vel_sub.h"
#include "jp_op3_demo/walk_com_pub.h"

namespace robotis_jp {
  class CmdVelToWalk : public SoccerDemo {
  public:
    CmdVelToWalk();
    ~CmdVelToWalk();

  protected:
    void setCmdVelToWalkEnable();
    void startCmdVelToWalkMode();
    void setCmdVelToWalkDisable();
    void stopCmdVelToWalkMode();

    // new objects
    CmdVel cmd_vel_; // handle input
    WalkCom walk_com_; // handle output

    // new parameters
    bool on_walk_com_;
    bool start_walk_com_;
    bool stop_walk_com_;
    int walk_com_status_;
    bool restart_cmd_vel_to_walk_;
  }; // class
} // namespace
#endif // macro
