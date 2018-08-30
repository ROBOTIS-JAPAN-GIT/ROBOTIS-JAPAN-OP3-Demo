#ifndef CMD_VEL_DEMO_H
#define CMD_VEL_DEMO_H

#include "op3_demo/soccer_demo.h"
#include "jp_op3_demo/cmd_vel_sub.h"
#include "jp_op3_demo/walk_com_pub.h"

namespace robotis_jp {
  class CmdVelDemo : public SoccerDemo {
  public:
    CmdVelDemo();
    ~CmdVelDemo();

  protected:
    void startCmdVelMode();
    void stopCmdVelMode();

    CmdVelSub cmd_vel_sub_;
    WalkComPub walk_com_pub_;

  }; // class
} // namespace
#endif // macro
