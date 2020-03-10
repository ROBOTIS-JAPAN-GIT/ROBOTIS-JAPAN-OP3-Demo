#include "jp_op3_demo/walk_com.h"

namespace robotis_op {

  WalkCom::WalkCom()
    : BallFollower(),
      on_walk_com(false),
      current_x_move(0),
      current_y_move(0),
      current_yaw_move(0) {
  }

  WalkCom::~WalkCom() {}

  void WalkCom::startWalkCom() {
    on_walk_com_ = true;
    ROS_INFO("Start walking command generation.");
    setWalkingCommand("start");

    bool result = getWalkingParam();
    if (result == true) {
        hip_pitch_offset_ = current_walking_param_.hip_pitch_offset;
        curr_period_time_ = current_walking_param_.period_time;
    }
    else {
        hip_pitch_offset_ = 7.0 * M_PI / 180;
        curr_period_time_ = 0.6;
    }
  }

  void WalkCom::stopWalkCom() {
    on_walk_com_ = false;
    ROS_INFO("Stop walking command generation.");
    setWalkingCommand("stop");
  }

  void WalkCom::waitWalkCom() { // unnessessary...?
    setWalkingParam(0.0, 0.0, 0.0);
  }


}
