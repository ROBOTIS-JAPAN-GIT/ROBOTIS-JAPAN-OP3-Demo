/* Author : Yoshimaru Tanaka */

#include "jp_op3_demo/cmd_vel_to_walk.h"

namespace robotis_op {
  CmdVelToWalk::CmdVelToWalk()
    : SoccerDemo(),
      on_walk_com_(false),
      start_walk_com_(false),
      stop_walk_com_(false),
      walk_com_status_(false) {
    // default constructor called here
  } // Constructor
  CmdVelToWalk::~CmdVelToWalk()
    : ~SoccerDemo() {
    // default destructor called here
  } // Destructor

  void CmdVelToWalk::setCmdVelToWalkEnable() {
    enable_ = true;
    startCmdVelToWalkMode();
  } // setCmdVelToWalkEnable

  void CmdVelToWalk::setCmdVelToWalkDisable() {
    cmd_vel_.stopCmdVel();
    walk_com_.stopWalkCom();

    enable_ = false;
    wait_count_ = 0;
    on_walk_com_ = false;
    restart_cmd_vel_to_walk_ = false;
    start_walk_com_ = false;
    stop_walk_com_ = false;
    stop_fallen_check_ = false;

    cmd_vel_status_ = CmdVel::Waiting;
  } // setCmdVelToWalkDisable

  void CmdVelToWalk::process() {
    if(enable_ == false)
        return;

    int cmd_vel_status;
    cmd_vel_status = cmd_vel_.processCmdVel();

    if (start_walk_com_ == true) {
      cmd_vel_.startCmdVel();
      walk_com_.startWalkCom();
      start_walk_com_ = false;
    }

    if (stop_walk_com_ == true) {
      cmd_vel_.stopCmdVel();
      walk_com_.stopWalkCom();

      wait_count_ = 0;
    }

    if (wait_count_ <= 0) {
      if (on_walk_com_ == true) {
        switch(cmd_vel_status) {
        case CmdVel::On:
          walk_com_.processWalkCom(cmd_vel_.getLinearX(), cmd_vel_.getLinearY(), cmd_vel_.getAngularYaw);
          if (cmd_vel_status_ != cmd_vel_status)
            setRGBLED(0x1F, 0x1F, 0x1F);
          break;

        case CmdVel::Off:
          walk_com_.waitWalkCom();
          if (cmd_vel_status_ != cmd_vel_status)
            setRGBLED(0, 0, 0);
          break;

        default:
          break;

        } // switch
        if (cmd_vel_status_ != cmd_vel_status)
          cmd_vel_status_ = cmd_vel_status;
      } // if

      // check fallen states
      switch (stand_state_) {
      case Stand: {
        if (restart_cmd_vel_to_walk_ == true) {
          restart_cmd_vel_to_walk_ = false;
          startCmdVelToWalkMode();
          break;
        } // if
        break;
      } // case
      default : {
        walk_com_.stopWalkCom();
        handleFallen(stand_state_);
        break;
      }
      } // switch
    } // if

    else {
      walt_count_ -= 1;
    } // else

  } // process

  void CmdVelToWalk::processThread() {
    bool result = false;

    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    cmd_vel_.startCmdVel();

    // node loop
    while (ros::ok()) {
      if (enable_ == true)
        process();

      //relax to fit output rate
      loop_rate.sleep();
    } // loop
  } // processThread

  void CmdVelToWalk::callbackThread() {
    SoccerDemo::callbackThread();
  }

  void CmdVelToWalk::startCmdVelToWalkMode() { // done
    setModuleToDemo("action_module");

    playMotion(WalkingReady);

    setBodyModuleToDemo("walking_module");

    ROS_INFO("Start Command Velocity to Walk Command Mode.");
    on_walk_com = true;
    start_walk_com = true;
  } // startCmdVelToWalkMode

  vold CmdVelToWalk::stopCmdVelToWalkMode() { // done
    ROS_INFO("Stop Command Velocity to Walk Command Mode.")
    on_walk_com_ = false;
    stop_walk_com_ = true;
  } // stopCmdVelToWalkMode

  bool CmdVelToWalk::handleFallen(int fallen_status) {
    if (fallen_status == Stand)
      {
        return false;
      }

    // change to motion module
    setModuleToDemo("action_module");

    // getup motion
    switch (fallen_status) {
    case Fallen_Forward:
      std::cout << "Getup Motion [F]: " << std::endl;
      playMotion(is_grass_ ? GetUpFront + ForGrass : GetUpFront);
      break;

    case Fallen_Behind:
      std::cout << "Getup Motion [B]: " << std::endl;
      playMotion(is_grass_ ? GetUpBack + ForGrass : GetUpBack);
      break;

    default:
      break;
    }

    while(isActionRunning() == true)
      usleep(100 * 1000);

    usleep(650 * 1000);

    if (on_walk_com_ == true)
      restart_cmd_vel_to_walk_ = true;

    // reset state
    on_walk_com_ = false;

    return true;
  } // handleFallen

} // namespace
