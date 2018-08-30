/* Author: Yoshimaru Tanaka */

#include "jp_op3_demo/cmd_vel_node.h"

namespace robotis_op {
  CmdVelNode::CmdVelNode()
    : SoccerDemo(),
      CMD_VEL_TIMEOUT_(0.1),
      MAX_X_MOVE_(0.050),
      MAX_Y_MOVE_(0.050),
      MAX_YAW_MOVE_(0.025),
      IN_PLACE_FB_STEP(-3.0 * 0.001),
      SPOT_FB_OFFSET(0.0 * 0.001),
      SPOT_RL_OFFSET(0.0 * 0.001),
      SPOT_ANGLE_OFFSET(0.0),
      current_x_move_(0.005),
      current_y_move_(0.005),
      current_r_angle_(0),
      curr_period_time_(0.5),
      prev_time_(0,0),
      on_cmd_vel_(false) {
    // SoccerDemo()
    prev_time_ = ros::Time::now();

    boost::thread queue_thread = boost::thread(boost::bind(&CmdVelNode::callbackThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&CmdVelNode::processThread, this));

  }

  CmdVelNode::~CmdVelNode() {
    // ~SoccerDemo()
  }

  void CmdVelNode::process() {

    ros::Time curr_time = ros::Time::now();
    ros::Duration dur = curr_time - prev_time_;
    double delta_time = dur.nsec * 0.000000001 + dur.sec;
    ROS_INFO("delta time : %f", delta_time);

    if (enable_ == false || delta_time > CMD_VEL_TIMEOUT_) {
      setWalkingCommand("stop");
      on_cmd_vel_ = false;
      ROS_INFO("cmd_vel is time out");
      // setWalkingParam(0.0, 0.0, 0.0);
      return;
    }

    // ROS_INFO("%d", stand_state_);

    switch (stand_state_) {
    case Stand:
      {
        if (enable_ == false) {
          enable_ = true;
          startCmdVelMode();
          break;
        } // if
      } // case Stand
      // fallen state : Fallen_Forward, Fallen_Behind
    default:
      {
        if (enable_ == true) {
          enable_ = false;
        }
        stopCmdVelMode();
        handleFallen(stand_state_);
        break;
      } // default
    } // switch

  }

  // main loop
  void CmdVelNode::processThread() {
    //set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    //node loop
    while (ros::ok()) {
      // ROS_INFO("%d", enable_);
      if (enable_ == true)
        process();
      //relax to fit output rate
      loop_rate.sleep();
      ROS_INFO("Node loop");
    }

  } // processThread

  void CmdVelNode::callbackThread() {

    ros::NodeHandle nh(ros::this_node::getName());

    // module_control_pub_ is used ...?
    module_control_pub_ = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
    motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
    rgb_led_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

    cmd_vel_sub_ = nh.subscribe("/cmd_vel/", 1, &CmdVelNode::cmdVelCallback, this);
    imu_data_sub_ = nh.subscribe("/robotis/open_cr/imu", 1, &CmdVelNode::imuDataCallback, this);
    buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &CmdVelNode::buttonHandlerCallback, this);
    demo_command_sub_ = nh.subscribe("/robotis/demo_command", 1, &CmdVelNode::demoCommandCallback, this);

    is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
    set_joint_module_client_ = nh.serviceClient<robotis_controller_msgs::SetJointModule>("/robotis/set_present_joint_ctrl_modules");

    // walking pub sub
    // current_joint_states_sub_ = nh_.subscribe("/robotis/goal_joint_states", 10, &BallFollower::currentJointStatesCallback, this);
    set_walking_command_pub_ = nh.advertise<std_msgs::String>("/robotis/walking/command", 0);
    set_walking_param_pub_ = nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);
    get_walking_param_client_ = nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis/walking/get_params");

    while (nh.ok()) {
        ros::spinOnce();
        usleep(1000);
    }
  }


  void CmdVelNode::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    SoccerDemo::imuDataCallback(msg);
  }

  void CmdVelNode::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg) {
    SoccerDemo::buttonHandlerCallback(msg);
  }

  void CmdVelNode::demoCommandCallback(const std_msgs::String::ConstPtr& msg) {
    SoccerDemo::demoCommandCallback(msg);
  }

  void CmdVelNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    // disable if condition is not met.
    // if (enable_ == false || stop_fallen_check_ == true) {
    // if (enable_ == false) {
    //   setWalkingCommand("stop");
    //   return;
    // }

    // start command is sent only once!
    // if (on_cmd_vel_ == false) {
    //   on_cmd_vel_ = true;
    //   setWalkingCommand("start");
    // }

    setWalkingCommand("start");

    // Create walking command ...
    double pt = current_walking_param_.period_time;
    ROS_INFO("%f", pt); // 0.6

    double x = msg->linear.x * pt;
    if (x > 0)
      x = x < MAX_X_MOVE_ ? x : MAX_X_MOVE_;
    else
      x = x > -MAX_X_MOVE_ ? x : -MAX_X_MOVE_;
    ROS_INFO("%f", MAX_X_MOVE_); // 0.02
    ROS_INFO("%f", x); // 0.3
    ROS_INFO("%f", abs(x)); // 0.3

    double y = msg->linear.y * pt;
    if (y > 0)
      y = y < MAX_Y_MOVE_ ? y : MAX_Y_MOVE_;
    else
      y = y > -MAX_Y_MOVE_ ? y : -MAX_Y_MOVE_;
    ROS_INFO("%f", y);

    double yaw = msg->angular.z * pt;
    if (yaw > 0)
      yaw = yaw < MAX_YAW_MOVE_ ? yaw : MAX_YAW_MOVE_;
    else
      yaw = yaw > -MAX_YAW_MOVE_ ? yaw : -MAX_YAW_MOVE_;
    ROS_INFO("%f", yaw);

    setWalkingParam(x, y, yaw);

    ros::Time curr_time = ros::Time::now();
    ros::Duration dur = curr_time - prev_time_;
    double delta_time = dur.nsec * 0.000000001 + dur.sec;
    prev_time_ = curr_time;
  }

  void  CmdVelNode::startCmdVelMode() {

    enable_ = true;
    // on_cmd_vel = true;

    setModuleToDemo("action_module");
    playMotion(WalkingReady);
    setBodyModuleToDemo("walking_module");

    ROS_INFO("Start Command Velocity Mode");
    // setWalkingCommand("start");

    bool result = getWalkingParam();
    if (result == true) {
      // hip_pitch_offset_ = current_walking_param_.hip_pitch_offset;
      curr_period_time_ = current_walking_param_.period_time;
    } else {
      // hip_pitch_offset_ = 7.0 * M_PI / 180;
      curr_period_time_ = 0.5;
    }
  }

  void  CmdVelNode::stopCmdVelMode() {

    ROS_INFO("Start Command Velocity Mode");
    setWalkingCommand("stop");
    on_cmd_vel_ = false;
    enable_ = false;
  }

  void CmdVelNode::setWalkingCommand(const std::string &command) {
    // get param
    if (command == "start") {
      getWalkingParam();
      setWalkingParam(IN_PLACE_FB_STEP, 0, 0, true);
    }

    std_msgs::String _command_msg;
    _command_msg.data = command;
    set_walking_command_pub_.publish(_command_msg);

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Send Walking command : " << command);
  }

  void CmdVelNode::setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance) {

    current_walking_param_.balance_enable = balance;
    current_walking_param_.x_move_amplitude = x_move + SPOT_FB_OFFSET;
    current_walking_param_.y_move_amplitude = y_move + SPOT_RL_OFFSET;
    current_walking_param_.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET;

    set_walking_param_pub_.publish(current_walking_param_);

    current_x_move_ = x_move;
    current_y_move_ = y_move;
    current_r_angle_ = rotation_angle;
  }

  bool CmdVelNode::getWalkingParam() {

    op3_walking_module_msgs::GetWalkingParam walking_param_msg;

    if (get_walking_param_client_.call(walking_param_msg)) {
      current_walking_param_ = walking_param_msg.response.parameters;
      // update ui
      ROS_INFO_COND(DEBUG_PRINT, "Get walking parameters");
      return true;
    } else {
      ROS_ERROR("Fail to get walking parameters.");
      return false;
    }
  }

  void CmdVelNode::setDemoEnable() {
    startCmdVelMode();
  }

  void CmdVelNode::setDemoDisable() {
    stopCmdVelMode();
  }
} // namespace
