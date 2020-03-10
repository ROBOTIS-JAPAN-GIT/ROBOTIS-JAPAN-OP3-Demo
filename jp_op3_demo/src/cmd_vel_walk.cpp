/* Author : Yoshimaru Tanaka */

#include "jp_op3_demo/cmd_vel_walk.h"

namespace robotis_op {
  CmdVelWalk::CmdVelWalk()
    : DEBUG_PRINT_(false),
      SPIN_RATE_(20),
      CMD_VEL_TIMEOUT_(0.100),
      MAX_X_MOVE_(0.030), // 0.050
      MAX_Y_MOVE_(0.020), // 0.050
      MAX_R_ANGLE_(0.250), // 0.025
      IN_PLACE_FB_STEP_(-3.0 * 0.001),
      SPOT_FB_OFFSET_(0.0 * 0.001),
      SPOT_RL_OFFSET_(0.0 * 0.001),
      SPOT_ANGLE_OFFSET_(0.0),
      on_start_(false),
      curr_period_time_(0.0),
      curr_x_move_(0.0),
      curr_y_move_(0.0),
      curr_r_angle_(0.0) {

    // NOTE: walking_module should be turned on from GUI

    ros::NodeHandle nh(ros::this_node::getName());

    std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/gui_config.yaml";
    std::string path = nh.param<std::string>("demo_config", default_path);
    // parseJointNameFromYaml(path);

    boost::thread queue_thread = boost::thread(boost::bind(&CmdVelWalk::callbackThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&CmdVelWalk::processThread, this));

    // goInitPose(); //

    prev_time_ = ros::Time::now();

  }

  CmdVelWalk::~CmdVelWalk() {
    // none
  }

  void CmdVelWalk::process() {

    // prev_time_ is the last time stamp of callback
    ros::Time curr_time = ros::Time::now();
    ros::Duration dur = curr_time - prev_time_;
    double delta_time = dur.nsec * 0.000000001 + dur.sec;
    ROS_INFO("delta time : %f", delta_time);

    if (delta_time > CMD_VEL_TIMEOUT_) {
      setWalkingCommand("stop");
      on_start_ = false;
      ROS_INFO("cmd_vel is time out");
      return;
    }

  }

  void CmdVelWalk::processThread() {
    ros::Rate loop_rate(SPIN_RATE_);

    // initialize
    ROS_INFO("Start Command Velocity Mode");

    bool result = getWalkingParam();
    if (result == true) {
      // hip_pitch_offset_ = curr_walking_param_.hip_pitch_offset;
      curr_period_time_ = curr_walking_param_.period_time;
    } else {
      // why is hip pitch offset necessary...?
      // hip_pitch_offset_ = 7.0 * M_PI / 180;
      curr_period_time_ = 0.5;
    }

    //node loop
    while (ros::ok()) {
      process();
      loop_rate.sleep();
    }
  }

  void CmdVelWalk::callbackThread() {

    ros::NodeHandle nh(ros::this_node::getName());

    cmd_vel_sub_ =
      nh.subscribe("/cmd_vel/", 1, &CmdVelWalk::cmdVelCallback, this);

    walking_command_sub_ =
      nh.subscribe("robotis/walking/command", 1, &CmdVelWalk::walkingCommandCallback, this);

    init_pose_pub_ =
      nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
    // goInitPose();

    set_walking_command_pub_ =
      nh.advertise<std_msgs::String>("/robotis/walking/command", 0);

    set_walking_param_pub_ =
      nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);

    get_walking_param_client_ =
      nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis/walking/get_params");

    while (nh.ok()) {
      ros::spinOnce();
      usleep(1000);
    }

  }

  void CmdVelWalk::walkingCommandCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "start")
      on_start_ = true;
    else
      on_start_ = false;
    // curr_command_ = msg->data;
  }

  void CmdVelWalk::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    if (on_start_ == false) {
      setWalkingCommand("start");
    }

    // Create walking command ...
    // now, only adjust move amp
    // but, period time is also adjustable
    // so, set min and max pt to generate the parameter.
    double pt = curr_walking_param_.period_time;

    double x = msg->linear.x * pt;
    if (x > 0)
      x = x < MAX_X_MOVE_ ? x : MAX_X_MOVE_;
    else
      x = x > -MAX_X_MOVE_ ? x : -MAX_X_MOVE_;

    double y = msg->linear.y * pt;
    if (y > 0)
      y = y < MAX_Y_MOVE_ ? y : MAX_Y_MOVE_;
    else
      y = y > -MAX_Y_MOVE_ ? y : -MAX_Y_MOVE_;

    double yaw = msg->angular.z * pt;
    if (yaw > 0)
      yaw = yaw < MAX_R_ANGLE_ ? yaw : MAX_R_ANGLE_;
    else
      yaw = yaw > -MAX_R_ANGLE_ ? yaw : -MAX_R_ANGLE_;

    setWalkingParam(x, y, yaw);

    ros::Time curr_time = ros::Time::now();
    prev_time_ = curr_time;

  }

  bool CmdVelWalk::getWalkingParam() {

    op3_walking_module_msgs::GetWalkingParam walking_param_msg;

    if (get_walking_param_client_.call(walking_param_msg)) {
      curr_walking_param_ = walking_param_msg.response.parameters;
      // update ui
      ROS_INFO_COND(DEBUG_PRINT_, "Get walking parameters");
      return true;
    } else {
      ROS_ERROR("Fail to get walking parameters.");
      return false;
    }
  }

  void CmdVelWalk::setWalkingCommand(const std::string &command) {
    // get param
    if (command == "start") {
      getWalkingParam();
      setWalkingParam(IN_PLACE_FB_STEP_, 0, 0, true);
    }

    std_msgs::String _command_msg;
    _command_msg.data = command;
    if (set_walking_command_pub_)
      set_walking_command_pub_.publish(_command_msg);
    else
      return;

    ROS_INFO_STREAM_COND(DEBUG_PRINT_, "Send Walking command : " << command);
  }

  void CmdVelWalk::setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance) {

    curr_walking_param_.balance_enable = balance;
    curr_walking_param_.x_move_amplitude = x_move + SPOT_FB_OFFSET_;
    curr_walking_param_.y_move_amplitude = y_move + SPOT_RL_OFFSET_;
    curr_walking_param_.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET_;

    set_walking_param_pub_.publish(curr_walking_param_);

    curr_x_move_ = x_move;
    curr_y_move_ = y_move;
    curr_r_angle_ = rotation_angle;
  }

  void CmdVelWalk::goInitPose() {
    std_msgs::String init_msg;
    init_msg.data = "ini_pose";

    init_pose_pub_.publish(init_msg);
  }

} // namespace robotis_op
