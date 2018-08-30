/* Yoshimaru Tanaka */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "jp_op3_demo/cmd_vel.h"


enum Demo_Status {
  Ready = 0,
  CmdVel = 1,
  DemoCount = 2,
};

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
void goInitPose();
void playSound(const std::string &path);
void setLED(int led);
bool checkManagerRunning(std::string& manager_name);
void dxlTorqueChecker();

// node main
int main(int argc, char **argv) {

  // init ROS
  ros::init(argc, argv, "jp_demo_node");

  // create ros wrapper object
  robotis_op::OPDemo *current_demo = NULL;
  robotis_op::CmdVel *cmd_vel = new robotis_op::CmdVel();

  ros::NodeHandle nh(ros::this_node::getName());

  // pub and sub
  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  // wait for starting of manager
  std::string manager_name = "/op3_manager";
  while (ros::ok()) {
    ros::Duration(1.0).sleep();

    if (checkManagerRunning(manager_name) == true) {
      break;
      ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
    }
    ROS_WARN("Waiting for op3 manager");
  }

  // turn on RGB LED
  setLED(0x01 | 0x02 | 0x04);

  //node loop
  while (ros::ok()) {
    // process
    if (apply_desired == true) {
      switch (desired_status)
        {
        case Ready: {
          if (current_demo != NULL)
            current_demo->setDemoDisable();
          current_demo = NULL;
          goInitPose();
          ROS_INFO_COND(DEBUG_PRINT, "[Go to Demo READY!]");
          break;
        }

        case CmdVel: {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = cmd_vel_node;
          current_demo->setDemoEnable();
          ROS_INFO_COND(DEBUG_PRINT, "[Start] Soccer Demo");
          break;
        }

        default: {
          break;
        }
      }

      apply_desired = false;
      current_status = desired_status;
    }

    // execute pending callbacks
    ros::spinOnce();

    // relax to fit output rate
    loop_rate.sleep();
  }

  return 0;
}

void goInitPose() {
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub.publish(init_msg);
}

void dxlTorqueChecker() {
  std_msgs::String check_msg;
  check_msg.data = "check";

  dxl_torque_pub.publish(check_msg);
}
