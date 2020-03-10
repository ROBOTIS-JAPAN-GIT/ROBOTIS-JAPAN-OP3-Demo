#include <ros/ros.h>
#include <std_msgs/String.h>

#include "jp_op3_demo/cmd_vel_walk.h"
// #include "robotis_math/robotis_linear_algebra.h"
// #include "robotis_controller_msgs/SyncWriteItem.h"

bool checkManagerRunning(std::string& manager_name); // prototype declaration

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

int main(int argc, char **argv) {

  ros::init(argc, argv, "cmd_vel_walk");

  // robotis_op::CmdVelWalk *cmd_vel_walk = new robotis_op::CmdVelWalk();
  robotis_op::CmdVelWalk *cmd_vel_walk = new robotis_op::CmdVelWalk();

  // necessary...?
  ros::NodeHandle nh(ros::this_node::getName());

  ros::start();
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

  while (ros::ok()) {

    // dxlTorqueChecker();

    ros::spinOnce();
    loop_rate.sleep();

  } // while

} // main

bool checkManagerRunning(std::string& manager_name) {
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++) {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }

  ROS_ERROR("Can't find op3_manager");
  return false;
}
