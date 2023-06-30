// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_example_controller_pulse.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace franka_example_controllers {

bool CartesianPoseExampleControllerPulse::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -0.7, 0, -1.9 , 0, 1.2, 0}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  std::string line;
  std::string traj_file_path = ros::package::getPath("franka_example_controllers") + "/config/traj.csv";
  std::ifstream traj_file(traj_file_path);
  while(getline(traj_file,line))
  {
    std::istringstream lineStream(line);
    double x, z, phi;
    lineStream >> x >> z >> phi;
    traj_x.push_back(x);
    traj_z.push_back(z);
    traj_phi.push_back(phi);
  }

  return true;
}

void CartesianPoseExampleControllerPulse::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianPoseExampleControllerPulse::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  index += period.toNSec() / 1000000;
  if (index >= traj_x.size()) {
    index = traj_x.size() - 1;
  }

  std::array<double, 16> new_pose = initial_pose_;
  new_pose[12] -= traj_x[index];
  new_pose[14] -= traj_z[index];
  
  if (index % 50 == 0) {
    ROS_INFO_STREAM("index: " << index << "\t x: " << traj_x[index] << "\t z:" << traj_z[index] << "\t phi:" << traj_phi[index]);
    ROS_INFO_STREAM("index: " << index << "\t x: " << new_pose[12] << "\t z:" << new_pose[14]);
  }

  cartesian_pose_handle_->setCommand(new_pose);
  
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleControllerPulse,
                       controller_interface::ControllerBase)
