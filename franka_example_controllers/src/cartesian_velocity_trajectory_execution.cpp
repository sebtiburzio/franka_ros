// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_velocity_trajectory_execution.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool CartesianVelocityTrajectoryExecution::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianVelocityExampleController: Robot is not in the expected starting position "
            "for running this example. Run `roslaunch franka_example_controllers "
            "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
            "first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  time_max = 4.0;
  constexpr double kTimeStep = 0.001;          // [s]
  angle = M_PI / 4.0;
  double v_max = 0.05;
  double v = 0;  // [rad/s]
  double t = 0;  // [s]
  double cycle;
  while (t < time_max) {
    cycle = std::floor(pow(-1.0, (t - std::fmod(t, time_max)) / time_max));
    v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * t));
    trajectory.push_back(v);
    t += kTimeStep;
  }

  return true;
}

void CartesianVelocityTrajectoryExecution::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  index = 0;
}

void CartesianVelocityTrajectoryExecution::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;
  if (elapsed_time_.toSec() < time_max) {
    
    index += period.toNSec() / 1000000;
    if (index >= trajectory.size()) {
      index = trajectory.size() - 1;
    }

    double v_x = std::cos(angle) * trajectory[index];
    double v_z = -std::sin(angle) * trajectory[index];
    if (index % 50 == 0) {
      ROS_INFO_STREAM("index: " << index << "\t v_x: " << v_x << "\t v_z:" << v_z);
    }
    std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
    velocity_cartesian_handle_->setCommand(command);

  } else {
    std::array<double, 6> command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    velocity_cartesian_handle_->setCommand(command);  
  }
}

void CartesianVelocityTrajectoryExecution::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityTrajectoryExecution,
                       controller_interface::ControllerBase)
