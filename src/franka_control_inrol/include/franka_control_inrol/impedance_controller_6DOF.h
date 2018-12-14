// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>

#include <franka_control_inrol/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include "franka_control_inrol/JointPkg.h" // msg 사용 전 compile 먼저!
#include "franka_control_inrol/TorquePkg.h"
#include "franka_control_inrol/Vector6.h"

namespace franka_control_inrol {

class ImpedanceController6DOF : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Vector3d ft_env_data_force_;
  Eigen::Vector3d ft_arm_data_force_;
  Eigen::Vector3d ft_env_data_torque_;
  Eigen::Vector3d ft_arm_data_torque_;
  Eigen::Vector3d des_vel_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_control_inrol::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_control_inrol::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::Subscriber sub_FT_data_env_;
  ros::Subscriber sub_FT_data_arm_;
  void FTArmDataCallback(const geometry_msgs::WrenchStampedPtr& msg);
  void FTEnvDataCallback(const geometry_msgs::WrenchStampedPtr& msg);

  ros::Subscriber sub_des_vel_;
  void desvelCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::Subscriber sub_gain_cmd_;
  void complianceParamCallback_DW(const geometry_msgs::PoseStampedConstPtr& msg);

  rosbag::Bag center_bag;


  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> car_pos_publisher_;
  realtime_tools::RealtimePublisher<franka_control_inrol::Vector6> dX_pub_;
  realtime_tools::RealtimePublisher<franka_control_inrol::JointPkg> joint_pkg_pub_;
  realtime_tools::RealtimePublisher<franka_control_inrol::TorquePkg> torque_pkg_pub_;

};

}  // namespace franka_example_controllers
