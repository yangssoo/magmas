// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_control_inrol/path_generator.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot.h>
#include "pseudo_inversion.h"

using namespace std;
using namespace Eigen;

double enter_time_;
double prior_time_;

namespace franka_control_inrol {

bool PathGenerator::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {

std::array<double, 7> lower_torque_thresholds_acceleration = {20.0,20.0,18.0,18.0,16.0,14.0,12.0};
std::array<double, 7> upper_torque_thresholds_acceleration = {20.0,20.0,18.0,18.0,16.0,14.0,12.0};
std::array<double, 7> lower_torque_thresholds_nominal = {20.0,20.0,18.0,18.0,16.0,14.0,12.0};
std::array<double, 7> upper_torque_thresholds_nominal = {20.0,20.0,18.0,18.0,16.0,14.0,12.0};
std::array<double, 6> lower_force_thresholds_acceleration = {20.0,20.0,20.0,25.0,25.0,25.0};
std::array<double, 6> upper_force_thresholds_acceleration = {20.0,20.0,20.0,25.0,25.0,25.0};
std::array<double, 6> lower_force_thresholds_nominal = {20.0,20.0,20.0,25.0,25.0,25.0};
std::array<double, 6> upper_force_thresholds_nominal = {20.0,20.0,20.0,25.0,25.0,25.0};

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &PathGenerator::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter radius, defaulting to: " << radius_);
  }
  if (std::fabs(radius_) < 0.005) {
    ROS_INFO_STREAM("JointImpedanceExampleController: Set radius to small, defaulting to: " << 0.1);
    radius_ = 0.1;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter vel_max, defaulting to: " << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter acceleration_time, defaulting to: "
        << acceleration_time_);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

/*
  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);
*/

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface =
      robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting cartesian pose interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(new franka_hw::FrankaCartesianPoseHandle(
        cartesian_pose_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting cartesian pose handle from interface: "
        << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  //torques_publisher_.init(node_handle, "torque_comparison", 1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}






void PathGenerator::starting(const ros::Time& time) {

  initial_pose_ = state_handle_->getRobotState().O_T_EE_d;

  PG_initial_time_ = time;
  initial_state_ = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state_.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state_.q.data());
  Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state_.O_T_EE.data()));

  position_d_ = initial_transform.translation();
  position_d_target_ = initial_transform.translation();
  orientation_d_ = Quaterniond(initial_transform.linear());
  orientation_d_target_ = Quaterniond(initial_transform.linear());

}





void PathGenerator::update(const ros::Time& time,
                                             const ros::Duration& period) {
  if (vel_current_ < vel_max_) {
    vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
  }
  vel_current_ = std::fmin(vel_current_, vel_max_);

  angle_ += period.toSec() * vel_current_ / std::fabs(radius_);
  if (angle_ > 2 * M_PI) {
    angle_ -= 2 * M_PI;
  }

cartesian_stiffness_ << 2000, 0, 0, 0, 0, 0,
0, 2000, 0, 0, 0, 0,
0, 0, 2000, 0, 0, 0,
0, 0, 0, 100, 0, 0,
0, 0, 0, 0, 100, 0,
0, 0, 0, 0, 0, 100;
cartesian_damping_ << 100, 0, 0, 0, 0, 0,
0, 100, 0, 0, 0, 0,
0, 0, 100, 0, 0, 0,
0, 0, 0, 10, 0, 0,
0, 0, 0, 0, 10, 0,
0, 0, 0, 0, 0, 10;

/*
  Affine3d target_transform(Translation3d(position_d_)*Matrix3d(orientation_d_));
  Matrix4d target_transform_matrix(target_transform.matrix());
*/


 

//test_sum = sqrt(test_sum);

/*
for (int i=0;i<16;i++)
{
	pose_desired[i] = target_transform_matrix(i);
}
*/
//test_sum = sqrt(test_sum);



//  pose_desired[13] += delta_y;
//  pose_desired[14] += delta_z;

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis_eig(coriolis.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state_.O_T_EE.data()));
  Eigen::Vector3d initial_position(initial_transform.translation());
  Eigen::Quaterniond initial_orientation(initial_transform.linear());
  
  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }



 Matrix4d target_transform_matrix;
  target_transform_matrix.setZero();
  target_transform_matrix.block(0, 0, 3, 3) = orientation_d_.toRotationMatrix();
  target_transform_matrix.block(0, 3, 3, 1) = position_d_;
//  target_transform_matrix.block(0, 0, 3, 3) = initial_orientation.toRotationMatrix();
//  target_transform_matrix.block(0, 3, 3, 1) = initial_position;
  target_transform_matrix(3, 3) = 1.0;

  double delta_y = radius_ * (1 - std::cos(angle_));
  double delta_z = radius_ * std::sin(angle_);

  //std::array<double, 16> pose_desired = initial_pose_;

//double tmp_matrix[16];
//double test_sum=0;
std::array<double, 16> pose_desired;


for (int i=0;i<4;i++)
{
	for (int j=0;j<4;j++)
	{
 		pose_desired[4*i+j] = target_transform_matrix(j,i);
	}
}
  state_handle_->setCommand(pose_desired);

//print
if (time.toSec() - enter_time_ > 0.5)
{
	enter_time_ = time.toSec();
	//cout << target_transform.matrix() <<
	cout << "===================" << endl;

//	cout << position.transpose() << endl;
//	cout << " " << endl;
	cout << position_d_.transpose() << endl;
	cout << " " << endl;
	cout << initial_position.transpose() << endl;
	cout << " " << endl;
	cout << orientation_d_.coeffs() << endl;
	cout << " " << endl;
	cout << initial_orientation.coeffs() << endl;
	cout << " " << endl;
//	cout << position_d_target_.transpose() << endl;
//	cout << " " << endl;

	//for (int i = 0; i < 7; i++)	cout << tau_d_saturated[i] <<"   "<<
	// endl;
	//cout << pose_desired[12] << " " << pose_desired[13] << " " << pose_desired[14] << " " <<endl;
	//cout << " " << endl;
/*
cout<< Eigen::Matrix4d::Map(robot_state.O_T_EE.data()) <<endl;
	cout << " " << endl;
cout<< target_transform_matrix2 << endl;
	cout << "!!!!!!!!!!!!!!!" << endl;
*/
}




//cartesian impedance control
  // position error
//  position_d_ << pose_desired[12],pose_desired[13],pose_desired[14];
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  orientation_d_ = initial_orientation;
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  for (int i=0; i<7; i++)  { q_d_nullspace_(i) =  robot_state.q_d[i]; }
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task  + coriolis_eig;// + tau_nullspace;
  std::array<double, 7> tau_d_calculated_car;
  for (size_t i = 0; i < 7; ++i) {
  tau_d_calculated_car[i] = tau_d(i);




}


//////////////////////////////////////////////////////
//joint impedance control
  std::array<double, 7> tau_d_calculated_joint;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated_joint[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                          d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
//  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);
std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated_car, robot_state.tau_J_d);


  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }



// position_d_ and orientation_d_ update
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  Eigen::AngleAxisd aa_orientation_d(orientation_d_);
  Eigen::AngleAxisd aa_orientation_d_target(orientation_d_target_);
  aa_orientation_d.axis() = filter_params_ * aa_orientation_d_target.axis() +
                            (1.0 - filter_params_) * aa_orientation_d.axis();
  aa_orientation_d.angle() = filter_params_ * aa_orientation_d_target.angle() +
                             (1.0 - filter_params_) * aa_orientation_d.angle();
  orientation_d_ = Eigen::Quaterniond(aa_orientation_d);

}


std::array<double, 7> PathGenerator::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}


void PathGenerator::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
} // orientation_d_target_ & position_d_target_ callback from subscriber equilibrium_pose
  // you can move equilibrium position and orientation from publishing geometry_msgs::PoseStampedConstPtr

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_control_inrol::PathGenerator,
                       controller_interface::ControllerBase)
