// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_control_inrol/impedance_controller_6DOF.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <fstream>

#include "pseudo_inversion.h"


using namespace Eigen;
using namespace std;

double en_time_;
double en_time_txt_;
double en_time_center_;

namespace franka_control_inrol {

ofstream car_data("/home/inrol/research/DWSon/peg_in_hole/data/car_data.txt");
ofstream FT_arm_data("/home/inrol/research/DWSon/peg_in_hole/data/FT_arm_data.txt");
ofstream FT_env_data("/home/inrol/research/DWSon/peg_in_hole/data/FT_env_data.txt");
ofstream torque_data("/home/inrol/research/DWSon/peg_in_hole/data/torque_data.txt");
ofstream joint_data("/home/inrol/research/DWSon/peg_in_hole/data/joint_data.txt");
ofstream center_data("/home/inrol/research/DWSon/peg_in_hole/data/center_data.txt");
ofstream gain_data("/home/inrol/research/DWSon/peg_in_hole/data/gain_data.txt");

bool ImpedanceController6DOF::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &ImpedanceController6DOF::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_des_vel_ = node_handle.subscribe(
      "/des_vel", 20, &ImpedanceController6DOF::desvelCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_FT_data_arm_ = node_handle.subscribe(
      "/ft_sensor/FT_arm", 20, &ImpedanceController6DOF::FTArmDataCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_FT_data_env_ = node_handle.subscribe(
      "/ft_sensor/FT_env", 20, &ImpedanceController6DOF::FTEnvDataCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());


  sub_gain_cmd_ = node_handle.subscribe(
      "/gain_cmd", 20, &ImpedanceController6DOF::complianceParamCallback_DW, this,
      ros::TransportHints().reliable().tcpNoDelay());


  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  car_pos_publisher_.init(node_handle, "car_pos_pub", 1);
  dX_pub_.init(node_handle, "dX_pub",1);
  joint_pkg_pub_.init(node_handle, "joint_pkg_pub",1);
  torque_pkg_pub_.init(node_handle, "torque_pkg_pub",1);


  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_.reset(
      new dynamic_reconfigure::Server<franka_control_inrol::compliance_paramConfig>(
          dynamic_reconfigure_compliance_param_node_));
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&ImpedanceController6DOF::complianceParamCallback, this, _1, _2));




  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void ImpedanceController6DOF::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;

  // ros bag open
//center_bag.open("/center.bag", rosbag::bagmode::Write);
}

void ImpedanceController6DOF::update(const ros::Time& time,
                                                 const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J(robot_state.tau_J.data()); // measured
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  VectorXd des_car_vel(6);
  des_car_vel.head(3) = des_vel_;
  des_car_vel.tail(3) << 0,0,0;

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  // Cartesian PD control with damping ratio = 1

  // calibration test ////////////////////////
//   cartesian_stiffness_.setZero();
//   cartesian_stiffness_(2, 2) = 1000.0;
//   cartesian_damping_.setZero();
  ////////////////////////////////////////////


  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq - des_car_vel));
  // nullspace PD control with damping ratio = 1
/*
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
*/
  Matrix<double, 7, 1> q_null_des;
  //q_null_des = q;
  //q_null_des(2,0) = 0;
  q_null_des = q_null_des.setZero();
  nullspace_stiffness_ = 100;
  double joint3_K = 100;
  double joint3_B = 20;
//   tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
//                     jacobian.transpose() * jacobian_transpose_pinv) *
//                        (nullspace_stiffness_ * (q_null_des - q)-
// 			(1.7 * sqrt(nullspace_stiffness_)) * dq);

  // Desired torque
  tau_d << tau_task  + coriolis; // + tau_nullspace;
  //tau_d << tau_task;//  + coriolis; // + tau_nullspace;
  tau_d(2) = -joint3_K*q(2)-joint3_B*dq(2); // fix 3 joint
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  //publish
  car_pos_publisher_.msg_.pose.position.x = position(0);
  car_pos_publisher_.msg_.pose.position.y = position(1);
  car_pos_publisher_.msg_.pose.position.z = position(2);
  car_pos_publisher_.msg_.pose.orientation.x = orientation.x();
  car_pos_publisher_.msg_.pose.orientation.y = orientation.y();
  car_pos_publisher_.msg_.pose.orientation.z = orientation.z();
  car_pos_publisher_.msg_.pose.orientation.w = orientation.w();

for (size_t i = 0; i < 7; i++)
{
  dX_pub_.msg_.v[i] = (jacobian * dq)(i);

  joint_pkg_pub_.msg_.q[i] = q[i];
  joint_pkg_pub_.msg_.dq[i] = dq[i];

  torque_pkg_pub_.msg_.tau_J[i] = tau_J[i];
  torque_pkg_pub_.msg_.tau_J_d[i] = tau_J_d[i];
  torque_pkg_pub_.msg_.tau_d[i] = tau_d[i];
  torque_pkg_pub_.msg_.tau_task[i] = tau_task[i];
  torque_pkg_pub_.msg_.gravity[i] = gravity[i];
  torque_pkg_pub_.msg_.coriolis[i] = coriolis[i];
  torque_pkg_pub_.msg_.tau_nullspace[i] = tau_nullspace[i];
}
  car_pos_publisher_.unlockAndPublish();
  dX_pub_.unlockAndPublish();
  joint_pkg_pub_.unlockAndPublish();
  torque_pkg_pub_.unlockAndPublish();

  

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  Eigen::AngleAxisd aa_orientation_d(orientation_d_);
  Eigen::AngleAxisd aa_orientation_d_target(orientation_d_target_);
  aa_orientation_d.axis() = filter_params_ * aa_orientation_d_target.axis() +
                            (1.0 - filter_params_) * aa_orientation_d.axis();
  aa_orientation_d.angle() = filter_params_ * aa_orientation_d_target.angle() +
                             (1.0 - filter_params_) * aa_orientation_d.angle();
  orientation_d_ = Eigen::Quaterniond(aa_orientation_d);

//print

Vector3d ft_force_global = Matrix3d(orientation) * ft_arm_data_force_;
Vector3d ft_torque_global = Matrix3d(orientation) * ft_arm_data_torque_;
Vector3d euler_mea = orientation.toRotationMatrix().eulerAngles(2, 1, 0);
Vector3d euler_des = orientation_d_.toRotationMatrix().eulerAngles(2, 1, 0);
Vector3d euler_des_target = orientation_d_target_.toRotationMatrix().eulerAngles(2, 1, 0);

if (time.toSec() - en_time_ > 0.5)
{
	en_time_ = time.toSec();
	cout <<
	"====================================================" << endl <<
	"            pos : " << position.transpose() << endl <<
        "    orientation : " << euler_mea.transpose() << endl <<
        "  ft_data_force : " << ft_force_global.transpose() << endl <<
        " ft_data_torque : " << ft_torque_global.transpose() << endl;
       // " R : " << orientation.toRotationMatrix() << endl ;
//	" car_stiffness  : " << endl << cartesian_stiffness_ << endl;
        
} // end if (time.toSec() - en_time_ > 0.5)


if(position(2) < 0.080)
{
    if((time.toSec() - en_time_center_) > 0.7)
    {
        en_time_center_ = time.toSec();
    franka_control_inrol::center_data << time << "   " << 
    position.transpose() << "     " << // 1 - 3
    euler_mea.transpose() << "   " << // 4 - 6
    endl;

    }
}




//ofstream txt out!!
if (position(2) < 0.17)
{
    if (time.toSec() - en_time_txt_ > 0.004)
    {
        en_time_txt_ = time.toSec();
        franka_control_inrol::car_data.precision(17);
        franka_control_inrol::car_data <<time << "     " <<
            position.transpose() << "     " << // 1 - 3
            euler_mea.transpose() << "   " << // 4 - 6
            (jacobian * dq).transpose() << "     " << // mea dX // 7 - 12
            position_d_.transpose() << "     " << // 13 - 15
            euler_des.transpose() << "     " << // 16 - 18
            position_d_target_.transpose() << "     " << // 19 - 21
            euler_des_target.transpose() << "     " << // 22 - 24
            endl;

        franka_control_inrol::FT_arm_data.precision(17);
        franka_control_inrol::FT_arm_data << time << "     " <<
            ft_arm_data_force_.transpose() << "        " <<
            ft_arm_data_torque_.transpose() << "        " <<
            endl;

        franka_control_inrol::FT_env_data.precision(17);
        franka_control_inrol::FT_env_data << time << "     " <<
            ft_env_data_force_.transpose() << "        " <<
            ft_env_data_torque_.transpose() << "        " <<
            endl;

        franka_control_inrol::joint_data.precision(17);
        franka_control_inrol::joint_data << time << "      " <<
            q.transpose() << "     " <<
            dq.transpose() << "     " <<
            endl;

        franka_control_inrol::torque_data.precision(17);
        franka_control_inrol::torque_data << time << "    " <<
            tau_J.transpose() << "      " << // measured // 7 (1-7)
            tau_J_d.transpose() << "      " << // desired - prior input // 7 (8-14)
            tau_d.transpose() << "      " << // desired - calculated //7 (15-21)
            tau_task.transpose() << "      " << // J.transpose() * error PD //7 (22-28)
            gravity.transpose() << "     " << // gravity term // 7 (29-35)
            coriolis.transpose() << "      " << //7
            tau_nullspace.transpose() << "      " <<  //7
            endl;

        franka_control_inrol::gain_data.precision(17);
        franka_control_inrol::gain_data << time << "    " <<
            cartesian_stiffness_(0,0)<< "      " <<
            cartesian_stiffness_(1,1)<< "      " <<
            cartesian_stiffness_(2,2)<< "      " <<
            cartesian_stiffness_(3,3)<< "      " <<
            cartesian_stiffness_(4,4)<< "      " <<
            cartesian_stiffness_(5,5)<< "      " <<
            cartesian_stiffness_(5,5)<< "      " <<
            cartesian_damping_(0,0) <<  "      " <<
            cartesian_damping_(1,1)<<  "      " <<
            cartesian_damping_(2,2)<<  "      " <<
            cartesian_damping_(3,3)<<  "      " <<
            cartesian_damping_(4,4)<<  "      " <<
            cartesian_damping_(5,5)<<  "      " <<
            joint3_K <<   "      " <<
            joint3_B <<   "      " <<
            endl;

    } //  end if (time.toSec() - en_time_txt_ > 0.02)
} // if z

}//end update






Eigen::Matrix<double, 7, 1> ImpedanceController6DOF::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}






void ImpedanceController6DOF::complianceParamCallback(
    franka_control_inrol::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_(5,5) = 1; // orientation z damping
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}


void ImpedanceController6DOF::complianceParamCallback_DW(const geometry_msgs::PoseStampedConstPtr& msg)
{
  cartesian_stiffness_target_.topLeftCorner(3, 3).diagonal() << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
  cartesian_stiffness_target_.bottomRightCorner(3, 3).diagonal() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3).diagonal()
      << 2.0 * sqrt(msg->pose.position.x),2.0 * sqrt(msg->pose.position.y),2.0 * sqrt(msg->pose.position.z) ;
  cartesian_damping_target_.bottomRightCorner(3, 3).diagonal()
      << 2.0 * sqrt(msg->pose.orientation.x),2.0 * sqrt(msg->pose.orientation.y),2.0 * sqrt(msg->pose.orientation.z) ;
  cartesian_damping_target_(5,5) = 1; // orientation z damping
  nullspace_stiffness_target_ = msg->pose.orientation.w;
}




void ImpedanceController6DOF::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void ImpedanceController6DOF::FTArmDataCallback(
	const geometry_msgs::WrenchStampedPtr& msg) {
       ft_arm_data_force_ << msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z;
       ft_arm_data_torque_ << msg->wrench.torque.x,msg->wrench.torque.y,msg->wrench.torque.z;
      if (ft_arm_data_force_.norm()>55 && ft_arm_data_force_.norm()<65)
	{ cout << "force over 55 N" << endl; }
      else if (ft_arm_data_force_.norm()>=65)
	{cout << "force over 65 N!!!!!!!!" << endl; }
      if (ft_arm_data_torque_.norm()>3 && ft_arm_data_torque_.norm()<4)
	{ cout << "torque over 3" << endl; }
      else if (ft_arm_data_torque_.norm()>=4)
	{ cout << "torque over 4!!!!!!!!!" << endl; }
}

void ImpedanceController6DOF::FTEnvDataCallback(
	const geometry_msgs::WrenchStampedPtr& msg) {
       ft_env_data_force_ << msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z;
       ft_env_data_torque_ << msg->wrench.torque.x,msg->wrench.torque.y,msg->wrench.torque.z;
      if (ft_env_data_force_.norm()>55 && ft_env_data_force_.norm()<65)
	{ cout << "force over 55 N" << endl; }
      else if (ft_env_data_force_.norm()>=65)
	{cout << "force over 65 N!!!!!!!!" << endl; }
      if (ft_env_data_torque_.norm()>3 && ft_env_data_torque_.norm()<4)
	{ cout << "torque over 3" << endl; }
      else if (ft_env_data_torque_.norm()>=4)
	{ cout << "torque over 4!!!!!!!!!" << endl; }
}


void ImpedanceController6DOF::desvelCallback(
    const geometry_msgs::PoseStampedConstPtr& msg){
      des_vel_ << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;

}


}  // namespace franka_control_inrol

PLUGINLIB_EXPORT_CLASS(franka_control_inrol::ImpedanceController6DOF,
                       controller_interface::ControllerBase)
