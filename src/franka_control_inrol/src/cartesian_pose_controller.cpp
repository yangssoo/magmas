// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#define PI 3.141592

#include <franka_control_inrol/cartesian_pose_controller.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <fstream>

using namespace std;
using namespace Eigen;

namespace franka_control_inrol {

ofstream cc_car_data("/home/inrol/research/DWSon/tool_identification/data/car_data.txt");
ofstream cc_FT_arm_data("/home/inrol/research/DWSon/tool_identification/data/FT_arm_data.txt");
ofstream cc_torque_data("/home/inrol/research/DWSon/tool_identification/data/torque_data.txt");
ofstream cc_joint_data("/home/inrol/research/DWSon/tool_identification/data/joint_data.txt");

using namespace franka_control_inrol;

//DW function

array<double,16> eig2array(Matrix4d eig);

double ent_time_;

bool CartesianPoseController::init(hardware_interface::RobotHW* robot_hardware,
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
    cartesian_pose_handle_.reset(new franka_hw::FrankaCartesianPoseHandle(
        cartesian_pose_interface_->getHandle(arm_id + "_robot")));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  // subscriber
  sub_FT_data_ = node_handle.subscribe(
      "/ft_sensor/netft_data", 20, &CartesianPoseController::FTDataCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());


  franka_hw::FrankaModelInterface* model_interface =
      robot_hardware->get<franka_hw::FrankaModelInterface>();
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



  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
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

  return true;
}

void CartesianPoseController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
  ent_time_ = elapsed_time_.toSec();
}



array<double,16> eig2array(Matrix4d eig)
{
  array<double,16> result;
  for (int i=0;i<16;i++)  result[i] = eig(i);
  return result;
} // end fun eig2array

double smooth(double time, double transition_interval , double des_vel)
{
  double result;

  if (time <= transition_interval)
  {
    result = 0.5*des_vel*(time-transition_interval/PI*sin(time*PI/transition_interval));
  }
  else
  {
    result = des_vel * (time-transition_interval) + 0.5*des_vel*transition_interval ;
  }
  

  return result;
}



void CartesianPoseController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  Map<Matrix<double, 4, 4> > O_T_EE_d_eig(initial_pose_.data());
  double radius = 0.3;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);

  Map<Matrix<double, 4, 4> > initial_pose_eig(initial_pose_.data());
  Affine3d initial_pose_eig_affine(initial_pose_eig);

  Affine3d T_d_eig_affine;
  T_d_eig_affine.translation() = initial_pose_eig_affine.translation();

  //T_d_eig_affine.linear() = Matrix3d(AngleAxisd(0.5*(1-cos(elapsed_time_.toSec()*1.0)),Vector3d::UnitY()))*initial_pose_eig_affine.linear();
  //T_d_eig_affine.linear() = Matrix3d(AngleAxisd(smooth(elapsed_time_.toSec(),2,0.1),Vector3d::UnitY()))*initial_pose_eig_affine.linear();
  //T_d_eig_affine.linear() = Matrix3d(AngleAxisd(smooth(elapsed_time_.toSec(),2,0.1),Vector3d::UnitX()))*initial_pose_eig_affine.linear();
  T_d_eig_affine.linear() = Matrix3d(AngleAxisd(smooth(elapsed_time_.toSec(),2,0.1),Vector3d::UnitZ()))*initial_pose_eig_affine.linear();

//  Matrix4d T_d_eig = initial_pose_eig;
  array<double, 16> new_pose = eig2array(T_d_eig_affine.matrix());
//  Map<Matrix<double, 4, 4> > T_d_eig(new_pose.data());
//  array<double, 16> T_d_array = eig2array(T_d_eig);
//  new_pose[12] -= delta_x;
//  new_pose[14] -= delta_z;

  cartesian_pose_handle_->setCommand(new_pose);


franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();

Map<Matrix<double, 7,1>> q(robot_state.q.data());
Map<Matrix<double, 7,1>> dq(robot_state.dq.data());
std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

Affine3d transform(Matrix4d::Map(robot_state.O_T_EE.data()));
Vector3d position(transform.translation());
Quaterniond orientation(transform.linear());
Vector3d euler_mea = orientation.toRotationMatrix().eulerAngles(2, 1, 0);
Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());



if (elapsed_time_.toSec() - en_time_txt_ > 0.02)
{
en_time_txt_ = elapsed_time_.toSec();
cc_car_data <<elapsed_time_.toSec() << "     " <<
	position.transpose() << "     " <<
	euler_mea.transpose() << "   " <<
	(jacobian * dq).transpose() << "     " << // mea dX
  //   position_d_.transpose() << "     " <<
	// euler_des.transpose() << "     " <<
	// position_d_target_.transpose() << "     " <<
	// euler_des_target.transpose() << "     " <<
	endl;

cc_FT_arm_data << elapsed_time_.toSec() << "     " <<
	ft_data_force_.transpose() << "        " <<
	ft_data_torque_.transpose() << "        " <<
	endl;

cc_joint_data << elapsed_time_.toSec() << "      " <<
	q.transpose() << "     " <<
	dq.transpose() << "     " <<
	endl;

// cc_torque_data << time << "    " <<
// 	tau_J.transpose() << "      " << // measured // 7 (1-7)
// 	tau_J_d.transpose() << "      " << // desired - prior input // 7 (8-14)
// 	tau_d.transpose() << "      " << // desired - calculated //7 (15-21)
// 	tau_task.transpose() << "      " << // J.transpose() * error PD //7 (22-28)
// 	gravity.transpose() << "     " << // gravity term // 7 (29-35)
// 	coriolis.transpose() << "      " << //7
// 	tau_nullspace.transpose() << "      " <<  //7
// 	endl;
} //  end if (time.toSec() - en_time_txt_ > 0.02)



  if (elapsed_time_.toSec() - ent_time_ > 0.5 )
    {
     ent_time_ = elapsed_time_.toSec();
     cout << T_d_eig_affine.matrix() << endl;
    }
} // end update


void CartesianPoseController::FTDataCallback(
	const geometry_msgs::WrenchStampedPtr& msg) {
       ft_data_force_ << msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z;
       ft_data_torque_ << msg->wrench.torque.x,msg->wrench.torque.y,msg->wrench.torque.z;
      if (ft_data_force_.norm()>55 && ft_data_force_.norm()<65)
	{ cout << "force over 55 N" << endl; }
      else if (ft_data_force_.norm()>=65)
	{cout << "force over 65 N!!!!!!!!" << endl; }
      if (ft_data_torque_.norm()>3 && ft_data_torque_.norm()<4)
	{ cout << "torque over 3" << endl; }
      else if (ft_data_torque_.norm()>=4)
	{ cout << "torque over 4!!!!!!!!!" << endl; }
}



}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_control_inrol::CartesianPoseController,
                       controller_interface::ControllerBase)
