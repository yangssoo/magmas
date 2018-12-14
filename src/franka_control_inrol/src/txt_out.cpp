#include "ros/ros.h"
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sstream>
#include <iostream>
#include <ros/node_handle.h>
#include "franka_control_inrol/JointPkg.h" // msg 사용 전 compile 먼저!
#include "franka_control_inrol/TorquePkg.h"
#include "franka_control_inrol/Vector6.h"

using namespace std;
using namespace Eigen;

Eigen::Vector3d position_d_target_;
Eigen::Quaterniond orientation_d_target_;
Eigen::Vector3d ft_env_data_force_;
Eigen::Vector3d ft_arm_data_force_;
Eigen::Vector3d ft_env_data_torque_;
Eigen::Vector3d ft_arm_data_torque_;
Eigen::Vector3d des_vel_;
Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  double nullspace_stiffness_target_{20.0};


ros::Subscriber sub_equilibrium_pose_;
void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

ros::Subscriber sub_FT_data_arm_;
void FTArmDataCallback(const geometry_msgs::WrenchStampedPtr& msg);

ros::Subscriber sub_FT_data_env_;
void FTEnvDataCallback(const geometry_msgs::WrenchStampedPtr& msg);

ros::Subscriber sub_des_vel_;
void desvelCallback(const geometry_msgs::PoseStampedConstPtr& msg);

ros::Subscriber sub_gain_cmd_;
void complianceParamCallback_DW(const geometry_msgs::PoseStampedConstPtr& msg);

ros::Subscriber sub_joint_pkg_;
void JointPkgCallback_txtout(const franka_control_inrol::JointPkg& msg);

ros::Subscriber sub_torque_pkg_;
void TorquePkgCallback_txtout(const franka_control_inrol::TorquePkg& msg);


int main(int argc, char **argv)
{

    ros::init(argc, argv, "txt_out");
    ros::NodeHandle n;



    return 0;
}

void JointPkgCallback_txtout(
	const franka_control_inrol::JointPkg& msg) {
	  //cur_car_position_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
	  //cur_car_orientation_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
	  //cur_car_R_ = Matrix3d(cur_car_orientation_);
	//cout << cur_car_position_.transpose() << endl;
}

void TorquePkgCallback_txtout(
	const franka_control_inrol::TorquePkg& msg) {
	  //cur_car_position_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
	  //cur_car_orientation_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
	  //cur_car_R_ = Matrix3d(cur_car_orientation_);
	//cout << cur_car_position_.transpose() << endl;
}

void complianceParamCallback_DW(const geometry_msgs::PoseStampedConstPtr& msg)
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


void equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void FTArmDataCallback(
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

void FTEnvDataCallback(
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


void desvelCallback(
    const geometry_msgs::PoseStampedConstPtr& msg){
      des_vel_ << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;

}
