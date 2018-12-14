#include "ros/ros.h"
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <iostream>
#include <termios.h>
#include <ros/node_handle.h>
#include <franka_control_inrol/compliance_paramConfig.h>
#include <rosbag/bag.h>

using namespace std;
using namespace Eigen;

# define PI 3.14159265


// function prototypes for kbhit()
static struct termios initial_settings, new_settings;
static int peek_character = -1;
int _kbhit();
int _getch();
void init_keyboard();
void close_keyboard();

// publisher & subcriber
ros::Publisher poseStampedPub_;
ros::Publisher velStampedPub_;
ros::Publisher gainPub_;
ros::Subscriber posetampedSub_;
void PeginHoleStatesubCallback(const geometry_msgs::PoseStampedConstPtr& msg);
Vector3d cur_car_position_(0,0,0);
Quaterniond cur_car_orientation_;
Matrix3d cur_car_R_;

// DW functions
void keyboardmode(Vector3d initial_position, Matrix3d initial_R);
void rot_gen(Vector3d position, Matrix3d start_R, Matrix3d end_R, double des_vel_mag);
void set_gain(double pos_x,double pos_y, double pos_z, double ori_x, double ori_y, double ori_z, double null);
void pub_posestamp(Vector3d position, Matrix3d R);



void keyboardmode(Vector3d initial_position, Matrix3d initial_R)
{

   init_keyboard();
   Vector3d position_des(initial_position);
   Matrix3d rot_mat_des(initial_R);
   Quaterniond des_orientation;
   geometry_msgs::PoseStamped poseStamped;
   double position_step = 0.0005;
   double ang_step = 0.5/180.0*PI;
   int while_on = 1; // termination variable

   while (ros::ok() && while_on)
{
  ros::spinOnce();
   if (_kbhit()) 
   {
	    switch(_getch()) {
		case 97: // a => +x
		position_des(0) += position_step;
		    break;
		case 100: // d => -x
		position_des(0) -= position_step;
		    break;
		case 115: // s => -z
		position_des(2) -= position_step;
		    break;
		case 119: // w => +z
		position_des(2) += position_step;
		    break;
		case 113: // q => +y
		position_des(1) += position_step;
		    break;
		case 101: // e => -y
		position_des(1) -= position_step;
		    break;
		case 122: // z => y axis rotate
		rot_mat_des = rot_mat_des * AngleAxisd(ang_step,Vector3d::UnitY());
		    break;
		case 120: // x => -y axis rotate
		rot_mat_des = rot_mat_des * AngleAxisd(-ang_step,Vector3d::UnitY());
		    break;
		case 116: // t => terminate
		while_on = 0;
		    break;
		case 103: // g => ground, set zero to current position
		position_des = cur_car_position_;
		des_orientation = cur_car_orientation_;
		    break;
	    }
	   //std::cout<<"beep :"<< _getch() << std::endl;
   }

   poseStamped.pose.position.x = position_des(0);
   poseStamped.pose.position.y = position_des(1);
   poseStamped.pose.position.z = position_des(2);

   des_orientation = Quaterniond(rot_mat_des);
   poseStamped.pose.orientation.x = des_orientation.x();
   poseStamped.pose.orientation.y = des_orientation.y();
   poseStamped.pose.orientation.z = des_orientation.z();
   poseStamped.pose.orientation.w = des_orientation.w();

   poseStampedPub_.publish(poseStamped);
//   ros::spinOnce();
}
}




void rot_gen(Vector3d position, Matrix3d start_R, Matrix3d end_R, double des_vel_mag)
{

   double des_vel_mag_deg = des_vel_mag /180.0*PI;
   Vector3d position_des(position);
   Matrix3d rot_mat_des(start_R);
   Quaterniond des_orientation;
   geometry_msgs::PoseStamped poseStamped;

   Matrix3d transition_R(start_R);

   double cur_time= ros::Time::now().toSec();
   double ent_time= ros::Time::now().toSec();
   double str_time= ros::Time::now().toSec();

   Matrix3d test_rot;

   while(ros::ok())
   {
     ros::spinOnce();
     cur_time = ros::Time::now().toSec();
     transition_R = end_R * start_R.transpose();

     Vector3d basis_axis = AngleAxisd(transition_R).axis();
     double total_angle = AngleAxisd(transition_R).angle();
     double dangle = (cur_time-str_time)*des_vel_mag_deg;

     rot_mat_des = AngleAxisd(dangle, basis_axis)*start_R;
     des_orientation = Quaterniond(rot_mat_des);

     if (cur_time - ent_time > 0.5)
	{
		ent_time = ros::Time::now().toSec();	
		cout <<
		"=========================================" << endl <<
		"rot axis : " << basis_axis.transpose() << endl <<
		"rot angl : " << dangle*180/PI << " deg / " << total_angle*180/PI << " deg " << endl;
	}

     if (abs(total_angle - dangle) < 0.0001)
	{
		
		cout << "motion end" << endl;
		rot_mat_des = end_R;

	   position_des = position;
	   poseStamped.pose.position.x = position_des(0);
	   poseStamped.pose.position.y = position_des(1);
	   poseStamped.pose.position.z = position_des(2);

	   des_orientation = Quaterniond(rot_mat_des);
	   poseStamped.pose.orientation.x = des_orientation.x();
	   poseStamped.pose.orientation.y = des_orientation.y();
	   poseStamped.pose.orientation.z = des_orientation.z();
	   poseStamped.pose.orientation.w = des_orientation.w();

	   poseStampedPub_.publish(poseStamped);

		break;
	}

   position_des = position;
   poseStamped.pose.position.x = position_des(0);
   poseStamped.pose.position.y = position_des(1);
   poseStamped.pose.position.z = position_des(2);

   des_orientation = Quaterniond(rot_mat_des);
   poseStamped.pose.orientation.x = des_orientation.x();
   poseStamped.pose.orientation.y = des_orientation.y();
   poseStamped.pose.orientation.z = des_orientation.z();
   poseStamped.pose.orientation.w = des_orientation.w();

   poseStampedPub_.publish(poseStamped);

   } // end while

} // end rot_gen






void line_gen(Vector3d start_position, Matrix3d start_R, Vector3d end_position,Matrix3d end_R, double des_vel_mag)
{
   ros::Time start_time = ros::Time::now();
   ros::Time cur_time;
   ros::Time prior_time;
   double time_enter_print = start_time.toSec();
   double dt;
   double time_from_start;
   Vector3d prior_position_des;
   Vector3d position_des(start_position);
   Matrix3d rot_mat_des(start_R);
   geometry_msgs::PoseStamped poseStamped;
   geometry_msgs::PoseStamped velStamped;
   double transition_angle=0;
   double transition_target_angle=0;
   double position_switch = 1;
   double orientation_switch = 1;

  while (position_switch == 1 || orientation_switch == 1 )
  {
    if (ros::ok() == false)
{
 close_keyboard();
 break;
}

//position transition
   prior_time = cur_time;
   cur_time = ros::Time::now();
   dt = cur_time.toSec() - prior_time.toSec();
   time_from_start = (cur_time.toSec() - start_time.toSec());

   prior_position_des = position_des;
   Vector3d target_direction = (end_position - start_position);
   target_direction = target_direction/target_direction.norm();

//angle transition
   Matrix3d transition_R = end_R* start_R.transpose() ;
   AngleAxisd transition_angleaxis(transition_R);
   Vector3d basis_axis;
   if (transition_angleaxis.angle() >= PI)
	{
		basis_axis = -transition_angleaxis.axis();
		transition_target_angle =PI - transition_angleaxis.angle(); 
	}
   else
	{
		basis_axis = transition_angleaxis.axis();
		transition_target_angle =transition_angleaxis.angle(); 
	}
 
   double ang_vel_coeff = 4; //ratio by vel_mag
   double ang_vel = des_vel_mag *ang_vel_coeff;
   double result_ang_vel = ang_vel;
   double result_pos_vel = des_vel_mag;
//cout << ang_duration << "   " << pos_duration<< endl;

/*
// delete because of error
//position or angle transition => which one is longer?
double ang_duration = abs(transition_target_angle) / ang_vel;
double pos_duration = (end_position - start_position).norm() / des_vel_mag;

if (ang_duration > pos_duration)
	{
		if (ang_duration < 0.01 || pos_duration < 0.01)
		{
		  cout << "im here!!" << endl;
		  result_ang_vel = ang_vel;
		  result_pos_vel= 0.01;
		}
		else
		{
			result_ang_vel=ang_vel;
			result_pos_vel=(end_position - start_position).norm() / ang_duration;
		}
	}
else
	{
		if (ang_duration < 0.01 || pos_duration < 0.01)
		{
		  result_ang_vel = 0.1;
		  result_pos_vel=des_vel_mag;
		}
		else
		{
		  result_ang_vel=transition_target_angle / pos_duration;
		  result_pos_vel=des_vel_mag;
		}
	}
*/

//position trajectory
   if (position_switch == 1){
   	position_des = start_position + target_direction * result_pos_vel *time_from_start ;
	   if ((position_des - end_position).norm()<0.00005){
		position_switch = 0;
	   }
   }
   else position_des = end_position;

   poseStamped.pose.position.x = position_des(0);
   poseStamped.pose.position.y = position_des(1);
   poseStamped.pose.position.z = position_des(2);


//orientation trajectory
   if (orientation_switch == 1){
           transition_angle = result_ang_vel*time_from_start;
	   rot_mat_des =  AngleAxisd(transition_angle,basis_axis)*start_R;
	   if (abs(transition_angle-transition_target_angle)<0.0001){
	   	orientation_switch = 0;
	   }
   }
   else {
 	rot_mat_des = end_R;
   }

   Quaterniond des_orientation(rot_mat_des);
   poseStamped.pose.orientation.x = des_orientation.x();
   poseStamped.pose.orientation.y = des_orientation.y();
   poseStamped.pose.orientation.z = des_orientation.z();
   poseStamped.pose.orientation.w = des_orientation.w();

   velStamped.pose.position.x = (position_des(0) - prior_position_des(0))/dt;
   velStamped.pose.position.y = (position_des(1) - prior_position_des(1))/dt;
   velStamped.pose.position.z = (position_des(2) - prior_position_des(2))/dt;

// safty => target is too far from cur / stop

  // orientation error
/*  if (des_orientation.coeffs().dot(cur_car_orientation_.coeffs()) < 0.0) {
    des_orientation.coeffs() << -des_orientation.coeffs();
  }
*/
  // "difference" quaternion
  Quaterniond error_quaternion(des_orientation * cur_car_orientation_.inverse());
  // convert to axis angle
  AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  Vector3d orientation_error = error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  AngleAxisd position_des_angle_axis(des_orientation);
  Vector3d des_position_vector3d = position_des_angle_axis.axis() * position_des_angle_axis.angle();

   if (cur_time.toSec() - time_enter_print > 0.5){
	time_enter_print = cur_time.toSec();
	cout << "====================================" << endl;
   	cout << 
        " target_pos : " << position_des.transpose() << endl <<
        "  cmd_angle : " << des_position_vector3d.transpose() << endl <<
        "cmd_angle_error : " << orientation_error.transpose() << endl;
   }


   poseStampedPub_.publish(poseStamped);
   velStampedPub_.publish(velStamped);

   ros::spinOnce();
} // end while
	cout << "*************motion end***************" << endl;
} // function end








void set_gain(double pos_x,double pos_y, double pos_z, double ori_x, double ori_y, double ori_z, double null)
{
   geometry_msgs::PoseStamped gainStamped;
   gainStamped.pose.position.x = pos_x;
   gainStamped.pose.position.y = pos_y;
   gainStamped.pose.position.z = pos_z;
   gainStamped.pose.orientation.x = ori_x;
   gainStamped.pose.orientation.y = ori_y;
   gainStamped.pose.orientation.z = ori_z;
   gainStamped.pose.orientation.w = null;
   gainPub_.publish(gainStamped);
}





void pub_posestamp(Vector3d position, Matrix3d R)
{
   Quaterniond orientation(R);
   geometry_msgs::PoseStamped posestamp;
   posestamp.pose.position.x = position(0);
   posestamp.pose.position.y = position(1);
   posestamp.pose.position.z = position(2);
   posestamp.pose.orientation.x = orientation.x();
   posestamp.pose.orientation.y = orientation.y();
   posestamp.pose.orientation.z = orientation.z();
   posestamp.pose.orientation.w = orientation.w();
   poseStampedPub_.publish(posestamp);
}







void allignment_sin(Vector3d fix_position, Matrix3d basis_R, double allignment_dev, Vector3d ini_ori_basis_axis)
{
   double cur_time= ros::Time::now().toSec();
   double str_time= ros::Time::now().toSec();
   Matrix3d R_des;
   Vector3d pos_des = fix_position;
   double vel = 2.2;

   while(ros::ok())
	{
	  cur_time= ros::Time::now().toSec();
	  ros::spinOnce();
          init_keyboard();
	  if (_kbhit())
		{
		cout << "*******allignment_sin end*********" << endl;
		break;
		}

    if ((cur_time-str_time)/100.0 < 0.075)
    {
      pos_des = fix_position + -(cur_time-str_time)/250.0 * Vector3d::UnitZ();
    }
    else
    {
      cout << "*******allignment_sin end*********" << endl;
      break;
    }
	  R_des = AngleAxisd(sin(vel*(cur_time-str_time))*allignment_dev,ini_ori_basis_axis)*basis_R;
	  pub_posestamp(pos_des,R_des);
	}

}





void peg_in_hole_stategy(Vector3d initial_position,Matrix3d initial_R,Vector3d peg_position,Matrix3d task_start_R, Vector3d ini_ori_basis_axis)
{
   Vector3d position_des(initial_position);
   Matrix3d rot_mat_des(initial_R);
   geometry_msgs::PoseStamped poseStamped;

   ros::spinOnce();

   // 0. stop for calibration
   set_gain(3500,3500,3500,60,60,10,30);
   Vector3d peg_stop_position = peg_position;
   peg_stop_position(2) += 0.05;
   line_gen(initial_position, initial_R, peg_stop_position, initial_R, 0.08);
   sleep(2);
   rot_gen(peg_stop_position, initial_R, task_start_R, 15);

   // 1. rough search
   set_gain(2500,2500,1500,60,60,10,30);
      Vector3d allign_direction = ini_ori_basis_axis.cross(Vector3d::UnitZ());
   if(abs(ini_ori_basis_axis(1) -1) < 0.001 )
   {   
     peg_position = peg_position + 0.007 * allign_direction;
   }
   else
   {
     peg_position = peg_position + 0.001 * allign_direction;
   }
   line_gen(peg_stop_position, task_start_R, peg_position, task_start_R, 0.08);

   // 2. drop to attraction resion
   set_gain(30,30,700,60,60,5,30);
   Vector3d peg_position_search;


  if(abs(ini_ori_basis_axis(1) -1) < 0.001 )
  {   peg_position_search = peg_position + 0.10 * allign_direction;
  }
  else
  {
  peg_position_search = peg_position + 0.02 * allign_direction;
  }

   peg_position_search(2) -= 0.050; // z depth for search
   line_gen(peg_position, task_start_R, peg_position_search, task_start_R, 0.015);

   // 3. allignment
   set_gain(60,60,700,90,90,5,30);
   peg_position_search(0) = cur_car_position_(0); // initialize x
   peg_position_search(1) = cur_car_position_(1); // initialize y
   //Matrix3d end_R(AngleAxisd(-0.10,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX()));
Matrix3d end_R;
if(abs(ini_ori_basis_axis(1) +1) < 0.001 )
{   end_R = AngleAxisd(-10 * PI/180.0,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX());}
else
{
   end_R=AngleAxisd(-6 * PI/180.0,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX());}
   //Matrix3d end_R(AngleAxisd(-40 * PI/180.0,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX()));
   rot_gen(peg_position_search, task_start_R, end_R, 15); //vel : deg/s

   // 4. swing
   set_gain(80,80,700,45,45,5,30);
   Matrix3d alligned_R(AngleAxisd(PI,Vector3d::UnitX()));
   double allignment_dev = 3 *PI/180.0;
   allignment_sin(peg_position_search,alligned_R,allignment_dev, ini_ori_basis_axis);

   // 5. stay and manual control to insert
   //keyboardmode(peg_position_search, alligned_R); // stay for inserting

   // 6. insert back and go to task end position
   set_gain(100,100,1000,60,60,5,30);
   Vector3d task_end_position = cur_car_position_;
   task_end_position(2) += 0.20;
   line_gen(peg_position_search, alligned_R, task_end_position, alligned_R, 0.04);
   //keyboardmode(task_end_position, alligned_R);


} //peg_in_hole_stategy


void sin_tran(Vector3d fix_position, Matrix3d fix_R, Vector3d des_dir, double dur)
{
   double cur_time= ros::Time::now().toSec();
   double str_time= ros::Time::now().toSec();
   Matrix3d R_des = fix_R;
   Vector3d pos_des = fix_position;

  set_gain(400,400,800,50,50,5,30);
  while(ros::ok() && (cur_time-str_time) < dur)
  {
	  ros::spinOnce();
    cur_time= ros::Time::now().toSec();
    pos_des = fix_position + 0.015*sin(2*PI*(cur_time-str_time)) * des_dir;
    //R_des = AngleAxisd(sin(vel*(cur_time-str_time))*allignment_dev,ini_ori_basis_axis)*basis_R;
    pub_posestamp(pos_des,R_des);
  }
  cout << "sin_tran end ======" << endl;
}


void sin_rot(Vector3d fix_position, Matrix3d fix_R, Vector3d ini_ori_basis_axis,double dur)
{
   double cur_time= ros::Time::now().toSec();
   double str_time= ros::Time::now().toSec();
   Matrix3d R_des = fix_R;
   Vector3d pos_des = fix_position;

  set_gain(400,400,800,30,30,5,30);
  while(ros::ok()&& (cur_time-str_time) < dur)
  {
	  ros::spinOnce();
    cur_time= ros::Time::now().toSec();
    //pos_des = fix_position + 0.020*sin(2*(cur_time-str_time) * des_dir;
    R_des = AngleAxisd(7*PI/180.0*sin(2*PI*(cur_time-str_time)),ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX());
    pub_posestamp(pos_des,R_des);
  }
  cout << "sin_rot end ======" << endl;
}


void peg_in_hole_swing(Vector3d initial_position,Matrix3d initial_R,Vector3d peg_position,Matrix3d task_start_R, Vector3d ini_ori_basis_axis)
{

   ros::spinOnce();

   // 0. stop for calibration
   set_gain(3500,3500,3500,60,60,10,30);
   Vector3d peg_stop_position = peg_position;
   peg_stop_position(2) += 0.04;
   line_gen(initial_position, initial_R, peg_stop_position, initial_R, 0.08);
   sleep(2);
   rot_gen(peg_stop_position, initial_R, task_start_R, 15);


   // 1. rough search
   set_gain(3500,3500,1500,60,60,10,30);
   Vector3d allign_direction = ini_ori_basis_axis.cross(Vector3d::UnitZ());
   peg_position = peg_position + 0.001 * allign_direction;
   line_gen(peg_stop_position, task_start_R, peg_position, task_start_R, 0.08);

   // 2. drop to attraction resion
   set_gain(30,30,700,60,60,5,30);
   Vector3d peg_position_search = peg_position + 0.01 * allign_direction;
   peg_position_search(2) -= 0.030; // z depth for search
   line_gen(peg_position, task_start_R, peg_position_search, task_start_R, 0.015);

   // 3. allignment
   set_gain(30,30,500,80,80,5,30);
   peg_position_search(0) = cur_car_position_(0); // initialize x
   peg_position_search(1) = cur_car_position_(1); // initialize y
   //Matrix3d end_R(AngleAxisd(-0.10,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX()));
   Matrix3d end_R(AngleAxisd(-7 * PI/180.0,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX()));
   rot_gen(peg_position_search, task_start_R, end_R, 15); //vel : deg/s

   // 4. swing
   set_gain(30,30,800,45,45,5,30);
   Matrix3d alligned_R(AngleAxisd(PI,Vector3d::UnitX()));
   double allignment_dev = 4 *PI/180.0;
   allignment_sin(peg_position_search,alligned_R,allignment_dev, ini_ori_basis_axis);
   
   // 5. stay and manual control to insert
   sin_tran(cur_car_position_,Matrix3d(cur_car_orientation_),Vector3d::UnitX(),5);
   sin_tran(cur_car_position_,Matrix3d(cur_car_orientation_),Vector3d::UnitY(),5);
   sin_rot(cur_car_position_, Matrix3d(cur_car_orientation_), Vector3d::UnitX(),5);
   sin_rot(cur_car_position_, Matrix3d(cur_car_orientation_), Vector3d::UnitY(),5);

   // 6. insert back and go to task end position
   set_gain(100,100,1000,60,60,5,30);
   Vector3d task_end_position = cur_car_position_;
   task_end_position(2) += 0.20;
   line_gen(peg_position_search, alligned_R, task_end_position, alligned_R, 0.04);
   //keyboardmode(task_end_position, alligned_R);


} //peg_in_hole_stategy


void Hole_height(Vector3d hole_position)
{
  int dx = 0.05;
  int dy = 0.05;
  Vector3d hole_search_pos1 = hole_position;
  hole_search_pos1(2) += 0.01;
  hole_search_pos1(1) -= dy/2.0;
  hole_search_pos1(0) -= dx/2.0;
  Vector3d hole_search_pos2 = hole_position;
  hole_search_pos2(2) += 0.01;
  hole_search_pos2(1) += dy/2.0;
  hole_search_pos2(0) -= dx/2.0;
  Vector3d hole_search_pos3 = hole_position;
  hole_search_pos3(2) += 0.01;
  hole_search_pos3(1) += dy/2.0;
  hole_search_pos3(0) += dx/2.0;
  Vector3d hole_search_pos4 = hole_position;
  hole_search_pos4(2) += 0.01;
  hole_search_pos4(1) -= dy/2.0;
  hole_search_pos4(0) += dx/2.0;

  Matrix3d R1(AngleAxisd(PI,Vector3d::UnitX()));
  Vector3d point1(0.3,0.0,0.48);

    line_gen(cur_car_position_, Matrix3d(cur_car_orientation_), hole_search_pos1 ,R1, 0.07);
    line_gen(hole_search_pos1, R1, hole_search_pos2 ,R1, 0.07);
    line_gen(hole_search_pos2, R1, hole_search_pos3 ,R1, 0.07);
    line_gen(hole_search_pos3, R1, hole_search_pos4 ,R1, 0.07);
    line_gen(cur_car_position_, R1, point1 ,R1, 0.07);

}



void base_plate_calibration()
{
    Matrix3d R1(AngleAxisd(PI,Vector3d::UnitX()));
  // base plate simulation
  set_gain(3500,3500,900,30,30,8,30);
  Vector3d test_zero(0.350, 0.100, 0.1);
  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      test_zero << 0.300 + 0.05 * j, 0.100 - 0.05 * i , 0.07;
      line_gen(cur_car_position_, Matrix3d(cur_car_orientation_), test_zero, R1, 0.07);
      Vector3d test_str;
      test_str = cur_car_position_;
      test_str(2) = -0.060;
      line_gen(cur_car_position_, Matrix3d(cur_car_orientation_), test_str, R1, 0.01);
      Matrix3d test_str_R(Matrix3d(cur_car_orientation_)*AngleAxisd(-100*PI/180.0,Vector3d::UnitZ()));
      rot_gen(test_str, Matrix3d(cur_car_orientation_), test_str_R, 10);
      sleep(1);
      Matrix3d test_end_R(Matrix3d(cur_car_orientation_)*AngleAxisd(+100*PI/180.0,Vector3d::UnitZ()));
      rot_gen(test_str, Matrix3d(cur_car_orientation_), test_end_R, 10);
      sleep(1);
      Matrix3d test_end_R2(Matrix3d(cur_car_orientation_)*AngleAxisd(+100*PI/180.0,Vector3d::UnitZ()));
      rot_gen(test_str, Matrix3d(cur_car_orientation_), test_end_R2, 10);  
    }
  }
}

void go_into_hole(Vector3d initial_position ,Vector3d peg_position)
{
   Vector3d ini_ori_basis_axis(1,0,0);
   double ini_ori_pitch = 30;
   Vector3d allign_direction = ini_ori_basis_axis.cross(Vector3d::UnitZ());
   Vector3d task_start_position = peg_position - 0.0005*allign_direction;
   Matrix3d task_start_R(AngleAxisd(ini_ori_pitch/180.0*PI,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX()));
   Matrix3d initial_R(AngleAxisd(PI,Vector3d::UnitX()));

   ros::spinOnce();

   // 0. stop for calibration
   set_gain(3500,3500,3500,60,60,10,30);
   Vector3d peg_stop_position = peg_position;
   peg_stop_position(2) += 0.04;
   line_gen(initial_position, initial_R, peg_stop_position, initial_R, 0.08);
   sleep(2);
   rot_gen(peg_stop_position, initial_R, task_start_R, 15);


   // 1. rough search
   set_gain(3500,3500,1500,60,60,10,30);
   peg_position = peg_position + 0.001 * allign_direction;
   line_gen(peg_stop_position, task_start_R, peg_position, task_start_R, 0.08);

   // 2. drop to attraction resion
   set_gain(30,30,700,60,60,5,30);
   Vector3d peg_position_search = peg_position + 0.01 * allign_direction;
   peg_position_search(2) -= 0.030; // z depth for search
   line_gen(peg_position, task_start_R, peg_position_search, task_start_R, 0.015);

   // 3. allignment
   set_gain(30,30,500,80,80,5,30);
   peg_position_search(0) = cur_car_position_(0); // initialize x
   peg_position_search(1) = cur_car_position_(1); // initialize y
   //Matrix3d end_R(AngleAxisd(-0.10,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX()));
   Matrix3d end_R(AngleAxisd(-7 * PI/180.0,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX()));
   rot_gen(peg_position_search, task_start_R, end_R, 15); //vel : deg/s

   // 4. swing
   set_gain(30,30,800,45,45,5,30);
   Matrix3d alligned_R(AngleAxisd(PI,Vector3d::UnitX()));
   double allignment_dev = 0 *PI/180.0;
   allignment_sin(peg_position_search,alligned_R,allignment_dev, ini_ori_basis_axis);
   //rot_gen(peg_position_search, end_R, alligned_R, 15); //vel : deg/s

}


void peg_static_experiment(Vector3d initial_position ,Vector3d peg_position)
{
  go_into_hole(initial_position,peg_position);
  pub_posestamp(cur_car_position_,Matrix3d(cur_car_orientation_));
  sleep(2);

  double des_height = cur_car_position_(2);

  Vector3d hole_center;
  hole_center = cur_car_position_;
  double hole_floor = cur_car_position_(2);

  Matrix3d hole_algn;
  hole_algn = Matrix3d(cur_car_orientation_);

  pub_posestamp(hole_center,hole_algn);
  sleep(2);
  
  set_gain(300,300,900,20,20,5,30);
  double dir_no = 8;
  for (size_t j = 0; j < 7; j++)
  { 
    hole_center(0) = cur_car_position_(0);
    hole_center(1) =  cur_car_position_(1);
    hole_center(2) = hole_floor + 0.010 * (j+1);   
      pub_posestamp(hole_center,hole_algn);
      sleep(1);
    
    for (size_t i = 0; i < dir_no; i++)
    {
      double push_dir = 360.0/dir_no * i * PI / 180.0;
      double push_depth = 0.020;
      Vector3d target_car_pos;
      target_car_pos(0) = hole_center(0) + push_depth * cos(push_dir);
      target_car_pos(1) = hole_center(1) + push_depth * sin(push_dir);
      target_car_pos(2) =  hole_center(2);
      line_gen(hole_center,hole_algn,target_car_pos,hole_algn,0.08);
      sleep(2);

      pub_posestamp(hole_center,hole_algn);
      sleep(1);
    }

    for (size_t i = 0; i < dir_no; i++)
    {
      double push_dir = 360.0/dir_no * i * PI / 180.0;
      Matrix3d mom_gen_des(AngleAxisd(6*PI/180.0,AngleAxisd(push_dir,Vector3d::UnitZ())*Vector3d::UnitX())*AngleAxisd(PI,Vector3d::UnitX()));
      rot_gen(hole_center,hole_algn, mom_gen_des,8);
      sleep(2);

      pub_posestamp(hole_center,hole_algn);
      sleep(1);
    }

  }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_gen_node");
  ros::NodeHandle n;

  poseStampedPub_ = n.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 10, true);
  velStampedPub_ = n.advertise<geometry_msgs::PoseStamped>("des_vel", 10, true);
  gainPub_ = n.advertise<geometry_msgs::PoseStamped>("gain_cmd", 10, true);
  ros::Rate loop_rate(1000); // HZt
  double cur_time = ros::Time::now().toSec();

  //planning variables
  Matrix3d start_R;
  Matrix3d end_R;
  double des_vel_mag = 0.05; // m/s
  Vector3d point1(0.3,0.0,0.48);
  Vector3d point2(0.475,-0.1,0.48);
  Vector3d point3(0.500,0.2,0.48);
  Matrix3d R1(AngleAxisd(PI,Vector3d::UnitX()));
  Matrix3d R2(AngleAxisd(-PI/6,Vector3d::UnitY())*R1);
  Matrix3d R3(AngleAxisd(+PI/4,Vector3d::UnitX())*R2);
  

  posetampedSub_ = n.subscribe(
      "/impedance_controller_6DOF/car_pos_pub", 20, &PeginHoleStatesubCallback,
      ros::TransportHints().reliable().tcpNoDelay());

//cur to start position
  while(cur_car_position_.norm() < 0.001)
{
  ros::spinOnce();
}
set_gain(3500,3500,3500,100,100,10,30);
 line_gen(cur_car_position_, Matrix3d(cur_car_orientation_), point1 ,R1, 0.07);

// rotation offset test
// set_gain(0,0,0,7,7,10,30);
// rot_gen(cur_car_position_, Matrix3d(cur_car_orientation_), R1, 10);
// sleep(2);
// set_gain(0,0,0,7,7,7,30);
// Matrix3d test_str_R(Matrix3d(cur_car_orientation_)*AngleAxisd(-100*PI/180.0,Vector3d::UnitZ()));
// rot_gen(cur_car_position_, Matrix3d(cur_car_orientation_), test_str_R, 10);
// sleep(2);
// Matrix3d test_end_R(Matrix3d(cur_car_orientation_)*AngleAxisd(+100*PI/180.0,Vector3d::UnitZ()));
// rot_gen(cur_car_position_, Matrix3d(cur_car_orientation_), test_end_R, 10);
// sleep(2);
// Matrix3d test_end_R2(Matrix3d(cur_car_orientation_)*AngleAxisd(+100*PI/180.0,Vector3d::UnitZ()));
// rot_gen(cur_car_position_, Matrix3d(cur_car_orientation_), test_end_R2, 10);

//base_plate_calibration();


  //planning

/*
  for (int i=0;i<3;i++)
{
  line_gen(point1, R1, point2 ,R2, des_vel_mag);
  line_gen(point2, R2, point3, R3, des_vel_mag);
  line_gen(point3, R3, point1, R1, des_vel_mag);
}
*/

//set_gain(3500,3500,1000,30,30,10,30);
//keyboardmode(cur_car_position_,Matrix3d(cur_car_orientation_));


// static push experiment
//Vector3d hole_position(0.570,0.0,0.110); // real hole position - al
Vector3d hole_position(0.570,0.0,0.210); // real hole position - al
peg_static_experiment(cur_car_position_,hole_position);




//peg-in-hole experiment
//Vector3d hole_position(0.570,0.0,0.110); // real hole position - al
//Vector3d hole_position(0.570,0.0,0.0933); // real hole position - steel
//Vector3d hole_position(0.570,0.0,0.2933);
//  Vector3d hole_position(0.570,0.0,0.28);
//  Vector3d hole_position(0.454,0.008,0.10); // make error
//  Vector3d hole_position(0.3,0.0,0.30); // for rotation test

//Hole_height(hole_position); // hole height measure
// Vector3d ini_ori_basis_axis;
// for (size_t i = 0; i < 4; i++)
// {
//   for (size_t j = 0; j < 3; j++)
//   {
//   double ini_ori_yaw = i * 90; // deg // (-) donnot work?? why??
//   double ini_ori_pitch = 22  + 5*j; // deg // initial 35
//   cout << i << " " << j << endl;
//   ini_ori_basis_axis << 1,0,0;
//   ini_ori_basis_axis = Matrix3d(AngleAxisd(ini_ori_yaw/180.0*PI,Vector3d::UnitZ())) * ini_ori_basis_axis;
//   cout << ini_ori_basis_axis.transpose() << endl;
//   Vector3d allign_direction = ini_ori_basis_axis.cross(Vector3d::UnitZ());
//   Vector3d task_start_position = hole_position - 0.0005*allign_direction;
//   Matrix3d task_start_R(AngleAxisd(ini_ori_pitch/180.0*PI,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX()));
//   peg_in_hole_stategy(cur_car_position_,Matrix3d(cur_car_orientation_),task_start_position,task_start_R, ini_ori_basis_axis);
//   } // end j
// } // end i


//   //swing motion
//   //Vector3d hole_position(0.575,0.0,0.110); // real hole position
//   ini_ori_basis_axis << 1,0,0;
//   double ini_ori_yaw = 0; // deg
//   double ini_ori_pitch = 30; // deg // initial 35
//   ini_ori_basis_axis = AngleAxisd(ini_ori_yaw/180.0*PI,Vector3d::UnitZ()) * ini_ori_basis_axis;
//   Vector3d allign_direction = ini_ori_basis_axis.cross(Vector3d::UnitZ());
//   Vector3d task_start_position = hole_position - 0.001*allign_direction;
//   Matrix3d task_start_R(AngleAxisd(ini_ori_pitch/180.0*PI,ini_ori_basis_axis)*AngleAxisd(PI,Vector3d::UnitX()));
//   peg_in_hole_swing(cur_car_position_,Matrix3d(cur_car_orientation_),task_start_position,task_start_R, ini_ori_basis_axis);




  return 0;
}






void PeginHoleStatesubCallback(
	const geometry_msgs::PoseStampedConstPtr& msg) {
	  cur_car_position_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
	  cur_car_orientation_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
	      msg->pose.orientation.z, msg->pose.orientation.w;
	  cur_car_R_ = Matrix3d(cur_car_orientation_);
	//cout << cur_car_position_.transpose() << endl;
}






///////////////////////////////////////////////////////////
// Keyboard input : kbhit funtion
int _kbhit() {
	unsigned char ch;
	int nread;

	if (peek_character != -1)
		return 1;
	new_settings.c_cc[VMIN] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
	nread = read(0, &ch, 1);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &new_settings);
	if (nread == 1) {
		peek_character = ch;
		return 1;
	}
	return 0;
}

int _getch() {
	char ch;

	if (peek_character != -1) {
		ch = peek_character;
		peek_character = -1;
		return ch;
	}
	read(0, &ch, 1);
	return ch;
}

void init_keyboard() {
	tcgetattr(0, &initial_settings);
	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_cc[VMIN] = 1;
	new_settings.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard() {
	tcsetattr(0, TCSANOW, &initial_settings);
}
///////////////////////////////////////////////////////////


