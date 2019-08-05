#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define Hz 10000.0

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "RRT_planner.h"
#include "NHRRT_planner.h"

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class Controller : public rrt, nh_rrt
{

public:
	Controller(ros::NodeHandle &nh_);
	~Controller();
	enum ARM_CONTROL_MODE
	{
		INIT,
		APPROACH1,
		APPROACH2,
		APPROACH3,
		APPROACH4,
		APPROACH5,
		APPROACH6,
		APPROACH7,
		GOTO_DOOR,
		DEFAULT
	};

public:
	double duration_;
	VectorXd maxAcceleration;
	VectorXd maxVelocity;
	list<VectorXd> waypoints;
	ros::Publisher cubic_pub;
	void compute();
	void initModel();
	bool JointPIDControl(double duration, bool left);
	bool MobilePIDControl(double duration);
	void setMode(ARM_CONTROL_MODE mode);
	void readdata(VectorXd &ql, VectorXd &mob_pos, VectorXd &goal_pose, VectorXd &EE_pos, MatrixXd &EE_rot);
	void writedata(VectorXd &ql, VectorXd &mob_pos);
	void initPosition(VectorXd &ql, VectorXd &qr);
	std_msgs::Bool cubic_done;
        FILE *save_data_p;

	// Math library

	// variable Setting
	struct jointState
	{
		VectorXd qInit_;
		VectorXd qGoal_;
		VectorXd q_;
		VectorXd qdot_;
	};
	struct JointLimit
	{
		VectorXd lower;
		VectorXd upper;
		VectorXd lower_rad;
		VectorXd upper_rad;
	};
	struct jointTarget
	{
		VectorXd q_;
		VectorXd cubic_q_;
		VectorXd desired_q_;
	};

	double mass_[dof];
	Math::Vector3d inertia_[dof];

	shared_ptr<Model> model_l, model_r;
	unsigned int body_id_l[dof], body_id_r[dof];
	Body body_l[dof], body_r[dof];
	Joint joint_l[dof], joint_r[dof];

	jointState joint_left_, joint_right_;
	jointTarget joint_target_left_, joint_target_right_;
	JointLimit joint_limit_left_, joint_limit_right_;

	Vector3d axis_l[dof];
	Math::Vector3d joint_position_global_l[dof];
	Math::Vector3d joint_position_local_l[dof];
	Math::Vector3d com_position_l[dof];
	Matrix3d rot_diff;
	Vector3d rot_diff_vec;
	VectorXd target_;

	double playTime_;
	double Hz_;
	double controlStartTime_;
	double current_time;

	ARM_CONTROL_MODE controlMode_;
	bool isModeChanged;

	Robotmodel _robot_left, _robot_right;
	int target_num, target_state;
	MatrixXd _joint_target;
	MatrixXd _joint_target2;
	MatrixXd _joint_target3;
	MatrixXd target;
	int row1;
	int row2;
	int row3;
	Matrix3d Rot_l;
	Matrix6d Rot_l_tot;

	Matrix3d Rot_Iden;
	Matrix3d Rot_left;
	Matrix3d Rot_right;
	MatrixXd u_ddot, t_parabolic, t_linear, u_dot;
	VectorXd goal_pose_;
	VectorXd p_Hermite_x;
	Vector3d EE_pos_;
	Matrix3d EE_rot_;

	bool planning_done;
	void RRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd &joint_target, bool left);
	void CRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd &joint_target, bool left);

	void TRAC_IK_solver(Vector6d pose, bool left, Robotmodel &model);
	TRAC_IK::TRAC_IK ik_solver(KDL::Chain chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, double timeout_in_secs = 0.005, double error = 1e-5, TRAC_IK::SolveType type = TRAC_IK::Speed);
	std::string urdf_param;
	Matrix4d trans;

	bool approach1_2;

	/////////////// NH_RRT/////////////////////////
	struct baseState
	{
		VectorXd qInit_;
		VectorXd qGoal_;
		VectorXd q_;
		VectorXd qdot_;
	};
	struct BaseLimit
	{
		VectorXd lower;
		VectorXd upper;
	};
	struct mobileTarget
	{
		VectorXd q_;
		VectorXd cubic_q_;
		VectorXd desired_q_;
	};
	baseState base_;
	BaseLimit base_limit_;
	mobileTarget mobile_target_;
	Vector3d base_pos_;
	VectorXd target_pos_;

	Trajectory *Traj_;
	void NHRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd &joint_target);
	VectorXd u0_dof, uf_dot;

	double fRand(double min, double max)
	{
		double f = (double)rand() / RAND_MAX;
		return min + f * (max - min);
	};

	static const Matrix3d Vec_2_Rot(const Vector3d &vec)
	{
		Matrix3d Rot;
		double angle = vec.norm();

		if (angle == 0.0)
			Rot = Matrix3d::Identity();
		else
		{
			Vector3d axis = vec / angle;
			Matrix3d V;
			V.setZero();
			V(0, 1) = -axis(2);
			V(0, 2) = axis(1);
			V(1, 0) = axis(2);
			V(1, 2) = -axis(0);
			V(2, 0) = -axis(1);
			V(2, 1) = axis(0);

			Rot = Matrix3d::Identity() + V * sin(angle) + (1 - cos(angle)) * V * V;
		}
		return Rot;
	};
};
#endif