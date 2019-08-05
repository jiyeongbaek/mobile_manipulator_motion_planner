#include "controller.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

using std::ofstream;

Controller::Controller(ros::NodeHandle &nh_) : rrt(nh_), nh_rrt(nh_)
{
	save_data_p = fopen("/home/dyros/catkin_ws/src/save_data_p.txt", "w");

	planning_done = false;
	joint_left_.qdot_.resize(dof);
	joint_left_.q_.resize(dof);
	joint_left_.qInit_.resize(dof);
	joint_left_.qGoal_.resize(dof);

	joint_left_.qdot_.setZero();
	joint_left_.q_.setZero();
	joint_left_.qInit_.setZero();
	joint_left_.qGoal_.setZero();

	joint_right_.qdot_.resize(dof);
	joint_right_.q_.resize(dof);
	joint_right_.qInit_.resize(dof);
	joint_right_.qGoal_.resize(dof);

	joint_right_.qdot_.setZero();
	joint_right_.q_.setZero();
	joint_right_.qInit_.setZero();
	joint_right_.qGoal_.setZero();

	joint_target_left_.q_.resize(dof);
	joint_target_left_.q_.setZero();
	joint_target_left_.cubic_q_.resize(dof);
	joint_target_left_.desired_q_.resize(dof);
	joint_target_left_.desired_q_.setZero();

	joint_target_right_.q_.resize(dof);
	joint_target_right_.q_.setZero();
	joint_target_right_.cubic_q_.resize(dof);
	joint_target_right_.desired_q_.resize(dof);
	joint_target_right_.desired_q_.setZero();

	joint_limit_right_.upper.resize(dof);
	joint_limit_right_.lower.resize(dof);
	joint_limit_left_.upper.resize(dof);
	joint_limit_left_.lower.resize(dof);

	joint_limit_right_.upper_rad.resize(dof);
	joint_limit_right_.lower_rad.resize(dof);
	joint_limit_left_.upper_rad.resize(dof);
	joint_limit_left_.lower_rad.resize(dof);

	for (int i = 0; i < dof; i++)
	{
		joint_limit_left_.lower(i) = -166.0;
		joint_limit_left_.upper(i) = 166.0;
	}
	for (int i = 0; i < dof; i++)
	{
		joint_limit_right_.lower(i) = -166.0;
		joint_limit_right_.upper(i) = 166.0;
	}

	joint_limit_left_.lower(1) = -101.0;
	joint_limit_left_.upper(1) = 101.0;

	joint_limit_left_.lower(3) = -176.0;
	joint_limit_left_.upper(3) = 4.0;

	joint_limit_left_.lower(5) = -1.0;
	joint_limit_left_.upper(5) = 215;

	joint_limit_right_.lower(1) = -101.0;
	joint_limit_right_.upper(1) = 101.0;

	joint_limit_right_.lower(3) = -176.0;
	joint_limit_right_.upper(3) = 4.0;

	joint_limit_right_.lower(5) = -1.0;
	joint_limit_right_.upper(5) = 215;

	joint_limit_left_.upper_rad = joint_limit_left_.upper / 180.0 * M_PI;
	joint_limit_left_.lower_rad = joint_limit_left_.lower / 180.0 * M_PI;

	joint_limit_right_.upper_rad = joint_limit_right_.upper / 180.0 * M_PI;
	joint_limit_right_.lower_rad = joint_limit_right_.lower / 180.0 * M_PI;

	model_l = make_shared<Model>();
	model_l->gravity = Vector3d(0., 0, -9.81);

	playTime_ = 0.0;

	left = true;
	cubic_done.data = false;
	maxVelocity.resize(dof);
	maxVelocity.setZero();
	maxAcceleration.resize(dof);
	maxAcceleration.setZero();
	for (size_t i = 0; i < dof; i++)
	{
		maxAcceleration(i) = 2.0;
		maxVelocity(i) = 0.7;
	}

	//////////////////////////////// mobile base ////////////////////////////////

	base_limit_.upper.resize(3);
	base_limit_.upper.setZero();
	base_limit_.lower.resize(3);
	base_limit_.lower.setZero();

	base_limit_.upper(0) = 5.0;
	base_limit_.lower(0) = 0.0;

	base_limit_.upper(1) = 5.0;
	base_limit_.lower(1) = 0.0;

	base_limit_.upper(2) = M_PI;
	base_limit_.lower(2) = -M_PI;

	base_.qGoal_.resize(3);
	base_.qGoal_.setZero();
	base_.qInit_.resize(3);
	base_.qInit_.setZero();

	rrt::DoF_size = dof;

	nh_rrt::upper_limit_goal_bias.resize(3);
	nh_rrt::upper_limit_goal_bias.setZero();
	nh_rrt::lower_limit_goal_bias.resize(3);
	nh_rrt::lower_limit_goal_bias.setZero();

	mobile_target_.q_.resize(3);
	mobile_target_.q_.setZero(3);
	mobile_target_.cubic_q_.resize(3);
	mobile_target_.cubic_q_.setZero(3);
	mobile_target_.desired_q_.resize(3);
	mobile_target_.desired_q_.setZero(3);

	target_.resize(6);
	target_.setZero();
	cubic_pub = nh_.advertise<std_msgs::Bool>("/rrt_planned_done", 1);

	EE_pos_.setZero();
	EE_rot_.setZero();

	approach1_2 = false;

}
Controller::~Controller()
{
}
void Controller::initModel()
{
	model_l = make_shared<Model>();

	model_l->gravity = Vector3d(0., 0, -9.81);

	Rot_l = Matrix3d::Identity();

	for (int i = 0; i < 2; i++)
	{
		Rot_l_tot.block(3 * i, 3 * i, 3, 3) = Rot_l;
	}

	axis_l[0] = Rot_l * Eigen::Vector3d::UnitZ();
	axis_l[1] = Rot_l * Eigen::Vector3d::UnitY();
	axis_l[2] = Rot_l * Eigen::Vector3d::UnitZ();
	axis_l[3] = Rot_l * (-1.0 * Eigen::Vector3d::UnitY());
	axis_l[4] = Rot_l * Eigen::Vector3d::UnitZ();
	axis_l[5] = Rot_l * (-1.0 * Eigen::Vector3d::UnitY());
	axis_l[6] = Rot_l * (-1.0 * Eigen::Vector3d::UnitZ());

	for (int i = 0; i < dof; i++)
	{
		mass_[i] = 1.0;
		inertia_[i] = Vector3d(1.0, 1.0, 1.0);
	}

	joint_position_global_l[0] = Vector3d(0, 0, 0.33);
	joint_position_global_l[1] = Vector3d(0, 0, 0.33);
	joint_position_global_l[2] = Vector3d(0, 0, 0.649);
	joint_position_global_l[3] = Vector3d(0.0825, 0, 0.649);
	joint_position_global_l[4] = Vector3d(0.0, -0.0, 1.033);
	joint_position_global_l[5] = Vector3d(0.0, -0.0, 1.033);
	joint_position_global_l[6] = Vector3d(0.0762, -0.0, 1.077);

	joint_position_local_l[0] = joint_position_global_l[0];
	for (int i = 1; i < dof; i++)
		joint_position_local_l[i] = joint_position_global_l[i] - joint_position_global_l[i - 1];

	com_position_l[0] = Vector3d(0.0, -0.0346, 0.2575);
	com_position_l[1] = Vector3d(0.0, 0.0344, 0.4094);
	com_position_l[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_l[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_l[4] = Vector3d(0.0, 0.0423, 0.9243);
	com_position_l[5] = Vector3d(0.0421, -0.0103, 1.0482);
	//	com_position_l[6] = Vector3d(0.4090, -0.5400, 1.5129); // link7_respondable
	com_position_l[6] = Vector3d(0.088, -0.0, 0.8407); // panda_finger_joint 6
	for (int i = 0; i < dof; i++)
		com_position_l[i] -= joint_position_global_l[i];

	for (int i = 0; i < dof; i++)
	{
		body_l[i] = Body(mass_[i], com_position_l[i], inertia_[i]);
		joint_l[i] = Joint(JointTypeRevolute, axis_l[i]);

		if (i == 0)
		{
			body_id_l[i] = model_l->AddBody(0, Math::Xtrans(joint_position_local_l[i]), joint_l[i], body_l[i]);
		}
		else
		{
			body_id_l[i] = model_l->AddBody(body_id_l[i - 1], Math::Xtrans(joint_position_local_l[i]), joint_l[i], body_l[i]);
		}
	}
}

void Controller::setMode(ARM_CONTROL_MODE mode)
{
	isModeChanged = true;
	controlMode_ = mode;
}
void Controller::compute()
{
	_robot_left.model = model_l;
	_robot_left.q = joint_left_.q_;
	for (int i = 0; i < dof; i++)
	{
		_robot_left.body_id[i] = body_id_l[i];
		_robot_left.com_id[i] = com_position_l[i];
	}
	_robot_left.Rot = Rot_l;

	bool joint_flag = false, task_flag = false;

	if (isModeChanged)
	{
		isModeChanged = false;
		controlStartTime_ = playTime_;
		base_.qInit_ = base_.q_;
		joint_left_.qInit_ = joint_left_.q_;
		target_state = 1;
		waypoints.clear();
	}
	switch (controlMode_)
	{
	case INIT:
		//JointPIDControl(0.1*Hz, false); // false : Right Arm Control, true : Left Arm Control
		joint_target_left_.q_(0) = 60.0 * DEGREE;
		joint_target_left_.q_(1) = -45.0 * DEGREE;
		joint_target_left_.q_(2) = 0.0 * DEGREE;
		joint_target_left_.q_(3) = -120.0 * DEGREE;
		joint_target_left_.q_(4) = 0.0 * DEGREE;
		joint_target_left_.q_(5) = 90.0 * DEGREE;
		joint_target_left_.q_(6) = 0.0 * DEGREE;
		JointPIDControl(0.1 * Hz, true);

		break;
	case APPROACH1:
	{
		if (controlStartTime_ == playTime_)
		{
			rrt::box_num = 1;
			rrt::box_num2 = 1;
			// box
			rrt::Box1[0].fAxis = Vector3d(2.0, 2.0, 0.12);
			rrt::Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[0].vPos = Vector3d(0.0, -0.0, -0.02);

			rrt::Box1[1].fAxis = Vector3d(0.01, 0.025, 0.135); // axis length
			rrt::Box1[1].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[1].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[1].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[1].vPos = Vector3d(0.40, 0.00, 0.34); // axis center pos

			rrt::Box1[2].fAxis = Vector3d(0.1, 0.1, 0.1);
			rrt::Box1[2].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[2].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[2].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[2].vPos = Vector3d(0.3, -0.0, 0.7);

			rrt::Box2[1].fAxis = Vector3d(0.02, 0.02, 0.20);
			rrt::Box2[2].fAxis = Vector3d(0.02, 0.02, 0.20);

			rot_diff = Rotate_with_Z(-M_PI / 4.0);

			Quaterniond a1(rot_diff);
			rot_diff_vec = a1.toRotationMatrix().eulerAngles(0, 1, 2);
			target_.head(3) << 0.375, 0.350, 0.316;
			target_.tail(3) = rot_diff_vec;
			TRAC_IK_solver(target_, true, _robot_left);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true);

			rrt::box_num = 0;
			rrt::box_num2 = 0;
			rrt::C.resize(6, 2);
			rrt::C.setZero(); // max , min
			rrt::C(0, 0) = 0.01;
			rrt::C(1, 0) = 0.02;
			rrt::C(2, 0) = 0.02;
			rrt::C(3, 0) = 0.1;
			rrt::C(4, 0) = 0.1;
			rrt::C(5, 0) = 0.1;

			rrt::C(0, 1) = -0.01;
			rrt::C(1, 1) = -0.01;
			rrt::C(2, 1) = -0.02;
			rrt::C(3, 1) = -0.1;
			rrt::C(4, 1) = -0.1;
			rrt::C(5, 1) = -0.1;

			target_.head(3) << 0.375, 0.350, 0.316;
			target_.tail(3) = rot_diff_vec;
			target_(2) -= 0.078;


			joint_target_left_.desired_q_ = joint_left_.qInit_;

			joint_left_.qInit_ = joint_left_.qGoal_;

			TRAC_IK_solver(target_, true, _robot_left);

			rrt::refer_pos(0) = target_(0);
			rrt::refer_pos(1) = target_(1);
			rrt::refer_pos(2) = 0.04;
			rrt::refer_rot = Rotate_with_X(rot_diff_vec(0)) * Rotate_with_Y(rot_diff_vec(1)) * Rotate_with_Z(rot_diff_vec(2));

			rrt::constraint_axis[0] = true;
			rrt::constraint_axis[1] = true;
			rrt::constraint_axis[2] = false;


			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true);

			// row1 = _joint_target.rows();
			// row2 = _joint_target2.rows() / 20;
			// target.resize(row1 + row2 + 1, 7);
			// target.topRows(row1) = _joint_target;

			// for (int i = 0; i < row2; i++)
			// {
			// 	target.row(row1 + i) = _joint_target2.row((1 + i) * 20);
			// }

			// target.bottomRows(1) = _joint_target2.bottomRows(1); //joint_left_.qGoal_.transpose();

			MatrixXd target_1;
			target_1 = MergeRRTResults(_joint_target, _joint_target2, 1);
			cout << target_1*DEGREE << endl;
			cout << "*******************" << endl;

			// cout << target * DEGREE << endl;
			// cout << "*******************" << endl;

			for (size_t i = 0; i < target_1.rows(); i++)
			{
				waypoints.push_back(target_1.row(i));
			}
			Traj_ = new Trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
			Traj_->outputPhasePlaneTrajectory();
			cubic_done.data = true;
			cubic_pub.publish(cubic_done);
			cout << "duration " << Traj_->getDuration() << endl;
		}
		duration_ = Traj_->getDuration();
		if ((playTime_ - controlStartTime_) / 100 < duration_)
		{
			for (size_t i = 0; i < dof; i++)
			{
				joint_target_left_.desired_q_(i) = Traj_->getPosition((playTime_ - controlStartTime_) / 100)[i] * DEGREE;
			}
		}

		// double t= (playTime_ - controlStartTime_) / 100;
		//   fprintf(save_data_p, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", (playTime_ - controlStartTime_) / 100 , Traj_->getPosition(t)[0], Traj_->getPosition(t)[1], Traj_->getPosition(t)[2], Traj_->getPosition(t)[3], Traj_->getPosition(t)[4], Traj_->getPosition(t)[5], Traj_->getPosition(t)[6]);

		//	cout << joint_target_left_.desired_q_.transpose() << endl;
		// joint_flag = false;
		// target_num = target.rows();

		// if (target_num != target_state)
		// 	joint_target_left_.q_ = target.row(target_state) * DEGREE;

		// joint_flag = JointPIDControl(2.0 * Hz, true);

		// if (joint_flag && target_state < target_num)
		// {
		// 	controlStartTime_ = playTime_;
		// 	joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
		// 	target_state++;
		// }

		break;
	}

	case APPROACH2:
		if (controlStartTime_ == playTime_)
		{
			approach1_2 = true;	
			target_.head(3) << EE_pos_(0), EE_pos_(1), EE_pos_(2);
			rot_diff = Rotate_with_Z(-M_PI / 4.0);
			Quaterniond a2(rot_diff);
			rot_diff_vec = a2.toRotationMatrix().eulerAngles(0, 1, 2);
			target_.tail(3) = rot_diff_vec;
			cout << target_.head(3).transpose() << endl;
			// rot_diff = Rotate_with_Z(-M_PI / 4.0);
			// Quaterniond a2(rot_diff);
			// rot_diff_vec = a2.toRotationMatrix().eulerAngles(0, 1, 2);
			// target_.head(3) << 0.375, 0.350, 0.316;
			// target_.tail(3) = rot_diff_vec;	

			target_(2) += 0.23;

			TRAC_IK_solver(target_, true, _robot_left);

			rrt::C.resize(6, 2);
			rrt::C.setZero(); // max , min
			rrt::C(0, 0) = 0.01;
			rrt::C(1, 0) = 0.02;
			rrt::C(2, 0) = 0.02;
			rrt::C(3, 0) = 0.1;
			rrt::C(4, 0) = 0.1;
			rrt::C(5, 0) = 0.1;

			rrt::C(0, 1) = -0.01;
			rrt::C(1, 1) = -0.01;
			rrt::C(2, 1) = -0.02;
			rrt::C(3, 1) = -0.1;
			rrt::C(4, 1) = -0.1;
			rrt::C(5, 1) = -0.1;


			rrt::refer_pos(0) = EE_pos_(0);
			rrt::refer_pos(1) = EE_pos_(1);
			rrt::refer_pos(2) = EE_pos_(2);
			rrt::refer_rot = Rotate_with_X(rot_diff_vec(0)) * Rotate_with_Y(rot_diff_vec(1)) * Rotate_with_Z(rot_diff_vec(2));
			
			rrt::constraint_axis[0] = true;
			rrt::constraint_axis[1] = true;
			rrt::constraint_axis[2] = false;
			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true);

			rrt::box_num = 1;
			rrt::box_num2 = 3;

			// box
			rrt::Box1[0].fAxis = Vector3d(2.0, 2.0, 0.001);
			rrt::Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[0].vPos = Vector3d(0.0, -0.0, 0.10);

			rrt::Box2[1].fAxis = Vector3d(0.05, 0.05, 0.10);
			rrt::Box2[2].fAxis = Vector3d(0.05, 0.05, 0.10);

			rot_diff = Rotate_with_X(-M_PI/2) * Rotate_with_Z(M_PI/4.0);

			Quaterniond a1(rot_diff);
			rot_diff_vec = a1.toRotationMatrix().eulerAngles(0, 1, 2);

			target_.head(3) << 0.514, -0.015, 0.45;
			target_.tail(3) = rot_diff_vec;

			joint_left_.qInit_ = joint_left_.qGoal_;

			TRAC_IK_solver(target_, true, _robot_left);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true); // true -> left planning
																						// _joint_target : Matrix of configurations (degrees)

			// row1 = _joint_target.rows() / 20;
			// row2 = _joint_target2.rows();
			// target.resize(row1 + row2, 7);

			// for (int i = 0; i < row1; i++)
			// {
			// 	target.row(i) = _joint_target.row(i * 20);
			// }
			// target.row(row1) = _joint_target.bottomRows(1); //row(row1*20);
			// target.bottomRows(row2 - 1) = _joint_target2.bottomRows(row2 - 1);

			MatrixXd target_1;
			target_1 = MergeRRTResults(_joint_target, _joint_target2, 2);
			cout << target_1*DEGREE << endl;
			cout << "*******************" << endl;

			// cout << target * DEGREE << endl;
			// cout << "*******************" << endl;

			for (size_t i = 0; i < target_1.rows(); i++)
			{
				waypoints.push_back(target_1.row(i));
			}
			Traj_ = new Trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
			Traj_->outputPhasePlaneTrajectory();
			cubic_done.data = true;
			cubic_pub.publish(cubic_done);
			cout << "duration " << Traj_->getDuration() << endl;
		}

		duration_ = Traj_->getDuration();
		if ((playTime_ - controlStartTime_) / 100 < duration_)
		{
			for (size_t i = 0; i < dof; i++)
			{
				joint_target_left_.desired_q_(i) = Traj_->getPosition((playTime_ - controlStartTime_) / 100)[i] * DEGREE;
			}
		}

		break;

	case APPROACH3:
	{
		if (controlStartTime_ == playTime_)
		{

			if(approach1_2){
			/********** move along y dir ************/
			rrt::box_num = 0;
			rrt::box_num2 = 0;
			rrt::C.resize(6, 2);
			rrt::C.setZero(); // max , min

			rrt::C(0, 0) = 0.01;
			rrt::C(1, 0) = 0.02;
			rrt::C(2, 0) = 0.02;
			rrt::C(3, 0) = 0.1;
			rrt::C(4, 0) = 0.1;
			rrt::C(5, 0) = 0.1;

			rrt::C(0, 1) = -0.01;
			rrt::C(1, 1) = -0.01;
			rrt::C(2, 1) = -0.02;
			rrt::C(3, 1) = -0.1;
			rrt::C(4, 1) = -0.1;
			rrt::C(5, 1) = -0.1;

			rot_diff = Rotate_with_X(-M_PI/2) * Rotate_with_Z(M_PI/4.0);
			Quaterniond a2(rot_diff);
			rot_diff_vec = a2.toRotationMatrix().eulerAngles(0, 1, 2);

			target_.head(3) << 0.514, -0.015, 0.45;
			target_(1) += 0.3;
			target_.tail(3) = rot_diff_vec;


			TRAC_IK_solver(target_, true, _robot_left);

			rrt::refer_pos(0) = target_(0);
			rrt::refer_pos(1) = target_(1);
			rrt::refer_pos(2) = target_(2);;
			rrt::refer_rot = Rotate_with_X(rot_diff_vec(0)) * Rotate_with_Y(rot_diff_vec(1)) * target_(rot_diff_vec(2));


			rrt::constraint_axis[0] = true;
			rrt::constraint_axis[1] = false;
			rrt::constraint_axis[2] = true;


			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true);
			}
/************************************************************************************************************************/
			rrt::box_num = 2;
			rrt::box_num2 = 1;

			rrt::Box1[0].fAxis = Vector3d(2.0, 2.0, 0.12);
			rrt::Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[0].vPos = Vector3d(0.0, -0.0, -0.02);

			rrt::Box1[1].fAxis = Vector3d(0.01, 0.025, 0.135); // axis length
			rrt::Box1[1].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[1].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[1].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[1].vPos = Vector3d(0.514, -0.015, 0.32); // axis center pos


			rot_diff =Rotate_with_Z(-M_PI / 4.0);
			Quaterniond a3(rot_diff);
			rot_diff_vec = a3.toRotationMatrix().eulerAngles(0, 1, 2);
			target_.head(3) <<  0.389, 0.500, 0.351;
			target_.tail(3) = rot_diff_vec;

			if (approach1_2)
				joint_left_.qInit_ = joint_left_.qGoal_;

			TRAC_IK_solver(target_, true, _robot_left);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true);

/************************************************************************************************************************/
			rrt::box_num = 0;
			rrt::box_num2 = 0;
			rrt::C.resize(6, 2);
			rrt::C.setZero(); // max , min
			rrt::C(0, 0) = 0.01;
			rrt::C(1, 0) = 0.02;
			rrt::C(2, 0) = 0.02;
			rrt::C(3, 0) = 0.1;
			rrt::C(4, 0) = 0.1;
			rrt::C(5, 0) = 0.1;

			rrt::C(0, 1) = -0.01;
			rrt::C(1, 1) = -0.01;
			rrt::C(2, 1) = -0.02;
			rrt::C(3, 1) = -0.1;
			rrt::C(4, 1) = -0.1;
			rrt::C(5, 1) = -0.1;

			target_(2) -= 0.188;
			joint_left_.qInit_ = joint_left_.qGoal_;

			TRAC_IK_solver(target_, true, _robot_left);

			rrt::refer_pos(0) = target_(0);
			rrt::refer_pos(1) = target_(1);
			rrt::refer_pos(2) = target_(2);
			rrt::refer_rot = Rotate_with_X(rot_diff_vec(0)) * Rotate_with_Y(rot_diff_vec(1)) * Rotate_with_Z(rot_diff_vec(2));

			rrt::constraint_axis[0] = true;
			rrt::constraint_axis[1] = true;
			rrt::constraint_axis[2] = false;

			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target3, true);

			MatrixXd target_2;

			if (approach1_2)
			{
				MatrixXd target_1;
				target_1 = MergeRRTResults(_joint_target, _joint_target2, 2);
				target_2 = MergeRRTResults(target_1, _joint_target3, 1);
			}
			else
			{
				target_2 = MergeRRTResults(_joint_target2, _joint_target3, 1);
			}
			/************************************************************************************************************************/

			// row1 = _joint_target.rows() / 20;  //CRRT row1 + 1
			// row2 = _joint_target2.rows();	  //RRT row 2 - 2` 
			// row3 = _joint_target3.rows() / 20; //CRRT row3 

			// target.resize(row1 + row2 + row3 + 2, 7);

			// for (int i = 0; i < row1; i++)
			// {
			// 	target.row(i) = _joint_target.row(i * 20);
			// }
			// target.row(row1) = _joint_target.bottomRows(1);

			// //target.topRows(row1) = _joint_target;

			// for (int i = 0; i < row2-1; i++)
			// {
			// 	target.row(row1 + 1+ i) = _joint_target2.row(1 + i);
			// }
			// target.row(row1+row2) = _joint_target2.bottomRows(1);
			// for (int i = 0; i < row3; i++)
			// {
			// 	target.row(row1 + 1+ row2 + i) = _joint_target3.row(1 +i * 20);
			// }

			// target.bottomRows(1) = _joint_target3.bottomRows(1); //joint_left_.qGoal_.transpose();


/************************************************************************************************************************/

			for (size_t i = 0; i < target_2.rows(); i++)
			{
				waypoints.push_back(target_2.row(i));
			}
			Traj_ = new Trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
			Traj_->outputPhasePlaneTrajectory();
			cubic_done.data = true;
			cubic_pub.publish(cubic_done);
		}

		duration_ = Traj_->getDuration();

		if ((playTime_ - controlStartTime_) / 100 < duration_)
		{
			for (size_t i = 0; i < dof; i++)
			{
				joint_target_left_.desired_q_(i) = Traj_->getPosition((playTime_ - controlStartTime_) / 100)[i] * DEGREE;
			}
			//cout << joint_target_left_.desired_q_.transpose() << endl;
		}

		break;
	}

	case APPROACH4:
		if (controlStartTime_ == playTime_)
		{



			target_.head(3) << EE_pos_(0), EE_pos_(1), EE_pos_(2);
			target_(2) += 0.25;
			rot_diff = Rotate_with_Z(-M_PI / 4.0);
			Quaterniond a3(rot_diff);
			rot_diff_vec = a3.toRotationMatrix().eulerAngles(0, 1, 2);
			target_.tail(3) = rot_diff_vec;

			TRAC_IK_solver(target_, true, _robot_left);

			rrt::C.resize(6, 2);
			rrt::C.setZero(); // max , min
			rrt::C(0, 0) = 0.01;
			rrt::C(1, 0) = 0.02;
			rrt::C(2, 0) = 0.02;
			rrt::C(3, 0) = 0.1;
			rrt::C(4, 0) = 0.1;
			rrt::C(5, 0) = 0.1;

			rrt::C(0, 1) = -0.01;
			rrt::C(1, 1) = -0.01;
			rrt::C(2, 1) = -0.02;
			rrt::C(3, 1) = -0.1;
			rrt::C(4, 1) = -0.1;
			rrt::C(5, 1) = -0.1;

			rrt::refer_pos(0) = target_(0);
			rrt::refer_pos(1) = target_(1);
			rrt::refer_pos(2) = target_(2);
			rrt::refer_rot = Rotate_with_X(rot_diff_vec(0)) * Rotate_with_Y(rot_diff_vec(1)) * Rotate_with_Z(rot_diff_vec(2));

			rrt::constraint_axis[0] = true;
			rrt::constraint_axis[1] = true;
			rrt::constraint_axis[2] = false;


			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true);

			rrt::box_num = 2;
			rrt::box_num2 = 4;

			// box
			rrt::Box1[0].fAxis = Vector3d(2.0, 2.0, 0.001);
			rrt::Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[0].vPos = Vector3d(0.0, -0.0, 0.10);

			rrt::Box1[1].fAxis = Vector3d(0.03, 0.01, 0.135);
			rrt::Box1[1].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[1].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[1].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[1].vPos = Vector3d(0.514652, -0.155728, 0.326979);

			rrt::Box2[1].fAxis = Vector3d(0.05, 0.05, 0.10);
			rrt::Box2[2].fAxis = Vector3d(0.05, 0.05, 0.10);

			rrt::Box2[3].fAxis = Vector3d(0.135, 0.03, 0.01);

			rot_diff = Rotate_with_X(M_PI / 2.0) * Rotate_with_Z(-M_PI / 2.0) * Rotate_with_Z(-M_PI / 4.0);

			Quaterniond a1(rot_diff);
			rot_diff_vec = a1.toRotationMatrix().eulerAngles(0, 1, 2);

			target_.head(3) << 0.505, 0.14, 0.307;
			target_.tail(3) = rot_diff_vec;

			joint_left_.qInit_ = joint_left_.qGoal_;

			TRAC_IK_solver(target_, true, _robot_left);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true); // true -> left planning

			// row1 = _joint_target.rows() / 20;
			// row2 = _joint_target2.rows();
			// target.resize(row1 + row2, 7);

			// for (int i = 0; i < row1; i++)
			// {
			// 	target.row(i) = _joint_target.row(i * 20);
			// }
			// target.row(row1) = _joint_target.bottomRows(1); //row(row1*20);
			// target.bottomRows(row2 - 1) = _joint_target2.bottomRows(row2 - 1);
			// cout << target << endl;
			// cout << "*******************" << endl;

			MatrixXd target_1;
			target_1 = MergeRRTResults(_joint_target, _joint_target2, 2);
			cout << target_1*DEGREE << endl;
			cout << "*******************" << endl;



			for (size_t i = 0; i < target_1.rows(); i++)
			{
				waypoints.push_back(target_1.row(i));
			}
			Traj_ = new Trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
			Traj_->outputPhasePlaneTrajectory();
			cubic_done.data = true;
			cubic_pub.publish(cubic_done);
			cout << "duration " << Traj_->getDuration() << endl;
		}
		duration_ = Traj_->getDuration();
		if ((playTime_ - controlStartTime_) / 100 < duration_)
		{
			for (size_t i = 0; i < dof; i++)
			{
				joint_target_left_.desired_q_(i) = Traj_->getPosition((playTime_ - controlStartTime_) / 100)[i] * DEGREE;
			}
		}

		break;

	case APPROACH5:
		if (controlStartTime_ == playTime_)
		{
			rrt::box_num = 1;
			rrt::box_num2 = 1;
			// box
			rrt::Box1[0].fAxis = Vector3d(2.0, 2.0, 0.12);
			rrt::Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[0].vPos = Vector3d(0.0, -0.0, -0.02);

			rrt::Box1[1].fAxis = Vector3d(0.01, 0.025, 0.135); // axis length
			rrt::Box1[1].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[1].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[1].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[1].vPos = Vector3d(0.40, 0.00, 0.34); // axis center pos

			rrt::Box1[2].fAxis = Vector3d(0.1, 0.1, 0.1);
			rrt::Box1[2].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[2].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[2].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[2].vPos = Vector3d(0.3, -0.0, 0.7);

			rrt::Box2[1].fAxis = Vector3d(0.02, 0.02, 0.20);
			rrt::Box2[2].fAxis = Vector3d(0.02, 0.02, 0.20);

			rot_diff << 0.999996, -0.00143145, 0.00232364,
				0.00143701, 0.999996, -0.00239149,
				-0.00232021, 0.00239482, 0.999994;

			Quaterniond a1(rot_diff);
			rot_diff_vec = a1.toRotationMatrix().eulerAngles(0, 1, 2);
			target_.head(3) << 0.325561, -0.169888, 0.326; // 0.12862;
			target_.tail(3) = rot_diff_vec;
			TRAC_IK_solver(target_, true, _robot_left);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true);

			rrt::box_num = 0;
			rrt::box_num2 = 0;
			rrt::C.resize(6, 2);
			rrt::C.setZero(); // max , min
			rrt::C(0, 0) = 0.01;
			rrt::C(1, 0) = 0.02;
			rrt::C(2, 0) = 0.02;
			rrt::C(3, 0) = 0.1;
			rrt::C(4, 0) = 0.1;
			rrt::C(5, 0) = 0.1;

			rrt::C(0, 1) = -0.01;
			rrt::C(1, 1) = -0.01;
			rrt::C(2, 1) = -0.02;
			rrt::C(3, 1) = -0.1;
			rrt::C(4, 1) = -0.1;
			rrt::C(5, 1) = -0.1;

			target_(2) -= 0.088;
			joint_left_.qInit_ = joint_left_.qGoal_;

			TRAC_IK_solver(target_, true, _robot_left);

			rrt::refer_pos(0) = 0.325561;
			rrt::refer_pos(1) = -0.169888;
			rrt::refer_pos(2) = 0.04;
			rrt::refer_rot = Rotate_with_X(rot_diff_vec(0)) * Rotate_with_Y(rot_diff_vec(1)) * Rotate_with_Z(rot_diff_vec(2));

			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true);
			// _joint_target : Matrix of configurations (degrees)
		}

		row1 = _joint_target.rows();
		row2 = _joint_target2.rows() / 20;
		target.resize(row1 + row2 + 1, 7);
		target.topRows(row1) = _joint_target;

		for (int i = 0; i < row2; i++)
		{
			target.row(row1 + i) = _joint_target2.row(1 + i * 20);
		}

		target.bottomRows(1) = _joint_target2.bottomRows(1); //joint_left_.qGoal_.transpose();

		joint_flag = false;
		target_num = target.rows();

		if (target_state == 1)
			joint_left_.qInit_ = joint_left_.q_;

		if (target_state < target_num + 1)
			joint_target_left_.q_ = target.row(target_state - 1) * DEGREE;

		joint_flag = JointPIDControl(2.0 * Hz, true);

		if (joint_flag && target_state < target_num)
		{
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
		}

		break;

	case APPROACH6:
		if (controlStartTime_ == playTime_)
		{
			rrt::box_num = 1;
			rrt::box_num2 = 2;

			// rrt::Box1[0].fAxis = Vector3d(0.01, 0.025, 0.135); // axis length
			// rrt::Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			// rrt::Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			// rrt::Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			// rrt::Box1[0].vPos = Vector3d(0.5, -0.25, 0.135); // axis center pos

			// box
			rrt::Box1[1].fAxis = Vector3d(2.0, 2.0, 0.1);
			rrt::Box1[1].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[1].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[1].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[1].vPos = Vector3d(0.0, -0.0, -0.02);

			// rrt::Box2[1].fAxis = Vector3d(0.02, 0.02, 0.20);
			// rrt::Box2[2].fAxis = Vector3d(0.02, 0.02, 0.20);

			rrt::Box2[1].fAxis = Vector3d(0.01, 0.025, 0.135); // axis length
			rrt::Box2[1].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box2[1].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box2[1].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box2[1].vPos = Vector3d(0.40, 0.00, 0.34); // axis center pos

			// current pos : 0.412993 -0.60146 0.259897
			// [ INFO] [1563944666.711415690]: rotation :  0.0815008  -0.527167   0.845844
			// -0.0837616  -0.849288  -0.521242
			//   0.993147 -0.0283675  -0.113374
			rot_diff << 0.72602, -0.685597, 0.0534038,
				-0.68595, -0.727508, -0.0143102,
				0.0486627, -0.0262428, -0.99847;

			Quaterniond a1(rot_diff);
			rot_diff_vec = a1.toRotationMatrix().eulerAngles(0, 1, 2);

			target_.head(3) << 0.337516, 0.296872, 0.129907;
			target_.tail(3) = rot_diff_vec;
			TRAC_IK_solver(target_, true, _robot_left);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true); // true -> left planning
																					   // _joint_target : Matrix of configurations (degrees)
		}
		joint_flag = false;
		target_num = _joint_target.rows();

		if (target_num != target_state)
			joint_target_left_.q_ = _joint_target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(2.0 * Hz, true);

		if (joint_flag && target_state < target_num)
		{
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
		}

		break;

	case APPROACH7:
		if (controlStartTime_ == playTime_)
		{
			rrt::box_num = 0;
			rrt::box_num2 = 0;

			rrt::Box1[0].fAxis = Vector3d(0.1, 0.1, 0.1); // axis length
			rrt::Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[0].vPos = Vector3d(0.375, 0.1, 0.8); // axis center pos

			// box
			rrt::Box1[1].fAxis = Vector3d(2.0, 2.0, 0.1);
			rrt::Box1[1].vAxis[0] = Vector3d(1, 0, 0);
			rrt::Box1[1].vAxis[1] = Vector3d(0, 1, 0);
			rrt::Box1[1].vAxis[2] = Vector3d(0, 0, 1);
			rrt::Box1[1].vPos = Vector3d(0.0, -0.0, -0.02);

			rrt::Box2[1].fAxis = Vector3d(0.02, 0.02, 0.20);
			rrt::Box2[2].fAxis = Vector3d(0.02, 0.02, 0.20);

			rrt::C.resize(6, 2);
			rrt::C.setZero(); // max , min
			rrt::C(0, 0) = 0.01;
			rrt::C(1, 0) = 0.02;
			rrt::C(2, 0) = 0.02;
			rrt::C(3, 0) = 0.1;
			rrt::C(4, 0) = 0.1;
			rrt::C(5, 0) = 0.1;

			rrt::C(0, 1) = -0.01;
			rrt::C(1, 1) = -0.01;
			rrt::C(2, 1) = -0.02;
			rrt::C(3, 1) = -0.1;
			rrt::C(4, 1) = -0.1;
			rrt::C(5, 1) = -0.1;

			TRAC_IK_solver(goal_pose_, true, _robot_left);

			rrt::refer_pos(0) = 0.375;
			rrt::refer_pos(1) = 0.1;
			rrt::refer_pos(2) = 0.04;
			rrt::refer_rot = Rotate_with_X(M_PI / 2.0) * Rotate_with_Y(0.0) * Rotate_with_Z(M_PI / 4.0);

			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true);

			// _joint_target : Matrix of configurations (degrees)
		}

		target_num = _joint_target.rows() / 20;

		if (target_state <= target_num)
			joint_target_left_.q_ = _joint_target.row((target_state)*20) * DEGREE;

		joint_flag = JointPIDControl(2.0 * Hz, true);

		if (joint_flag && target_state <= target_num + 1)
		{
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state

			if (target_state == target_num + 1)
			{
				joint_target_left_.q_ = joint_left_.qGoal_;
			}
			target_state++;
			//cout << target_state << endl;
		}

		break;

	case GOTO_DOOR:
		if (controlStartTime_ == playTime_)
		{

			base_.qGoal_(0) = 4.4;
			base_.qGoal_(1) = 2.0;
			base_.qGoal_(2) = 0.0;

			nh_rrt::Obs[0].pos(0) = 1.45;
			nh_rrt::Obs[0].pos(1) = 2.3250;
			nh_rrt::Obs[0].radius = 0.56;

			nh_rrt::Obs[1].pos(0) = 2.850;
			nh_rrt::Obs[1].pos(1) = 0.1250;
			nh_rrt::Obs[1].radius = 0.56;

			nh_rrt::obs_num = 2;
			nh_rrt::base_length = 1.14;
			nh_rrt::base_width = 0.7;

			NHRRT_planning(base_.qInit_, base_.qGoal_, _joint_target);
			// _joint_target : Matrix of configurations (x, y, theta)
			target_state = 0;
		}
		joint_flag = false;
		target_num = _joint_target.rows();
		// //getchar();

		if (target_num != target_state)
			mobile_target_.q_ = _joint_target.row(target_state);

		joint_flag = MobilePIDControl(0.1 * Hz);

		if (joint_flag && target_state < target_num)
		{
			controlStartTime_ = playTime_;
			base_.qInit_ = mobile_target_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
		}

		break;

	case DEFAULT:
		joint_target_left_.desired_q_ = joint_left_.qInit_;
		joint_target_right_.desired_q_ = joint_right_.qInit_;
		//std::cout << joint_left_.qInit_.transpose() << std::endl;

		break;
	}

	playTime_++;
}

// \B3\BB\B9\AB function
bool Controller::JointPIDControl(double duration, bool left)
{
	bool res = false;
	if (left == true)
	{
		for (int i = 0; i < dof; i++)
			joint_target_left_.cubic_q_(i) = JCubic(playTime_, controlStartTime_, controlStartTime_ + duration, joint_left_.qInit_(i), 0.0, joint_target_left_.q_(i), 0.0);
		joint_target_left_.desired_q_ = joint_target_left_.cubic_q_;
	}
	else
	{
		for (int i = 0; i < dof; i++)
			joint_target_right_.cubic_q_(i) = JCubic(playTime_, controlStartTime_, controlStartTime_ + duration, joint_right_.qInit_(i), 0.0, joint_target_right_.q_(i), 0.0);
		joint_target_right_.desired_q_ = joint_target_right_.cubic_q_;
	}
	if (controlStartTime_ + duration < playTime_)
		res = true;

	return res;
}

bool Controller::MobilePIDControl(double duration)
{
	bool res = false;
	for (int i = 0; i < 3; i++)
		mobile_target_.cubic_q_(i) = JCubic(playTime_, controlStartTime_, controlStartTime_ + duration, base_.qInit_(i), 0.0, mobile_target_.q_(i), 0.0);
	mobile_target_.desired_q_ = mobile_target_.cubic_q_;

	if (controlStartTime_ + duration < playTime_)
		res = true;

	return res;
}
void Controller::readdata(VectorXd &ql, VectorXd &mob_pos, VectorXd &goal_pose, VectorXd &EE_pos, MatrixXd &EE_rot)
{
	joint_left_.q_ = ql;
	base_.q_ = mob_pos;
	goal_pose_ = goal_pose;
	EE_pos_ = EE_pos;
	EE_rot_ = EE_rot;
}
void Controller::writedata(VectorXd &ql, VectorXd &mob_pos)
{
	ql = joint_target_left_.desired_q_;
	mob_pos = mobile_target_.desired_q_;
}

void Controller::TRAC_IK_solver(Vector6d pose, bool left, Robotmodel &model)
{
	double eps = 1e-7;
	double num_samples = 1000;
	double timeout = 0.005;
	std::string chain_start = "panda_link0";
	std::string chain_end = "panda_link8";
	TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

	KDL::Chain chain;
	KDL::JntArray ll, ul; //lower joint limits, upper joint limits

	bool valid = tracik_solver.getKDLChain(chain);

	if (!valid)
	{
		ROS_ERROR("there was no valid KDL chain found");
		return;
	}

	valid = tracik_solver.getKDLLimits(ll, ul);

	if (!valid)
	{
		ROS_ERROR("there was no valid KDL joint limits found");
		return;
	}

	assert(chain.getNrOfJoints() == ll.data.size());
	assert(chain.getNrOfJoints() == ul.data.size());

	// Set up KDL IK
	KDL::ChainFkSolverPos_recursive fk_solver(chain);									  // Forward kin. solver
	KDL::ChainIkSolverVel_pinv vik_solver(chain);										  // PseudoInverse vel solver
	KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps); // Joint Limit Solver
	// 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

	// Create Nominal chain configuration midway between all joint limits
	KDL::JntArray nominal(chain.getNrOfJoints());
	for (size_t j = 0; j < nominal.data.size(); j++)
	{
		nominal(j) = (ll(j) + ul(j)) / 2.0;
	}

	// Create desired number of valid, random joint configurations
	std::vector<KDL::JntArray> JointList;
	KDL::JntArray q(chain.getNrOfJoints());

	for (uint i = 0; i < num_samples; i++)
	{
		for (uint j = 0; j < ll.data.size(); j++)
		{
			q(j) = fRand(ll(j), ul(j));
		}
		JointList.push_back(q);
	}

	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration diff;

	KDL::JntArray result;
	KDL::Frame end_effector_pose;

	ROS_INFO_STREAM("Target pose(global) : " << pose.transpose());

	for (int i = 0; i < 3; i++)
	{
		end_effector_pose.p(i) = pose(i);
	}
	Matrix3d Rot_d;
	Rot_d = Rotate_with_X(pose(3)) * Rotate_with_Y(pose(4)) * Rotate_with_Z(pose(5));

	KDL::Rotation A;
	A.data[0] = Rot_d(0, 0);
	A.data[1] = Rot_d(0, 1);
	A.data[2] = Rot_d(0, 2);
	A.data[3] = Rot_d(1, 0);
	A.data[4] = Rot_d(1, 1);
	A.data[5] = Rot_d(1, 2);
	A.data[6] = Rot_d(2, 0);
	A.data[7] = Rot_d(2, 1);
	A.data[8] = Rot_d(2, 2);
	end_effector_pose.M = A;

	int rc;

	double total_time = 0;
	uint success = 0;

	ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " random samples");

	for (uint i = 0; i < num_samples; i++)
	{
		//fk_solver.JntToCart(JointList[i], end_effector_pose);
		double elapsed = 0;
		start_time = boost::posix_time::microsec_clock::local_time();
		rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
		std::vector<double> config;
		config.clear();

		if (rc >= 0)
		{
			for (int i = 0; i < dof; i++)
			{
				config.push_back(result.data(i) * 180 / M_PI);
			}
			if (!rrt::CheckCollision(model, config))
			{
				//cout <<"no colli!!" << endl;
				break;
			}
			else
			{
				continue;
			}
		}
	}
	//ROS_INFO_STREAM("TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample");
	ROS_INFO_STREAM("Result data : " << result.data.transpose() * 180 / M_PI);

	if (left)
	{
		joint_left_.qGoal_ = result.data;
	}
	else
	{
		joint_right_.qGoal_ = result.data;
	}
}

void Controller::RRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd &joint_target, bool left)
{

	rrt::left = left;
	if (left)
	{
		rrt::lower_limit = joint_limit_left_.lower;
		rrt::upper_limit = joint_limit_left_.upper;
		rrt::DoF_size = dof;
	}
	else
	{
		rrt::lower_limit = joint_limit_right_.lower;
		rrt::upper_limit = joint_limit_right_.upper;
		rrt::DoF_size = dof;
	}
	rrt::qinit = q_start;
	rrt::qgoal = q_goal;

	//////////////// until here okay

	std::ofstream outFile("path_result.txt", ios::out); // open file for "writing"
	bool a = false;

	while (!a)
		if (left)
		{

			a = rrt::StartRRT(_robot_left, outFile);
		}
		else
			a = rrt::StartRRT(_robot_right, outFile);
	ofstream outFile2("path_result2.txt", ios::out); // "writing"
	// path_result -> Smooth -> path_result2
	ifstream inFile("path_result.txt"); // "reading"
	rrt::SmoothPath(outFile2, inFile);

	outFile2.close();
	MatrixXd joint_temp(100, dof);
	if (!left)
		joint_temp.resize(100, dof);

	ifstream inFile2("path_result2.txt"); // "reading"
	int size = 0;
	std::vector<std::string> parameters;
	char inputString[1000];
	while (!inFile2.eof())
	{ // eof : end of file
		inFile2.getline(inputString, 1000);
		boost::split(parameters, inputString, boost::is_any_of(","));
		if (left)
		{
			if (parameters.size() == dof)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
				size++;
			}
		}
		else
		{
			if (parameters.size() == dof)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
	}
	cout << "trajectory size" << size << endl;
	inFile2.close();
	if (left)
		joint_target = joint_temp.topLeftCorner(size, dof);
	else
		joint_target = joint_temp.topLeftCorner(size, dof);
}
void Controller::CRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd &joint_target, bool left)
{
	rrt::left = left;
	if (left)
	{
		rrt::lower_limit = joint_limit_left_.lower;
		rrt::upper_limit = joint_limit_left_.upper;
		rrt::DoF_size = dof;
	}
	else
	{
		rrt::lower_limit = joint_limit_right_.lower;
		rrt::upper_limit = joint_limit_right_.upper;
		rrt::DoF_size = dof;
	}
	rrt::qinit = q_start;
	rrt::qgoal = q_goal;

	ofstream outFile("path_result.txt", ios::out);
	bool a = false;
	while (!a)
		if (left)
			a = rrt::StartCRRT(_robot_left, outFile);
		else
			a = rrt::StartCRRT(_robot_right, outFile);

	//cout << "3" << endl;
	outFile.close();
	MatrixXd joint_temp(5000, dof);
	if (!left)
		joint_temp.resize(5000, dof);

	ifstream inFile2("path_result.txt");
	int size = 0;
	std::vector<std::string> parameters;
	char inputString[50000];
	while (!inFile2.eof())
	{
		inFile2.getline(inputString, 50000);
		boost::split(parameters, inputString, boost::is_any_of(","));
		if (left)
		{
			if (parameters.size() == dof)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
		else
		{
			if (parameters.size() == dof)
			{
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
	}
	inFile2.close();
	cout << "size" << size << endl;

	if (left)
		joint_target = joint_temp.topLeftCorner(size, dof);
	else
		joint_target = joint_temp.topLeftCorner(size, dof);
}

void Controller::NHRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd &joint_target)
{

	nh_rrt::lower_limit_base = base_limit_.lower;
	nh_rrt::upper_limit_base = base_limit_.upper;

	nh_rrt::qinit = q_start;
	nh_rrt::qgoal = q_goal;

	nh_rrt::upper_limit_goal_bias(0) = q_goal(0) + 0.2;
	nh_rrt::upper_limit_goal_bias(1) = q_goal(1) + 0.2;
	nh_rrt::upper_limit_goal_bias(2) = q_goal(2) + 0.05;

	nh_rrt::lower_limit_goal_bias(0) = q_goal(0) - 0.2;
	nh_rrt::lower_limit_goal_bias(1) = q_goal(1) - 0.2;
	nh_rrt::lower_limit_goal_bias(2) = q_goal(2) - 0.05;

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"
	bool a = false;
	while (!a)
		a = nh_rrt::StartNHRRT(outFile);

	outFile.close();
	MatrixXd joint_temp(200, 3);
	//cout << "input" << endl;
	ifstream inFile2("path_result.txt"); // "reading"
	int size = 0;
	std::vector<std::string> parameters;
	char inputString[1000];
	while (!inFile2.eof())
	{ // eof : end of file
		inFile2.getline(inputString, 1000);
		boost::split(parameters, inputString, boost::is_any_of(","));
		if (parameters.size() == 3)
		{
			for (int j = 0; j < parameters.size(); j++)
				joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
			size++;
		}
	}
	inFile2.close();
	joint_target = joint_temp.topLeftCorner(size, 3);
}
