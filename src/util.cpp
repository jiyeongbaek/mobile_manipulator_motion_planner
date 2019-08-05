#pragma once
#include "util.h"

#include <iostream>
using namespace Eigen;
using namespace std;

Matrix3d Rotate_with_X(const double rAngle)
{
	Matrix3d _Rotate_wth_X;

	_Rotate_wth_X(0, 0) = 1.0;
	_Rotate_wth_X(1, 0) = 0.0;
	_Rotate_wth_X(2, 0) = 0.0;

	_Rotate_wth_X(0, 1) = 0.0;
	_Rotate_wth_X(1, 1) = cos(rAngle);
	_Rotate_wth_X(2, 1) = sin(rAngle);

	_Rotate_wth_X(0, 2) = 0.0;
	_Rotate_wth_X(1, 2) = -sin(rAngle);
	_Rotate_wth_X(2, 2) = cos(rAngle);

	return (_Rotate_wth_X);
}

Matrix3d Rotate_with_Y(const double rAngle)
{
	Matrix3d _Rotate_wth_Y(3, 3);

	_Rotate_wth_Y(0, 0) = cos(rAngle);
	_Rotate_wth_Y(1, 0) = 0.0;
	_Rotate_wth_Y(2, 0) = -sin(rAngle);

	_Rotate_wth_Y(0, 1) = 0.0;
	_Rotate_wth_Y(1, 1) = 1.0;
	_Rotate_wth_Y(2, 1) = 0.0;

	_Rotate_wth_Y(0, 2) = sin(rAngle);
	_Rotate_wth_Y(1, 2) = 0.0;
	_Rotate_wth_Y(2, 2) = cos(rAngle);

	return (_Rotate_wth_Y);
}

Matrix3d Rotate_with_Z(const double rAngle)
{
	Matrix3d _Rotate_wth_Z(3, 3);

	_Rotate_wth_Z(0, 0) = cos(rAngle);
	_Rotate_wth_Z(1, 0) = sin(rAngle);
	_Rotate_wth_Z(2, 0) = 0.0;

	_Rotate_wth_Z(0, 1) = -sin(rAngle);
	_Rotate_wth_Z(1, 1) = cos(rAngle);
	_Rotate_wth_Z(2, 1) = 0.0;

	_Rotate_wth_Z(0, 2) = 0.0;
	_Rotate_wth_Z(1, 2) = 0.0;
	_Rotate_wth_Z(2, 2) = 1.0;

	return (_Rotate_wth_Z);
}

Matrix3d Rot_arm(VectorXd q_current_)
{
	Matrix3d Rot_;
	Rot_ = Rotate_with_Z(q_current_(0)) * Rotate_with_Y(q_current_(1)) * Rotate_with_Z(q_current_(2)) * Rotate_with_Y(-q_current_(3)) * Rotate_with_Z(q_current_(4)) * Rotate_with_Y(-q_current_(5)) * Rotate_with_Z(-(q_current_(6)));
	return Rot_;
}
Matrix3d Rot_arm_link5(VectorXd q_current_)
{
	Matrix3d Rot_;
	Rot_ = Rotate_with_Z(q_current_(0)) * Rotate_with_Y(q_current_(1)) * Rotate_with_Z(q_current_(2)) * Rotate_with_Y(-q_current_(3)) * Rotate_with_Z(q_current_(4));
	return Rot_;
}
Matrix3d Rot_arm_link2(VectorXd q_current_)
{
	Matrix3d Rot_;
	Rot_ = Rotate_with_Z(q_current_(0)) * Rotate_with_Y(q_current_(1));
	return Rot_;
}

Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd)
{

	Vector3d phi;
	Vector3d s[3], v[3], w[3];

	for (int i = 0; i < 3; i++)
	{
		v[i] = Rot.block(0, i, 3, 1);
		w[i] = Rotd.block(0, i, 3, 1);
		s[i] = v[i].cross(w[i]);
	}
	phi = s[0] + s[1] + s[2];
	phi = -0.5 * phi;

	return phi;
}

double Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{

	double rx_t;
	if (rT < rT_0)
	{
		rx_t = rx_0;
	}
	else if (rT >= rT_0 && rT < rT_f)
	{
		rx_t = rx_0 + rx_dot_0 * (rT - rT_0) + (3 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2 * rx_dot_0 / ((rT_f - rT_0) * (rT_f - rT_0)) - rx_dot_f / ((rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0) + (-2 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0) * (rT - rT_0);
	}
	else
	{
		rx_t = rx_f;
	}
	return (rx_t);
}

double JCubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double &rx_f, double rx_dot_f)
{

	double rx_t;
	if (rT < rT_0)
	{
		rx_t = rx_0;
	}
	else if (rT >= rT_0 && rT < rT_f)
	{
		// if (abs(rx_0 - rx_f) > abs(rx_0 - (rx_f + 2 * M_PI)))
		// 	rx_f += 2 * M_PI;
		rx_t = rx_0 + rx_dot_0 * (rT - rT_0) + (3 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2 * rx_dot_0 / ((rT_f - rT_0) * (rT_f - rT_0)) - rx_dot_f / ((rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0) + (-2 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0) * (rT - rT_0);
	}
	else
	{
		rx_t = rx_f;
	}
	return (rx_t);
}

// double Waypoint_Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double &rx_f, double rx_dot_f)
// {

// 	double rx_t;
// 	if (rT < rT_0)
// 	{
// 		rx_t = rx_0;
// 	}
// 	else if (rT >= rT_0 && rT < rT_f)
// 	{
// 		double tf;
// 		tf = rT_f - rT_0;
// 		double a0, a1, a2, a3;
// 		double u0, u0_dot, uf, uf_dot;
// 		u0 = rx_0;
// 		uf = rx_f;
// 		u0_dot = rx_dot_0;
// 		uf_dot = rx_dot_f;

// 		a0 = u0;
// 		a1 = u0_dot; //m_init.vel(i);
// 		a2 = 3.0 / pow(tf, 2) * (uf - u0) - 2.0 / tf * u0_dot - 1.0 / tf * uf_dot;
// 		a3 = -2.0 / pow(tf, 3) * (uf - u0) + 1.0 / pow(tf, 2) * (uf_dot + u0_dot);

// 		rx_t = a0 + a1 * (rT - rT_0) + a2 * pow(rT - rT_0, 2) + a3 * pow(rT - rT_0, 3);
// 	}
// 	else
// 	{
// 		rx_t = rx_f;
// 	}
// 	return (rx_t);
// }

void Linear_Cubic(MatrixXd target, MatrixXd &u_ddot, MatrixXd &t_parabolic, MatrixXd &t_linear, MatrixXd &u_dot)
{
	int size = target.rows();
	target *= DEGREE;
	u_ddot.resize(size, dof);
	t_parabolic.resize(size, dof);
	t_linear.resize(size - 1, dof);
	u_dot.resize(size - 1, dof);

	u_ddot.setZero();
	t_parabolic.setZero();
	t_linear.setZero();
	u_dot.setZero();

	double td = 0.1;
	double uddot = 100; //absolute value of u_ddot (constant)
	// todo uddot
	// todo td12

	/**** udot(linear slope) ******/
	for (int j = 0; j < dof; j++)
	{
		for (int i = 0; i < size - 1; i++)
		{
			if (i == 0)
			{
				u_ddot(i, j) = sgn(target(i + 1, j) - target(i, j)) * fabs(uddot);
				t_parabolic(i, j) = td - sqrt(pow(td, 2) - 2 * (target(i + 1, j) - target(i, j)) / u_ddot(i, j));
				cout << pow(td, 2) - 2 * (target(i + 1, j) - target(i, j)) / u_ddot(i, j) << endl;
				u_dot(i, j) = (target(i + 1, j) - target(i, j)) / (td - 0.5 * t_parabolic(i, j));
			}
			else
			{
				u_dot(i, j) = (target(i + 1, j) - target(i, j)) / td;
			}
		}
	}
	std::cout << "u_dot" << std::endl;
	std::cout << u_dot << std::endl;

	/******* uddot******** */
	for (int j = 0; j < dof; j++)
	{
		for (int i = 0; i < size - 1; i++)
		{
			if (i == 0)
				u_ddot(i, j) = sgn(target(i + 1, j) - target(i, j)) * fabs(uddot);
			else
			{
				u_ddot(i, j) = sgn(u_dot(i + 1, j) - u_dot(i, j)) * fabs(uddot);
				if (i == size - 2)
					u_ddot(i, j) = sgn(u_dot(i - 1, j) - u_dot(i, j)) * fabs(uddot);
			}
		}
	}
	std::cout << "u_ddot" << std::endl;

	std::cout << u_ddot << std::endl;

	/**** the blend times at each path point  */
	for (int j = 0; j < dof; j++)
	{
		for (int i = 0; i < size; i++)
		{
			if (i == 0)
				t_parabolic(i, j) = td - sqrt(pow(td, 2) - 2 * (target(i + 1, j) - target(i, j)) / u_ddot(i, j));
			else
			{
				t_parabolic(i, j) = (u_dot(i + 1, j) - u_dot(i, j)) / u_ddot(i, j);
				if (i == size - 1) //last
					t_parabolic(i, j) = td - sqrt(pow(td, 2) + 2 * (target(i, j) - target(i - 1, j)) / u_ddot(i, j));
			}
		}
	}
	std::cout << "parab" << std::endl;

	std::cout << t_parabolic << std::endl;

	/***** the straight segments times */
	for (int j = 0; j < dof; j++)
	{
		for (int i = 0; i < size - 1; i++)
		{
			if (i == 0)
				t_linear(i, j) = td - t_parabolic(i, j) - 0.5 * t_parabolic(i + 1, j);
			else
			{
				t_linear(i, j) = td - 0.5 * t_parabolic(i, j) - 0.5 * t_parabolic(i + 1, j);
				if (i == size - 2) //last
					t_linear(i, j) = td - t_parabolic(i + 1, j) - 0.5 * t_parabolic(i, j);
			}
		}
	}

	std::cout << "linear" << std::endl;

	std::cout << t_linear << std::endl;
}

int sgn(double v)
{
	if (v < 0)
		return -1;
	if (v > 0)
		return 1;
	return 0;
}

void HermiteSpline_3rd(VectorXd target, VectorXd &pdot_x)
{
	int n = target.rows();
	
	MatrixXd M(n, n);
	M.setZero();
	for (int i = 0; i < n; i++)
	{
		if (i == 0)
		{
			M(0, 0) = 2.0;
			M(0, 1) = 1.0;
		}
		if (i == n - 1)
		{
			M(i, i - 1) = 1.0;
			M(i, i) = 2.0;
		}
		if (i != 0 && i != n - 1)
		{
			M(i, i - 1) = 1.0;
			M(i, i) = 4.0;
			M(i, i + 1) = 1.0;
		}
	}

	VectorXd temp_x(n);
	temp_x.setZero();
	// VectorXd pdot_x(n);
	// pdot_x.setZero();

	MatrixXd inv_M(n, n);
	inv_M.setZero();
	inv_M = M.inverse();

	for (int i = 0; i < n; i++)
	{
		if (i == 0)
		{
			temp_x(i) = 3 * (target(i + 1) - target(i));
		}
		if (i == n - 1)
		{
			temp_x(i) = 3 * (target(i) - target(i - 1));
		}
		if (i != 0 && i != n - 1)
		{
			temp_x(i) = 3 * (target(i + 1) - target(i - 1));
		}
	}

	pdot_x = inv_M * temp_x;

	// for (int i = 0; i < n - 1; i++)
	// {
	// 	p_Hermite_x(i) = target(i) + pdot_x(i) * u +
	// 					 (3.0 * (target(i + 1) - target(i)) - 2.0 * pdot_x(i) - pdot_x(i + 1)) * pow(u, 2.0) + (2.0 * (target(i) - target(i + 1)) + pdot_x(i) + pdot_x(i + 1)) * pow(u, 3.0);
	// }



}
// Trajectory HermiteSpline(const Path &path, const VectorXd &maxVelocity, const VectorXd &maxAcceleration, double timeStep)
// {
// 	Trajectory trajectory;
// 	trajectory.outputPhasePlaneTrajectory();
// 	if (trajectory.isValid())
// 	{
// 		double duration = trajectory.getDuration();

// 		// for (double t=0.0; t<duration;t+=0.1)
// 		// 	Cubic_q.row(t*10) = trajectory.getPosition(t);
// 		// cout << "Trajectory duration: " << duration << " s" << endl
// 		// 	 << endl;
// 	}
// 	else
// 	{
// 		cout << "Trajectory generation failed." << endl;
// 	}
// 	return trajectory;
// 	// string s;
// 	// cin >> s;
// }

Trajectory& HermiteSpline(MatrixXd target)
{

	list<VectorXd> waypoints;
	
	for (size_t i=0; i<target.rows(); i++)
	{
		waypoints.push_back(target.row(i));
	}
	
	VectorXd maxAcceleration(dof);
	VectorXd maxVelocity(dof);
	
	for (size_t i=0; i<dof;i++){
		maxAcceleration(i) =0.01;
		maxVelocity(i) = 0.05;
	}

	Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
	trajectory.outputPhasePlaneTrajectory();
	if (trajectory.isValid())
	{
		double duration = trajectory.getDuration();

		// for (double t=0.0; t<duration;t+=0.1)
		// 	Cubic_q.row(t*10) = trajectory.getPosition(t);
		// cout << "Trajectory duration: " << duration << " s" << endl
		// 	 << endl;
	}
	else
	{
		cout << "Trajectory generation failed." << endl;
	}
	return trajectory;
	// string s;
	// cin >> s;
}
