#include <iostream>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include "RRT_planner.h"
#include <string>
#include <cstring>

using namespace std;
using namespace Eigen;
static const long MAX_TIME = 180.0f;
static const int DoF = 7;

#define REACHED 0
#define ADVANCED 1
#define TRAPPED -1
#define q_Max  180.0

rrt::rrt(ros::NodeHandle nh_)
{

	rrt_pub = nh_.advertise<std_msgs::Bool>("/rrt_planned_done", 1);

}
rrt::~rrt(){

}

bool rrt::StartRRT(Robotmodel& model, std::ostream& sout) {
	_model = model;
	std::string fullinput;
	std::vector<std::string> startconfig;
	std::vector<std::string> endgoal;
	std::vector<std::string> weightsstr;
	std::vector<std::string> parameters;
	std::vector<std::string> inputs;
	tree._nodes.clear(); // NodeTree
	gTree._nodes.clear(); // NodeTree
	goal.clear(); // std::vector<double>
	start.clear();// std::vector<double>
	DOF_weights.clear();// std::vector<double>

	int ActiveDoFs = DoF_size;
	step_size = 0.5;
	std::string start_string[DoF];

	for (int i = 0; i < ActiveDoFs; i++) {
		start.push_back(qinit(i) * 180.0 / M_PI);
		goal.push_back(qgoal(i) * 180.0 / M_PI);
		DOF_weights.push_back(1.0);
	}

	RRTNode n(start, 0); // ������ RRT
	tree.addNode(n);
	c_tree = &tree; // c_tree : pointer of NodeTree
	t_turn = 0;
	g2 = goal;
	RRTNode g(goal, 0); // ���� RRT
	gTree.addNode(g);
	// That's why we called "BiRRT"

	bool finished = false;
	int iter = 1;

	while (iter < 500000) {
		planned_done.data = false;
		rrt_pub.publish(planned_done);
		count = 0;
		if (t_turn == 1) { // BiRRT 
			t_turn = 2;
			g2 = tree._nodes.back()->getConfiguration(); // back() : access last element of vector
			c_tree = &gTree;
		}
		else {
			t_turn = 1;
			g2 = gTree._nodes.back()->getConfiguration();
			c_tree = &tree;
		}
		std::vector<double> node = RandomConfig();
		//std::cout << iter << std::endl;

		if (this->Connect(node) == REACHED && isGoal) {
			std::cout << "FINISHED " << " " << "iteration:" << " " << iter << std::endl;
			finished = true;
			break;
		}
		iter++;
	}

	if (finished) {
		planned_done.data = false;
		rrt_pub.publish(planned_done);

		path = tree.getPath();
		vector<vector<double>> p2;
		p2 = gTree.getPath();
		reverse(p2.begin(), p2.end());
		path.insert(path.end(), p2.begin(), p2.end());

		for (int i = 0; i < path.size(); i++) {
			//cout << i << endl;
			std::vector<double> node = path[i];
			for (int j = 0; j<node.size() - 1; j++)
				sout << node[j] << ",";

			sout << node[node.size() - 1] << "\n";
		}
	}
	else {
		return false;
	}
	// planned_done.data = true;
	// rrt_pub.publish(planned_done);
	return true;
}
bool rrt::StartCRRT(Robotmodel& model, std::ostream& sout) {
	_model = model;
	std::string fullinput;
	std::vector<std::string> startconfig;
	std::vector<std::string> endgoal;
	std::vector<std::string> weightsstr;
	std::vector<std::string> parameters;
	std::vector<std::string> inputs;
	tree._nodes.clear();
	gTree._nodes.clear();
	goal.clear();
	start.clear();
	DOF_weights.clear();

	int ActiveDoFs = DoF_size;
	step_size = 0.5;
	std::string start_string[DoF];

	for (int i = 0; i < ActiveDoFs; i++) {
		start.push_back(qinit(i) * 180.0 / M_PI);
		goal.push_back(qgoal(i) * 180.0 / M_PI);
		DOF_weights.push_back(1.0);
	}
	RRTNode n(start, 0); // start tree
	tree.addNode(n);
	c_tree = &tree;
	t_turn = 0;
	g2 = goal;
	RRTNode g(goal, 0); // goal tree
	gTree.addNode(g);

	bool finished = false;
	int iter = 1;

	while (iter < 5000000) {
		planned_done.data = false;
		rrt_pub.publish(planned_done);
		/////// Start tree -> Goal tree's the nearest configuration
		g2 = gTree._nodes.back()->getConfiguration();// goal node 

		c_tree = &tree;//		// start tree 

		std::vector<double> node = RandomConfig(); 
		// 10% : random / 90% : goal tree config 

		// goal tree -> start tree 
		int a = this->ConstrainedConnect(node);

		if (a == REACHED && isGoal) { // REACHED : norm(g2, node) < step size
			std::cout << "FINISHED " << " " << "iteration:" << " " << iter << std::endl;
			finished = true;
			break;
		}

		/////// Goal tree ->  Start tree's the nearest configuration
		g2 = tree._nodes.back()->getConfiguration();
		c_tree = &gTree;
		a = this->ConstrainedConnect(node);
		if (a == REACHED) {
			std::cout << "FINISHED " << " " << "iteration:" << " " << iter << std::endl;
			finished = true;
			break;
		}
		iter++;
  	}
	if (finished) {
		planned_done.data = false;
		rrt_pub.publish(planned_done);

		path = tree.getPath();
		vector<vector<double>> p2;
		p2 = gTree.getPath();
		reverse(p2.begin(), p2.end());
		path.insert(path.end(), p2.begin(), p2.end());
		//cout << "path size" << path.size() << endl;
		for (int i = 0; i < path.size(); i++) {
			std::vector<double> node = path[i];
			for (int j = 0; j<node.size() - 1; j++)
				sout << node[j] << ",";

			sout << node[node.size() - 1] << "\n";
		}
	}
	else {
		return false;
	}
	return true;
}
std::vector<double> rrt::RandomConfig() {
	double goalb = (double)rand() / RAND_MAX;
	if (goalb < 0.1) {
		isGoal = true;
		return g2;
	} // goal bias�� ���� ���� ���żӵ��� �������ٳ�!
	// why ?? 

	isGoal = false;
	std::vector<double> R;
	do {
		for (int i = 0; i < start.size(); i++) {
			double jointrange = upper_limit(i) - lower_limit(i);
			double r = ((double)rand() / (double)RAND_MAX)*jointrange;
			R.push_back(lower_limit(i) + r);
		}
	} while (R.size() != goal.size());
	return R;
}
int rrt::Connect(std::vector<double> &node) { // Connect current node & current tree

	std::vector<double> robot;
	RRTNode* near = c_tree->getNearest(robot, node, DOF_weights);  // Get nearest node of NodeTree
	int s = this->Extend(node, near);
	while (s != TRAPPED) {
		if (s == REACHED) {
		//	cout << " reached" << endl;
			return REACHED;
		}
		s = this->Extend(node, near);
	}
	return TRAPPED;
}
int rrt::Extend(std::vector<double> &node, RRTNode* &near) {
	std::vector<double> robot;
	double distance = sqrt(near->getDistance(robot, node, DOF_weights));
	std::vector<double> qnew;

	// 'near' is close to 'node'
	if (distance < step_size) {
	    if (!CheckCollision(_model, node)) { // False : Collision-free
			RRTNode *old = near;
			near = (new RRTNode(node, old)); // parent node�� near��, configuration�� node
			c_tree->addNode(*near); // Tree�� node �߰�
			return REACHED;
		}
		else {
			count++;
			return TRAPPED; // Collision !!
		}
	}

	for (int i = 0; i< goal.size(); i++) {
		qnew.push_back(near->getConfiguration()[i] + ((node[i] - near->getConfiguration()[i]) / distance)*step_size);
	}
	bool check = false; 
	check = CheckCollision(_model, qnew);
	if (!check) {
		RRTNode *old = near;

		near = (new RRTNode(qnew, old));
		c_tree->addNode(*near);
		return ADVANCED; // 'near' extend 'qnew'
	}
	else {

		return TRAPPED;
	}
}
int rrt::ConstrainedConnect(std::vector<double> &node) {
	//&node : random configuration
	std::vector<double> robot;
	RRTNode* near = c_tree->getNearest(robot, node, DOF_weights); 
	int s = this->ConstrainedExtend(node, near);
	return s;
}
int rrt::ConstrainedExtend(std::vector<double> &node, RRTNode* &near) {
	// q_near & q_rand
	std::vector<double> robot;
	std::vector<double> qs, qs_old;
	bool project_flag = false;

	for (int i = 0; i < goal.size(); i++) {
		qs.push_back(near->getConfiguration()[i]);
		qs_old.push_back(near->getConfiguration()[i]);
	}

	while (true) {
		if (std_norm(node, qs, DoF_size) < step_size) {
			node = qs;
			return REACHED;
		}
		else if (std_norm(node, qs, DoF_size) - std_norm(qs_old, node, DoF_size) > 0.0) {
			node = qs_old;
			return TRAPPED;
		}
		qs_old = qs;

		for (int i = 0; i< goal.size(); i++)
			qs[i] = (qs[i] + min(step_size, std_norm(node, qs, DoF_size)) * (node[i]-qs[i]) / std_norm(node, qs, DoF_size));


		project_flag = ProjectConfig(_model, qs_old, qs); // project qs onto constraint manifold
		//return qs

		if (!project_flag) {
			cout << "project config trapped" << endl;
			node = qs_old;
			return TRAPPED;
		}
		else {
			if (!CheckCollision(_model, qs)) {
				RRTNode *old = near; //qs_old
				near = (new RRTNode(qs, old)); // 
				c_tree->addNode(*near);
			}
			else {
				cout << "Collision trapped" << endl;
				node = qs_old;
				return TRAPPED;
			}
		}
	}
}
bool rrt::ProjectConfig(Robotmodel model, std::vector<double> qold, std::vector<double> &qs) {
	bool flag = false;
	model.q.resize(dof);
	// Tc
	Matrix4d T0_c, T0_obj, Tc_obj;
	T0_c.setIdentity();
	T0_c.topRightCorner(3, 1) = refer_pos;

	MatrixXd J_temp(6, DoF_size), J(6, DoF_size), eye(6, 6);

	eye.setIdentity();
	VectorXd d_c(6), dx(6), q_error(6);
	dx.setZero();
	Vector3d phi;
	Vector3d s[3], v[3], w[3];
	Matrix3d Rotd;
	Rotd = refer_rot;

	// from degree to radian
	// to calculate jacobian in RBDL 
	for (int i = 0; i < DoF_size; i++) {
		qs[i] = qs[i] * M_PI / 180.0;
		qold[i] = qold[i] * M_PI / 180.0;
	}

	while (true) {
		for (int i = 0; i < DoF_size; i++)
			model.q(i) = qs[i];


		Matrix3d Rot_temp = Rot_arm(model.q);
		Vector3d pos_temp = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] ); // get position and rotation in EE frame;

		T0_obj.setIdentity();
		T0_obj.topLeftCorner(3, 3) = Rot_temp;
		T0_obj.topRightCorner(3, 1) = pos_temp;
		Tc_obj = T0_c.inverse() * T0_obj;

		d_c.head(3) = Tc_obj.topRightCorner(3, 1);
		d_c(3) = atan2(-Tc_obj(1, 2), Tc_obj(2, 2));
		d_c(4) = asin(Tc_obj(0, 2));
		d_c(5) = atan2(-Tc_obj(0, 1), Tc_obj(0, 0));

		for (int i = 0; i < 3; i++) {
			if (d_c(i) > C(i, 0)  ) // max 
				dx(i) = d_c(i) - C(i, 0);
			else if (d_c(i) < C(i, 1) ) // min
				dx(i) = d_c(i) - C(i, 1);
			else
				dx(i) = 0.0;
		}

		for (int i = 0; i < 3; i++)
			if (constraint_axis[i] == false)
				dx(i) = 0.0;

		for (int i = 0; i < 3; i++) {
			v[i] = Rot_temp.block(0, i, 3, 1);
			w[i] = Rotd.block(0, i, 3, 1);
			s[i] = v[i].cross(w[i]);
		}
		phi = s[0] + s[1] + s[2];
		phi = -0.5* phi;
		dx.tail(3) = 1.0*phi;


		// Algorithm 4 - line 3
		if (dx.norm() < 0.03) {
			flag = true;
			break;
		}

		// Algorithm 4 - line 4
		if (left) {
			CalcPointJacobian6D(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] , J_temp, true);
			J.topLeftCorner(3, DoF_size) = J_temp.bottomLeftCorner(3, DoF_size);
			J.bottomLeftCorner(3, DoF_size) = J_temp.topLeftCorner(3, DoF_size);
		}
		else {
			CalcPointJacobian6D(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] , J_temp, true);
			J.topLeftCorner(3, DoF_size) = J_temp.bottomLeftCorner(3, DoF_size);
			J.bottomLeftCorner(3, DoF_size) = J_temp.topLeftCorner(3, DoF_size);
		}

		// Algorithm 4 - line 5
		q_error = J.transpose() * (J*J.transpose()).inverse() * dx;

		// Algorithm 4 - line 6
		for (int i = 0; i < DoF_size; i++)
			qs[i] -= q_error(i);

		// 
		// for (int i = 0; i < DoF_size; i++)
		// 	if ((qs[i]) > M_PI)
		// 		qs[i] -= 2 * M_PI;
		// 	else if (qs[i] < -M_PI)
		// 		qs[i] += 2 * M_PI;

		// Algorithm 4 - line 7 : stuck here
		//|| std_norm(qs, qold, DoF_size) > 2 * step_size / 180.0 * M_PI
		if (!OutsideJointLimit(qs)) {
			flag = false;
			break;
		}

	}

	// rad -> deg
	for (int i = 0; i < DoF_size; i++) {
		qs[i] = qs[i] / M_PI * 180.0;
	}

	
	return flag;
}
double rrt::getDistance(std::vector<double> &a, std::vector<double> &b) {

	//Gets squared Euclidean Distance on C-Space between node and a given
	double accumulated = 0.0f;
	std::vector<double> diff = b;

	for (int i = 0; i< diff.size(); i++) {
		diff[i] -= a[i];
		//double dif = _configuration[i] - (config)[i];
		accumulated += (diff[i] * diff[i] * DOF_weights[i] * DOF_weights[i]);
	}
	return sqrt(accumulated);
}
bool rrt::SmoothPath(std::ostream& sout, std::istream& sinput)
{
	std::string fullinput;
	//Parse input
	while (sinput) {
		std::string input;
		sinput >> input;
		fullinput += input;
	}
	int nSmooth = 200; // atoi(fullinput.c_str());

	while (nSmooth) { // while(x) -> while( x != 0 )
		ShortcutSmoothing();
	//	std::cout << " path length: " << path.size() << std::endl;
		nSmooth--;
	}
	
	//std::cout<< (std::clock() - startTime)/CLOCKS_PER_SEC << std::endl;
	//std::cout <<"Length: "<<path.size();
	for (int i = 0; i< path.size(); i++) {
		std::vector<double> node = path[i];
		//sout << "[";
		for (int j = 0; j<node.size() - 1; j++) {
			sout << node[j] << ",";
		}
		sout << node[node.size() - 1] << "\n";
	}

	return true;
}
bool rrt::ShortcutSmoothing() {
	if (path.size() <= 2) return false;
	int i = ((double)rand() / RAND_MAX)*path.size();
	int j = ((double)rand() / RAND_MAX)*path.size();
	while (abs(j - i)<2) {
		i = ((double)rand() / RAND_MAX)*path.size();
		j = ((double)rand() / RAND_MAX)*path.size();
	}
	if (j < i) { //make sure i is the smaller number
		int a = j;
		j = i;
		i = a;
	}

	if (CheckTraj(path[i], path[j])) {
		path.erase(path.begin() + i + 1, path.begin() + j);
		return true;
	}
	return false;
}
bool rrt::CheckTraj(std::vector<double> &a, std::vector<double> &b) {
	std::vector<double> u = getUnitVector(a, b); // a->b
	std::vector<double> p = a;
	//std::cout << &p << "  " << &a << std::endl;
	while (p != b) {
		if (CheckCollision(_model, p)) { return false;
		break;
		}
		if (getDistance(p, b) < step_size / 2) {
			p = b;
		}
		else {
			for (int i = 0; i < u.size(); i++) {
				p[i] += u[i] * step_size / 2;
			}
		}
	}
	return true;
}
std::vector<double> rrt::getUnitVector(std::vector<double> &a, std::vector<double> &b) {

	//Gets squared Euclidean Distance on C-Space between node and a given
	double accumulated = 0.0f;
	std::vector<double> diff = b;

	for (int i = 0; i< diff.size(); i++) {
		diff[i] -= a[i];//double dif = _configuration[i] - (config)[i];
		accumulated += (diff[i] * diff[i] * DOF_weights[i] * DOF_weights[i]);
	}
	accumulated = sqrt(accumulated);
	for (int i = 0; i < diff.size(); i++) {
		diff[i] /= accumulated;
	}
	return diff;
}
bool rrt::CheckCollision(Robotmodel model, std::vector<double> &config) {
	model.q.resize(DoF_size);
	for (int i = 0; i < DoF_size; i++)
		model.q(i) = config[i] * M_PI / 180.0;

	if (!left) {
		Matrix3d Rot_temp = model.Rot*Rot_arm(model.q); // end-effector
		Box2[0].vAxis[1] = Rot_temp.col(0);
		Box2[0].vAxis[1] = Rot_temp.col(1);
		Box2[0].vAxis[2] = Rot_temp.col(2);
		Box2[0].fAxis = Vector3d(0.05, 0.05, 0.05);
		Box2[0].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] , true);

	}
	else {
		Matrix3d Rot_temp = model.Rot*Rot_arm(model.q);// end-effector
		Box2[0].vAxis[0] = Rot_temp.col(0);
		Box2[0].vAxis[1] = Rot_temp.col(1);
		Box2[0].vAxis[2] = Rot_temp.col(2);
		Box2[0].fAxis = Vector3d(0.05, 0.05, 0.10);
		Box2[0].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] , true);
	
			// door scenario 
		Matrix3d Rot_temp2 = model.Rot*Rot_arm_link5(model.q); // link 5
		Box2[1].vAxis[0] = Rot_temp2.col(0);
		Box2[1].vAxis[1] = Rot_temp2.col(1);
		Box2[1].vAxis[2] = Rot_temp2.col(2);
		Box2[1].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[4], model.com_id[4], true);

		Matrix3d Rot_temp3 = model.Rot*Rot_arm_link2(model.q); // link 2
		Box2[2].vAxis[0] = Rot_temp3.col(0);
		Box2[2].vAxis[1] = Rot_temp3.col(1);
		Box2[2].vAxis[2] = Rot_temp3.col(2);
		Box2[2].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[1], model.com_id[1], true);


		// stick 
		// Box2[3].vAxis[0] = Rot_temp.col(0);
		// Box2[3].vAxis[1] = Rot_temp.col(1);
		// Box2[3].vAxis[2] = Rot_temp.col(2);
		// Box2[3].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[6], model.com_id[6], true);

		Box2[3].vAxis[0] = Rot_temp.col(0);
		Box2[3].vAxis[1] = Rot_temp.col(1);
		Box2[3].vAxis[2] = Rot_temp.col(2);
		Box2[3].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] , true);


	}
	bool chk;
	if (box_num == 0) {
		chk = false;
		return chk;
	}
	for (int i = 0; i < box_num; i++) {
		for (int j = 0;j < box_num2; j++) {
		chk = CheckOBBCollision(&Box1[i], &Box2[j]);
		if (chk) {
			return chk;
		}
	}
	}
	return chk;
}
bool rrt::OutsideJointLimit(std::vector<double> q) {
	for (int i = 0;i < q.size();i++) {
		if (q[i] > upper_limit(i)) {
			return false;
		}
		else if (q[i] < lower_limit(i)) {
			return false;
		}
	}
	return true;

}
bool rrt::CheckOBBCollision(ST_OBB* box0, ST_OBB* box1) // Collision-free : False
{
	// compute difference of box centers,D=C1-C0
	Vector3d D = Vector3d(box1->vPos(0) - box0->vPos(0), box1->vPos(1) - box0->vPos(1), box1->vPos(2) - box0->vPos(2));

	float C[3][3];    //matrix C=A^T B,c_{ij}=Dot(A_i,B_j)
	float absC[3][3]; //|c_{ij}|
	float AD[3];      //Dot(A_i,D)
	float R0, R1, R;    //interval radii and distance between centers
	float R01;        //=R0+R1

					  //A0
	C[0][0] = FDotProduct(box0->vAxis[0], box1->vAxis[0]);// vAxis : direction // 3D Dot product
	C[0][1] = FDotProduct(box0->vAxis[0], box1->vAxis[1]);
	C[0][2] = FDotProduct(box0->vAxis[0], box1->vAxis[2]);
	AD[0] = FDotProduct(box0->vAxis[0], D);
	absC[0][0] = (float)fabsf(C[0][0]);
	absC[0][1] = (float)fabsf(C[0][1]);
	absC[0][2] = (float)fabsf(C[0][2]);
	R = (float)fabsf(AD[0]);
	R1 = box1->fAxis(0) * absC[0][0] + box1->fAxis(1) * absC[0][1] + box1->fAxis(2) * absC[0][2];
	R01 = box0->fAxis(0) + R1;
	if (R > R01)
		return 0;

	//A1
	C[1][0] = FDotProduct(box0->vAxis[1], box1->vAxis[0]);
	C[1][1] = FDotProduct(box0->vAxis[1], box1->vAxis[1]);
	C[1][2] = FDotProduct(box0->vAxis[1], box1->vAxis[2]);
	AD[1] = FDotProduct(box0->vAxis[1], D);
	absC[1][0] = (float)fabsf(C[1][0]);
	absC[1][1] = (float)fabsf(C[1][1]);
	absC[1][2] = (float)fabsf(C[1][2]);
	R = (float)fabsf(AD[1]);
	R1 = box1->fAxis(0) * absC[1][0] + box1->fAxis(1) * absC[1][1] + box1->fAxis(2) * absC[1][2];
	R01 = box0->fAxis(1) + R1;
	if (R > R01)
		return 0;

	//A2
	C[2][0] = FDotProduct(box0->vAxis[2], box1->vAxis[0]);
	C[2][1] = FDotProduct(box0->vAxis[2], box1->vAxis[1]);
	C[2][2] = FDotProduct(box0->vAxis[2], box1->vAxis[2]);
	AD[2] = FDotProduct(box0->vAxis[2], D);
	absC[2][0] = (float)fabsf(C[2][0]);
	absC[2][1] = (float)fabsf(C[2][1]);
	absC[2][2] = (float)fabsf(C[2][2]);
	R = (float)fabsf(AD[2]);
	R1 = box1->fAxis(0) * absC[2][0] + box1->fAxis(1) * absC[2][1] + box1->fAxis(2) * absC[2][2];
	R01 = box0->fAxis(2) + R1;
	if (R > R01)
		return 0;

	//B0
	R = (float)fabsf(FDotProduct(box1->vAxis[0], D));
	R0 = box0->fAxis(0) * absC[0][0] + box0->fAxis(1) * absC[1][0] + box0->fAxis(2) * absC[2][0];
	R01 = R0 + box1->fAxis(0);
	if (R > R01)
		return 0;

	//B1
	R = (float)fabsf(FDotProduct(box1->vAxis[1], D));
	R0 = box0->fAxis(0) * absC[0][1] + box0->fAxis(1) * absC[1][1] + box0->fAxis(2) * absC[2][1];
	R01 = R0 + box1->fAxis(1);
	if (R > R01)
		return 0;

	//B2
	R = (float)fabsf(FDotProduct(box1->vAxis[2], D));
	R0 = box0->fAxis(0) * absC[0][2] + box0->fAxis(1) * absC[1][2] + box0->fAxis(2) * absC[2][2];
	R01 = R0 + box1->fAxis(2);
	if (R > R01)
		return 0;

	//A0xB0
	R = (float)fabsf(AD[2] * C[1][0] - AD[1] * C[2][0]);
	R0 = box0->fAxis(1) * absC[2][0] + box0->fAxis(2) * absC[1][0];
	R1 = box1->fAxis(1) * absC[0][2] + box1->fAxis(2) * absC[0][1];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A0xB1
	R = (float)fabsf(AD[2] * C[1][1] - AD[1] * C[2][1]);
	R0 = box0->fAxis(1) * absC[2][1] + box0->fAxis(2) * absC[1][1];
	R1 = box1->fAxis(0) * absC[0][2] + box1->fAxis(2) * absC[0][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A0xB2
	R = (float)fabsf(AD[2] * C[1][2] - AD[1] * C[2][2]);
	R0 = box0->fAxis(1) * absC[2][2] + box0->fAxis(2) * absC[1][2];
	R1 = box1->fAxis(0) * absC[0][1] + box1->fAxis(1) * absC[0][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A1xB0
	R = (float)fabsf(AD[0] * C[2][0] - AD[2] * C[0][0]);
	R0 = box0->fAxis(0) * absC[2][0] + box0->fAxis(2) * absC[0][0];
	R1 = box1->fAxis(1) * absC[1][2] + box1->fAxis(2) * absC[1][1];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A1xB1
	R = (float)fabsf(AD[0] * C[2][1] - AD[2] * C[0][1]);
	R0 = box0->fAxis(0) * absC[2][1] + box0->fAxis(2) * absC[0][1];
	R1 = box1->fAxis(0) * absC[1][2] + box1->fAxis(2) * absC[1][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A1xB2
	R = (float)fabsf(AD[0] * C[2][2] - AD[2] * C[0][2]);
	R0 = box0->fAxis(0) * absC[2][2] + box0->fAxis(2) * absC[0][2];
	R1 = box1->fAxis(0) * absC[1][1] + box1->fAxis(1) * absC[1][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A2xB0
	R = (float)fabsf(AD[1] * C[0][0] - AD[0] * C[1][0]);
	R0 = box0->fAxis(0) * absC[1][0] + box0->fAxis(1) * absC[0][0];
	R1 = box1->fAxis(1) * absC[2][2] + box1->fAxis(2) * absC[2][1];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A2xB1
	R = (float)fabsf(AD[1] * C[0][1] - AD[0] * C[1][1]);
	R0 = box0->fAxis(0) * absC[1][1] + box0->fAxis(1) * absC[0][1];
	R1 = box1->fAxis(0) * absC[2][2] + box1->fAxis(2) * absC[2][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A2xB2
	R = (float)fabsf(AD[1] * C[0][2] - AD[0] * C[1][2]);
	R0 = box0->fAxis(0) * absC[1][2] + box0->fAxis(1) * absC[0][2];
	R1 = box1->fAxis(0) * absC[2][1] + box1->fAxis(1) * absC[2][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	return 1;
}
MatrixXd rrt::MergeRRTResults(MatrixXd joint_target1, MatrixXd joint_target2, int case_){
	// case 0 : RRT / RRT
	// case 1 : RRT / CBiRRT
	// case 2 : CBiRRT / RRT
	// case 3 : CBiRRT / CBiRRT 
	MatrixXd merged_joint_target;
	int row[2];
	int reminder[2];
	switch(case_ )
	{
		case 0 :
			row[0] = joint_target1.rows();
			row[1] = joint_target2.rows();
			merged_joint_target.resize(row[0] + row[1] -1, dof);
			merged_joint_target.topRows(row[0]) = joint_target1.topRows(row[0]);
			merged_joint_target.bottomRows(row[1]-1) = joint_target2.bottomRows(row[1]-1);
		break;
		case 1 :
			row[0] = joint_target1.rows();
			row[1] = joint_target2.rows() / 20;
			reminder[1] = joint_target2.rows() % 20;
			if (reminder[1] == 0)
			{
				merged_joint_target.resize(row[0] + row[1], dof);
				merged_joint_target.topRows(row[0]) = joint_target1.topRows(row[0]);
				for (int i = 0; i < row[1]; i++)
					merged_joint_target.row(row[0] + i) = joint_target2.row(20 * (i + 1));
			}
			else{
				merged_joint_target.resize(row[0] + row[1] + 1,dof);
				merged_joint_target.topRows(row[0]) = joint_target1.topRows(row[0]);
				for (int i = 0; i < row[1]; i++)
					merged_joint_target.row(row[0] + i) = joint_target2.row(20 * (i + 1));

				merged_joint_target.bottomRows(1) = joint_target2.bottomRows(1);
			}
			break; 
		case 2 :
			row[0] = joint_target1.rows() /20;
			row[1] = joint_target2.rows() ;
			reminder[0] = joint_target1.rows() % 20;
			if (reminder[0] == 0)
			{
				merged_joint_target.resize(row[0] + row[1],dof);
				for (int i = 0; i < row[0]; i++)
					merged_joint_target.row(i) = joint_target1.row(20 * (i));

				merged_joint_target.bottomRows(row[1]) = joint_target2.bottomRows(row[1]);
			}
			else{
				merged_joint_target.resize(row[0] + row[1] + 1,dof);
				for (int i = 0; i < row[0] + 1; i++)
					merged_joint_target.row(i) = joint_target1.row(20 * (i));

				merged_joint_target.bottomRows(row[1]) = joint_target2.bottomRows(row[1]);
			}
		break;
		case 3 :
			row[0] = joint_target1.rows() /20;
			row[1] = joint_target2.rows() /20;
			reminder[0] = joint_target1.rows() % 20;
			reminder[1] = joint_target2.rows() % 20;
			if (reminder[0] == 0){
				if (reminder[1] == 0){
					merged_joint_target.resize(row[0] + row[1] + 1,dof);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 1 + i) = joint_target2.row(20 * (i+1));
				}
				else{
					merged_joint_target.resize(row[0] + row[1] + 2,dof);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 1 + i) = joint_target2.row(20 * (i+1));

					merged_joint_target.bottomRows(1) = joint_target2.bottomRows(1);
				}
			}
			else{
				if (reminder[1] == 0){
					merged_joint_target.resize(row[0] + row[1] + 2,dof);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					merged_joint_target.row(row[0]+1) = joint_target1.bottomRows(1);

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 2 + i) = joint_target2.row(20 * (i+1));
				}
				else{
					merged_joint_target.resize(row[0] + row[1] + 3,dof);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					merged_joint_target.row(row[0]+1) = joint_target1.bottomRows(1);

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 2 + i) = joint_target2.row(20 * (i+1));

					merged_joint_target.bottomRows(1) = joint_target2.bottomRows(1);
				}
			}
		break;
	}


	return merged_joint_target;
}
