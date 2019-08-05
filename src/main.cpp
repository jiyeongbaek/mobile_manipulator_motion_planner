#include <iostream>
#include <string>
#include "ROS_bridge.hpp"
#include "controller.h"

#include <sensor_msgs/JointState.h>

// for vrep keyboard event
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <std_msgs/String.h>

using namespace std;
std::string state_;

void state_cb(const std_msgs::String::ConstPtr& msg)
  {
	
    state_ = msg->data;

  }
static constexpr uint32_t HashCode(const char *p) {

    return *p ? static_cast<uint32_t>(*p) + 33 * HashCode(p + 1) :  5381;

  }


int main(int argc, char** argv)
{
	ros::init(argc, argv, "assembly_vrep");
	ros::NodeHandle nh("~");
	
	const double hz = 1000;
	ROS_bridge rb(nh, hz);
	
	ros::Subscriber state_trans_sub = nh.subscribe("/assembly/state_transition", 100, &state_cb);
	Controller ac(nh);
	nh.param("urdf_param", ac.urdf_param, std::string("/robot_description"));

	ac.initModel();

	bool isSimulationRun = true;
	bool exitFlag = false;	
	std_msgs::Bool msg;
	std_msgs::Bool msg_open;
	ros::Publisher gripper_pub = nh.advertise<std_msgs::Bool>("/gripper_close", 100);
	ros::Publisher gripper_open_pub = nh.advertise<std_msgs::Bool>("/gripper_open", 100);
	while (ros::ok() && !exitFlag)
	{
		ros::spinOnce();
		ac.readdata(rb.current_ql_, rb.current_mob_pos_, rb.desired_goal_pose_, rb.current_position_, rb.rotation_M);
		uint32_t hash = HashCode(state_.c_str());
		ac.joint_target_left_.desired_q_ = rb.current_ql_;
		switch (hash)
			{
			case HashCode("approach1-1"):
				ROS_INFO("APPROACH1");
				ac.setMode(Controller::APPROACH1);
				break;
			
			case HashCode("approach1-2"):
				ROS_INFO("APPROACH2");
				ac.setMode(Controller::APPROACH2);
				break;
			case HashCode("approach2-1"):
				ROS_INFO("APPROACH3");
				ac.setMode(Controller::APPROACH3);
				break;
			case HashCode("approach2-2"):
				ROS_INFO("APPROACH4");
				ac.setMode(Controller::APPROACH4);
				break;
			case HashCode("approach3-1"):
				ROS_INFO("APPROACH5");
				ac.setMode(Controller::APPROACH5);
				break;
			case HashCode("approach3-2"):
				ROS_INFO("APPROACH6");
				ac.setMode(Controller::APPROACH6);
				break;
		
			case HashCode("grasp"):
				ROS_INFO("GRASPING");
				msg.data = true;
				gripper_pub.publish(msg);
				break;

			case HashCode("open"):
				ROS_INFO("GRIPPER OPEN");
				msg_open.data = true;
				gripper_open_pub.publish(msg_open);
				break;
			
			// case 'q':
			// 	isSimulationRun = false;
			// 	exitFlag = true;
			// 	break;
			
			
			// svb.wait();
		}
		state_ = 'default';
		ac.compute();
		ac.writedata(rb.desired_ql_, rb.desired_mob_pos_);
		rb.write_robot(); // publish desired q, success
		//vb.wait();		
		
	
	}
	ros::shutdown();
	
	return 0;
}