
#ifndef SIMULATION_INTERFACE_H
#define SIMULATION_INTERFACE_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "controller.h"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>

#define TOTAL_DOF 9 // 9 dof robot(7 + 2 gripper)
#define SIM_DT 0.01 // 10ms
#define PI 3.14159265359
#define deg2rad(deg) ((deg)*PI / 180.0)
#define rad2deg(rad) ((rad)*180.0 / PI)

class ROS_bridge
{
public:
    ROS_bridge(ros::NodeHandle nh_, double hz_);
    ~ROS_bridge();

    void ljoint_cb(const sensor_msgs::JointStateConstPtr &msg);
    void rjoint_cb(const sensor_msgs::JointStateConstPtr &msg);
    void mob_cb(const geometry_msgs::Pose2DConstPtr &msg);
    void goal_pose_cb(const std_msgs::Float32MultiArrayConstPtr &msg);
    void current_pose_cb(const geometry_msgs::PoseConstPtr &msg);

    void write_robot();
    void wait();

    Eigen::VectorXd current_ql_, current_qr_;
    Eigen::VectorXd desired_ql_, desired_qr_;
    Eigen::VectorXd current_mob_pos_;
    Eigen::VectorXd desired_mob_pos_;
    Eigen::VectorXd desired_goal_pose_;
    Eigen::VectorXd current_position_;
    Eigen::MatrixXd rotation_M;

private:
    ros::Publisher ros_ljoint_set_pub_;
    ros::Publisher ros_rjoint_set_pub_;
    ros::Publisher ros_gripper_set_pub;
    ros::Publisher ros_mob_set_pub_;

    ros::Subscriber ros_ljoint_state_sub_;
    ros::Subscriber ros_rjoint_state_sub_;
    ros::Subscriber ros_mob_state_sub_;
    ros::Subscriber ros_ee_goal_state_sub_;
    ros::Subscriber ros_current_pose_sub_;
    ros::Rate rate_;
    sensor_msgs::JointState ljoint_cmd_;
    sensor_msgs::JointState rjoint_cmd_;
    sensor_msgs::JointState gripper_cmd_;
    geometry_msgs::Pose2D mob_cmd_;
};

#endif
