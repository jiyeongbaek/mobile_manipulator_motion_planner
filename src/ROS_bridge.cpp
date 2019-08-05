#include "ROS_bridge.hpp"

ROS_bridge::ROS_bridge(ros::NodeHandle nh_,double hz_):
    rate_(hz_)
  {    

    current_ql_.resize(dof);
    current_ql_.setZero();
    desired_ql_.resize(dof);
    desired_ql_.setZero();

    current_qr_.resize(dof);
    current_qr_.setZero();
    desired_qr_.resize(dof);
    desired_qr_.setZero();

    current_mob_pos_.resize(3);
    current_mob_pos_.setZero();
    desired_mob_pos_.resize(3);
    desired_mob_pos_.setZero();

    desired_goal_pose_.resize(6);
    desired_goal_pose_.setZero();

    ljoint_cmd_.position.resize(dof);
    rjoint_cmd_.position.resize(dof);

    current_position_.resize(3);
    current_position_.setZero();

    rotation_M.resize(3, 3);
    rotation_M.setZero();

    ros_ljoint_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/planned_arm_trajectory", 1);
    ros_mob_set_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/planned_mob_trajectory", 1);
    ros_ljoint_state_sub_ = nh_.subscribe("/joint_states", 100, &ROS_bridge::ljoint_cb, this);
    ros_mob_state_sub_ = nh_.subscribe("/odometry/filtered", 100, &ROS_bridge::mob_cb, this);

    ros_ee_goal_state_sub_ = nh_.subscribe("/goal_pose", 100, &ROS_bridge::goal_pose_cb, this);
    ros_current_pose_sub_ = nh_.subscribe("/franka_states/current_pose", 1, &ROS_bridge::current_pose_cb, this);
  }
  ROS_bridge::~ROS_bridge()
  {
    ros_ljoint_set_pub_.shutdown();
    ros_mob_set_pub_.shutdown();
  }

  void ROS_bridge::mob_cb(const geometry_msgs::Pose2DConstPtr &msg){

  current_mob_pos_(0) = msg->x;
  current_mob_pos_(1) = msg->y;
  current_mob_pos_(2) = msg->theta;
  }

  void ROS_bridge::ljoint_cb(const sensor_msgs::JointStateConstPtr &msg)
  {
    for (size_t i = 0; i < 7; i++)
    {
      current_ql_[i] = msg->position[i];
    }
  }

  void ROS_bridge::rjoint_cb(const sensor_msgs::JointStateConstPtr &msg)
  {
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      current_qr_[i] = msg->position[i];
    }
  }

  void ROS_bridge::goal_pose_cb(const std_msgs::Float32MultiArrayConstPtr& msg)
  {

    for (size_t i = 0; i < 6; i++)
      desired_goal_pose_[i] = msg->data[i];
  }

  void ROS_bridge::current_pose_cb(const geometry_msgs::PoseConstPtr &msg)
  {
    current_position_(0) = msg->position.x;
    current_position_(1) = msg->position.y;
    current_position_(2) = msg->position.z;

    Eigen::Quaterniond a(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    rotation_M = a.toRotationMatrix();
  }

  void ROS_bridge::write_robot()
  {
    for(size_t i=0;i<dof;i++) {
        ljoint_cmd_.position[i] = desired_ql_[i];
    }
    mob_cmd_.x = desired_mob_pos_(0);
    mob_cmd_.y = desired_mob_pos_(1);
    mob_cmd_.theta = desired_mob_pos_(2);

    ros_mob_set_pub_.publish(mob_cmd_);
    ros_ljoint_set_pub_.publish(ljoint_cmd_);
    rate_.sleep();
 
  }

  void ROS_bridge::wait()
  {
    // while(ros::ok())
    // {
    //   ros::spinOnce();
    // }
  }