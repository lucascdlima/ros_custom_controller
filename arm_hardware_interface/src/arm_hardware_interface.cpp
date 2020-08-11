#include "arm_hardware_interface/arm_hardware_interface.h"

ManipulatorRobot::ManipulatorRobot(ros::NodeHandle& nh) : nh_(nh)
{
 init();
 //controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
 //nh_.param("/ROBOT/hardware_interface/loop_hz", loop_hz_, 0.1);
 //ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
         //non_realtime_loop_ = nh_.createTimer(update_freq, &TR1HardwareInterface::update, this);

}

void ManipulatorRobot::init()
{

  //nh_.getParam("/ROBOT/hardware_interface/joints", joint_names_); //I need create a ros param config with joint names to set this variable
  //num_jnts_ = joint_names_.size();
  num_jnts_ = 1;
  joint_names_.resize(num_jnts_);
 /* for (unsigned int i = 0; i < num_jnts_; ++i)
  {
    char buffer [50];
    std::sprintf(buffer,"Joint%d",i);
    joint_names_[i] = std::string(buffer);
  }*/
  joint_names_[0] = "Joint1";

  joint_position_.resize(num_jnts_);
  joint_velocity_.resize(num_jnts_);
  joint_effort_.resize(num_jnts_);
  joint_position_command_.resize(num_jnts_);
  joint_velocity_command_.resize(num_jnts_);
  joint_effort_command_.resize(num_jnts_);

  for (unsigned int i = 0; i < num_jnts_; ++i)
  {
    // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
   // jnt_state_interface_.registerHandle(jointStateHandle);

    // Create position joint interface
   // hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);

   // jnt_pos_interface_.registerHandle(jointPositionHandle);

    // Create effort joint interface
    hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
    jnt_effort_interface_.registerHandle(jointEffortHandle);
  }

  //registerInterface(&jnt_state_interface_);
  //registerInterface(&jnt_pos_interface_);
  registerInterface(&jnt_effort_interface_);
  //registerInterface(&positionJointSoftLimitsInterface);
  ROS_INFO("hardware interface initiated");

}

void ManipulatorRobot::read(const ros::Time &etime, const ros::Duration &duration)
{
  for (unsigned int i = 0; i < num_jnts_; i++) {
              joint_position_[i] = 0.1;
          }
  ROS_INFO("read executed");
}

void ManipulatorRobot::write(const ros::Time &etime, const ros::Duration &duration)
{
  for (unsigned int i = 0; i < num_jnts_; i++) {
              joint_effort_command_[i] = 0.2;
          }
  ROS_INFO("write executed");
}

/*void ManipulatorRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}
*/

