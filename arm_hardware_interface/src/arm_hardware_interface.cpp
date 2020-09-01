#include "arm_hardware_interface/arm_hardware_interface.h"


ManipulatorRobot::ManipulatorRobot(ros::NodeHandle& nh) : nh_(nh)
{

}

bool ManipulatorRobot::init()
{
  //Get the joints names set as a ros parameter of our robot hardware interface
  if (!nh_.getParam("/arm_robot/hardware_interface/joints", joint_names_)){
    ROS_ERROR("Could not find joints names");
    return false;
  }

  num_jnts_ = joint_names_.size();

  ROS_INFO_STREAM("Number of joints = "<<num_jnts_);

  joint_position_.resize(num_jnts_);
  joint_velocity_.resize(num_jnts_);
  joint_effort_.resize(num_jnts_);
  joint_position_command_.resize(num_jnts_);
  joint_velocity_command_.resize(num_jnts_);
  joint_effort_command_.resize(num_jnts_);
  joints_states.resize(num_jnts_);

  for (unsigned int i = 0; i < num_jnts_; ++i)
  {
    // Creates joint state handle and provide refecente to joints states and command vectors
    hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);

    // Creates joint handle, passes effort command vector reference and registers it in a EffortJointInterface
    hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
    jnt_effort_interface_.registerHandle(jointEffortHandle);
  }

  //Registers the EffortJointInterface in the ros interface manager
  registerInterface(&jnt_effort_interface_);

  ROS_INFO("hardware interface initiated");

  return true;

}

void ManipulatorRobot::read(const ros::Time &etime, const ros::Duration &duration)
{
  //Pass joints data from joints handles (ros hardware interface) to internal joints states vectors
  for (uint i = 0; i < num_jnts_; i++) {
    joint_position_[i] = joints_states[i].position;
    joint_velocity_[i] = joints_states[i].velocity;
    joint_effort_[i] = joints_states[i].effort;
  }

}

void ManipulatorRobot::write(const ros::Time &etime, const ros::Duration &duration, ros::Publisher &pub)
{
  std_msgs::Float64MultiArray msg;
  msg.data.resize(num_jnts_);

  //Sets ros msg to publish effort commands received from joint handle (ros hardware interface)
  for (uint i = 0; i < num_jnts_; i++)
  {
    msg.data[i] = joint_effort_command_[i];
  }
  pub.publish(msg);
}

void ManipulatorRobot::SubJointsStates(const sensor_msgs::JointStateConstPtr& msg)
{
  //Get joints states received from simulated hardware through ROS topic
  for(uint i=0; i<num_jnts_; i++)
  {
    joints_states[i].effort = msg->effort[i];
    joints_states[i].position = msg->position[i];
    joints_states[i].velocity = msg->velocity[i];
  }

}


