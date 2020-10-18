#include "arm_hardware_interface/arm_hardware_interface.h"


ManipulatorRobot::ManipulatorRobot(ros::NodeHandle& nh) : nh_(nh)
{

}

bool ManipulatorRobot::init()
{
  //Get the ros param joints names (joints to be controlled)
  if (!nh_.getParam("/arm_robot/hardware_interface/joints", joint_names_)){
    ROS_ERROR("Could not find joints names");
    return false;
  }

  num_jnts_ = joint_names_.size();

  ROS_INFO_STREAM("Number of joints = "<<num_jnts_);

  joint_position_.resize(num_jnts_);
  joint_velocity_.resize(num_jnts_);
  joint_effort_.resize(num_jnts_);
  joint_effort_command_.resize(num_jnts_);
  joints_states.resize(num_jnts_);

  for (unsigned int i = 0; i < num_jnts_; ++i)
  {
    // Creates joint state handle and provide reference to joints states and command vectors
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

/**
 * @brief ManipulatorRobot::read Overloaded method to read data from hardware interface (robot).
 * Get joints positons, velocities and efforts from hardware interface, set respective values to internal attributes and send to
 * ros controller.
 * @param etime ros elapsed time since controller start.
 * @param duration ros duration time between each call of read method.
 */
void ManipulatorRobot::read(const ros::Time &etime, const ros::Duration &duration)
{
  //Pass joints data from joints handles (ros hardware interface) to internal joints states vectors
  for (uint i = 0; i < num_jnts_; i++) {
    joint_position_[i] = joints_states[i].position;
    joint_velocity_[i] = joints_states[i].velocity;
    joint_effort_[i] = joints_states[i].effort;
  }

}

/**
 * @brief ManipulatorRobot::write Overloaded method to write data (commands) to hardware interface (robot).
 * Gets the effort commands sent from ros node controller and sends to hardware by ros topic publish interface.
 * @param etime ros elapsed time since controller start.
 * @param duration ros duration time between each call of write method.
 * @param pub ros Publisher object to send messages to hardware through network.
 */
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

/**
 * @brief ManipulatorRobot::SubJointsStates Ros subscriber callback.
 * Method called each time a message with hardware joint states is received from a ros topic. After joints states
 * are received the values are passed to ros controller through read method call.
 * @param msg Message received with joints states (effort, position and velocities)
 */
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


