#include "robot_kdl/robot_kdl.h"
#include <iostream>

using namespace KDL;
using namespace std;
using namespace Eigen;


RobotKDL::RobotKDL(const char* urdf_file, ros::NodeHandle &n, ros::Duration loop_period)
{
  urdf_file_path = urdf_file;
  nh = n;
  dt = loop_period.toSec();
  chain_dynamics = nullptr;
}

bool RobotKDL::Init()
{
   //Tries to parse robot urdf model to KDL::chain class
  if (!kdl_parser::treeFromFile(urdf_file_path, robot_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return 0;
  }

  ROS_INFO("KDL tree constructed by urdf");

  num_joints = robot_tree.getNrOfJoints();

  KDL::SegmentMap robot_segment_map = robot_tree.getSegments();

  //Tries to get a chain from specified link segments (base to end effector)
  if(!robot_tree.getChain("base_link","link_7", robot_chain))
  {
    ROS_ERROR("chain not acquired correctly!!");
    return 0;
  }

  std::vector<Segment> robot_segments = robot_chain.segments;
  jnts_names.resize(num_joints);

  //Get movable joints names in the chain
  for(unsigned int i=1; i<robot_segments.size(); i++)
    jnts_names[i-1] = robot_segments[i].getJoint().getName();

  jnts_pos.resize(num_joints);
  jnts_vel.resize(num_joints);
  jnts_effort_command.resize(num_joints);
  jnts_acc.resize(num_joints);
  SetToZero(jnts_pos);
  SetToZero(jnts_vel);
  SetToZero(jnts_effort_command);
  SetToZero(jnts_acc);

  KDL::Vector grav (0,0,-9.81);
  chain_dynamics = new ChainDynParam(robot_chain,grav);

  M_inertia.resize(num_joints);
  C_coriolis.resize(num_joints);
  G_gravity.resize(num_joints);

  iter_count = 0;

  return 1;

}

/**
 * @brief RobotKDL::UpdateDynamic Method to update robot dynamic data.
 * Gets current joints states and effort commands received from controller and updates lagrange
 * parameters of manipulator forward dynamics (Inertia matrix, Coriolis and Gravity vectors).
 * @return Joints accelerations in KDL::JntArray.
 */
JntArray RobotKDL::UpdateDynamic()
{
  //Gets manipulator dynamic parameters based on joints states
  chain_dynamics->JntToMass(jnts_pos,M_inertia);
  chain_dynamics->JntToCoriolis(jnts_pos,jnts_vel,C_coriolis);
  chain_dynamics->JntToGravity(jnts_pos,G_gravity);

  //Compute joints accelerations using Lagrange dynamic model and forward dynamics
  jnts_acc.data = M_inertia.data.inverse()*(jnts_effort_command.data - C_coriolis.data - G_gravity.data);

  iter_count+=1;

  return jnts_acc;

}

void RobotKDL::SetEffortCommand(const VectorXd& jnts_effort)
{
  jnts_effort_command.data = jnts_effort;
}

void RobotKDL::SetJointsPosition(const VectorXd& jnts_position)
{
  jnts_pos.data = jnts_position;
}

void RobotKDL::SetJointsVelocity(const VectorXd& jnts_velocity)
{
  jnts_vel.data = jnts_velocity;
}

JntArray RobotKDL::GetJointsVelocity()
{
  return jnts_vel;
}

JntArray RobotKDL::GetJointsPosition()
{
  return jnts_pos;
}

uint RobotKDL::GetNumJoints()
{
  return num_joints;
}

RobotKDL::~RobotKDL()
{
  if(chain_dynamics!=nullptr)
    delete chain_dynamics;
}

/**
 * @brief RobotKDL::SubEffortCommand Ros Callback to get effort command messages from controller.
 * This callback is subscribed to ros topic /arm_effort_command for receiving effort commands from
 * controller and set internal attribute.
 * @param msg Array with effort commands for each joint.
 */
void RobotKDL::SubEffortCommand(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  for (unsigned int i=0; i<num_joints; i++)
    jnts_effort_command.data[i] = msg->data[i];
}

/**
 * @brief RobotKDL::SetJointStatesMsg Method to set joints states msg with manipulator joints data
 * updated from internal simulation. Joints states will be published later on in the /arm_joints_states ros topic.
 * @param msg ros message with joints states.
 */
void RobotKDL::SetJointStatesMsg(sensor_msgs::JointState &msg)
{
  msg.name.resize(num_joints);
  msg.effort.resize(num_joints);
  msg.position.resize(num_joints);
  msg.velocity.resize(num_joints);

  for(unsigned int i=0; i<num_joints; i++)
  {
    msg.name[i] = jnts_names[i];
    msg.effort[i] = jnts_effort_command.data[i];
    msg.position[i] = jnts_pos.data(i);
    msg.velocity[i] = jnts_vel.data(i);
    msg.header.stamp = ros::Time::now();
  }

}

/**
 * @brief RobotKDL::InitControlParam Method to set internal control params when internal control is
 * used in simulation.
 */
void RobotKDL::InitControlParam()
{
  wn = 2*EIGEN_PI/4;
  jnt_u.resize(num_joints);
  KDL::SetToZero(jnt_u);

  Kp.resize(num_joints,num_joints);
  Kd.resize(num_joints,num_joints);
  Kp = MatrixXd::Identity(num_joints, num_joints);
  Kd = MatrixXd::Identity(num_joints, num_joints);

  q_des.resize(num_joints);
  dq_des.resize(num_joints);
  ddq_des.resize(num_joints);
}

/**
 * @brief RobotKDL::ComputedTorqueControlExample Method used for testing purposes.
 * If ros parameter internal_control is set to true the simulation of manipulator dynamics is performed
 * along with a internal control. Effort commands are then set by this method using computed torque control approach.
 */
void RobotKDL::ComputedTorqueControlExample()
{

  for(uint i=0;i<num_joints;i++)
  {
    //Caculates desired joints positions, velocities and accelarations evolutions
    q_des(i) = sin(dt*iter_count*wn);
    dq_des(i) = wn*cos(dt*iter_count*wn);
    ddq_des(i) = -wn*wn*sin(dt*iter_count*wn);
  }

  //Updates robot dynamics parameters
  chain_dynamics->JntToMass(jnts_pos,M_inertia);
  chain_dynamics->JntToCoriolis(jnts_pos,jnts_vel,C_coriolis);
  chain_dynamics->JntToGravity(jnts_pos,G_gravity);

  //Calculates computed torque control and sets joints command effort data
  jnt_u.data = Kp*(q_des - jnts_pos.data) + Kd*(dq_des - jnts_vel.data);
  jnts_effort_command.data = M_inertia.data*(ddq_des + jnt_u.data ) + C_coriolis.data + G_gravity.data;

}
