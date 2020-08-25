#include "robot_kdl/robot_kdl.h"
#include <iostream>

using namespace KDL;
using namespace std;
using namespace Eigen;

RobotKDL::RobotKDL()
{

}

RobotKDL::RobotKDL(const char* urdf_file, ros::NodeHandle &n)
{
  //Tries to parse robot urdf model to KDL::chain class
  if (!kdl_parser::treeFromFile(urdf_file, robot_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    cout<<"Failed to construct kdl tree by urdf"<<endl;
  }
  else
  {
    cout<<" KDL tree constructed by urdf !!! "<<endl;
    num_joints = robot_tree.getNrOfJoints();

    KDL::SegmentMap robot_segment_map = robot_tree.getSegments();

    //Tries to get a chain from specified link segments (base to end effector)
    if(!robot_tree.getChain("base_link","link_7", robot_chain))
    {
      cout<< "chain not acquired correctly!!"<<endl;
    }
    else
    {
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


    }
  }


}

JntArray RobotKDL::UpdateDynamic()
{
  /* Function to update manipulator dynamics and return joints accelerations based on robot forward dynamics.
     This function is used in simulation of robot dynamics.
  */

  //Gets manipulator dynamic parameters based on joints states
  chain_dynamics->JntToMass(jnts_pos,M_inertia);
  chain_dynamics->JntToCoriolis(jnts_pos,jnts_vel,C_coriolis);
  chain_dynamics->JntToGravity(jnts_pos,G_gravity);

  //Compute joints accelerations using Lagrange dynamic model and forward dynamics
  jnts_acc.data = M_inertia.data.inverse()*(jnts_effort_command.data - C_coriolis.data - G_gravity.data);

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

RobotKDL::~RobotKDL()
{

}

void RobotKDL::SubEffortCommand(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  /* Callback function to subscribe ros topic of commanded joints torques and set internal
    effort joint command attribute. */

  VectorXd command;
  command.resize(num_joints);

  for (unsigned int i=0; i<num_joints; i++)
    command(i) = msg->data[i];

  SetEffortCommand(command);

}

void RobotKDL::SetJointStatesMsg(sensor_msgs::JointState &msg)
{
  /* Function to set a JointState msg with manipulator joints data updated
     by internal simulation */

  msg.name.resize(num_joints);
  msg.effort.resize(num_joints);
  msg.position.resize(num_joints);
  msg.velocity.resize(num_joints);

  for(unsigned int i=0; i<num_joints; i++)
  {
    msg.name[i] = jnts_names[i];
    msg.effort[i] = 0;
    msg.position[i] = jnts_pos.data(i);
    msg.velocity[i] = jnts_vel.data(i);
    msg.header.stamp = ros::Time::now();
  }

}

void RobotKDL::InitControlParam(ros::Time stime)
{
  /*Function to initiate parameters used in computation of manipulator dynamics. This function
    is used only when internal control is performed. */

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

  start_time = stime.now();
}

void RobotKDL::ComputedTorqueControlExample(ros::Time time)
{
  /* Calculates the computed torque control based on manipulator Forward Dynamics and sets
     joints effort commands.
  */

  for(uint i=0;i<num_joints;i++)
  {
    //Caculates desired joints positions, velocities and accelarations evolutions based on ros time

    q_des(i) = sin((time.toSec() - start_time.toSec())*wn);
    dq_des(i) = wn*cos((time.toSec() - start_time.toSec())*wn);
    ddq_des(i) = -wn*wn*sin((time.toSec() - start_time.toSec())*wn);
  }

  //Updates robot dynamics parameters
  chain_dynamics->JntToMass(jnts_pos,M_inertia);
  chain_dynamics->JntToCoriolis(jnts_pos,jnts_vel,C_coriolis);
  chain_dynamics->JntToGravity(jnts_pos,G_gravity);

  //Calculates computed torque control and sets joints command effort data
  jnt_u.data = Kp*(q_des - jnts_pos.data) + Kd*(dq_des - jnts_vel.data);
  jnts_effort_command.data = M_inertia.data*(ddq_des + jnt_u.data ) + C_coriolis.data + G_gravity.data;


}
