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

    KDL::SegmentMap::iterator itr;


    for (itr = ++robot_segment_map.begin(); itr != robot_segment_map.end() ; ++itr)
    {
      cout <<"segment name = " << itr->second.segment.getName()<<endl;
      cout<<"joint type = "<<itr->second.segment.getJoint().getTypeName()<<endl;
      cout<<"Inertia matrix"<<Map<Matrix<double,3,3,RowMajor> >(itr->second.segment.getInertia().getRotationalInertia().data)<<endl;
    }

    string root_segment = robot_segment_map.begin()->second.segment.getName();
    string tip_segment = (--robot_segment_map.end())->second.segment.getName();//itr2->second.segment.getName();
    cout <<"root segment = " << root_segment<<endl;
    cout<<"tip segment = "<<tip_segment<<endl;

    if(!robot_tree.getChain("base_link","link_7", robot_chain))
    {
      cout<< "chain not acquired correctly!!"<<endl;
    }
    else
    {
      std::vector<Segment> robot_segments = robot_chain.segments;
      jnts_names.resize(num_joints);

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




  /*  chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
  chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
  chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
  chain.addSegment(Segment(Joint(Joint::RotZ)));
  chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
  chain.addSegment(Segment(Joint(Joint::RotZ)));*/


}

JntArray RobotKDL::UpdateDynamic()
{


  /* for(unsigned int i=0;i<num_joints;i++)
  {
    jnts_effort_command.data(i) = 1.0;
    jnts_pos.data(i) = 1.5;

  }*/
  /* vector<Segment> chain_segments = robot_chain.segments;
  for (int i =0; i<num_joints; i++)
  {
    cout<<"Chain joint name "<<"["<<i<<"]"<<"="<<chain_segments[i].getName()<<endl;
  }*/

  chain_dynamics->JntToMass(jnts_pos,M_inertia);

  chain_dynamics->JntToCoriolis(jnts_pos,jnts_vel,C_coriolis);

  chain_dynamics->JntToGravity(jnts_pos,G_gravity);

  jnts_acc.data = M_inertia.data.inverse()*(jnts_effort_command.data - C_coriolis.data - G_gravity.data);

  /*
  cout<<"M inertia = "<<endl;
  cout<<M_inertia.data<<endl;
  cout<<"C coriolis = "<<C_coriolis.data<<endl;
  cout<<"G gravity = "<<G_gravity.data<<endl;
  cout<<"Joints accelerations = "<<jnts_acc.data<<endl;*/

  return jnts_acc;

}

void RobotKDL::SetEffortCommand(const VectorXd& jnts_effort)
{
  /*Map<Matrix<double,7,1>>effort_temp(effort_jnts);
  jnts_effort_command.data = effort_temp.derived();*/
  jnts_effort_command.data = jnts_effort;
}

void RobotKDL::SetJointsPosition(const VectorXd& jnts_position)
{
  /*Map<Matrix<double,7,1>>jnt_pos_temp(jnts_position);
  jnts_pos.data = jnt_pos_temp.derived();*/
  jnts_pos.data = jnts_position;
}

void RobotKDL::SetJointsVelocity(const VectorXd& jnts_velocity)
{
  /* Map<Matrix<double,7,1>>jnt_vel_temp(jnts_velocity);
  jnts_vel.data = jnt_vel_temp.derived();*/
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
  VectorXd command;
  command.resize(num_joints);

  for (unsigned int i=0; i<num_joints; i++)
    command(i) = msg->data[i];

  SetEffortCommand(command);

}

void RobotKDL::SetJointStatesMsg(sensor_msgs::JointState &msg)
{
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

int main(int argc, char* argv[])
{

  // RobotKDL myrobot("/home/lucaslima/catkin_test_ws/src/robot_kdl/kuka_urdf/kuka_urdf_test.urdf");
  //myrobot.UpdateDynamic();

  return 0;

}

