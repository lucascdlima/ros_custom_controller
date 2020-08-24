#include "arm_controller/arm_effort_controller.h"

using namespace KDL;
using namespace Eigen;

namespace arm_controller_ns{

ArmEffortController::ArmEffortController()
{

}
ArmEffortController::~ArmEffortController()
{

}

bool ArmEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{

  if (!hw)
  {
    ROS_ERROR("The given robot was NULL");
    return false;
  }
  nh_ = n;

  if (!nh_.getParam("/arm_effort_controller_obj/joints", joint_names_)){
    ROS_ERROR("Could not find joints names");
    return false;
  }

  if (!nh_.getParam("/arm_effort_controller_obj/urdf_model", urdf_model_)){
    ROS_ERROR("Could not find robot urdf model path");
    return false;
  }

  if (!kdl_parser::treeFromFile(urdf_model_, robot_tree))
  {
    ROS_ERROR("Failed to construct KDL::tree from urdf file");
    return false;
  }

  num_jnts_ = static_cast<uint>(joint_names_.size());
  joints_.resize(num_jnts_);
  for(uint i=0;i<num_jnts_;i++)
    joints_[i] = hw->getHandle(joint_names_[i]);

  if(!robot_tree.getChain("base_link","link_7", robot_chain))
  {
    ROS_ERROR("Failed to get KDL::chain");
    return false;
  }

  std::vector<Segment> robot_segments = robot_chain.segments;

  jnts_pos.resize(num_jnts_);
  jnts_vel.resize(num_jnts_);
  jnts_effort_command.resize(num_jnts_);
  jnts_acc.resize(num_jnts_);
  SetToZero(jnts_pos);
  SetToZero(jnts_vel);
  SetToZero(jnts_effort_command);
  SetToZero(jnts_acc);

  KDL::Vector grav (0,0,-9.81);
  chain_dynamics = new ChainDynParam(robot_chain,grav);

  M_inertia.resize(num_jnts_);
  C_coriolis.resize(num_jnts_);
  G_gravity.resize(num_jnts_);

  wn = 2*EIGEN_PI/2;
  jnt_u.resize(num_jnts_);
  KDL::SetToZero(jnt_u);

  Kp = MatrixXd::Identity(7, 7);
  Kd = MatrixXd::Identity(7, 7);

  return true;
}

void ArmEffortController::starting(const ros::Time& time)
{
  ROS_INFO("Arm Controller - STARTED ");
  start_time = time.now();
  ROS_INFO_STREAM( "Start time set: " <<start_time.toSec());
}

void ArmEffortController::update(const ros::Time& time, const ros::Duration& period)
{
  for(uint i=0;i<num_jnts_;i++)
  {
    jnts_pos.data(i) = joints_[i].getPosition();
    jnts_vel.data(i) = joints_[i].getVelocity();
    q_des(i) = sin((time.now().toSec() - start_time.toSec())*wn);
    dq_des(i) = wn*cos((time.now().toSec() - start_time.toSec())*wn);
    ddq_des(i) = -wn*wn*sin((time.now().toSec() - start_time.toSec())*wn);

  }

  chain_dynamics->JntToMass(jnts_pos,M_inertia);

  chain_dynamics->JntToCoriolis(jnts_pos,jnts_vel,C_coriolis);

  chain_dynamics->JntToGravity(jnts_pos,G_gravity);

  jnt_u.data = Kp*(q_des - jnts_pos.data) + Kd*(dq_des - jnts_vel.data);
  jnts_effort_command.data = M_inertia.data*(ddq_des + jnt_u.data ) + C_coriolis.data + G_gravity.data;

  for(uint i=0;i<num_jnts_;i++)
    joints_[i].setCommand(jnts_effort_command.data(i));



  // ROS_INFO_STREAM( "Time now - start time : " << time.now().toSec() - start_time.toSec());
  //ROS_INFO("Arm Controller - Updating ");
}


}

PLUGINLIB_EXPORT_CLASS(arm_controller_ns::ArmEffortController, controller_interface::ControllerBase)
