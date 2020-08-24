#ifndef ARM_EFFORT_CONTROLLER_H
#define ARM_EFFORT_CONTROLLER_H

#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/console.h>
#include <Eigen/LU>


namespace arm_controller_ns{

class ArmEffortController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

private:

  KDL::Chain robot_chain;
  KDL::Tree robot_tree;
  std::string urdf_model_;

  double init_pos_;
  ros::NodeHandle nh_;

  std::vector<std::string> joint_names_;

  unsigned int num_jnts_;
  std::vector<hardware_interface::JointHandle> joints_;

  KDL::JntArray jnts_vel;
  KDL::JntArray jnts_pos;
  KDL::JntArray jnts_acc;
  KDL::JntArray jnts_effort_command;

  KDL::ChainDynParam* chain_dynamics;
  KDL::JntSpaceInertiaMatrix M_inertia;
  KDL::JntArray C_coriolis;
  KDL::JntArray G_gravity;

  double pi;
  double wn;

  KDL::JntArray jnt_u;

  Eigen::Matrix<double,7,7> Kp;
  Eigen::Matrix<double,7,7> Kd;

  Eigen:: Matrix<double,7,1> q_des;
  Eigen:: Matrix<double,7,1> dq_des;
  Eigen:: Matrix<double,7,1> ddq_des;

  ros::Time start_time;

public:
  ArmEffortController();
  ~ArmEffortController();
  //double command_;
  //double command_array[7];
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

};
}

#endif // ARM_EFFORT_CONTROLLER_H
