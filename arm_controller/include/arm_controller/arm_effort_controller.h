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

  KDL::Chain robot_chain; //handle a chain of joint-link segments
  KDL::Tree robot_tree; //handle a group of chains
  std::string urdf_model_; //path to urdf manipulator model file

  double init_pos_;
  ros::NodeHandle nh_;

  std::vector<std::string> joint_names_;

  unsigned int num_jnts_;
  std::vector<hardware_interface::JointHandle> joints_;

  //Stores joints positions, velocities, accelerations and effort commands
  KDL::JntArray jnts_vel;
  KDL::JntArray jnts_pos;
  KDL::JntArray jnts_acc;
  KDL::JntArray jnts_effort_command;

  KDL::ChainDynParam* chain_dynamics;  //attribute to compute dynamic parameters of a manipulator

  //Lagrange dynamic model components
  KDL::JntSpaceInertiaMatrix M_inertia;
  KDL::JntArray C_coriolis;
  KDL::JntArray G_gravity;

  KDL::JntArray jnt_u;

  Eigen::MatrixXd Kp; //Proportional constants of control
  Eigen::MatrixXd Kd; //Derivative constants of control

  double wn; //angular frequency of desired joints trajectory (sinusoid)

  Eigen:: VectorXd q_des; //joints desired position
  Eigen:: VectorXd dq_des; //joints desired velocities
  Eigen:: VectorXd ddq_des; //joints desired acceleration

  ros::Time start_time;

public:
  ArmEffortController();
  ~ArmEffortController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

};
}

#endif // ARM_EFFORT_CONTROLLER_H
