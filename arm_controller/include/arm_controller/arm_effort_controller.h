#ifndef ARM_EFFORT_CONTROLLER_H
#define ARM_EFFORT_CONTROLLER_H

#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>


namespace arm_controller_ns{

class ArmEffortController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

private:
  //pr2_mechanism_model::RobotState *robot_;
  //pr2_mechanism_model::JointState* joint_state_;
  //pr2_mechanism_model::JointState* joint_states_array[7];

  double init_pos_;
  ros::NodeHandle node_;
  ros::Subscriber sub_command_;
  ros::Publisher pub_joint_states_;
  std::vector<std::string> joint_names_;
  //void commandCB();//const std_msgs::Float64MultiArrayConstPtr& msg);

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
