#ifndef ARM_HRDWARE_INTERFACE_H
#define ARM_HRDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

struct MyJointState{
  double position;
  double velocity;
  double effort;
  std::string name;
};

class ManipulatorRobot : public hardware_interface::RobotHW
{
public:
  ManipulatorRobot(ros::NodeHandle& nh);
  bool init();
  void read(const ros::Time &etime, const ros::Duration &duration);
  void write(const ros::Time &etime, const ros::Duration &duration, ros::Publisher &pub);
  void SubJointsStates(const sensor_msgs::JointStateConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  double loop_hz_;

  hardware_interface::JointStateInterface jnt_state_interface_;

  hardware_interface::EffortJointInterface jnt_effort_interface_;

  unsigned int num_jnts_;
  int control_mode_;
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;

  std::vector<MyJointState> joints_states;

};

#endif // MANIPULATOR_HW_INTERFACE_H
