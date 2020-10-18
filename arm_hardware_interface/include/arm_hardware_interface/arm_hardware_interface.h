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

/**
 * @brief The ManipulatorRobot class
 * Class of a manipulator robot derived from ros base class hardware_interface::RobotHW to implement
 * methods for reading data (joints states) from and writing data (effort commands) to robot hardware.
 */
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

  hardware_interface::EffortJointInterface jnt_effort_interface_;

  unsigned int num_jnts_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;

  //Shared data between hardware interface and controller (both in controller side)
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;

  //Store data received from network connection (ros topic)
  std::vector<MyJointState> joints_states;

};

#endif // MANIPULATOR_HW_INTERFACE_H
