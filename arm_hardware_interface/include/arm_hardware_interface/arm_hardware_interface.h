#ifndef ARM_HRDWARE_INTERFACE_H
#define ARM_HRDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


class ManipulatorRobot : public hardware_interface::RobotHW
{
public:
  ManipulatorRobot(ros::NodeHandle& nh);
  void init();
  //void update(const ros::TimerEvent& e);
  void read(const ros::Time &etime, const ros::Duration &duration);
  void write(const ros::Time &etime, const ros::Duration &duration);

private:
  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  //hardware_interface::PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
  double loop_hz_;
  //boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  //double p_error_, v_error_, e_error_;
  hardware_interface::JointStateInterface jnt_state_interface_;

  hardware_interface::EffortJointInterface jnt_effort_interface_;
  //boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  //hardware_interface::PositionJointInterface jnt_pos_interface;

  // Shared memory
  unsigned int num_jnts_;
  int control_mode_; // position, velocity, or effort
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  /*std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;*/


};

#endif // MANIPULATOR_HW_INTERFACE_H
