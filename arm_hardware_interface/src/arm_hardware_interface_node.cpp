#include <ros/ros.h>
#include "arm_hardware_interface/arm_hardware_interface.h"
#include "controller_manager/controller_manager.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_hardware_interface_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ManipulatorRobot manipulator_robot(nh);

  //Try to initiate hardware interface correctly, if returns false ends node
  if(manipulator_robot.init())
  {
    //Register the robot hardware interface in the controller manager to make it available to be controlled
    controller_manager::ControllerManager cm(&manipulator_robot,nh);

    //Subscribe joints states and publish effort commands to the simulated robot
    ros::Subscriber joints_states_sub = nh.subscribe<sensor_msgs::JointState>("arm_joints_states",10,&ManipulatorRobot::SubJointsStates, &manipulator_robot);
    ros::Publisher effor_command_pub = nh.advertise<std_msgs::Float64MultiArray>("arm_effort_command",10);


    ros::Duration period(0.02); //Period (1/rate) of thread to update robots data

    while (ros::ok())
    {
      manipulator_robot.read(ros::Time::now(), period);
      cm.update(ros::Time::now(), period);
      manipulator_robot.write(ros::Time::now(), period,effor_command_pub);

      ros::spinOnce();

      period.sleep();

    }
  }
  spinner.stop();
  return 0;
}
