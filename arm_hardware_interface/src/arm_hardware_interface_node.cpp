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

  //Tries to initiate hardware interface correctly, if returns false ends node
  if(manipulator_robot.init())
  {
    //Registers the robot hardware interface in the controller manager to make it available to be controlled
    controller_manager::ControllerManager cm(&manipulator_robot,nh);

    //Subscribes joints states from and publish effort commands to the simulated robot
    ros::Subscriber joints_states_sub = nh.subscribe<sensor_msgs::JointState>("arm_joints_states",10,&ManipulatorRobot::SubJointsStates, &manipulator_robot);
    ros::Publisher effor_command_pub = nh.advertise<std_msgs::Float64MultiArray>("arm_effort_command",10);


    ros::Duration period(0.0025); //Period (1/rate) of thread to update robots data
    ros::Time time_now;

    while (ros::ok())
    {
      time_now = ros::Time::now();
      manipulator_robot.read(time_now, period);
      cm.update(time_now, period);
      manipulator_robot.write(time_now, period,effor_command_pub);

      ros::spinOnce();

      period.sleep();

    }
  }
  spinner.stop();
  return 0;
}
