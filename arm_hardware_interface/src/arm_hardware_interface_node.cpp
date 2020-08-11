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

  controller_manager::ControllerManager cm(&manipulator_robot,nh);


  ros::Duration period(0.2); //20 Hz rate for update
  //ros::Rate loop_rate(10);


   while (ros::ok())
   {
     manipulator_robot.read(ros::Time::now(), period);
     cm.update(ros::Time::now(), period);
     manipulator_robot.write(ros::Time::now(), period);



     //ros::spinOnce();

     period.sleep();

   }

   spinner.stop();
   return 0;
}
