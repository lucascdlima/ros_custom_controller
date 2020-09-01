#include <ros/ros.h>
#include "robot_kdl/robot_kdl.h"
#include <iostream>
#include <ros/callback_queue.h>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_sim_node");

  ros::NodeHandle nh("~");

  ros::Rate loop_rate(100);
  ros::Duration loop_period(loop_rate);

  bool internal_control;

  if(!nh.param("internal_control", internal_control,false))
    ROS_INFO("Param internal_control not found - set as default");

  //Creates the robot object based in urdf model
  RobotKDL myrobot("/home/lucaslima/catkin_test_ws/src/robot_kdl/kuka_urdf/kuka_urdf_test.urdf", nh, loop_period);

  //Tries to initiate all robot model parameters
  if(!myrobot.Init())
  {
    ROS_ERROR("Robot model simulation NOT started correctly");
    return 0;
  }
  //Creates ros publisher to broadcast joints states and ros subscriber to receive joints effort commands from arm_controller package
  ros::Publisher joints_states_pub = nh.advertise<sensor_msgs::JointState>("/arm_joints_states", 1);
  ros::Subscriber effort_command_sub;

  if(!internal_control)
    effort_command_sub = nh.subscribe<std_msgs::Float64MultiArray>("/arm_effort_command", 1, &RobotKDL::SubEffortCommand, &myrobot);

  KDL::JntArray jnt_acc;
  KDL::JntArray jnt_vel;
  KDL::JntArray jnt_pos;

  jnt_acc.resize(myrobot.num_joints);
  jnt_vel.resize(myrobot.num_joints);
  jnt_pos.resize(myrobot.num_joints);

  KDL::SetToZero(jnt_acc);
  KDL::SetToZero(jnt_vel);
  KDL::SetToZero(jnt_pos);

  if(internal_control)
  {
    myrobot.InitControlParam(ros::Time::now());
    ROS_INFO("Simulation using inernal control");
   }

  ros::Duration elapsed_time;
  ros::Time end_time;
  ros::Time start_time;
  double dt = loop_period.toSec();

  ROS_INFO("Node simulation loop started");

  while (ros::ok())
  {
    start_time = ros::Time::now();

    if(internal_control)
      myrobot.ComputedTorqueControlExample();

    //Compute joint accelerations via Forward Dynamics encapsuled in UpdateDynamic() function
    jnt_acc = myrobot.UpdateDynamic();

    //Update joints velocities and position (Euler integration)
    jnt_vel.data = jnt_acc.data*dt + myrobot.GetJointsVelocity().data;
    jnt_pos.data = jnt_vel.data*dt + myrobot.GetJointsPosition().data;

    // Set new joints positions and velocities
    myrobot.SetJointsPosition(jnt_pos.data);
    myrobot.SetJointsVelocity(jnt_vel.data);

    sensor_msgs::JointState states_msg;
    myrobot.SetJointStatesMsg(states_msg);
    joints_states_pub.publish(states_msg);

    //Call all queued msgs (published and subscribed)
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));

    elapsed_time = ros::Time::now() - start_time;
    if(elapsed_time.toSec()>=loop_period.toSec())
      ROS_WARN("Loop exceeded time");
    else {
      (loop_period - elapsed_time).sleep();
    }

    //temporary for data display and test
   cout<<"Acceleration = "<<endl<<jnt_acc.data<<endl;
    //cout<<"loop cycle time: "<<etime<<" s"<<endl;
  }

  joints_states_pub.shutdown();
  if(!internal_control)
    effort_command_sub.shutdown();

  return 0;

}

