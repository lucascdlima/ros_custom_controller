#include <ros/ros.h>
#include "robot_kdl/robot_kdl.h"
#include <iostream>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_sim_node");

  ros::NodeHandle nh("~");

  ros::Rate loop_rate(200);

  bool internal_control;

  if(!nh.param("internal_control", internal_control,false))
    ROS_INFO("Param internal_control not found - set as default");

  //Creates the robot object based in urdf model
  RobotKDL myrobot("/home/lucaslima/catkin_test_ws/src/robot_kdl/kuka_urdf/kuka_urdf_test.urdf", nh);

  //Creates ros publisher to broadcast joints states and ros subscriber to receive joints effor commands from the controller ros node
  ros::Publisher joints_states_pub = nh.advertise<sensor_msgs::JointState>("/arm_joints_states", 1000);
  ros::Subscriber effort_command_sub;

  if(!internal_control)
    effort_command_sub = nh.subscribe<std_msgs::Float64MultiArray>("/arm_effort_command", 1000, &RobotKDL::SubEffortCommand, &myrobot);

  ROS_INFO("Effort subscriber and Joint_states publisher are Set");

  KDL::JntArray jnt_acc;
  KDL::JntArray jnt_vel;
  KDL::JntArray jnt_pos;

  jnt_acc.resize(myrobot.num_joints);
  jnt_vel.resize(myrobot.num_joints);
  jnt_pos.resize(myrobot.num_joints);

  KDL::SetToZero(jnt_acc);
  KDL::SetToZero(jnt_vel);
  KDL::SetToZero(jnt_pos);

  KDL::Vector grav (0.0,0.0,-9.81);
  KDL::ChainDynParam chain_dynamics(myrobot.robot_chain,grav);

  if(internal_control)
  {
    myrobot.InitControlParam(ros::Time::now());
    ROS_INFO("Simulation using inernal control");
   }

  ROS_INFO("Node simulating loop started");

  double etime;
  ros::Time last_time = ros::Time::now();
  ros::Time now_time;

  while (ros::ok())
  {
    now_time = ros::Time::now();
    etime = now_time.toSec() - last_time.toSec();
    last_time = now_time;

    if(internal_control)
      myrobot.ComputedTorqueControlExample(now_time);

    //Compute joint accelerations via Forward Dynamics encapsuled in UpdateDynamic() function
    jnt_acc = myrobot.UpdateDynamic();

    //Update joints velocities and position (Euler integration)
    jnt_vel.data = jnt_acc.data*etime + myrobot.GetJointsVelocity().data;
    jnt_pos.data = jnt_vel.data*etime + myrobot.GetJointsPosition().data;

    // Set new joints positions and velocities
    myrobot.SetJointsPosition(jnt_pos.data);
    myrobot.SetJointsVelocity(jnt_vel.data);

    sensor_msgs::JointState states_msg;
    myrobot.SetJointStatesMsg(states_msg);
    joints_states_pub.publish(states_msg);

    ros::spinOnce();
    loop_rate.sleep();

    //temporary for data display and test
    cout<<"Acceleration = "<<endl<<jnt_acc.data<<endl;
  }

  joints_states_pub.shutdown();
  if(!internal_control)
    effort_command_sub.shutdown();

  return 0;

}

