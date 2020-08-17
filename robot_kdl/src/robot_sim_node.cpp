#include <ros/ros.h>
#include "robot_kdl/robot_kdl.h"
#include <iostream>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_sim_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);


  RobotKDL myrobot("/home/lucaslima/catkin_test_ws/src/robot_kdl/kuka_urdf/kuka_urdf_test.urdf", nh);

  ros::Publisher joints_states_pub = nh.advertise<sensor_msgs::JointState>("arm_joints_states", 1000);
  ros::Subscriber effort_command_sub = nh.subscribe<std_msgs::Float64MultiArray>("arm_effort_command", 1000, &RobotKDL::SubEffortCommand, &myrobot);

  ROS_INFO("Effort subscriber and Joint_states publisher are Set");

  ros::Time last_time;
  last_time = ros::Time::now();
  KDL::JntArray jnt_acc;
  KDL::JntArray jnt_vel;
  KDL::JntArray jnt_pos;
  KDL::JntArray jnt_effort;
  jnt_effort.resize(myrobot.num_joints);
  jnt_acc.resize(myrobot.num_joints);
  jnt_vel.resize(myrobot.num_joints);
  jnt_pos.resize(myrobot.num_joints);

  double etime;
  ros::Time start_time;
  start_time = ros::Time::now();
  ros::Time now_time;

  KDL::SetToZero(jnt_acc);
  KDL::SetToZero(jnt_vel);
  KDL::SetToZero(jnt_pos);
  KDL::SetToZero(jnt_effort);

  KDL::Vector grav (0.0,0.0,-9.81);
  KDL::ChainDynParam chain_dynamics(myrobot.robot_chain,grav);

  double pi = 3.1415;
  double wn = 2*pi/2;

  KDL::JntArray jnt_u;
  jnt_u.resize(myrobot.num_joints);
  KDL::SetToZero(jnt_u);

  Eigen::Matrix<double,7,7> Kp = MatrixXd::Identity(7, 7);
  Eigen::Matrix<double,7,7> Kd = MatrixXd::Identity(7, 7);

  Eigen:: Matrix<double,7,1> q_des;
  Eigen:: Matrix<double,7,1> dq_des;
  Eigen:: Matrix<double,7,1> ddq_des;

  ROS_INFO("Node loop started");
  while (ros::ok())
  {

    /*
    // Computing desired joints positions for computed torque control
    for(unsigned int i=0; i<myrobot.num_joints; i++)
    {
      q_des(i) = sin((ros::Time::now().toSec() - start_time.toSec())*wn);
      dq_des(i) = wn*cos((ros::Time::now().toSec() - start_time.toSec())*wn);
      ddq_des(i) = -wn*wn*sin((ros::Time::now().toSec() - start_time.toSec())*wn);
    }

    //Initializing Inertial, Coriolis and Gravity of dynamic matrices
    KDL::JntSpaceInertiaMatrix M_c(myrobot.num_joints);
    chain_dynamics.JntToMass(jnt_pos,M_c);
    KDL::JntArray C_c(myrobot.num_joints);
    chain_dynamics.JntToCoriolis(jnt_pos,jnt_vel,C_c);
    KDL::JntArray G_c(myrobot.num_joints);
    chain_dynamics.JntToGravity(jnt_pos,G_c);

    //computing the coputed torque control
    jnt_u.data = Kp*(q_des - jnt_pos.data) + Kd*(dq_des - jnt_vel.data);
    jnt_effort.data=M_c.data*(ddq_des + jnt_u.data) + C_c.data + G_c.data;

    //Setting the commanded effort and
    myrobot.SetEffortCommand(jnt_effort.data);*/

    //Computing joint accelerations via Forward Dynamics encapsuled in UpdateDynamic() function
    jnt_acc = myrobot.UpdateDynamic();


  /*  KDL::JntSpaceInertiaMatrix M(myrobot.num_joints);
    chain_dynamics.JntToMass(jnt_pos,M);
    KDL::JntArray C(myrobot.num_joints);
    chain_dynamics.JntToCoriolis(jnt_pos,jnt_vel,C);
    KDL::JntArray G(myrobot.num_joints);
    chain_dynamics.JntToGravity(jnt_pos,G);
    KDL::JntArray jnt_acc(myrobot.num_joints);
    jnt_acc.data=M.data.inverse()*(jnt_effort.data - C.data - G.data);*/

    now_time = ros::Time::now();
    etime = now_time.toSec() - last_time.toSec();
    last_time = now_time;

   /* jnt_vel.data = jnt_vel.data + jnt_acc.data*etime;
    jnt_pos.data = jnt_pos.data + jnt_vel.data*etime;*/

   //Updating joints velocities and position (Euler integration)
   jnt_vel.data = jnt_acc.data*etime + myrobot.GetJointsVelocity().data;
   jnt_pos.data = jnt_vel.data*etime + myrobot.GetJointsPosition().data;

   // Setting new joints positions and velocities
   myrobot.SetJointsPosition(jnt_pos.data);
   myrobot.SetJointsVelocity(jnt_vel.data);

   ROS_INFO("Calculos realizados");

   sensor_msgs::JointState states_msg;
   myrobot.SetJointStatesMsg(states_msg);
   joints_states_pub.publish(states_msg);

    ros::spinOnce();
    loop_rate.sleep();

    // cout<<"Acceleration = "<<endl<<jnt_acc.data<<endl;
     //cout<<"etime = "<<etime<<endl;
    // cout<<"Effort = "<<endl;
     //cout<<jnt_effort.data<<endl;
  }

  joints_states_pub.shutdown();
  effort_command_sub.shutdown();

  return 0;

}

