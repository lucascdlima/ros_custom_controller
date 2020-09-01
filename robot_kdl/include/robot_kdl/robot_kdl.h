#ifndef ROBOT_KDL_H
#define ROBOT_KDL_H

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/console.h>
#include <Eigen/LU>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

using namespace Eigen;

class RobotKDL{

  public:

    RobotKDL(const char* urdf_file, ros::NodeHandle &n,ros::Duration loop_period);
    ~RobotKDL();

    bool Init();

    std::string urdf_file_path;
    ros::NodeHandle nh;
    KDL::Chain robot_chain; //handle for robot chain of join-link segments
    KDL::Tree robot_tree; //handle of robot chains

    unsigned int num_joints;
    std::vector<std::string> jnts_names;

    //Stores current joints positions, velocities, accelerations and torques
    KDL::JntArray jnts_vel;
    KDL::JntArray jnts_pos;
    KDL::JntArray jnts_acc;
    KDL::JntArray jnts_effort_command;

    KDL::ChainDynParam* chain_dynamics; //handle of chain dynamic parameters

    KDL::JntSpaceInertiaMatrix M_inertia; //lagrange inertia matrix of manipulator
    KDL::JntArray C_coriolis; //lagrange coriolis vector of manipulator
    KDL::JntArray G_gravity; //lagrange gravity vector of manipulator

    unsigned long int iter_count;

    KDL::JntArray UpdateDynamic();

    void SetEffortCommand(const VectorXd& jnts_effort);
    void SetJointsVelocity(const VectorXd& jnts_velocity);
    void SetJointsPosition(const VectorXd& jnts_position);

    KDL::JntArray GetJointsVelocity();
    KDL::JntArray GetJointsPosition();

    void SubEffortCommand(const std_msgs::Float64MultiArrayConstPtr& msg);
    void SetJointStatesMsg(sensor_msgs::JointState &msg);

    //Methods and attributes below used in internal control only
    void ComputedTorqueControlExample();
    void InitControlParam();

    KDL::JntArray jnt_u;

    Eigen::MatrixXd Kp; //Proportional constants of control
    Eigen::MatrixXd Kd; //Derivative constants of control

    double wn; //angular frequency of desired joints trajectory (sinusoid)
    double dt; //Integration interval based on simulation loop period

    Eigen:: VectorXd q_des; //joints desired position
    Eigen:: VectorXd dq_des; //joints desired velocities
    Eigen:: VectorXd ddq_des; //joints desired acceleration

    ros::Time start_time;

};

#endif // ROBOT_KDL_H
