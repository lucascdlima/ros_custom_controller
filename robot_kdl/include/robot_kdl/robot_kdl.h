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
    RobotKDL();
    RobotKDL(const char* urdf_file, ros::NodeHandle &n);
    ~RobotKDL();
    KDL::Chain robot_chain;
    KDL::Tree robot_tree;

    unsigned int num_joints;
    std::vector<std::string> jnts_names;
    KDL::JntArray jnts_vel;
    KDL::JntArray jnts_pos;
    KDL::JntArray jnts_acc;
    KDL::JntArray jnts_effort_command;
    KDL::ChainDynParam* chain_dynamics;
    KDL::JntSpaceInertiaMatrix M_inertia;
    KDL::JntArray C_coriolis;
    KDL::JntArray G_gravity;

    KDL::JntArray UpdateDynamic();
    /* void SetEffortCommand(double effort_jnts[]);
    void SetJointsPosition(double jnts_position[]);
    void SetJointsVelocity(double jnts_velocity[]);*/
    void SetEffortCommand(const VectorXd& jnts_effort);
    void SetJointsVelocity(const VectorXd& jnts_velocity);
    void SetJointsPosition(const VectorXd& jnts_position);

    KDL::JntArray GetJointsVelocity();
    KDL::JntArray GetJointsPosition();


    void SubEffortCommand(const std_msgs::Float64MultiArrayConstPtr& msg);
    void SetJointStatesMsg(sensor_msgs::JointState &msg);





};

#endif // ROBOT_KDL_H
