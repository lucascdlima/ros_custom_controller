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
/**
 * @brief The RobotKDL class
 * Class to handle a manipulator model based on KDL class and perform calculations such as
 * Forward and Inverse dynamics, and execute simulation of manipulator movements given an internal or external
 * controller.
 */
class RobotKDL{

  public:

    RobotKDL(const char* urdf_file, ros::NodeHandle &n,ros::Duration loop_period);
    ~RobotKDL();

    bool Init();

    KDL::JntArray UpdateDynamic();

    void SetEffortCommand(const VectorXd& jnts_effort);
    void SetJointsVelocity(const VectorXd& jnts_velocity);
    void SetJointsPosition(const VectorXd& jnts_position);

    KDL::JntArray GetJointsVelocity();
    KDL::JntArray GetJointsPosition();
    uint GetNumJoints();

    void SubEffortCommand(const std_msgs::Float64MultiArrayConstPtr& msg);
    void SetJointStatesMsg(sensor_msgs::JointState &msg);

    //Methods and attributes below used in internal control only
    void ComputedTorqueControlExample();
    void InitControlParam();

private:
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

    //Lagrange dynamic parameters
    KDL::JntSpaceInertiaMatrix M_inertia;
    KDL::JntArray C_coriolis;
    KDL::JntArray G_gravity;

    unsigned long int iter_count;

    KDL::JntArray jnt_u;

    //Internal control parameters
    //Control constants
    Eigen::MatrixXd Kp;
    Eigen::MatrixXd Kd;

    double wn; //angular frequency of desired joints trajectory (sinusoid)
    double dt; //Integration interval based on simulation loop period

    //Desired joints positions, velocities and accelerations
    Eigen:: VectorXd q_des;
    Eigen:: VectorXd dq_des;
    Eigen:: VectorXd ddq_des;

    ros::Time start_time;

};

#endif // ROBOT_KDL_H
