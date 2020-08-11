#include "arm_controller/arm_effort_controller.h"

namespace arm_controller_ns{

ArmEffortController::ArmEffortController()
{

}
ArmEffortController::~ArmEffortController()
{

}

bool ArmEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{

  if (!hw)
  {
    ROS_ERROR("The given robot was NULL");
    return false;
  }

  return true;
}

void ArmEffortController::starting(const ros::Time& time)
{
  ROS_INFO("Arm Controller - STARTED ");
}

void ArmEffortController::update(const ros::Time& time, const ros::Duration& period)
{
  ROS_INFO("Arm Controller - Updating ");
}


}

PLUGINLIB_EXPORT_CLASS(arm_controller_ns::ArmEffortController, controller_interface::ControllerBase)
