#include <hydrus/hydrus_tilted_lqi_controller.h>
#include <std_msgs/Bool.h>

using namespace aerial_robot_control;

HydrusTiltedLQIController::HydrusTiltedLQIController():
  UnderActuatedTiltedLQIController()
{
}

void HydrusTiltedLQIController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedTiltedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  static_thrust_available_pub_ = nh_.advertise<std_msgs::Bool>("static_thrust_available", 1);
}

bool HydrusTiltedLQIController::checkRobotModel()
{
  bool flag  = robot_model_->stabilityCheck();
  std_msgs::Bool static_thrust_available_msg;
  static_thrust_available_msg.data = flag;
  static_thrust_available_pub_.publish(static_thrust_available_msg);
  
  ROS_INFO_STREAM(flag);
  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
      return false;
    }

  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: invalid pose, stability is invalid");

      return false;
    }
  return true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQIController, aerial_robot_control::ControlBase);
 