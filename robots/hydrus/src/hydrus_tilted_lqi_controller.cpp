#include <hydrus/hydrus_tilted_lqi_controller.h>
#include <hydrus/hydrus_tilted_robot_model.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

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
  hydrus_robot_model_ = boost::dynamic_pointer_cast<HydrusTiltedRobotModel>(robot_model);
  static_thrust_available_pub_ = nh_.advertise<std_msgs::Bool>("static_thrust_available", 1);
  fc_t_min_pub_ = nh_.advertise<std_msgs::Float64>("fc_t_min", 1);
  fc_t_min_thre_pub_  = nh_.advertise<std_msgs::Float64>("fc_t_min_thre", 1);
  fc_rp_min_pub_ = nh_.advertise<std_msgs::Float64>("fc_rp_min", 1);
}

bool HydrusTiltedLQIController::update()
{
  bool static_flag  = robot_model_->stabilityCheck();
  std_msgs::Bool static_thrust_available_msg;
  static_thrust_available_msg.data = static_flag;
  static_thrust_available_pub_.publish(static_thrust_available_msg);

  double fc_t_min = robot_model_->getFeasibleControlTMin();
  std_msgs::Float64 fc_t_min_msg;
  fc_t_min_msg.data = fc_t_min;
  fc_t_min_pub_.publish(fc_t_min_msg);

  double fc_t_min_thre = robot_model_->getFeasibleControlTMinThre();
  std_msgs::Float64 fc_t_min_thre_msg;
  fc_t_min_thre_msg.data = fc_t_min_thre;
  fc_t_min_thre_pub_.publish(fc_t_min_thre_msg);

  double fc_rp_min = hydrus_robot_model_->getFeasibleControlRollPitchMin();
  std_msgs::Float64 fc_rp_min_msg;
  fc_rp_min_msg.data = fc_rp_min;
  fc_rp_min_pub_.publish(fc_rp_min_msg);

  if(!PoseLinearController::update()) return false;

  return true;
}

void HydrusTiltedLQIController::controlCore()
{
  UnderActuatedTiltedLQIController::controlCore();
}

void HydrusTiltedLQIController::sendCmd()
{
  UnderActuatedLQIController::sendCmd();
}

bool HydrusTiltedLQIController::checkRobotModel()
{

  /*ROS_INFO_STREAM(flag);*/
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
 
