#include <hydrus/hydrus_tilted_lqi_controller.h>
#include <hydrus/hydrus_tilted_robot_model.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

using namespace aerial_robot_control;

HydrusTiltedLQIController::HydrusTiltedLQIController():
  UnderActuatedTiltedLQIController(),
  external_wrench_pid_controllers_(0)
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
  target_wrench_sub_ = nh_.subscribe("target_wrench", 1, &HydrusTiltedLQIController::TargetWrenchCallback, this);

  double limit_sum=1.0e6, limit_p=1.0e6, limit_i=1.0e6, limit_d=1.0e6;
  double limit_err_p=1.0e6, limit_err_i=1.0e6, limit_err_d=1.0e6;
  double p_gain=1.0e6, i_gain=1.0e6, d_gain=1.0e6;

  nh_.getParam("limit_sum", limit_sum);
  nh_.getParam("limit_p", limit_p);
  nh_.getParam("limit_i", limit_i);
  nh_.getParam("limit_d", limit_d);
  nh_.getParam("limit_err_p", limit_err_p);
  nh_.getParam("limit_err_i", limit_err_i);
  nh_.getParam("limit_err_d", limit_err_d);
  nh_.getParam("p_gain", p_gain);
  nh_.getParam("i_gain", i_gain);
  nh_.getParam("d_gain", d_gain);

  external_wrench_pid_controllers_.push_back(PID("force_x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  external_wrench_pid_controllers_.push_back(PID("force_y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  external_wrench_pid_controllers_.push_back(PID("force_z", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  external_wrench_pid_controllers_.push_back(PID("torque_x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  external_wrench_pid_controllers_.push_back(PID("torque_y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  external_wrench_pid_controllers_.push_back(PID("torque_z", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));

}

void HydrusTiltedLQIController::TargetWrenchCallback(geometry_msgs::WrenchStamped msg)
{
  target_wrench_ = msg;
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
  // external wrench  pid controller
  UnderActuatedTiltedLQIController::controlCore();
  double du = ros::Time::now().toSec() - control_timestamp_;
  external_wrench_pid_controllers_.at(X).update(target_wrench_.wrench.force.x - estimate_wrench_.wrench.force.x, du, 0, 0);
  external_wrench_pid_controllers_.at(Y).update(target_wrench_.wrench.force.y - estimate_wrench_.wrench.force.y, du, 0, 0);
  external_wrench_pid_controllers_.at(Z).update(target_wrench_.wrench.force.z - estimate_wrench_.wrench.force.z, du, 0, 0);
  external_wrench_pid_controllers_.at(ROLL).update(target_wrench_.wrench.torque.x - estimate_wrench_.wrench.torque.x, du, 0, 0);
  external_wrench_pid_controllers_.at(PITCH).update(target_wrench_.wrench.torque.y - estimate_wrench_.wrench.torque.y, du, 0, 0);
  external_wrench_pid_controllers_.at(YAW).update(target_wrench_.wrench.torque.z - estimate_wrench_.wrench.torque.z, du, 0, 0);
  control_timestamp_ = ros::Time::now().toSec();

  // target wrench 
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  setTargetWrenchAccCog(target_wrench_acc_cog);

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
/*
void HydrusTiltedLQIController::controlCore()
{
  UnderActuatedTiltedLQIController::controlCore();
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  setTargetWrenchAccCog(target_wrench_acc_cog);
}
*/


void HydrusTiltedLQIController::controlCore()
{
  UnderActuatedTiltedLQIController::controlCore();
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  setTargetWrenchAccCog(target_wrench_acc_cog);
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQIController, aerial_robot_control::ControlBase);
 
