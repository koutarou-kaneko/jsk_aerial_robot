#include <hydrus/hydrus_tilted_lqi_controller.h>
#include <hydrus/hydrus_tilted_robot_model.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

using namespace aerial_robot_control;

HydrusTiltedLQIController::HydrusTiltedLQIController():
  UnderActuatedTiltedLQIController(),
  external_wrench_pid_controllers_(0),
  external_wrench_pid_reconf_servers_(0)
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
  feedforward_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("feedforward_wrench", 1);
  desire_wrench_sub_ = nh_.subscribe("desire_wrench", 1, &HydrusTiltedLQIController::DesireWrenchCallback, this);
  desire_wrench_ = Eigen::VectorXd::Zero(6);
  target_wrench_cog_ = Eigen::VectorXd::Zero(6);
  p_wrench_stamp_ = Eigen::VectorXd::Zero(6);

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle est_wrench_nh(control_nh, "est_wrench");
  ros::NodeHandle wrench_xy_nh(est_wrench_nh, "wrench_xy");
  ros::NodeHandle wrench_x_nh(est_wrench_nh, "wrench_x");
  ros::NodeHandle wrench_y_nh(est_wrench_nh, "wrench_y");
  ros::NodeHandle wrench_z_nh(est_wrench_nh, "wrench_z");
  ros::NodeHandle wrench_roll_pitch_nh(est_wrench_nh, "wrench_roll_pitch");
  ros::NodeHandle wrench_roll_nh(est_wrench_nh, "wrench_roll");
  ros::NodeHandle wrench_pitch_nh(est_wrench_nh, "wrench_pitch");
  ros::NodeHandle wrench_yaw_nh(est_wrench_nh, "wrench_yaw");

  double limit_sum, limit_p, limit_i, limit_d;
  double limit_err_p, limit_err_i, limit_err_d;
  double p_gain, i_gain, d_gain;

  auto loadParam = [&, this](ros::NodeHandle nh)
    {
      getParam<double>(nh, "limit_sum", limit_sum, 1.0e6);
      getParam<double>(nh, "limit_p", limit_p, 1.0e6);
      getParam<double>(nh, "limit_i", limit_i, 1.0e6);
      getParam<double>(nh, "limit_d", limit_d, 1.0e6);
      getParam<double>(nh, "limit_err_p", limit_err_p, 1.0e6);
      getParam<double>(nh, "limit_err_i", limit_err_i, 1.0e6);
      getParam<double>(nh, "limit_err_d", limit_err_d, 1.0e6);

      getParam<double>(nh, "p_gain", p_gain, 0.0);
      getParam<double>(nh, "i_gain", i_gain, 0.0);
      getParam<double>(nh, "d_gain", d_gain, 0.0);
    };

  /* xy */
  if(wrench_xy_nh.hasParam("p_gain"))
    {
      loadParam(wrench_xy_nh);
      external_wrench_pid_controllers_.push_back(PID("wrench_x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
      external_wrench_pid_controllers_.push_back(PID("wrench_y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));

      std::vector<int> indices = {X, Y};
      external_wrench_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_xy_nh));
      external_wrench_pid_reconf_servers_.back()->setCallback(boost::bind(&HydrusTiltedLQIController::cfgWrenchPidCallback, this, _1, _2, indices));

    }
  else
    {
      loadParam(wrench_x_nh);
      external_wrench_pid_controllers_.push_back(PID("wrench_x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
      external_wrench_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_x_nh));
      external_wrench_pid_reconf_servers_.back()->setCallback(boost::bind(&HydrusTiltedLQIController::cfgWrenchPidCallback, this, _1, _2, std::vector<int>(1, X)));

      loadParam(wrench_y_nh);
      external_wrench_pid_controllers_.push_back(PID("wrench_y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
      external_wrench_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_y_nh));
      external_wrench_pid_reconf_servers_.back()->setCallback(boost::bind(&HydrusTiltedLQIController::cfgWrenchPidCallback, this, _1, _2, std::vector<int>(1, Y)));
    }

  /* z */
  loadParam(wrench_z_nh);
  external_wrench_pid_controllers_.push_back(PID("wrench_z", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  external_wrench_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_z_nh));
  external_wrench_pid_reconf_servers_.back()->setCallback(boost::bind(&HydrusTiltedLQIController::cfgWrenchPidCallback, this, _1, _2, std::vector<int>(1, Z)));

  /* roll pitch */
  //getParam<double>(wrench_roll_pitch_nh, "start_integration_height", start_rp_integration_height_, 0.01);
  if(wrench_roll_pitch_nh.hasParam("p_gain"))
    {
      loadParam(wrench_roll_pitch_nh);
      external_wrench_pid_controllers_.push_back(PID("wrench_roll", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
      external_wrench_pid_controllers_.push_back(PID("wrench_pitch", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
      std::vector<int> indices = {ROLL, PITCH};
      external_wrench_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_roll_pitch_nh));
      external_wrench_pid_reconf_servers_.back()->setCallback(boost::bind(&HydrusTiltedLQIController::cfgWrenchPidCallback, this, _1, _2, indices));
    }
  else
    {
      loadParam(wrench_roll_nh);
      external_wrench_pid_controllers_.push_back(PID("wrench_roll", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
      external_wrench_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_roll_nh));
      external_wrench_pid_reconf_servers_.back()->setCallback(boost::bind(&HydrusTiltedLQIController::cfgWrenchPidCallback, this, _1, _2, std::vector<int>(1, ROLL)));

      loadParam(wrench_pitch_nh);
      external_wrench_pid_controllers_.push_back(PID("wrench_pitch", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
      external_wrench_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_pitch_nh));
      external_wrench_pid_reconf_servers_.back()->setCallback(boost::bind(&HydrusTiltedLQIController::cfgWrenchPidCallback, this, _1, _2, std::vector<int>(1, PITCH)));
    }

  /* yaw */
  loadParam(wrench_yaw_nh);
  //getParam<bool>(wrench_yaw_nh, "need_d_control", need_yaw_d_control_, false);
  external_wrench_pid_controllers_.push_back(PID("wrench_yaw", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  external_wrench_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_yaw_nh));
  external_wrench_pid_reconf_servers_.back()->setCallback(boost::bind(&HydrusTiltedLQIController::cfgWrenchPidCallback, this, _1, _2, std::vector<int>(1, YAW)));

}

void HydrusTiltedLQIController::cfgWrenchPidCallback(aerial_robot_control::PIDConfig &config, uint32_t level, std::vector<int> controller_indices)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if(config.pid_control_flag)
    {
      switch(level)
        {
        case Levels::RECONFIGURE_P_GAIN:
          for(const auto& index: controller_indices)
            {
              external_wrench_pid_controllers_.at(index).setPGain(config.p_gain);
              ROS_INFO_STREAM("change p gain for controller '" << external_wrench_pid_controllers_.at(index).getName() << "'");
            }
          break;
        case Levels::RECONFIGURE_I_GAIN:
          for(const auto& index: controller_indices)
            {
              external_wrench_pid_controllers_.at(index).setIGain(config.i_gain);
              ROS_INFO_STREAM("change i gain for controller '" << external_wrench_pid_controllers_.at(index).getName() << "'");
            }
          break;
        case Levels::RECONFIGURE_D_GAIN:
          for(const auto& index: controller_indices)
            {
              external_wrench_pid_controllers_.at(index).setDGain(config.d_gain);
              ROS_INFO_STREAM("change d gain for controller '" << external_wrench_pid_controllers_.at(index).getName() << "'");
            }
          break;
        default :
          break;
        }
    }
}

void HydrusTiltedLQIController::DesireWrenchCallback(geometry_msgs::WrenchStamped msg)
{
  Eigen::Vector3d desire_force_at_end;
  desire_force_at_end[0] = msg.wrench.force.x;
  desire_force_at_end[1] = msg.wrench.force.y;
  desire_force_at_end[2] = msg.wrench.force.z;
  hydrus_robot_model_->setTargetForceInLinkEnd(desire_force_at_end);
  Eigen::Vector3d desire_torque_at_cog = hydrus_robot_model_->getCompensateTorqueForLinkEndInCog();
  desire_wrench_[0] = desire_force_at_end[0];
  desire_wrench_[1] = desire_force_at_end[1];
  desire_wrench_[2] = desire_force_at_end[2];
  desire_wrench_[3] = desire_torque_at_cog[0];
  desire_wrench_[4] = desire_torque_at_cog[1];
  desire_wrench_[5] = desire_torque_at_cog[2];
  //std::cout << desire_wrench_ << std::endl;
  //std::cout << "-----------------------" << std::endl;

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

void HydrusTiltedLQIController::controlCore()
{
  double du = ros::Time::now().toSec() - control_timestamp_;
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
  //double target_ang_acc_z = pid_controllers_.at(YAW).result(); //candidate_yaw_term
  double target_ang_acc_z = candidate_yaw_term_;
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  Eigen::VectorXd p_wrench_diff = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd d_wrench_diff = Eigen::VectorXd::Zero(6);
  for (int i=0;i<=5;i++){
    p_wrench_diff[i] = desire_wrench_[i] - est_external_wrench_[i];
    double dp = p_wrench_diff[i] - p_wrench_stamp_[i];
    d_wrench_diff[i] = dp/du;
  }
  external_wrench_pid_controllers_.at(X).update(p_wrench_diff[0], du, d_wrench_diff[0], 0);
  external_wrench_pid_controllers_.at(Y).update(p_wrench_diff[1], du, d_wrench_diff[1], 0);
  external_wrench_pid_controllers_.at(Z).update(p_wrench_diff[2], du, d_wrench_diff[2], 0);
  external_wrench_pid_controllers_.at(ROLL).update(p_wrench_diff[3], du, d_wrench_diff[3], 0);
  external_wrench_pid_controllers_.at(PITCH).update(p_wrench_diff[4], du, d_wrench_diff[4], 0);
  external_wrench_pid_controllers_.at(YAW).update(p_wrench_diff[5], du, d_wrench_diff[5], 0);
  p_wrench_stamp_ = p_wrench_diff;

  geometry_msgs::WrenchStamped feedforward_wrench_msg;
  /*
  feedforward_wrench_msg.wrench.force.x = external_wrench_pid_controllers_.at(X).result();
  feedforward_wrench_msg.wrench.force.y = external_wrench_pid_controllers_.at(Y).result();
  feedforward_wrench_msg.wrench.force.z = external_wrench_pid_controllers_.at(Z).result();
  feedforward_wrench_msg.wrench.torque.x = external_wrench_pid_controllers_.at(ROLL).result();
  feedforward_wrench_msg.wrench.torque.y = external_wrench_pid_controllers_.at(PITCH).result();
  feedforward_wrench_msg.wrench.torque.z = external_wrench_pid_controllers_.at(YAW).result();
  */
  feedforward_wrench_msg.wrench.force.x = p_wrench_diff[0];
  feedforward_wrench_msg.wrench.force.y = p_wrench_diff[1];
  feedforward_wrench_msg.wrench.force.z = p_wrench_diff[2];
  feedforward_wrench_msg.wrench.torque.x = p_wrench_diff[3];
  feedforward_wrench_msg.wrench.torque.y = p_wrench_diff[4];
  feedforward_wrench_msg.wrench.torque.z = p_wrench_diff[5];

  feedforward_wrench_pub_.publish(feedforward_wrench_msg);
  /* feedforward */
  //target_base_thrust_ += p_wrench_diff;
  /*
  for(int i; i<=target_base_thrust_.size(); i++)
  {
  std::cout << target_base_thrust_[i] << std::endl;
  }
  std::cout << "---------------------------" << std::endl;
  */

  double mass_inv = 1/ hydrus_robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = hydrus_robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  Eigen::Vector3d des_torque;
  des_torque[0]=p_wrench_diff[3];
  des_torque[1]=p_wrench_diff[4];
  des_torque[2]=p_wrench_diff[5];
  Eigen::Vector3d des_acc_ang = inertia_inv * des_torque;
  target_acc_.setX(p_wrench_diff[0] * mass_inv);
  target_acc_.setY(p_wrench_diff[1] * mass_inv);
  target_acc_.setZ(p_wrench_diff[2] * mass_inv);
  target_acc_ang_.setZ(des_acc_ang[2]);
  
  setTargetWrenchAccCog(target_wrench_acc_cog);

}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQIController, aerial_robot_control::ControlBase);
 