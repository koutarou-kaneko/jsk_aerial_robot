#include <hydrus/hydrus_tilted_lqi_controller.h>
#include <hydrus/hydrus_tilted_robot_model.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf_conversions/tf_eigen.h>
#include <cstdlib>

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
  feedforward_acc_cog_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("feedforward_acc_cog", 1);
  feedforward_ang_acc_cog_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("feedforward_ang_acc_cog", 1);
  des_wrench_cog_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("des_wrench_cog", 1);
  attaching_flag_pub_ = nh_.advertise<std_msgs::Bool>("attaching_flag",1);
  desire_wrench_sub_ = nh_.subscribe("desire_wrench", 1, &HydrusTiltedLQIController::DesireWrenchCallback, this);
  desire_pos_sub_ = nh_.subscribe("uav/nav", 1, &HydrusTiltedLQIController::DesirePosCallback, this);
  acc_root_sub_ = nh_.subscribe("imu", 10, &HydrusTiltedLQIController::accRootCallback, this);
  hand_force_switch_sub_ = nh_.subscribe("hand_force_switch", 1, &HydrusTiltedLQIController::HandForceSwitchCallBack, this);
  estimated_external_wrench_in_cog_ = Eigen::VectorXd::Zero(6);
  desire_wrench_ = Eigen::VectorXd::Zero(6);
  desire_wrench_from_pos_ = Eigen::VectorXd::Zero(6);
  target_wrench_cog_ = Eigen::VectorXd::Zero(6);
  p_wrench_stamp_ = Eigen::VectorXd::Zero(6);
  feedforward_sum_ = Eigen::VectorXd::Zero(6);
  desire_pos_ = Eigen::Vector3d::Zero(6);
  attaching_flag_ = false;
  const_err_i_flag_ = false;

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

  getParam<double>(control_nh, "wrench_diff_gain", wrench_diff_gain_, 1.0);
  getParam<bool>(control_nh, "send_feedforward_switch_flag", send_feedforward_switch_flag_, false);
  getParam<double>(control_nh, "acc_shock_thres", acc_shock_thres_, 20.0);
  
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
  hydrus_robot_model_->setTargetForceInRootEnd(desire_force_at_end);
  Eigen::Vector3d des_torque_for_root_end_in_cog = hydrus_robot_model_->getCompensateTorqueForRootEndInCog();
  KDL::Frame root_end = hydrus_robot_model_->getRootEnd();
  KDL::Frame cog = hydrus_robot_model_->getCog<KDL::Frame>();
  Eigen::Vector3d des_force_for_root_end_in_cog = aerial_robot_model::kdlToEigen(cog.M.Inverse() * root_end.M) * desire_force_at_end;

  desire_wrench_[0] = des_force_for_root_end_in_cog[0];
  desire_wrench_[1] = des_force_for_root_end_in_cog[1];
  desire_wrench_[2] = des_force_for_root_end_in_cog[2];
  desire_wrench_[3] = des_torque_for_root_end_in_cog[0];
  desire_wrench_[4] = des_torque_for_root_end_in_cog[1];
  desire_wrench_[5] = des_torque_for_root_end_in_cog[2];
}

void HydrusTiltedLQIController::DesirePosCallback(aerial_robot_msgs::FlightNav msg)
{
  desire_pos_[0] = msg.target_pos_x;
  desire_pos_[1] = msg.target_pos_y;
  desire_pos_[2] = msg.target_pos_z;
  desire_pos_[5] = msg.target_yaw;
}

void HydrusTiltedLQIController::accRootCallback(const spinal::Imu msg)
{
  /*
  if((!attaching_flag_) && (msg.acc_data[0] > acc_shock_thres_))
  {
    attaching_flag_ = true;
  }*/
  /*it was not good
  if(attaching_flag_ && abs(est_external_wrench_[0])<=0.6 && abs(est_external_wrench_[1])<=0.6)
  {
    attaching_flag_ = false;
  }*/

}

void HydrusTiltedLQIController::HandForceSwitchCallBack(std_msgs::Int8 msg)
{
  int i = msg.data;
  if(i==1)
  {
    attaching_flag_ = true;
  }
  if(i==0)
  {
    attaching_flag_ = false;
  }
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
  int navi_state = navigator_->getNaviState();
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 euler = estimator_->getEuler(Frame::COG, estimate_mode_);
  double yaw_diff = desire_pos_[5] - euler.z();
  double pos_x_diff = desire_pos_[0] - pos.x();
  double pos_y_diff = desire_pos_[1] - pos.y();
  /*
  if(abs(pos_x_diff)<=0.1 && yaw_diff<=0.1)
  {
    attaching_flag_ = false;
  }  
  if(navi_state != 5 || navigator_->getForceLandingFlag())
  {
    attaching_flag_ = false;
  }*/

  std_msgs::Bool attaching_flag_msg;
  attaching_flag_msg.data = attaching_flag_;
  attaching_flag_pub_.publish(attaching_flag_msg);
  // during attaching
  if(attaching_flag_)
    {
      if(!const_err_i_flag_)
        {
          err_i_x_ = pid_controllers_.at(X).getErrI();
          err_i_y_ = pid_controllers_.at(Y).getErrI();
          err_i_z_ = pid_controllers_.at(Z).getErrI();
          const_err_i_flag_ = true;
        }
      pid_controllers_.at(X).setErrI(err_i_x_);
      pid_controllers_.at(Y).setErrI(err_i_y_);
      pid_controllers_.at(Z).setErrI(err_i_z_);
    }
  //else{const_err_i_flag_ = false;}

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
  //double target_ang_acc_z = pid_controllers_.at(YAW).result();
  double target_ang_acc_z = candidate_yaw_term_;
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  /*
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
  */

  /*
  geometry_msgs::WrenchStamped feedforward_wrench_msg;
  feedforward_wrench_msg.wrench.force.x = external_wrench_pid_controllers_.at(X).result();
  feedforward_wrench_msg.wrench.force.y = external_wrench_pid_controllers_.at(Y).result();
  feedforward_wrench_msg.wrench.force.z = external_wrench_pid_controllers_.at(Z).result();
  feedforward_wrench_msg.wrench.torque.x = external_wrench_pid_controllers_.at(ROLL).result();
  feedforward_wrench_msg.wrench.torque.y = external_wrench_pid_controllers_.at(PITCH).result();
  feedforward_wrench_msg.wrench.torque.z = external_wrench_pid_controllers_.at(YAW).result();
  */

  /* feedforward */
  double mass_inv = 1/ hydrus_robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = hydrus_robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  Eigen::Vector3d des_force,des_torque;
  Eigen::Matrix3d cog_rot;
  Eigen::Vector3d est_external_force_cog;
  Eigen::Vector3d est_external_force;
  est_external_force =  est_external_wrench_.head(3);
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);
  est_external_force_cog = cog_rot.inverse() * est_external_force;
  for(int i;i<3;i++)
    {
      des_force[i] = desire_wrench_[i] + est_external_force_cog[i];
      des_torque[i] = desire_wrench_[i+3] + est_external_wrench_[i+3];
    }

  Eigen::Vector3d des_acc = des_force * mass_inv;
  Eigen::Vector3d des_ang_acc = inertia_inv * des_torque;
  //ROS_INFO("send_feedforward_switch_flag: %d", send_feedforward_switch_flag_);
  Eigen::Vector3d feedforward_sum_3 = feedforward_sum_.head(3);
  Eigen::Vector3d feedforward_world = cog_rot * (des_acc + feedforward_sum_3);
  
  if(send_feedforward_switch_flag_ && attaching_flag_)
  {
    // target_pitch_ += des_acc[0];
    // target_roll_ += des_acc[1];
    navigator_->setTargetAccX(feedforward_world[0]);
    navigator_->setTargetAccY(feedforward_world[1]);
    navigator_->setTargetAngAccZ(des_ang_acc[2] + feedforward_sum_[5]);
    target_wrench_acc_cog[0] += feedforward_world[0];
    target_wrench_acc_cog[1] += feedforward_world[1];
    target_wrench_acc_cog[5] += des_ang_acc[2] + feedforward_sum_[5];

    feedforward_sum_.head(3) += des_acc * wrench_diff_gain_;
    feedforward_sum_.tail(3) += des_ang_acc * wrench_diff_gain_;

    std::cout << "send_feedforward" << std::endl;
  }
  if(!attaching_flag_)
  {
    navigator_->setTargetAccX(0);
    navigator_->setTargetAccY(0);
    navigator_->setTargetAngAccZ(0);
    feedforward_sum_ = Eigen::VectorXd::Zero(6);
  }
  if(pid_controllers_.at(X).result()<0.0)
  {
    //attaching_flag_ = false;
  }

  geometry_msgs::Vector3Stamped feedforward_acc_cog_msg;
  geometry_msgs::Vector3Stamped feedforward_ang_acc_cog_msg;
  geometry_msgs::WrenchStamped des_wrench_cog_msg;
  feedforward_acc_cog_msg.vector.x = feedforward_world[0];
  feedforward_acc_cog_msg.vector.y = feedforward_world[1];
  feedforward_acc_cog_msg.vector.z = feedforward_world[2];
  feedforward_ang_acc_cog_msg.vector.x = feedforward_sum_[3];
  feedforward_ang_acc_cog_msg.vector.y = feedforward_sum_[4];
  feedforward_ang_acc_cog_msg.vector.z = feedforward_sum_[5];
  des_wrench_cog_msg.wrench.force.x = des_force[0];
  des_wrench_cog_msg.wrench.force.y = des_force[1];
  des_wrench_cog_msg.wrench.force.z = des_force[2];
  des_wrench_cog_msg.wrench.torque.x = des_torque[0];
  des_wrench_cog_msg.wrench.torque.y = des_torque[1];
  des_wrench_cog_msg.wrench.torque.z = des_torque[2];
  feedforward_acc_cog_pub_.publish (feedforward_acc_cog_msg);
  feedforward_ang_acc_cog_pub_.publish(feedforward_ang_acc_cog_msg);
  des_wrench_cog_pub_.publish(des_wrench_cog_msg);
  setTargetWrenchAccCog(target_wrench_acc_cog);

}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQIController, aerial_robot_control::ControlBase);
 
