#include <hydrus/hydrus_tilted_lqi_controller.h>
#include <hydrus/hydrus_tilted_robot_model.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf_conversions/tf_eigen.h>
#include <cstdlib>

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
  feedforward_acc_cog_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("feedforward_acc_world", 1);
  feedforward_ang_acc_cog_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("feedforward_ang_acc_cog", 1);
  des_wrench_cog_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("des_wrench_cog", 1);
  attaching_flag_pub_ = nh_.advertise<std_msgs::Bool>("attaching_flag",1);
  filtered_est_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("filtered_est_external_wrench",1);
  desire_wrench_sub_ = nh_.subscribe("desire_wrench", 1, &HydrusTiltedLQIController::DesireWrenchCallback, this);
  filterd_ftsensor_sub_ = nh_.subscribe("/filterd_ftsensor", 1, &HydrusTiltedLQIController::FilterdFtsensorCallBack, this);
  desire_pos_sub_ = nh_.subscribe("uav/nav", 1, &HydrusTiltedLQIController::DesirePosCallback, this);
  hand_force_switch_sub_ = nh_.subscribe("hand_force_switch", 1, &HydrusTiltedLQIController::HandForceSwitchCallBack, this);
  estimated_external_wrench_in_cog_ = Eigen::VectorXd::Zero(6);
  desire_wrench_ = Eigen::VectorXd::Zero(6);
  filtered_ftsensor_wrench_ = Eigen::VectorXd::Zero(6);
  desire_wrench_from_pos_ = Eigen::VectorXd::Zero(6);
  target_wrench_cog_ = Eigen::VectorXd::Zero(6);
  p_wrench_stamp_ = Eigen::VectorXd::Zero(6);
  feedforward_sum_ = Eigen::VectorXd::Zero(6);
  desire_pos_ = Eigen::Vector3d::Zero(6);
  attaching_flag_ = false;
  const_err_i_flag_ = false;
  first_flag_ = true;

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<double>(control_nh, "wrench_diff_gain", wrench_diff_gain_, 1.0);
  getParam<bool>(control_nh, "send_feedforward_switch_flag", send_feedforward_switch_flag_, false);
  getParam<bool>(control_nh, "using_FTsensor", using_FTsensor_, false);
  getParam<double>(control_nh, "acc_shock_thres", acc_shock_thres_, 20.0);
  double cutoff_freq, sample_freq;
  getParam<double>(control_nh, "cutoff_freq", cutoff_freq, 25.0);
  getParam<double>(control_nh, "sample_freq", sample_freq, 100.0);
  lpf_est_external_wrench_ = IirFilter(sample_freq, cutoff_freq, 6);

  x_p_gain_ = pid_controllers_.at(X).getPGain();
  y_p_gain_ = pid_controllers_.at(Y).getPGain();
  
}


void HydrusTiltedLQIController::DesireWrenchCallback(geometry_msgs::WrenchStamped msg)
{
  Eigen::Vector3d desire_force_at_end;
  desire_force_at_end[0] = msg.wrench.force.x;
  desire_force_at_end[1] = msg.wrench.force.y;
  desire_force_at_end[2] = msg.wrench.force.z;
  hydrus_robot_model_->setTargetForceInRootEnd(desire_force_at_end);
  Eigen::Vector3d des_force_for_root_end_in_cog = hydrus_robot_model_->getCompensateForceForRootEndInCog();
  Eigen::Vector3d des_torque_for_root_end_in_cog = hydrus_robot_model_->getCompensateTorqueForRootEndInCog();
  // KDL::Frame root_end = hydrus_robot_model_->getRootEnd();
  // KDL::Frame cog = hydrus_robot_model_->getCog<KDL::Frame>();
  // Eigen::Vector3d des_force_for_root_end_in_cog = aerial_robot_model::kdlToEigen(cog.M.Inverse() * root_end.M) * desire_force_at_end;

  desire_wrench_[0] = des_force_for_root_end_in_cog[0];
  desire_wrench_[1] = des_force_for_root_end_in_cog[1];
  desire_wrench_[2] = des_force_for_root_end_in_cog[2];
  desire_wrench_[3] = des_torque_for_root_end_in_cog[0];
  desire_wrench_[4] = des_torque_for_root_end_in_cog[1];
  desire_wrench_[5] = des_torque_for_root_end_in_cog[2];
}

void HydrusTiltedLQIController::FilterdFtsensorCallBack(geometry_msgs::WrenchStamped msg)
{
  Eigen::Vector3d force_at_end, torque_at_end;
  KDL::Frame root_end = hydrus_robot_model_->getRootEnd();
  KDL::Frame cog = hydrus_robot_model_->getCog<KDL::Frame>();

  force_at_end[0] = msg.wrench.force.x;
  force_at_end[1] = msg.wrench.force.y;
  force_at_end[2] = msg.wrench.force.z;
  torque_at_end[0] = msg.wrench.torque.x;
  torque_at_end[1] = msg.wrench.torque.y;
  torque_at_end[2] = msg.wrench.torque.z;
  
  for(int i;i<3;i++)
  {
    if(force_at_end[i]>=4.0)
    {
      force_at_end[i]=4.0;
    }
  }

  Eigen::Vector3d force_for_root_end_in_cog = aerial_robot_model::kdlToEigen(cog.M.Inverse() * root_end.M) * force_at_end;

  filtered_ftsensor_wrench_[0] = force_for_root_end_in_cog[0];
  filtered_ftsensor_wrench_[1] = force_for_root_end_in_cog[1];
  filtered_ftsensor_wrench_[2] = force_for_root_end_in_cog[2];
  filtered_ftsensor_wrench_[3] = torque_at_end[0];
  filtered_ftsensor_wrench_[4] = torque_at_end[1];
  filtered_ftsensor_wrench_[5] = torque_at_end[2];

  // for(int i; i<6; i++){
  //   std::cout << filtered_ftsensor_wrench_[i] << ", ";
  // }
  // std::cout << std::endl;
}


void HydrusTiltedLQIController::DesirePosCallback(aerial_robot_msgs::FlightNav msg)
{
  desire_pos_[0] = msg.target_pos_x;
  desire_pos_[1] = msg.target_pos_y;
  desire_pos_[2] = msg.target_pos_z;
  desire_pos_[5] = msg.target_yaw;
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
  if(first_flag_)
  {
    lpf_est_external_wrench_.setInitValues(est_external_wrench_);
    first_flag_ = false;
  }
  Eigen::VectorXd filtered_est_external_wrench;
  filtered_est_external_wrench = lpf_est_external_wrench_.filterFunction(est_external_wrench_);

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

  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  //double target_ang_acc_z = pid_controllers_.at(YAW).result();
  double target_ang_acc_z = candidate_yaw_term_;
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  /* feedforward */
  double mass_inv = 1/ hydrus_robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = hydrus_robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  Eigen::Matrix3d cog_rot;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);

  Eigen::Vector3d target_force, target_torque;
  if(using_FTsensor_)
  {
  target_force = desire_wrench_.head(3) + filtered_ftsensor_wrench_.head(3);
  target_torque = desire_wrench_.tail(3) + filtered_ftsensor_wrench_.tail(3);
  }
  else
  {
  target_force = desire_wrench_.head(3) + cog_rot.inverse() * filtered_est_external_wrench.head(3);
  target_torque = desire_wrench_.tail(3) + cog_rot.inverse() * filtered_est_external_wrench.tail(3);
  }

  Eigen::Vector3d target_acc = mass_inv * target_force;
  Eigen::Vector3d target_ang_acc = inertia_inv * target_torque;
  Eigen::Vector3d feedforward_acc = cog_rot * (target_acc + feedforward_sum_.head(3));
  Eigen::Vector3d feedforward_ang_acc = cog_rot * (target_ang_acc + feedforward_sum_.tail(3));
  
  if(send_feedforward_switch_flag_ && attaching_flag_)
  {
    // target_pitch_ += target_acc[0];
    // target_roll_ += target_acc[1];
    navigator_->setTargetAccX(feedforward_acc[0]);
    navigator_->setTargetAccY(feedforward_acc[1]);
    // navigator_->setTargetAngAccZ(feedforward_ang_acc[2]);
    target_wrench_acc_cog[0] += feedforward_acc[0];
    target_wrench_acc_cog[1] += feedforward_acc[1];
    // target_wrench_acc_cog[5] += feedforward_ang_acc[2];

    feedforward_sum_.head(3) += target_acc * wrench_diff_gain_;
    feedforward_sum_.tail(3) += target_ang_acc * wrench_diff_gain_;

    std::cout << "send_feedforward" << std::endl;
  }
  if(!attaching_flag_)
  {
    navigator_->setTargetAccX(0);
    navigator_->setTargetAccY(0);
    // navigator_->setTargetAngAccZ(0);
    feedforward_sum_ = Eigen::VectorXd::Zero(6);
  }
  if(pid_controllers_.at(X).result()<0.0)
  {
    //attaching_flag_ = false;
  }
    
  // during attaching
  if(attaching_flag_)
    {
      if(!const_err_i_flag_)
        {
          err_i_x_ = pid_controllers_.at(X).getErrI();
          err_i_y_ = pid_controllers_.at(Y).getErrI();
          err_i_z_ = pid_controllers_.at(Z).getErrI();
          err_i_yaw_ = pid_controllers_.at(YAW).getErrI();
          x_p_gain_ = pid_controllers_.at(X).getPGain();
          y_p_gain_ = pid_controllers_.at(Y).getPGain();
          //err_p_y_ = pid_controllers_.at(Y).getErrP();
          const_err_i_flag_ = true;
        }
      pid_controllers_.at(X).setErrI(err_i_x_);
      pid_controllers_.at(Y).setErrI(err_i_y_);
      pid_controllers_.at(Z).setErrI(err_i_z_);
      pid_controllers_.at(YAW).setErrI(err_i_yaw_);
      //pid_controllers_.at(Y).setErrP(0);
      // pid_controllers_.at(X).setPGain(0.0);
      pid_controllers_.at(Y).setPGain(0.0);
    }
  if(!attaching_flag_)
  {
    // pid_controllers_.at(X).setPGain(x_p_gain_);
    pid_controllers_.at(Y).setPGain(y_p_gain_);
    const_err_i_flag_ = false;
  }
  
  UnderActuatedTiltedLQIController::controlCore();
  

  geometry_msgs::Vector3Stamped feedforward_acc_cog_msg;
  geometry_msgs::Vector3Stamped feedforward_ang_acc_cog_msg;
  geometry_msgs::WrenchStamped des_wrench_cog_msg;
  geometry_msgs::WrenchStamped filtered_est_external_wrench_msg;
  feedforward_acc_cog_msg.vector.x = feedforward_acc[0];
  feedforward_acc_cog_msg.vector.y = feedforward_acc[1];
  feedforward_acc_cog_msg.vector.z = feedforward_acc[2];
  feedforward_ang_acc_cog_msg.vector.x = feedforward_sum_[3];
  feedforward_ang_acc_cog_msg.vector.y = feedforward_sum_[4];
  feedforward_ang_acc_cog_msg.vector.z = feedforward_sum_[5];
  des_wrench_cog_msg.wrench.force.x = target_force[0];
  des_wrench_cog_msg.wrench.force.y = target_force[1];
  des_wrench_cog_msg.wrench.force.z = target_force[2];
  des_wrench_cog_msg.wrench.torque.x = target_torque[0];
  des_wrench_cog_msg.wrench.torque.y = target_torque[1];
  des_wrench_cog_msg.wrench.torque.z = target_torque[2];
  filtered_est_external_wrench_msg.wrench.force.x = filtered_est_external_wrench[0];
  filtered_est_external_wrench_msg.wrench.force.y = filtered_est_external_wrench[1];
  filtered_est_external_wrench_msg.wrench.force.z = filtered_est_external_wrench[2];
  filtered_est_external_wrench_msg.wrench.torque.x = filtered_est_external_wrench[3];
  filtered_est_external_wrench_msg.wrench.torque.y = filtered_est_external_wrench[4];
  filtered_est_external_wrench_msg.wrench.torque.z = filtered_est_external_wrench[5];

  feedforward_acc_cog_pub_.publish (feedforward_acc_cog_msg);
  feedforward_ang_acc_cog_pub_.publish(feedforward_ang_acc_cog_msg);
  des_wrench_cog_pub_.publish(des_wrench_cog_msg);
  filtered_est_external_wrench_pub_.publish(filtered_est_external_wrench_msg);
  setTargetWrenchAccCog(target_wrench_acc_cog);

}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQIController, aerial_robot_control::ControlBase);
 
