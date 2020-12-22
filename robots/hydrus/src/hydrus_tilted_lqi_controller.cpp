#include <hydrus/hydrus_tilted_lqi_controller.h>

using namespace aerial_robot_control;

void HydrusTiltedLQIController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  HydrusLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  desired_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  start_wall_touching_srv_ = nh.advertiseService("start_wall_touching", &HydrusTiltedLQIController::startWallTouching, this);
  set_horizontal_force_mode_srv_ = nh.advertiseService("set_horizontal_force_mode", &HydrusTiltedLQIController::setHorizontalForceMode, this);
  reset_horizontal_force_mode_srv_ = nh.advertiseService("reset_horizontal_force_mode", &HydrusTiltedLQIController::resetHorizontalForceMode, this);
  tilted_model_ = boost::dynamic_pointer_cast<HydrusTiltedRobotModel>(robot_model);
  navigator_ = navigator;

  //additional
  ff_wrench_pub_ = nh_.advertise<geometry_msgs::Vector3>("ff_wrench", 1);
  ff_wrench_noreset_pub_ = nh_.advertise<geometry_msgs::Vector3>("ff_wrench_noreset", 1);
  acc_root_sub_ = nh.subscribe("sensor_plugin/imu1/acc_root", 10, &HydrusTiltedLQIController::accRootCallback, this);

  //param
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<double>(control_nh, "acc_root_shock_thres", acc_root_shock_thres_, 4.5);

  pid_msg_.z.p_term.resize(1);
  pid_msg_.z.i_term.resize(1);
  pid_msg_.z.d_term.resize(1);
}

void HydrusTiltedLQIController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());

  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;
  
  Eigen::VectorXd f;


  if (not horizontal_force_mode_) {
    target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
    target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));

    f = robot_model_->getStaticThrust();
  } else {
    target_pitch_ = 0;
    target_roll_ = 0;

    // Don't calc here when optimize all axis
    //tilted_model_->calc3DoFThrust(ff_f_x_, ff_f_y_);
    f = tilted_model_->get3DoFThrust();
  }
  
  Eigen::VectorXd allocate_scales = f / f.sum() * robot_model_->getMass();
  Eigen::VectorXd target_thrust_z_term = allocate_scales * target_acc_w.length();

  // constraint z (also  I term)
  int index;
  double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - pid_controllers_.at(Z).getLimitSum();
  if(residual > 0)
    {
      pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getErrI() - residual / allocate_scales(index) / pid_controllers_.at(Z).getIGain());
      target_thrust_z_term *= (1 - residual / max_term);
    }

  for(int i = 0; i < motor_num_; i++)
    {
      target_base_thrust_.at(i) = target_thrust_z_term(i);
      pid_msg_.z.total.at(i) =  target_thrust_z_term(i);
    }

  allocateYawTerm();
}

void HydrusTiltedLQIController::allocateYawTerm()
{
  Eigen::VectorXd target_thrust_yaw_term = Eigen::VectorXd::Zero(motor_num_);
  Eigen::Vector4d p;
  if (horizontal_force_mode_ and wall_touching_) {
    auto cog = robot_model_->getCog<Eigen::Affine3d>();
    //auto ff_f_cog = cog.rotation().inverse() * Eigen::Vector3d(ff_f_x_, ff_f_y_, 0);
    double compensate = 1 * robot_model_->getMass() * (cog.translation()(1)*tilted_model_->ff_f_x_ - (cog.translation()(0)+0.08)*tilted_model_->ff_f_y_);
    ROS_INFO_STREAM_THROTTLE(0.1, "comp+ff: " << tilted_model_->ff_t_z_ + compensate);
    p << 0, 0, 0, tilted_model_->ff_t_z_ + compensate;
  } else {
    p << 0, 0, 0, 0;
  }
  auto ff_yaw_collective_thrust = p_mat_pseudo_inv_ * p;
  //ROS_INFO_STREAM_THROTTLE(0.1, "Feedforward term of thrust: " << ff_yaw_collective_thrust.transpose());
  for(int i = 0; i < motor_num_; i++)
    {
      double p_term = yaw_gains_.at(i)[0] * pid_controllers_.at(YAW).getErrP();
      double i_term = yaw_gains_.at(i)[1] * pid_controllers_.at(YAW).getErrI();
      double d_term = yaw_gains_.at(i)[2] * pid_controllers_.at(YAW).getErrD();
      target_thrust_yaw_term(i) = p_term + i_term + d_term + ff_yaw_collective_thrust(i);
      pid_msg_.yaw.p_term.at(i) = p_term;
      pid_msg_.yaw.i_term.at(i) = i_term;
      pid_msg_.yaw.d_term.at(i) = d_term;
    }
  // constraint yaw (also  I term)
  int index;
  double max_term = target_thrust_yaw_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - pid_controllers_.at(YAW).getLimitSum();
  if(residual > 0)
    {
      //pid_controllers_.at(YAW).setErrI(pid_controllers_.at(YAW).getErrI() - residual / yaw_gains_.at(index)[1]);
      target_thrust_yaw_term *= (1 - residual / max_term);
    }

  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  for(int i = 0; i < motor_num_; i++)
    {
      pid_msg_.yaw.total.at(i) =  target_thrust_yaw_term(i);

      if(yaw_gains_[i][2] > max_yaw_scale)
        {
          max_yaw_scale = yaw_gains_[i][2];
          candidate_yaw_term_ = target_thrust_yaw_term(i);
        }
    }
}

void HydrusTiltedLQIController::accRootCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  if ((not wall_touching_) and horizontal_force_mode_ and msg->vector.y > acc_root_shock_thres_) {
    wall_touching_ = true;
    ROS_INFO("Collided with the wall");
  }
}

bool HydrusTiltedLQIController::optimalGain()
{
  /* calculate the P_orig pseudo inverse */
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::MatrixXd P_dash  = inertia.inverse() * P.bottomRows(3); // roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9, motor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, 9);
  for(int i = 0; i < 3; i++)
    {
      A(2 * i, 2 * i + 1) = 1;
      B.row(2 * i + 1) = P_dash.row(i);
      C(i, 2 * i) = 1;
    }
  A.block(6, 0, 3, 9) = -C;

  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: B: \n"  <<  B );

  Eigen::VectorXd q_diagonals(9);
  q_diagonals << lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd P_trans = P.topRows(3) / robot_model_->getMass() ;
  Eigen::MatrixXd R_trans = P_trans.transpose() * P_trans;
  Eigen::MatrixXd R_input = Eigen::MatrixXd::Identity(motor_num_, motor_num_);
  Eigen::MatrixXd R = R_trans * trans_constraint_weight_ + R_input * att_control_weight_;

  double t = ros::Time::now().toSec();
  bool use_kleinman_method = true;
  if(K_.cols() == 0 || K_.rows() == 0) use_kleinman_method = false;
  if(!control_utils::care(A, B, R, Q, K_, use_kleinman_method))
    {
      ROS_ERROR_STREAM("error in solver of continuous-time algebraic riccati equation");
      return false;
    }

  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator: CARE: %f sec" << ros::Time::now().toSec() - t);
  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator:  K \n" <<  K_);

  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i) = Eigen::Vector3d(-K_(i,0), K_(i,6), -K_(i,1));
      pitch_gains_.at(i) = Eigen::Vector3d(-K_(i,2),  K_(i,7), -K_(i,3));
      yaw_gains_.at(i) = Eigen::Vector3d(-K_(i,4), K_(i,8), -K_(i,5));
    }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, 4));
  return true;
}

void HydrusTiltedLQIController::publishGain()
{
  HydrusLQIController::publishGain();

  double roll,pitch, yaw;
  robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

  spinal::DesireCoord coord_msg;
  coord_msg.roll = roll;
  coord_msg.pitch = pitch;
  desired_baselink_rot_pub_.publish(coord_msg);
}

void HydrusTiltedLQIController::rosParamInit()
{
  HydrusLQIController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");

  getParam<double>(lqi_nh, "trans_constraint_weight", trans_constraint_weight_, 1.0);
  getParam<double>(lqi_nh, "att_control_weight", att_control_weight_, 1.0);
}

bool HydrusTiltedLQIController::startWallTouching(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  horizontal_force_mode_ = true;
  wall_touching_ = false;
  navigator_->horizontal_mode_ = true;
  tilted_model_->horizontal_mode_ = true;
  ROS_INFO("start wall touching");
  double approach_force = -0.3;
  geometry_msgs::Vector3 ff_msg;
  ff_msg.x = approach_force;
  ff_msg.y = 0;
  ff_msg.z = 0;
  ff_wrench_pub_.publish(ff_msg);
  while (not wall_touching_) {
    // dist srv no nakade update mendoksuai
    //ff_msg.x = -0.5*(wall_dist_now / wall_dist_start);
    //ff_wrench_noreset_pub_.publish(ff_msg);
    ros::Duration(0.1).sleep();
  }
  for (; approach_force > -1.0; approach_force-=0.1) {
    ff_msg.x = approach_force;
    ff_msg.z = -1.0 - approach_force;
    ff_wrench_noreset_pub_.publish(ff_msg);
    ros::Duration(0.3).sleep();
  }
  ROS_INFO("finish wall touching");
  
  return true;
}

bool HydrusTiltedLQIController::setHorizontalForceMode(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  horizontal_force_mode_ = true;
  wall_touching_ = false;
  navigator_->horizontal_mode_ = true;
  tilted_model_->horizontal_mode_ = true;
  ROS_INFO("horizontal force mode set");
  return true;
}

bool HydrusTiltedLQIController::resetHorizontalForceMode(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  horizontal_force_mode_ = false;
  wall_touching_ = false;
  navigator_->horizontal_mode_ = false;
  tilted_model_->horizontal_mode_ = false;
  ROS_INFO("came back to normal control mode");
  return true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQIController, aerial_robot_control::ControlBase);
