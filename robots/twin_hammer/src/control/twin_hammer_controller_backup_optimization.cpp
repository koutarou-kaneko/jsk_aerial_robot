#include <twin_hammer/control/twin_hammer_controller.h>

using namespace aerial_robot_control;

namespace
{
  double minimizeThrust(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    TwinHammerController *planner = reinterpret_cast<TwinHammerController*>(planner_ptr);
    auto twin_hammer_model = planner->getHammerModel();
    double cost = 0.0;
    double alpha = 1.0; // weight for dp 
    int virtual_rotor_num = twin_hammer_model->getRotorNum()/2;
    for(int i=0; i<virtual_rotor_num; i++)
    {
      Eigen::Vector3d thrust1(x[0],x[1],x[2]);
      Eigen::Vector3d thrust2(x[4],x[5],x[6]);
      Eigen::Vector2d movement(x[3], x[7]);

      double thrusts_cost = thrust1[0]*thrust1[0] + thrust1[1]*thrust1[1] + thrust1[2]*thrust1[2] +  thrust2[0]*thrust2[0] + thrust2[1]*thrust2[1] + thrust2[2]*thrust2[2];
      double movement_cost = movement[0]*movement[0] + movement[1]*movement[1];
      cost += thrusts_cost + alpha*movement_cost;
      // if(!grad.empty())
      // {
      //   grad[i] = 2*f;
      //   grad[3*n+i] = 2*alpha*dp;
      // }
    }
    return cost;
  }

  void WrenchConstraint(unsigned m, double* result, unsigned n, const double* x, double *gradient, void *planner_ptr)
  {
    std::fill(result, result+6, 0.0);
    TwinHammerController *planner = reinterpret_cast<TwinHammerController*>(planner_ptr);
    auto twin_hammer_model = planner->getHammerModel();
    int virtual_rotor_num = twin_hammer_model->getRotorNum()/2;
    Eigen::VectorXd target_wrench = planner->getTargetWrenchAccCog();

    Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3*virtual_rotor_num);
    double mass_inv = 1/twin_hammer_model->getMass();
    Eigen::Matrix3d inertia = twin_hammer_model->getInertia<Eigen::Matrix3d>();
    Eigen::Matrix3d inertia_inv = (twin_hammer_model->getInertia<Eigen::Matrix3d>()).inverse();
    std::vector<Eigen::Vector3d> rotors_origin_from_cog = twin_hammer_model->getRotorsOriginFromCog<Eigen::Vector3d>();
    double rotor1_x = (rotors_origin_from_cog.at(0)[0] + rotors_origin_from_cog.at(2)[0])/2;
    double rotor1_y = (rotors_origin_from_cog.at(0)[1] + rotors_origin_from_cog.at(2)[1])/2;
    double rotor1_z = (rotors_origin_from_cog.at(0)[2] + rotors_origin_from_cog.at(2)[2])/2;
    double rotor2_x = (rotors_origin_from_cog.at(1)[0] + rotors_origin_from_cog.at(4)[0])/2;
    double rotor2_y = (rotors_origin_from_cog.at(1)[1] + rotors_origin_from_cog.at(4)[1])/2;
    double rotor2_z = (rotors_origin_from_cog.at(1)[2] + rotors_origin_from_cog.at(4)[2])/2;
    Eigen::Vector3d rotor1_origin_from_cog(rotor1_x, rotor1_y, rotor1_z);
    Eigen::Vector3d rotor2_origin_from_cog(rotor2_x, rotor2_y, rotor2_z);
    rotor1_origin_from_cog[1] += x[3];
    rotor2_origin_from_cog[1] += x[7];

    Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6,3);
    wrench_map.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);
    wrench_map.block(3,0,3,3) = aerial_robot_model::skew(rotor1_origin_from_cog);
    full_q_mat.middleCols(0,3) = wrench_map;
    wrench_map.block(3,0,3,3) = aerial_robot_model::skew(rotor2_origin_from_cog);
    full_q_mat.middleCols(3,3) = wrench_map;

    full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
    full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);
    Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);

    Eigen::VectorXd target_vectoring_f(6);
    target_vectoring_f << x[0],x[1],x[2],x[4],x[5],x[6];

    auto res = full_q_mat_inv * target_vectoring_f - target_wrench;
    for(int i=0; i<m; i++)
    {
      result[i] = res[i];
    }
  }
};


TwinHammerController::TwinHammerController():
  PoseLinearController()
{
}

void TwinHammerController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                  double ctrl_loop_rate
                                  )
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  twin_hammer_model_ = boost::dynamic_pointer_cast<TwinHammerModel>(robot_model);

  nl_solver_ = boost::make_shared<nlopt::opt>(nlopt::LD_SLSQP, 4*motor_num_/2);
  opt_x_.resize(4*motor_num_/2, 0.0);
  prev_opt_x_.resize(4*motor_num_/2, 0.0);
  opt_x_.at(2) = 10.0;
  opt_x_.at(6) = 10.0;
  nl_solver_ -> set_min_objective(minimizeThrust, this);
  nl_solver_ -> add_equality_mconstraint(WrenchConstraint, this, std::vector<double>(6,1e-8));
  nl_solver_ -> set_xtol_rel(1e-4);

  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_, 0);
  target_wrench_acc_cog_ = Eigen::VectorXd::Zero(6);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  haptics_switch_sub_ = nh_.subscribe("haptics_switch", 1, &TwinHammerController::HapticsSwitchCallback, this);
  haptics_wrench_sub_ = nh_.subscribe("haptics_wrench", 1, &TwinHammerController::HapticsWrenchCallback, this);

  haptics_switch_ = false;
  target_wrench_acc_cog_ = Eigen::VectorXd::Zero(6);  

  haptics_force_ = Eigen::Vector3d::Zero();
  haptics_torque_ = Eigen::Vector3d::Zero();

  rosParamInit();
}

void TwinHammerController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "use_haptics", use_haptics_flag_, true);
  getParam<double>(control_nh, "gimbal_roll_delta_angle", gimbal_roll_delta_angle_, 0.2);
  getParam<double>(control_nh, "gimbal_pitch_delta_angle", gimbal_pitch_delta_angle_, 0.2);
  
}

void TwinHammerController::HapticsSwitchCallback(std_msgs::Int8 msg)
{
  int i = msg.data;
  if(i==1){haptics_switch_ = true;}
  if(i==0){
    haptics_switch_ = false;
    haptics_force_ = Eigen::Vector3d::Zero();
    haptics_torque_ = Eigen::Vector3d::Zero();
  }
  // std::cout << "switch" << haptics_switch_ << std::endl;
}

void TwinHammerController::HapticsWrenchCallback(geometry_msgs::WrenchStamped msg)
{
  haptics_force_(0) = msg.wrench.force.x;
  haptics_force_(1) = msg.wrench.force.y;
  haptics_force_(2) = msg.wrench.force.z;
  haptics_torque_(0) = msg.wrench.torque.x;
  haptics_torque_(1) = msg.wrench.torque.y;
  haptics_torque_(2) = msg.wrench.torque.z;
  // std::cout << "wrench_x" << haptics_wrench_(0) << std::endl;
}

void TwinHammerController::controlCore()
{
  PoseLinearController::controlCore();
  tf::Vector3 target_acc_w(0.0, 0.0, 0.0);
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  if(use_haptics_flag_)
  {
    tf::Vector3 tf_haptics_force(haptics_force_.x(), haptics_force_.y(), haptics_force_.z());
    target_acc_w = tf_haptics_force;
  }
  else
  {
    tf::Vector3 pid_result(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
    target_acc_w = pid_result;
  }
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  // Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);  
  target_wrench_acc_cog_.head(3) = Eigen::Vector3d(target_acc_cog.x(),target_acc_cog.y(),target_acc_cog.z());

  double target_ang_acc_x = 0.0;
  double target_ang_acc_y = 0.0;
  double target_ang_acc_z = 0.0;

  if(use_haptics_flag_)
  {
    target_ang_acc_x = haptics_torque_.x();
    target_ang_acc_y = haptics_torque_.y();
    target_ang_acc_z = haptics_torque_.z();
  }
  else
  {
    target_ang_acc_x = pid_controllers_.at(ROLL).result();
    target_ang_acc_y = pid_controllers_.at(PITCH).result();
    target_ang_acc_z = pid_controllers_.at(YAW).result();
  }
  target_wrench_acc_cog_.tail(3) = Eigen::Vector3d(target_ang_acc_x,target_ang_acc_y,target_ang_acc_z);

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  double virtual_rotor_num = motor_num_/2;
  std::vector<double> lb(4*virtual_rotor_num, 0.0);
  std::vector<double> ub(4*virtual_rotor_num, 10.0);
  // thrust limit
  for(int i=0; i<virtual_rotor_num; i++)
  {
    lb.at(i) = 0.0;
    lb.at(i+1) = 0.0;
    lb.at(i+2) = 0.0;
    lb.at(i+3) = -0.212;     // This is hard cording. Rewrite to get link length from robot model 
    ub.at(i) = 15.0;
    ub.at(i+1) = 15.0;
    ub.at(i+2) = 15.0;
    ub.at(i+3) = 0.212;     // This is hard cording. Rewrite to get link length from robot model 
  }
  nl_solver_ -> set_lower_bounds(lb);
  nl_solver_ -> set_upper_bounds(ub);
  double min_f = 0;
  try
  {
    nlopt::result result = nl_solver_ -> optimize(opt_x_, min_f);
  }
  catch(std::exception &e)
  {
    std::cout << "nlopt failed: " << e.what() << std::endl;
  }

  /*
  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 4*virtual_rotor_num);

  double mass_inv = 1/twin_hammer_model_->getMass();
  Eigen::Matrix3d inertia = twin_hammer_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Matrix3d inertia_inv = (twin_hammer_model_->getInertia<Eigen::Matrix3d>()).inverse();
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = twin_hammer_model_->getRotorsOriginFromCog<Eigen::Vector3d>();

  double t = ros::Time::now().toSec();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6,3);
  wrench_map.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);
  int last_col = 0;

  for(int i=0; i<virtual_rotor_num; i++)
  {
    Eigen::Matrix3d skew_rotor_mat = aerial_robot_model::skew(rotors_origin_from_cog.at(i));
    wrench_map.block(3,0,3,3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));
    full_q_mat.middleCols(last_col, 3) = wrench_map;
    last_col += 4;
  }

  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);
  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
  target_vectoring_f_ = full_q_mat_inv * target_wrench_acc_cog_;
  last_col = 0;

  for(int i=0; i<virtual_rotor_num; i++)
  {
    Eigen::Vector3d f_i = target_vectoring_f_.segment(last_col,3);

    target_base_thrust_.at(i) = f_i.norm();
    // std::cout << "rotor" << i << ":target_base_thrust is " << target_base_thrust_.at(i) << std::endl;
 
    double gimbal_i_roll = atan2(-f_i.y(), f_i.z());
    double gimbal_i_pitch = atan2(f_i.x(), -f_i.y() * sin(gimbal_i_roll) + f_i.z() * cos(gimbal_i_roll));
    // target_gimbal_angles_.at(2*i) = gimbal_i_roll;
    // target_gimbal_angles_.at(2*i+1) = gimbal_i_pitch;
    // std::cout << "gimbal_i_roll is " << gimbal_i_roll << std::endl;
    // std::cout << "gimbal_i_pitch is " << gimbal_i_pitch << std::endl;
    last_col += 4;
  } 
  // std::cout << "-----------" << std::endl;
  */
}

void TwinHammerController::sendCmd()
{
  PoseLinearController::sendCmd();
  // spinal::FourAxisCommand flight_command_data;
  // flight_command_data.base_thrust = target_base_thrust_;
  // flight_cmd_pub_.publish(flight_command_data);

  // sensor_msgs::JointState gimbal_control_msg;
  // gimbal_control_msg.header.stamp = ros::Time::now();
  // for(int i=0; i<motor_num_; i++)
  // {
  //   gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
  // }
  // gimbal_control_pub_.publish(gimbal_control_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::TwinHammerController, aerial_robot_control::ControlBase);
