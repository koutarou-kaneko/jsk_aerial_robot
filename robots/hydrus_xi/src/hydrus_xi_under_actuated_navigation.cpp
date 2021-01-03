#include <hydrus_xi/hydrus_xi_under_actuated_navigation.h>

using namespace aerial_robot_navigation;

namespace
{
  int cnt = 0;
  int invalid_cnt = 0;

  double maximizeHorizontalForceSquare(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();
    /* update robot model */
    KDL::JntArray joint_positions = planner->getJointPositionsForPlan();
    
    auto root = robot_model->getSegmentsTf().at("link1");
    auto cog2root = aerial_robot_model::kdlToEigen(robot_model->getCog<KDL::Frame>().Inverse() * root);
    auto t1_mat = (cog2root * robot_model->forwardKinematics<Eigen::Affine3d>(std::string("thrust1"), planner->getJointPositionsForPlan())).matrix();
    auto t2_mat = (cog2root * robot_model->forwardKinematics<Eigen::Affine3d>(std::string("thrust2"), planner->getJointPositionsForPlan())).matrix();
    auto t3_mat = (cog2root * robot_model->forwardKinematics<Eigen::Affine3d>(std::string("thrust3"), planner->getJointPositionsForPlan())).matrix();
    auto t4_mat = (cog2root * robot_model->forwardKinematics<Eigen::Affine3d>(std::string("thrust4"), planner->getJointPositionsForPlan())).matrix();

    ROS_INFO_STREAM_THROTTLE(1, "cog2root:\n" << cog2root.matrix());
    ROS_INFO_THROTTLE(1, "t1: %lf %lf", t1_mat(0,2), t1_mat(1,2));
    ROS_INFO_THROTTLE(1, "t2: %lf %lf", t2_mat(0,2), t2_mat(1,2));
    ROS_INFO_THROTTLE(1, "t3: %lf %lf", t3_mat(0,2), t3_mat(1,2));
    ROS_INFO_THROTTLE(1, "t4: %lf %lf", t4_mat(0,2), t4_mat(1,2));

    for(int i = 0; i < x.size(); i++)
      joint_positions(planner->getControlIndices().at(i)) = x.at(i);

    robot_model->updateRobotModel(joint_positions);

    /* suppress for debugging
    if(!robot_model->stabilityCheck(planner->getPlanVerbose()))
      {
        invalid_cnt ++;
        std::stringstream ss;
        for(const auto& angle: x) ss << angle << ", ";
        if(planner->getPlanVerbose()) ROS_WARN_STREAM("nlopt, robot stability is invalid with gimbals: " << ss.str() << " (cnt: " << invalid_cnt << ")");
        return 0;
      }
    */

    invalid_cnt = 0;

    robot_model->calc3DoFThrust(planner->ff_f_xy_[0], planner->ff_f_xy_[1]);
    auto thrust = robot_model->get3DoFThrust();
    Eigen::Vector2d t_sum = {t1_mat(0,2)*thrust(0)+t2_mat(0,2)*thrust(1)+t3_mat(0,2)*thrust(2)+t4_mat(0,2)*thrust(3), t1_mat(1,2)*thrust(0)+t2_mat(1,2)*thrust(1)+t3_mat(1,2)*thrust(2)+t4_mat(1,2)*thrust(3)};

    //ROS_INFO_THROTTLE(1, "dir, thrust: %lf %lf %lf %lf %lf %lf", planner->h_f_direction_(0), planner->h_f_direction_(1), thrust(0), thrust(1), thrust(2), thrust(3));

    double average_force = thrust.sum() / thrust.size();
    double variant = 0;

    for(int i = 0; i < thrust.size(); i++)
      variant += ((thrust(i) - average_force) * (thrust(i) - average_force));

    variant = sqrt(variant / thrust.size());

    auto tw = robot_model->thrust_wrench_;
    //ROS_INFO_STREAM_THROTTLE(1, "obj func element: " << -0.1*tw(3)*tw(3) << " " << -0.1*tw(4)*tw(4) << " " << planner->h_f_direction_.dot(t_sum));
    return -planner->getUncontrolledTorqueWeight()*tw(3)*tw(3) + -planner->getUncontrolledTorqueWeight()*tw(4)*tw(4)/* + planner->h_f_direction_.dot(t_sum)*/;
    //return planner->getForceNormWeight() * robot_model->getMass() / thrust.norm()  + planner->getForceVariantWeight() / variant + planner->h_f_direction_.dot(t_sum);
  }

  double maximizeFCTMin(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();
    /* update robot model */
    KDL::JntArray joint_positions = planner->getJointPositionsForPlan();
    for(int i = 0; i < x.size(); i++)
      joint_positions(planner->getControlIndices().at(i)) = x.at(i);

    robot_model->updateRobotModel(joint_positions);

    if(!robot_model->stabilityCheck(planner->getPlanVerbose()))
      {
        invalid_cnt ++;
        std::stringstream ss;
        for(const auto& angle: x) ss << angle << ", ";
        if(planner->getPlanVerbose()) ROS_WARN_STREAM("nlopt, robot stability is invalid with gimbals: " << ss.str() << " (cnt: " << invalid_cnt << ")");
        return 0;
      }

    invalid_cnt = 0;

    Eigen::VectorXd force_v = robot_model->getStaticThrust();
    double average_force = force_v.sum() / force_v.size();
    double variant = 0;

    for(int i = 0; i < force_v.size(); i++)
      variant += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variant = sqrt(variant / force_v.size());

    return planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()  + planner->getForceVariantWeight() / variant + planner->getFCTMinWeight() * robot_model->getFeasibleControlTMin();
  }

  double maximizeFCTMinWide(const std::vector<double> &x_wide, std::vector<double> &grad, void *planner_ptr)
  {
    std::vector<double> x(4);
    for (int i=0; i<4; i++) {
      x[i] = x_wide[i];
    }
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();
    /* update robot model */
    KDL::JntArray joint_positions = planner->getJointPositionsForPlan();
    for(int i = 0; i < x.size(); i++)
      joint_positions(planner->getControlIndices().at(i)) = x.at(i);

    robot_model->updateRobotModel(joint_positions);

    /* suppress for debug
    if(!robot_model->stabilityCheck(planner->getPlanVerbose()))
      {
        invalid_cnt ++;
        std::stringstream ss;
        for(const auto& angle: x) ss << angle << ", ";
        if(planner->getPlanVerbose()) ROS_WARN_STREAM("nlopt, robot stability is invalid with gimbals: " << ss.str() << " (cnt: " << invalid_cnt << ")");
        return 0;
      }
    */
    invalid_cnt = 0;

    // Thrust norm/var
    Eigen::VectorXd force_v(4);
    force_v << x_wide[4], x_wide[5], x_wide[6], x_wide[7];
    double average_force = force_v.sum() / force_v.size();
    double variant = 0;

    for(int i = 0; i < force_v.size(); i++)
      variant += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variant = sqrt(variant / force_v.size());

    if (planner->getJointTorqueWeight() > 0) {
      // Joint Torque
      Eigen::VectorXd hori_thrust(4);
      for (int i=0; i<4; i++) {
        hori_thrust(i) = x_wide[i+4] + planner->robot_model_real_->thrusts_yaw_term_(i);
      }
      robot_model->calcCoGMomentumJacobian();
      robot_model->calcJointTorque(hori_thrust);
      auto jt = robot_model->getJointTorque();
      //ROS_INFO_STREAM_THROTTLE(0.1, jt.transpose());

      ROS_INFO_STREAM_THROTTLE(1, "obj func elem: " << -planner->getJointTorqueWeight() * (jt[1]*jt[1]+jt[3]*jt[3]) << " " << planner->getForceNormWeight() * robot_model->getMass() / force_v.norm() << " " << planner->getForceVariantWeight() / variant << " " << planner->getFCTMinWeight() * robot_model->getFeasibleControlTMin());
      return -planner->getJointTorqueWeight() * (jt[1]*jt[1]+jt[3]*jt[3]) + planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()  + planner->getForceVariantWeight() / variant + planner->getFCTMinWeight() * robot_model->getFeasibleControlTMin();
    } else {
      return planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()  + planner->getForceVariantWeight() / variant + planner->getFCTMinWeight() * robot_model->getFeasibleControlTMin();
    }
  }

  double maximizeBalanceWide(const std::vector<double> &x_wide, std::vector<double> &grad, void *planner_ptr)
  {
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();

    Eigen::VectorXd thrust(4);
    thrust << x_wide[4], x_wide[5], x_wide[6], x_wide[7];

    double average_force = thrust.sum() / thrust.size();
    double variant = 0;

    for(int i = 0; i < thrust.size(); i++)
      variant += ((thrust(i) - average_force) * (thrust(i) - average_force));

    variant = sqrt(variant / thrust.size());

    //ROS_INFO_STREAM_THROTTLE(1, "obj func elem: " << planner->getForceNormWeight() * robot_model->getMass() / thrust.norm() << " " << planner->getForceVariantWeight() / variant);
    return planner->getForceNormWeight() * robot_model->getMass() / thrust.norm()  + planner->getForceVariantWeight() / variant;
  }

  double maximizeMinYawTorque(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    cnt++;
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();

    /* update robot model */
    KDL::JntArray joint_positions = planner->getJointPositionsForPlan();
    for(int i = 0; i < x.size(); i++)
      joint_positions(planner->getControlIndices().at(i)) = x.at(i);

    robot_model->updateRobotModel(joint_positions);

    if(!robot_model->stabilityCheck(planner->getPlanVerbose()))
      {
        invalid_cnt ++;
        if(planner->getPlanVerbose()) ROS_WARN("nlopt, robot stability is invalid (cnt: %d)", invalid_cnt);
        return 0;
      }
    else
      {
        invalid_cnt = 0;

        /* 1. calculate the max and min yaw torque by LP */
        Eigen::VectorXd gradient = robot_model->calcWrenchMatrixOnCoG().row(5).transpose();
        Eigen::VectorXd max_u, min_u;
        double max_yaw, min_yaw;

        //std::cout << "yaw torque map: " << gradient.transpose() << std::endl;

        /* get min u and min yaw */
        planner->getYawRangeLPSolver().updateGradient(gradient);
        if(!planner->getYawRangeLPSolver().solve())
          {
            ROS_ERROR("cat not calcualte the min u by LP");
            planner->setMaxMinYaw(0);
          }
        else
          {
            min_u = planner->getYawRangeLPSolver().getSolution();
            //std::cout << "min_u: " << min_u.transpose() << std::endl;
            min_yaw = (gradient.transpose() * min_u)(0);
            if(min_yaw > 0)
              {
                ROS_WARN("the min yaw is positive: %f", min_yaw);
                min_yaw = 0;
              }
          }

        /* get max u and max yaw */
        Eigen::VectorXd reverse_gradient = - gradient;
        planner->getYawRangeLPSolver().updateGradient(reverse_gradient);
        if(!planner->getYawRangeLPSolver().solve())
          {
            ROS_ERROR("cat not calcualte the max u by LP");
            planner->setMaxMinYaw(0);
          }
        else
          {
            max_u = planner->getYawRangeLPSolver().getSolution();
            max_yaw = (gradient.transpose() * max_u)(0);
          }

        //ROS_INFO("LP: max: %f, min: %f", max_yaw, min_yaw); //debug
        planner->setMaxMinYaw(std::min(max_yaw, -min_yaw));
      }

    Eigen::VectorXd force_v = robot_model->getStaticThrust();
    double average_force = force_v.sum() / force_v.size();
    double variant = 0;

    for(int i = 0; i < force_v.size(); i++)
      variant += ((force_v(i) - average_force) * (force_v(i) - average_force));

    variant = sqrt(variant / force_v.size());

    return planner->getForceNormWeight() * robot_model->getMass() / force_v.norm()  + planner->getForceVariantWeight() / variant + planner->getYawTorqueWeight() * planner->getMaxMinYaw();
  }

  double baselinkRotConstraint(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto baselink_rot = planner->getRobotModelForPlan()->getCogDesireOrientation<Eigen::Matrix3d>();

    double ez_x = baselink_rot(0,2);
    double ez_y = baselink_rot(1,2);
    double ez_z = baselink_rot(2,2);
    double angle = atan2(sqrt(ez_x* ez_x + ez_y * ez_y), fabs(ez_z));

    return angle - planner->getBaselinkRotThresh();
  }


  double fcTMinConstraint(const std::vector<double> &x, std::vector<double> &grad, void *planner_ptr)
  {
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    return planner->getFCTMinThresh() - planner->getRobotModelForPlan()->getFeasibleControlTMin();
  }

  void thrustLimitConstraint(unsigned m, double* result, unsigned n, const double* x, double *gradient, void *planner_ptr)
  {
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();
    robot_model->calc3DoFThrust(planner->ff_f_xy_[0], planner->ff_f_xy_[1]);
    auto thrust = robot_model->get3DoFThrust();
    Eigen::VectorXd ret_e(8);
    ret_e << thrust-Eigen::VectorXd::Constant(4, robot_model->getThrustUpperLimit()), Eigen::VectorXd::Constant(4, robot_model->getThrustLowerLimit())-thrust;

    for (int i=0; i<m; i++) {
      result[i] = ret_e(i);
    }
  }
  
  void kinematicsConstraint(unsigned m, double* result, unsigned n, const double* x, double *gradient, void *planner_ptr)
  {
    HydrusXiUnderActuatedNavigator *planner = reinterpret_cast<HydrusXiUnderActuatedNavigator*>(planner_ptr);
    auto robot_model = planner->getRobotModelForPlan();
    KDL::JntArray joints = planner->getJointPositionsForPlan();
    //for(int i = 0; i < x.size(); i++)
    //  joint_positions(planner->getControlIndices().at(i)) = x.at(i);
    joints.data[0] = x[0];
    joints.data[3] = x[1];
    joints.data[6] = x[2];
    joints.data[9] = x[3];
    robot_model->updateRobotModel(joints);
    robot_model->calcWrenchMatrixOnRoot();
    auto Q = robot_model->calcWrenchMatrixOnCoG();
    planner->ffWrenchUpdate(planner->ff_f_xy_world_[0], planner->ff_f_xy_world_[1], planner->robot_model_real_->ff_t_z_);
    Eigen::VectorXd thrusts(4), wrench_des(6);
    thrusts << x[4], x[5], x[6], x[7];
    wrench_des << planner->ff_f_xy_[0], planner->ff_f_xy_[1], robot_model->getGravity()[2], 0, 0, 0;
    auto res = Q*thrusts-robot_model->getMass()*wrench_des;
    for (int i=0; i<6; i++) {
      result[i] = res[i];
    }
  }

};

HydrusXiUnderActuatedNavigator::HydrusXiUnderActuatedNavigator():
    opt_gimbal_angles_(0),
    prev_opt_gimbal_angles_(0),
    max_min_yaw_(0),
    control_gimbal_names_(0),
    control_gimbal_indices_(0)
{
}

HydrusXiUnderActuatedNavigator::~HydrusXiUnderActuatedNavigator()
{
  plan_thread_.join();
}

void HydrusXiUnderActuatedNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  BaseNavigator::initialize(nh, nhp, robot_model, estimator);

  robot_model_for_plan_ = boost::make_shared<HydrusTiltedRobotModel>(); // for planning, not the real robot model
  robot_model_real_ = boost::dynamic_pointer_cast<HydrusTiltedRobotModel>(robot_model);
  estimator_ = estimator;

  rosParamInit();

  gimbal_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  joints_torque_pub_ = nh_.advertise<geometry_msgs::Vector3>("joints_torque", 1);

  ff_wrench_sub_ = nh_.subscribe("ff_wrench", 10, &HydrusXiUnderActuatedNavigator::ffWrenchCallback, this);
  ff_wrench_noreset_sub_ = nh_.subscribe("ff_wrench_noreset", 10, &HydrusXiUnderActuatedNavigator::ffWrenchNoResetCallback, this);
  joint_fb_sub_ = nh_.subscribe("joint_states", 10, &HydrusXiUnderActuatedNavigator::jointStatesCallback, this);
  ff_f_xy_[0] = 0.01;
  ff_f_xy_[1] = 0.0;

  if(nh.hasParam("control_gimbal_names"))
    {
      nh.getParam("control_gimbal_names", control_gimbal_names_);
    }
  else
    {
      ROS_INFO("load control gimbal list from robot model");
      for(const auto& name: robot_model->getJointNames())
        {
          if(name.find("gimbal") != std::string::npos)
            {
              control_gimbal_names_.push_back(name);
              ROS_INFO_STREAM("add " << name);
            }
        }
    }

  /* nonlinear optimization for vectoring angles planner */
  vectoring_nl_solver_ = boost::make_shared<nlopt::opt>(nlopt::LN_COBYLA, control_gimbal_names_.size());
  vectoring_nl_solver_h_ = boost::make_shared<nlopt::opt>(nlopt::LN_COBYLA, 2*control_gimbal_names_.size());
  if(maximize_yaw_) {
    vectoring_nl_solver_->set_max_objective(maximizeMinYawTorque, this);
    vectoring_nl_solver_->add_inequality_constraint(fcTMinConstraint, this, 1e-8);
  } else {
    vectoring_nl_solver_->set_max_objective(maximizeFCTMin, this);
  }
  vectoring_nl_solver_->add_inequality_constraint(baselinkRotConstraint, this, 1e-8);
  
  //vectoring_nl_solver_h_->set_max_objective(maximizeHorizontalForceSquare, this);
  //vectoring_nl_solver_h_->add_inequality_mconstraint(thrustLimitConstraint, this, std::vector<double>(8, 1e-8));
  //vectoring_nl_solver_h_->set_max_objective(maximizeBalanceWide, this);
  vectoring_nl_solver_h_->set_max_objective(maximizeFCTMinWide, this);
  vectoring_nl_solver_h_->add_equality_mconstraint(kinematicsConstraint, this, {0.05, 0.05, 0.1, 0.001, 0.001, 0.001}/*std::vector<double>(6, 1e-2)*/);

  vectoring_nl_solver_->set_xtol_rel(1e-4); //1e-4
  vectoring_nl_solver_h_->set_xtol_rel(1e-4); //1e-4
  vectoring_nl_solver_->set_maxeval(1000); // 1000 times
  vectoring_nl_solver_h_->set_maxeval(1000); // 1000 times

  opt_gimbal_angles_tmp_ = {0, 0, 0, 0};
  opt_static_thrusts_ = {0, 0, 0, 0};
  opt_x_ = {M_PI, M_PI, M_PI, M_PI, 0, 0, 0, 0};

  /* linear optimization for yaw range */
  double rotor_num = robot_model->getRotorNum();

  // settings the LP solver
  yaw_range_lp_solver_.settings()->setVerbosity(false);
  yaw_range_lp_solver_.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  yaw_range_lp_solver_.data()->setNumberOfVariables(rotor_num);
  yaw_range_lp_solver_.data()->setNumberOfConstraints(rotor_num);

  // allocate LP problem matrices and vectors
  Eigen::SparseMatrix<double> hessian;
  hessian.resize(rotor_num, rotor_num);

  Eigen::VectorXd gradient = Eigen::VectorXd::Ones(rotor_num);
  Eigen::SparseMatrix<double> linear_cons;
  linear_cons.resize(rotor_num, rotor_num);
  for(int i = 0; i < linear_cons.cols(); i++) linear_cons.insert(i,i) = 1;

  Eigen::VectorXd lower_bound = Eigen::VectorXd::Ones(rotor_num) * robot_model->getThrustLowerLimit();
  Eigen::VectorXd upper_bound = Eigen::VectorXd::Ones(rotor_num) * robot_model->getThrustUpperLimit();

  yaw_range_lp_solver_.data()->setHessianMatrix(hessian);
  yaw_range_lp_solver_.data()->setGradient(gradient);
  yaw_range_lp_solver_.data()->setLinearConstraintsMatrix(linear_cons);
  yaw_range_lp_solver_.data()->setLowerBound(lower_bound);
  yaw_range_lp_solver_.data()->setUpperBound(upper_bound);

  // instantiate the yaw_range_lp_solver
  if(!yaw_range_lp_solver_.initSolver())
    throw std::runtime_error("can not init LP solver based on osqp");

  plan_thread_ = std::thread(boost::bind(&HydrusXiUnderActuatedNavigator::threadFunc, this));
}

void HydrusXiUnderActuatedNavigator::threadFunc()
{
  double plan_freq;
  ros::NodeHandle navi_nh(nh_, "navigation");
  getParam<double>(navi_nh, "plan_freq", plan_freq, 20.0);
  ros::Rate loop_rate(plan_freq);

  // sleep for initialization
  double plan_init_sleep;
  getParam<double>(navi_nh, "plan_init_sleep", plan_init_sleep, 2.0);
  ros::Duration(plan_init_sleep).sleep();

  while(ros::ok())
    {
      plan();
      loop_rate.sleep();
    }
}

void HydrusXiUnderActuatedNavigator::sanitizeJoints(std::vector<double>& joints)
{
  for (int i=0; i<joints.size(); i++) {
    if (joints[i] > M_PI) {
      joints[i] -= 2*M_PI;
    } else if (joints[i] < -M_PI) {
      joints[i] += 2*M_PI;
    }
  }
}

bool HydrusXiUnderActuatedNavigator::plan()
{
  joint_positions_for_plan_ = robot_model_->getJointPositions(); // real

  boost::shared_ptr<nlopt::opt> nl_solver_now;
  if (horizontal_mode_) {
    nl_solver_now = vectoring_nl_solver_h_;
    robot_model_for_plan_->horizontal_mode_ = true;
    setTargetYaw(last_target_yaw_ + joint_positions_for_plan_.data[2] - last_normal_joint1_angle_);
  } else {
    nl_solver_now = vectoring_nl_solver_;
    robot_model_for_plan_->horizontal_mode_ = false;
    last_normal_joint1_angle_ = joint_positions_for_plan_.data[2];
    last_target_yaw_ = getTargetRPY().getZ();
  }
  //ROS_INFO_STREAM_THROTTLE(0.1, "target link1 yaw: " << getTargetRPY().getZ()-joint_positions_for_plan_.data[2]);

  if(joint_positions_for_plan_.rows() == 0) return false;

  // initialize from the normal shape
  bool singluar_form = true;
  if(control_gimbal_indices_.size() == 0)
    {
      const auto& joint_names = robot_model_->getJointNames();
      const auto& joint_indices = robot_model_->getJointIndices();

      for(int i = 0; i < joint_names.size(); i++)
        {
          if(joint_names.at(i).find("joint") != std::string::npos)
            {
              if(fabs(joint_positions_for_plan_(joint_indices.at(i))) > 0.2) singluar_form = false;
            }
        }

      for(const auto& name: control_gimbal_names_)
        control_gimbal_indices_.push_back(robot_model_->getJointIndexMap().at(name));
    }

  /* find the optimal gimbal vectoring angles from nlopt */
  std::vector<double> lb(control_gimbal_indices_.size(), - M_PI);
  std::vector<double> ub(control_gimbal_indices_.size(), M_PI);
  std::vector<double> lbh(2*control_gimbal_indices_.size(), - M_PI);
  std::vector<double> ubh(2*control_gimbal_indices_.size(), M_PI);

  for (int i=0; i<opt_gimbal_angles_.size(); i++) {
    lbh.at(4+i) = robot_model_real_->getThrustLowerLimit();
    ubh.at(4+i) = robot_model_real_->getThrustUpperLimit();
  }

  /* update the range by using the last optimization result with the assumption that the motion is continuous */
  if(opt_gimbal_angles_.size() != 0)
    {
      double delta_angle = gimbal_delta_angle_;

      if(horizontal_mode_ and vectoring_reset_flag_)
        {
          delta_angle = M_PI; // reset
          vectoring_reset_flag_ = false;
          robot_model_real_->transition_flag_ = true;
          ROS_INFO("Vectoring angle optimization reset, transitioning");
        }

      if((not horizontal_mode_) and (!robot_model_for_plan_->stabilityCheck(false)))
        {
          delta_angle = M_PI; // reset
        }

      for(int i = 0; i < opt_gimbal_angles_.size(); i++)
        {
          lb.at(i) = opt_gimbal_angles_.at(i) - delta_angle;
          ub.at(i) = opt_gimbal_angles_.at(i) + delta_angle;
          lbh.at(i) = opt_gimbal_angles_.at(i) - delta_angle;
          ubh.at(i) = opt_gimbal_angles_.at(i) + delta_angle;
        }
    }
  else
    {
      /* heuristic assignment for the init state of vectoring angles */
      ROS_INFO("navigation opt gimbal initialize");
      opt_gimbal_angles_.resize(control_gimbal_indices_.size(), 0); // all angles  are zero

      // contraint bound is relaxed to perform global search

      // if control all gimbals:
      if(control_gimbal_indices_.size() == robot_model_->getRotorNum())
        {
          for(int i = 0; i < control_gimbal_indices_.size(); i++)
            {
              if(i%2 == 0) opt_gimbal_angles_.at(i) = M_PI;
            }

          // hard-coding: singular line form
          if(singluar_form && robot_model_->getRotorNum() == 4)
            {
              opt_gimbal_angles_.at(0) = M_PI / 2;
              opt_gimbal_angles_.at(1) = - M_PI / 2;
              opt_gimbal_angles_.at(2) = - M_PI / 2;
              opt_gimbal_angles_.at(3) = M_PI / 2;
            }
        }
    }

  if (not horizontal_mode_) {
    nl_solver_now->set_lower_bounds(lb);
    nl_solver_now->set_upper_bounds(ub);
  } else {
    nl_solver_now->set_lower_bounds(lbh);
    nl_solver_now->set_upper_bounds(ubh);
  }

  double start_time = ros::Time::now().toSec();
  double max_f = 0;
  try
    {
      nlopt::result result;
      if (not horizontal_mode_) {
        result = nl_solver_now->optimize(opt_gimbal_angles_, max_f);
        opt_x_ = {opt_gimbal_angles_.at(0), opt_gimbal_angles_.at(1), opt_gimbal_angles_.at(2), opt_gimbal_angles_.at(3), 9.0, 10.0, 10.0, 9.0};
      } else {
        opt_x_ = {opt_gimbal_angles_.at(0), opt_gimbal_angles_.at(1), opt_gimbal_angles_.at(2), opt_gimbal_angles_.at(3), opt_x_.at(4), opt_x_.at(5), opt_x_.at(6), opt_x_.at(7)};
        result = nl_solver_now->optimize(opt_x_, max_f);
        opt_gimbal_angles_ = {opt_x_.at(0), opt_x_.at(1), opt_x_.at(2), opt_x_.at(3)};
      }
      opt_static_thrusts_ = {opt_x_.at(4), opt_x_.at(5), opt_x_.at(6), opt_x_.at(7)};
      ROS_INFO_STREAM_THROTTLE(1, "res: " << int(result) << " maxf: " << max_f << " opt: " << opt_gimbal_angles_[0] << " " << opt_gimbal_angles_[1] << " " << opt_gimbal_angles_[2] << " " << opt_gimbal_angles_[3] << " " << opt_static_thrusts_[0] << " " << opt_static_thrusts_[1] << " " << opt_static_thrusts_[2] << " " << opt_static_thrusts_[3]);

      // Transition
      double thres = 0.1;
      if (horizontal_mode_ and robot_model_real_->transition_flag_) {
        bool transitioning = false;
        for (int i=0; i<4; i++) {
          auto gimbal_diff = opt_gimbal_angles_[i] - joint_pos_fb_[i];
          while (gimbal_diff > M_PI) {
            gimbal_diff = gimbal_diff - 2*M_PI;
          }
          while (gimbal_diff < -M_PI) {
            gimbal_diff = gimbal_diff + 2*M_PI;
          }
          if (std::abs(gimbal_diff) > thres) {
            if (gimbal_diff > 0) {
              lbh.at(i) = joint_pos_fb_.at(i) + 0.5*gimbal_delta_angle_;
              ubh.at(i) = joint_pos_fb_.at(i) + 1.5*gimbal_delta_angle_;
              opt_x_[i] = joint_pos_fb_.at(i) + gimbal_delta_angle_;
            } else {
              lbh.at(i) = joint_pos_fb_.at(i) - 1.5*gimbal_delta_angle_;
              ubh.at(i) = joint_pos_fb_.at(i) - 0.5*gimbal_delta_angle_;
              opt_x_[i] = joint_pos_fb_.at(i) - gimbal_delta_angle_;
            }
            transitioning = true;
          }
        }
        if (transitioning) {
            nl_solver_now->set_lower_bounds(lbh);
            nl_solver_now->set_upper_bounds(ubh);
            result = nl_solver_now->optimize(opt_x_, max_f);
            ROS_INFO_STREAM_THROTTLE(1, "tmp res: " << int(result) << " maxf: " << max_f << " opt: " << opt_x_[0] << " " << opt_x_[1] << " " << opt_x_[2] << " " << opt_x_[3] << " " << opt_x_[4] << " " << opt_x_[5] << " " << opt_x_[6] << " " << opt_x_[7]);
            opt_gimbal_angles_tmp_ = {opt_x_.at(0), opt_x_.at(1), opt_x_.at(2), opt_x_.at(3)};
            opt_static_thrusts_ = {opt_x_.at(4), opt_x_.at(5), opt_x_.at(6), opt_x_.at(7)};
        } else {
          // Converged
          ROS_INFO("Converged, transition flag reset");
          robot_model_real_->transition_flag_ = false;
        }
      }
      robot_model_real_->set3DoFThrust(opt_static_thrusts_);

      // Joint Torque
      Eigen::VectorXd hori_thrust(4);
      for (int i=0; i<4; i++) {
        hori_thrust(i) = opt_static_thrusts_[i] + robot_model_real_->thrusts_yaw_term_(i);
      }
      robot_model_real_->calcCoGMomentumJacobian();
      robot_model_real_->calcJointTorque(hori_thrust);
      auto jt = robot_model_real_->getJointTorque();
      geometry_msgs::Vector3 joints_t_to_send;
      joints_t_to_send.x = jt[1];
      joints_t_to_send.y = jt[3];
      joints_t_to_send.z = jt[5];
      joints_torque_pub_.publish(joints_t_to_send);
      /* debug print to make sure that optimization is to blame
      robot_model_real_->calcWrenchMatrixOnRoot();
      auto Q = robot_model_real_->calcWrenchMatrixOnCoG();
      Eigen::Vector4d thr(opt_static_thrusts_.at(0), opt_static_thrusts_.at(1), opt_static_thrusts_.at(2), opt_static_thrusts_.at(3));
      ROS_INFO_STREAM("opt thrust wrench recalc: " << Q*thr);
      */
      if (result != 4 and result != 5) {
        vectoring_reset_flag_ = true;
      }
      
      double roll,pitch,yaw;
      robot_model_for_plan_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

      if(prev_opt_gimbal_angles_.size() == 0) prev_opt_gimbal_angles_ = opt_gimbal_angles_;

      if(plan_verbose_)
        {
          std::cout << "nlopt: " << std::setprecision(7)
                    << ros::Time::now().toSec() - start_time  <<  "[sec], cnt: " << cnt;
          std::cout << ", found optimal gimbal angles: ";
          for(auto it: opt_gimbal_angles_) std::cout << std::setprecision(5) << it << " ";
          std::cout << ", max min yaw: " << max_min_yaw_;
          std::cout << ", fc t min: " << robot_model_for_plan_->getFeasibleControlTMin();
          std::cout << ", attitude: [" << roll << ", " << pitch;
          std::cout << "], force: [" << robot_model_for_plan_->getStaticThrust().transpose();
          std::cout << "]" << std::endl;
        }


      cnt = 0;
      invalid_cnt = 0;
    }
  catch(std::exception &e)
    {
      std::cout << "nlopt failed: " << e.what() << std::endl;
    }

  /* publish the gimbal angles if necessary */
  sensor_msgs::JointState gimbal_msg;
  gimbal_msg.header.stamp = ros::Time::now();

  for(int i = 0; i < control_gimbal_indices_.size(); i++)
    {
      gimbal_msg.name.push_back(control_gimbal_names_.at(i));
      if (robot_model_real_->transition_flag_) {
        gimbal_msg.position.push_back(opt_gimbal_angles_tmp_.at(i));
      } else {
        gimbal_msg.position.push_back(opt_gimbal_angles_.at(i));
      }
    }
  gimbal_ctrl_pub_.publish(gimbal_msg);

  prev_opt_gimbal_angles_ = opt_gimbal_angles_;

  return true;
}

void HydrusXiUnderActuatedNavigator::ffWrenchCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
  ffWrenchUpdate(msg->x, msg->y, msg->z);
  //ROS_INFO_STREAM("ff_converted: " << ff_f_xy_[0] << " " << ff_f_xy_[1] << " " << robot_model_real_->getCog<Eigen::Affine3d>().rotation().inverse().eulerAngles(0,1,2));
  
  vectoring_reset_flag_ = true;
}

void HydrusXiUnderActuatedNavigator::ffWrenchNoResetCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
  ffWrenchUpdate(msg->x, msg->y, msg->z);
}

void HydrusXiUnderActuatedNavigator::ffWrenchUpdate(double x, double y, double z)
{
  ff_f_xy_world_[0] = x;
  ff_f_xy_world_[1] = y;
  ff_f_xy_world_[2] = aerial_robot_estimation::G;
  auto world2base = estimator_->getOrientation(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH);
  auto base = world2base.inverse()*tf::Vector3(x,y,aerial_robot_estimation::G);
  ff_f_xy_[0] = base.x();
  ff_f_xy_[1] = base.y();
  ff_f_xy_[2] = 0.0;
  ff_f_xy_root_ = robot_model_real_->getCog<Eigen::Affine3d>().rotation() * ff_f_xy_;
  robot_model_real_->ff_f_x_ = ff_f_xy_root_[0];
  robot_model_real_->ff_f_y_ = ff_f_xy_root_[1];
  robot_model_real_->ff_t_z_ = z;
}

void HydrusXiUnderActuatedNavigator::jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_pos_fb_ = msg->position;
}

void HydrusXiUnderActuatedNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();
  ros::NodeHandle navi_nh(nh_, "navigation");
  getParam<bool>(navi_nh, "plan_verbose", plan_verbose_, false);
  getParam<bool>(navi_nh, "maximize_yaw", maximize_yaw_, false);
  getParam<double>(navi_nh, "gimbal_delta_angle", gimbal_delta_angle_, 0.2);
  getParam<double>(navi_nh, "force_norm_weight", force_norm_weight_, 2.0);
  getParam<double>(navi_nh, "force_variant_weight", force_variant_weight_, 0.01);
  getParam<double>(navi_nh, "yaw_torque_weight", yaw_torque_weight_, 1.0);
  getParam<double>(navi_nh, "fc_t_min_weight", fc_t_min_weight_, 1.0);
  getParam<double>(navi_nh, "uncontrolled_torque_weight", uncontrolled_torque_weight_, 0.1);
  getParam<double>(navi_nh, "joint_torque_weight", joint_torque_weight_, 0.0);
  getParam<double>(navi_nh, "baselink_rot_thresh", baselink_rot_thresh_, 0.02);
  getParam<double>(navi_nh, "fc_t_min_thresh", fc_t_min_thresh_, 2.0);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::HydrusXiUnderActuatedNavigator, aerial_robot_navigation::BaseNavigator);
