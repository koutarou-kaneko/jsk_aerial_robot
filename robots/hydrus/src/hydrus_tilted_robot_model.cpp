#include <hydrus/hydrus_tilted_robot_model.h>

HydrusTiltedRobotModel::HydrusTiltedRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon):
  HydrusRobotModel(init_with_rosparam, verbose, fc_t_min_thre, 0, epsilon, 4)
{
}

void HydrusTiltedRobotModel::calcStaticThrust()
{
  calcWrenchMatrixOnRoot(); // update Q matrix

  /* calculate the static thrust on CoG frame */
  /* note: can not calculate in root frame, since the projected f_x, f_y is different in CoG and root */
  Eigen::MatrixXd wrench_mat_on_cog = calcWrenchMatrixOnCoG();

  Eigen::VectorXd static_thrust = aerial_robot_model::pseudoinverse(wrench_mat_on_cog.middleRows(2, 4)) * getGravity().segment(2,4) * getMass();
  setStaticThrust(static_thrust);
}

void HydrusTiltedRobotModel::calcJointTorque(Eigen::VectorXd horizontal_mode_thrust)
{
  const auto& sigma = getRotorDirection();
  const auto& joint_positions = getJointPositions();
  const auto& inertia_map = getInertiaMap();
  const int joint_num = getJointNum();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();

  //Always
  calcBasicKinematicsJacobian(); // update thrust_coord_jacobians_

  joint_torque_ = Eigen::VectorXd::Zero(joint_num);

  // update coord jacobians for cog point and convert to joint torque
  int seg_index = 0;
  for(const auto& inertia : inertia_map)
    {
      cog_coord_jacobians_.at(seg_index) = RobotModel::getJacobian(joint_positions, inertia.first, inertia.second.getCOG());
      joint_torque_ -= cog_coord_jacobians_.at(seg_index).rightCols(joint_num).transpose() * inertia.second.getMass() * (-gravity_);
      seg_index ++;
    }

  // thrust
  for (int i = 0; i < rotor_num; ++i) {
    Eigen::VectorXd wrench;
    if (not flight_mode_ == FLIGHT_MODE_FULL) {
      wrench = thrust_wrench_units_.at(i) * static_thrust_(i);
    } else {
      wrench = thrust_wrench_units_.at(i) * horizontal_mode_thrust(i);
    }
    joint_torque_ -= thrust_coord_jacobians_.at(i).rightCols(joint_num).transpose() * wrench;
  }
}

void HydrusTiltedRobotModel::calc3DoFThrust(double ff_f_x, double ff_f_y)
{
  ROS_WARN_THROTTLE(1, "Deprecated: HydrusTiltedRobotModel::calc3DoFThrust()");
  calcWrenchMatrixOnRoot(); // update Q matrix
  /* calculate the static thrust on CoG frame */
  /* note: can not calculate in root frame, since the projected f_x, f_y is different in CoG and root */
  Eigen::MatrixXd wrench_mat_on_cog = calcWrenchMatrixOnCoG();
  //ROS_INFO_STREAM_THROTTLE(1, "Q:\n" << wrench_mat_on_cog);
  
  Eigen::MatrixXd Q_4(4,wrench_mat_on_cog.cols());
  Q_4 << wrench_mat_on_cog.topRows(3), wrench_mat_on_cog.bottomRows(1);
  //ROS_INFO_STREAM_THROTTLE(1, "Q_4^T:\n" << aerial_robot_model::pseudoinverse(Q_4));
  Eigen::VectorXd grav(4), ff_wrench(4);
  ff_wrench << ff_f_x, ff_f_y, 0, 0;
  grav << getGravity().head(3), getGravity().tail(1);
  grav = grav + ff_wrench;

  three_dof_thrust_ = aerial_robot_model::pseudoinverse(Q_4) * grav * getMass();
  thrust_wrench_ = wrench_mat_on_cog * three_dof_thrust_;
  ROS_INFO_STREAM_THROTTLE(1, "wrench out [N]: " << thrust_wrench_.transpose());
}

Eigen::VectorXd HydrusTiltedRobotModel::get3DoFThrust()
{
  return three_dof_thrust_;
}

void HydrusTiltedRobotModel::set3DoFThrust(std::vector<double> thrust_vector)
{
  three_dof_thrust_ << thrust_vector.at(0), thrust_vector.at(1), thrust_vector.at(2), thrust_vector.at(3);
}

void HydrusTiltedRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  if(getStaticThrust().minCoeff() < 0)
    {
      setCogDesireOrientation(0, 0, 0);
      return; // invalid robot state
    }

  /* special process to find the hovering axis for tilt model */
  Eigen::MatrixXd cog_rot_inv = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>().Inverse());
  Eigen::MatrixXd wrench_mat_on_cog = calcWrenchMatrixOnCoG();
  Eigen::VectorXd f = cog_rot_inv * wrench_mat_on_cog.topRows(3) * getStaticThrust();

  double f_norm_roll, f_norm_pitch;
  if (not flight_mode_ == FLIGHT_MODE_FULL) {
    // Underactuated control mode
    f_norm_roll = atan2(f(1), f(2));
    f_norm_pitch = atan2(-f(0), sqrt(f(1)*f(1) + f(2)*f(2)));
  } else {
    f_norm_roll = 0;
    f_norm_pitch = 0;
  }

  /* set the hoverable frame as CoG and reupdate model */
  setCogDesireOrientation(f_norm_roll, f_norm_pitch, 0);
  HydrusRobotModel::updateRobotModelImpl(joint_positions);

  if(getVerbose())
  {
    ROS_INFO_STREAM("f_norm_pitch: " << f_norm_pitch << "; f_norm_roll: " << f_norm_roll);
    ROS_INFO_STREAM("rescaled static thrust: " << getStaticThrust().transpose());
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(HydrusTiltedRobotModel, aerial_robot_model::RobotModel);
