#include <hydrus/hydrus_tilted_robot_model.h>

HydrusTiltedRobotModel::HydrusTiltedRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon):
  HydrusRobotModel(init_with_rosparam, verbose, fc_t_min_thre, 0, epsilon, 4)
{
  root_end_name_ = "link1_end";
  link_end_name_ = "link4_end";
}


void HydrusTiltedRobotModel::calcStaticThrust()
{
  calcWrenchMatrixOnRoot(); // update Q matrix

  /* calculate the static thrust on CoG frame */
  /* note: can not calculate in root frame, sine the projected f_x, f_y is different in CoG and root */
  Eigen::MatrixXd wrench_mat_on_cog = calcWrenchMatrixOnCoG();

  Eigen::VectorXd static_thrust = aerial_robot_model::pseudoinverse(wrench_mat_on_cog.middleRows(2, 4)) * getGravity().segment(2,4) * getMass();
  setStaticThrust(static_thrust);
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

  double f_norm_roll = atan2(f(1), f(2));
  double f_norm_pitch = atan2(-f(0), sqrt(f(1)*f(1) + f(2)*f(2)));

  /* set the hoverable frame as CoG and reupdate model */
  setCogDesireOrientation(f_norm_roll, f_norm_pitch, 0);
  HydrusRobotModel::updateRobotModelImpl(joint_positions);

  if(getVerbose())
  {
    ROS_INFO_STREAM("f_norm_pitch: " << f_norm_pitch << "; f_norm_roll: " << f_norm_roll);
    ROS_INFO_STREAM("rescaled static thrust: " << getStaticThrust().transpose());
  }

  /* calculate torque in cog frame to compensate force on end-effectors */
  const auto seg_tf_map = getSegmentsTf();
  KDL::Frame cog = getCog<KDL::Frame>();

  if(seg_tf_map.count(root_end_name_))
    {
  ROS_WARN_STREAM_ONCE("[model] there is a end-effector named " << root_end_name_);
  KDL::Frame root_end = seg_tf_map.at(root_end_name_);
  Eigen::Vector3d compensate_force_for_root_end_in_cog = aerial_robot_model::kdlToEigen(cog.M.Inverse() * root_end.M) * target_force_in_root_end_;
  KDL::Frame cog_to_root_end = cog.Inverse() * root_end;
  Eigen::MatrixXd cog_to_root_end_skew = aerial_robot_model::skew(Eigen::Vector3d(cog_to_root_end.p.x(), cog_to_root_end.p.y(), cog_to_root_end.p.z()));
  compensate_torque_for_root_end_in_cog_ = cog_to_root_end_skew * compensate_force_for_root_end_in_cog;
    }
  else
    {
  ROS_ERROR_STREAM_ONCE("[model] there is no end-effector named " << root_end_name_);
    }

  if(seg_tf_map.count(link_end_name_))
    {
  ROS_WARN_STREAM_ONCE("[model] there is a end-effector named " << link_end_name_);
  KDL::Frame link_end = seg_tf_map.at(link_end_name_);
  Eigen::Vector3d compensate_force_for_link_end_in_cog = aerial_robot_model::kdlToEigen(cog.M.Inverse() * link_end.M) * target_force_in_link_end_;
  KDL::Frame cog_to_link_end = cog.Inverse() * link_end;
  Eigen::MatrixXd cog_to_link_end_skew = aerial_robot_model::skew(Eigen::Vector3d(cog_to_link_end.p.x(), cog_to_link_end.p.y(), cog_to_link_end.p.z()));
  compensate_torque_for_link_end_in_cog_ = cog_to_link_end_skew * compensate_force_for_link_end_in_cog;
    }
  else
    {
  ROS_ERROR_STREAM_ONCE("[model] there is no end-effector named " << link_end_name_);
    }

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(HydrusTiltedRobotModel, aerial_robot_model::RobotModel);
