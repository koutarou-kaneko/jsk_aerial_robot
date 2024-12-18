#include <twin_hammer/model/twin_hammer_model.h>

TwinHammerModel::TwinHammerModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
  // tfBuffer_(),
  // tfListener_(tfBuffer_)
{
}

void TwinHammerModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  // unsigned int size = joint_positions.rows();
  // std::cout << "joint positions [" ;
  // for (unsigned int i=0; i<size; i++){
  //   std::cout << joint_positions(i);
  //   if(i<size-1){
  //     std::cout << ",";
  //   }
  // }
  // std::cout << "]" << std::endl;
  RobotModel::updateRobotModelImpl(joint_positions);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TwinHammerModel, aerial_robot_model::RobotModel);