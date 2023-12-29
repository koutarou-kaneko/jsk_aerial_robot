// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <hydrus/hydrus_tilted_robot_model.h>
#include <aerial_robot_control/control/under_actuated_tilted_lqi_controller.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
#include <aerial_robot_control/control/utils/pid.h>
#include <thread>

namespace aerial_robot_control
{
  class HydrusTiltedLQIController: public UnderActuatedTiltedLQIController
  {
  public:
    HydrusTiltedLQIController();
    ~HydrusTiltedLQIController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

  protected:
    bool checkRobotModel() override;
  private:
    boost::shared_ptr<HydrusTiltedRobotModel> hydrus_robot_model_;
    std::vector<PID> external_wrench_pid_controllers_;
    std::vector<boost::shared_ptr<PidControlDynamicConfig> > external_wrench_pid_reconf_servers_;
    ros::Publisher static_thrust_available_pub_;
    ros::Publisher fc_t_min_pub_;
    ros::Publisher fc_t_min_thre_pub_;
    ros::Publisher fc_rp_min_pub_;
    ros::Publisher feedforward_acc_cog_pub_;
    ros::Publisher feedforward_ang_acc_cog_pub_;
    ros::Publisher des_wrench_cog_pub_;
    ros::Publisher attaching_flag_pub_;
    ros::Publisher filtered_est_external_wrench_pub_;
    ros::Subscriber desire_wrench_sub_;
    ros::Subscriber desire_pos_sub_;
    ros::Subscriber acc_root_sub_;
    ros::Subscriber hand_force_switch_sub_;
    Eigen::VectorXd estimated_external_wrench_in_cog_;
    Eigen::VectorXd desire_wrench_;
    Eigen::VectorXd desire_wrench_from_pos_;
    Eigen::VectorXd target_wrench_cog_;
    Eigen::VectorXd p_wrench_stamp_;
    Eigen::VectorXd feedforward_sum_;
    Eigen::VectorXd desire_pos_;
    bool send_feedforward_switch_flag_;
    bool attaching_flag_, const_err_i_flag_, first_flag_;
    double err_i_x_, err_i_y_, err_i_z_, err_p_y_;
    double wrench_diff_gain_;
    double acc_shock_thres_;
    IirFilter lpf_est_external_wrench_;
    bool update() override;
    void controlCore() override;
    void sendCmd() override;
    void cfgWrenchPidCallback(aerial_robot_control::PIDConfig &config, uint32_t level, std::vector<int> controller_indices);
    void DesireWrenchCallback(geometry_msgs::WrenchStamped msg);
    void DesirePosCallback(aerial_robot_msgs::FlightNav msg);
    void accRootCallback(const spinal::Imu msg);
    void HandForceSwitchCallBack(std_msgs::Int8 msg);
  };

};
