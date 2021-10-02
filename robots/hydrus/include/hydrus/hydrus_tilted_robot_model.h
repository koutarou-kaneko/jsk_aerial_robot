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

#include <hydrus/hydrus_robot_model.h>

class HydrusTiltedRobotModel : public HydrusRobotModel {
public:
  HydrusTiltedRobotModel(bool init_with_rosparam = true,
                         bool verbose = false,
                         double fc_t_min_thre = 0,
                         double epsilon = 10);
  virtual ~HydrusTiltedRobotModel() = default;

  virtual void calcStaticThrust() override;
  void calcJointTorque(Eigen::VectorXd horizontal_mode_thrust);
  void calc3DoFThrust(double, double);
  Eigen::VectorXd get3DoFThrust();
  void set3DoFThrust(std::vector<double> thrust_vector);
  Eigen::VectorXd thrust_wrench_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd thrusts_yaw_term_ = Eigen::VectorXd::Zero(4);
  static constexpr int FLIGHT_MODE_HOVERING = 0;
  static constexpr int FLIGHT_MODE_FULL = 1;
  static constexpr int FLIGHT_MODE_TRANSITION_FOR = 2;
  static constexpr int FLIGHT_MODE_TRANSITION_BACK = 3;
  int flight_mode_ = FLIGHT_MODE_HOVERING;
  bool vectoring_reset_flag_ = false;
  double ff_f_x_ = 0, ff_f_y_ = 0, ff_t_z_ = 0, yaw_comp_ = 0;
private:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
  Eigen::VectorXd three_dof_thrust_ = Eigen::VectorXd::Zero(4);
};
