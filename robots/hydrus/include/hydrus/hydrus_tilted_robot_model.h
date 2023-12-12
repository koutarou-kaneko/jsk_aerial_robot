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

  KDL::Frame getRootEnd() {return root_end_;}
  KDL::Frame getLinkEnd() {return link_end_;}
  void setTargetForceInRootEnd(Eigen::Vector3d target_force) {target_force_in_root_end_ = target_force;}
  void setTargetForceInLinkEnd(Eigen::Vector3d target_force) {target_force_in_link_end_ = target_force;}
  Eigen::Vector3d getCompensateTorqueForRootEndInCog() {return compensate_torque_for_root_end_in_cog_;}
  Eigen::Vector3d getCompensateTorqueForLinkEndInCog() {return compensate_torque_for_link_end_in_cog_;}

private:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

  KDL::Frame root_end_;
  KDL::Frame link_end_;
  std::string root_end_name_;
  std::string link_end_name_;
  Eigen::Vector3d target_force_in_root_end_;
  Eigen::Vector3d target_force_in_link_end_;
  Eigen::Vector3d compensate_torque_for_root_end_in_cog_;
  Eigen::Vector3d compensate_torque_for_link_end_in_cog_;

};
