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

#include <hydrus/hydrus_lqi_controller.h>
#include <hydrus/hydrus_tilted_robot_model.h>

#include <spinal/DesireCoord.h>

namespace aerial_robot_control
{
  class HydrusTiltedLQIController: public HydrusLQIController
  {
  public:
    HydrusTiltedLQIController() {}
    virtual ~HydrusTiltedLQIController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

  protected:

    ros::Publisher desired_baselink_rot_pub_, ff_wrench_pub_, ff_wrench_noreset_pub_, nav_msg_pub_;
    ros::ServiceServer start_wall_touching_srv_;
    ros::ServiceServer set_horizontal_force_mode_srv_;
    ros::ServiceServer reset_horizontal_force_mode_srv_;
    ros::Subscriber acc_root_sub_;

    boost::shared_ptr<HydrusTiltedRobotModel> tilted_model_;
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator_;
    double trans_constraint_weight_;
    double att_control_weight_;
    bool horizontal_force_mode_ = false, wall_touching_ = false;
    double acc_root_shock_thres_ = 20.0;
    double plane_axis_rad_ = 0.0;
    double contact_point_x_ = 0.3;
    double contact_point_y_ = 0.3;

    double z_limit_;

    void accRootCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
    void allocateYawTerm() override;
    void controlCore() override;
    bool optimalGain() override;
    void publishGain() override;
    void rosParamInit() override;
    bool startWallTouching(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    bool setHorizontalForceMode(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    bool resetHorizontalForceMode(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  };
};
