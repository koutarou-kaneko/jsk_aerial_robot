// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

/* ros */
#include <ros/ros.h>

/* base class */
#include <aerial_robot_base/sensor/base_plugin.h>

/* kalman filters */
#include <kalman_filter/kf_pos_vel_acc_plugin.h>

/* ros msg */
#include <sensor_msgs/Range.h>
#include <spinal/Barometer.h>

using namespace Eigen;
using namespace std;

namespace
{
  double range_previous_secs;
}

namespace sensor_plugin
{
  class Alt :public sensor_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, StateEstimator* estimator, string sensor_name, int index)
    {
      SensorBase::initialize(nh, nhp, estimator, sensor_name, index);
      rosParamInit();

      kf_loader_ptr_ = boost::shared_ptr< pluginlib::ClassLoader<kf_plugin::KalmanFilter> >(new pluginlib::ClassLoader<kf_plugin::KalmanFilter>("kalman_filter", "kf_plugin::KalmanFilter"));
      baro_bias_kf_  = kf_loader_ptr_->createInstance("aerial_robot_base/kf_baro_bias");
      baro_bias_kf_->initialize(string(""), 0);

      baro_lpf_filter_ = IirFilter(sample_freq_, cutoff_freq_);
      baro_lpf_high_filter_ = IirFilter(sample_freq_, high_cutoff_freq_);

      /* range sensor */
      std::string topic_name;
      getParam<std::string>("range_sensor_sub_name", topic_name, string("/distance"));
      range_sensor_sub_ = nh_.subscribe(topic_name, 10, &Alt::rangeCallback, this);

      if(estimator_->getGpsHandlers().size() == 1)
        {
          alt_pub_ = nh_.advertise<aerial_robot_msgs::States>("data",10);
          alt_mode_sub_ = nh_.subscribe("estimate_alt_mode", 1, &Alt::altEstimateModeCallback, this);
        }

      /* barometer */
      //barometer_sub_ = nh_.subscribe<spinal::Barometer>(barometer_sub_name_, 1, &Alt::baroCallback, this, ros::TransportHints().tcpNoDelay());
    }

    ~Alt() {}

    Alt():
      sensor_plugin::SensorBase(string("alt")),
      /* range sensor */
      raw_range_sensor_value_(0),
      prev_raw_range_sensor_value_(0),
      raw_range_pos_z_(0),
      prev_raw_range_pos_z_(0),
      raw_range_vel_z_(0),
      min_range_(-1),
      max_range_(-1),
      range_sensor_hz_(0),
      init_range_value_(0),
      /* barometer */
      raw_baro_pos_z_(0),
      baro_pos_z_(0),
      prev_raw_baro_pos_z_(0),
      baro_vel_z_(0),
      raw_baro_vel_z_(0),
      baro_temp_(0),
      high_filtered_baro_pos_z_(0),
      prev_high_filtered_baro_pos_z_(0),
      high_filtered_baro_vel_z_(0),
      alt_estimate_mode_(ONLY_BARO_MODE),
      inflight_state_(false)
    {
      alt_state_.states.resize(2);
      alt_state_.states[0].id = "range_sensor";
      alt_state_.states[0].state.resize(2);
      alt_state_.states[1].id = "baro";
      alt_state_.states[1].state.resize(3);

      /* set health chan num */
      setHealthChanNum(2);
    }

    /* the height estimation related function */
    static constexpr uint8_t ONLY_BARO_MODE = 0; //we estimate the height only based the baro, but the bias of baro is constexprant(keep the last eistamted value)
    static constexpr uint8_t WITH_BARO_MODE = 1; //we estimate the height using range sensor etc. without the baro, but we are estimate the bias of baro
    static constexpr uint8_t WITHOUT_BARO_MODE = 2; //we estimate the height using range sensor etc. with the baro, also estimating the bias of baro

  private:
    /* ros */
    ros::Publisher alt_pub_;
    ros::Subscriber alt_mode_sub_;
    /* range sensor */
    ros::Subscriber range_sensor_sub_;
    /* barometer */
    ros::Subscriber barometer_sub_;
    ros::Publisher barometer_pub_;

    /* ros param */
    /* range sensor */
    tf::Vector3 range_origin_; /* the origin of range based on cog of UAV */
    double range_noise_sigma_;
    int initialize_cnt_max_;
    int initialize_cnt_;
    /* barometer */
    string barometer_sub_name_;
    double baro_noise_sigma_, baro_bias_noise_sigma_;
    /* the iir filter for the barometer for the first filtering stage */
    double sample_freq_, cutoff_freq_, high_cutoff_freq_;

    /* base variables */
    /* range sensor */
    double raw_range_sensor_value_, prev_raw_range_sensor_value_;
    double raw_range_pos_z_, prev_raw_range_pos_z_, raw_range_vel_z_;
    double min_range_, max_range_;
    double ascending_check_range_; /* merge range around the min range */
    double init_range_value_; /* averange value */
    float range_sensor_hz_;
    int alt_estimate_mode_;

    /* barometer */
    /* the kalman filter for the baro bias estimation */
    boost::shared_ptr< pluginlib::ClassLoader<kf_plugin::KalmanFilter> > kf_loader_ptr_;
    boost::shared_ptr<kf_plugin::KalmanFilter> baro_bias_kf_;
    IirFilter baro_lpf_filter_, baro_lpf_high_filter_;
    bool inflight_state_; //the flag for the inflight state
    double raw_baro_pos_z_, baro_pos_z_, prev_raw_baro_pos_z_, prev_baro_pos_z_;
    double raw_baro_vel_z_, baro_vel_z_;
    double high_filtered_baro_pos_z_, prev_high_filtered_baro_pos_z_, high_filtered_baro_vel_z_;
    double baro_temp_; // the temperature of the chip

    aerial_robot_msgs::States alt_state_;

    void rangeCallback(const sensor_msgs::RangeConstPtr & range_msg)
    {
      double current_secs = range_msg->header.stamp.toSec();

      if(!updateBaseLink2SensorTransform()) return;

      /* consider the orientation of the uav */
      raw_range_sensor_value_ = range_msg->range;
      raw_range_pos_z_ = -(estimator_->getOrientation(Frame::BASELINK, StateEstimator::EGOMOTION_ESTIMATE) * (sensor_tf_* tf::Vector3(0, 0, range_msg->range))).z();

      /* initizalize phase */
      if(initialize_cnt_ > 0)
        {
          if(initialize_cnt_ == initialize_cnt_max_)
            {
              range_previous_secs = current_secs;
              setStatus(Status::INIT);
            }

          initialize_cnt_--;
          sensor_hz_ += (current_secs - range_previous_secs);
          init_range_value_ += raw_range_sensor_value_;

          /* initialize */
          if(initialize_cnt_ == 0)
            {
              /* set the range of the sensor value */
              if(min_range_ < 0) min_range_ = range_msg->min_range;
              if(max_range_ < 0) max_range_ = range_msg->max_range;

              /* check the sanity of the range sensor value */
              if(range_msg->max_range <= range_msg->min_range)
                {
                  initialize_cnt_ = 1;
                  ROS_ERROR("range sensor: the min/max range is not correct");
                  return;
                }

              sensor_hz_ /= (float)(initialize_cnt_ - 1);
              init_range_value_ /= (float)initialize_cnt_max_;

              ROS_ERROR("Test: min range: %f, max range: %f, init_range_value_: %f", min_range_, max_range_, init_range_value_);

              /* check the sanity of the first height */
              if(init_range_value_ < min_range_ || init_range_value_ > max_range_)
                {
                  /* sonar sensor should be here */
                  ROS_WARN("the range sensor is too close to the ground");

                  /* set the undescending mode because we may only use imu for z(alt) estimation */
                  estimator_->setUnDescendMode(true);
                }
              else
                {
                  /* fuser for 0: egomotion, 1: experiment */
                  for(int mode = 0; mode < 2; mode++)
                    {
                      if(!getFuserActivate(mode)) continue;

                      for(auto& fuser : estimator_->getFuser(mode))
                        {
                          boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                          int id = kf->getId();

                          if(id & (1 << State::Z_BASE))
                            {
                              kf->setMeasureFlag();
                              kf->setInitState(raw_range_sensor_value_, 0);
                            }
                        }
                    }
                }

              /* change the alt estimate mode */
              alt_estimate_mode_ = WITHOUT_BARO_MODE;


              /* set the status for Z (altitude) */
              estimator_->setStateStatus(State::Z_BASE, StateEstimator::EGOMOTION_ESTIMATE, true);

              setStatus(Status::ACTIVE); //active

              ROS_WARN("%s: the hz is %f, estimate mode is %d",
                       (range_msg->radiation_type == sensor_msgs::Range::ULTRASOUND)?string("sonar sensor").c_str():string("infrared sensor").c_str(), 1.0 / sensor_hz_, alt_estimate_mode_);
            }
        }


      /* first ascending phase */
      if(estimator_->getUnDescendMode() &&
         raw_range_sensor_value_ < min_range_ + ascending_check_range_ &&
         raw_range_sensor_value_ > min_range_ &&
         prev_raw_range_sensor_value_ < min_range_ &&
         prev_raw_range_sensor_value_ > min_range_ - ascending_check_range_)
        {

          ROS_WARN("%s: confirm ascending to sanity height, start sf correction process, previous height: %f", (range_msg->radiation_type == sensor_msgs::Range::ULTRASOUND)?string("sonar sensor").c_str():string("infrared sensor").c_str(), prev_raw_range_pos_z_);

          /* release the non-descending mode, use the range sensor for z(alt) estimation */
          estimator_->setUnDescendMode(false);

          for(int mode = 0; mode < 2; mode++)
            {
              if(!getFuserActivate(mode)) continue;

              for(auto& fuser : estimator_->getFuser(mode))
                {
                  ROS_INFO("debug sonar: init test:");
                  boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                  int id = kf->getId();
                  if(id & (1 << State::Z_BASE))
                    {
                      kf->setInitState(raw_range_pos_z_, 0);
                      kf->setMeasureFlag();
                    }
                }
            }
        }


      /* update */
      raw_range_vel_z_ = (raw_range_pos_z_ - prev_raw_range_pos_z_) / (current_secs - range_previous_secs); 
      range_previous_secs = current_secs;
      prev_raw_range_pos_z_ = raw_range_pos_z_;
      prev_raw_range_sensor_value_ = raw_range_sensor_value_;
      if(initialize_cnt_ > 0) return;

      /* terrain check and height estimate */
      alt_state_.header.stamp.fromSec(range_msg->header.stamp.toSec());
      rangeEstimateProcess();

      /* publish phase */
      alt_state_.states[0].state[0].x = raw_range_pos_z_;
      alt_state_.states[0].state[0].y = raw_range_vel_z_;

      alt_pub_.publish(alt_state_);
      updateHealthStamp(1); //channel: 1
    }

    void rangeEstimateProcess()
    {
      if(getStatus() == Status::INVALID) return;

      Matrix<double, 1, 1> temp = MatrixXd::Zero(1, 1);

      for(int mode = 0; mode < 2; mode++)
        {
          if(!getFuserActivate(mode)) continue;

          for(auto& fuser : estimator_->getFuser(mode))
            {
              string plugin_name = fuser.first;
              boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
              int id = kf->getId();
              if(id & (1 << State::Z_BASE))
                {
                  if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                    {
                      /* correction */
                      VectorXd measure_sigma(1); measure_sigma << range_noise_sigma_;
                      VectorXd meas(1); meas <<  raw_range_pos_z_;
                      vector<double> params = {kf_plugin::POS};

                      kf->correction(meas, measure_sigma,
                                     time_sync_?(alt_state_.header.stamp.toSec()):-1, params);
                    }
                }
            }
        }
    }

    void baroCallback(const spinal::BarometerConstPtr & baro_msg)
    {
      static double baro_previous_secs;
      double current_secs = baro_msg->stamp.toSec();

      raw_baro_pos_z_ = baro_msg->altitude;
      baro_temp_ = baro_msg->temperature;

      /*First Filtering: IIR filter */
      /* position */
      baro_pos_z_ = baro_lpf_filter_.filterFunction(raw_baro_pos_z_);
      high_filtered_baro_pos_z_ = baro_lpf_high_filter_.filterFunction(raw_baro_pos_z_);
      /* velocity */
      raw_baro_vel_z_ = (raw_baro_pos_z_ - prev_raw_baro_pos_z_)/(current_secs - baro_previous_secs);
      baro_vel_z_ = (baro_pos_z_ - prev_baro_pos_z_)/(current_secs - baro_previous_secs);
      high_filtered_baro_vel_z_ = (high_filtered_baro_pos_z_ - prev_high_filtered_baro_pos_z_)/(current_secs - baro_previous_secs);

      /* the true initial phase for baro based estimattion for inflight state */
      /* since the value of pressure will decrease during the rising of the propeller rotation speed */
      if(((alt_estimate_mode_ == ONLY_BARO_MODE  && high_filtered_baro_vel_z_ > 0.1)
          || alt_estimate_mode_ == WITHOUT_BARO_MODE)
         && estimator_->getFlyingFlag() && !inflight_state_)
        {//the inflight state should be with the velocity of 0.1(up)
          inflight_state_ = true;
          ROS_WARN("barometer: start the inflight barometer height estimation");

          /* the initialization of the baro bias kf filter */
          VectorXd input_sigma(1); input_sigma << baro_bias_noise_sigma_;
          baro_bias_kf_->setPredictionNoiseCovariance(input_sigma);
          baro_bias_kf_->setInputFlag();
          baro_bias_kf_->setMeasureFlag();
          baro_bias_kf_->setInitState(-baro_pos_z_, 0);
        }
      /* reset */
      if(estimator_->getLandedFlag()) inflight_state_ = false;
      baroEstimateProcess(baro_msg->stamp);

      /* publish */
      alt_state_.header.stamp = baro_msg->stamp;
      alt_state_.states[1].state[0].x = raw_baro_pos_z_;
      alt_state_.states[1].state[0].y = raw_baro_vel_z_;
      alt_state_.states[1].state[1].x = baro_pos_z_;
      alt_state_.states[1].state[1].y = baro_vel_z_;
      alt_state_.states[1].state[2].x = high_filtered_baro_pos_z_;
      alt_state_.states[1].state[2].y = high_filtered_baro_vel_z_;

      alt_pub_.publish(alt_state_);

      /* update */
      baro_previous_secs = current_secs;
      prev_raw_baro_pos_z_ = raw_baro_pos_z_;
      prev_baro_pos_z_ = baro_pos_z_;
      prev_high_filtered_baro_pos_z_ = high_filtered_baro_pos_z_;
      updateHealthStamp(0); //channel: 0
    }

    void baroEstimateProcess(ros::Time stamp)
    {
      if(getStatus() == Status::INVALID) return;

      if(!inflight_state_) return;

      switch(alt_estimate_mode_)
        {
        case ONLY_BARO_MODE :
          for(int mode = 0; mode < 2; mode++)
            {
              if(!getFuserActivate(mode)) continue;

              for(auto& fuser : estimator_->getFuser(mode))
                {
                  string plugin_name = fuser.first;
                  boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                  int id = kf->getId();
                  if(id & (1 << State::Z_BASE))
                    {
                      if(!kf->getFilteringFlag())
                        {
                          //ROS_FATAL("baro: can not estiamte the height by baro(ONLY_BARO_MODE), because the filtering flag is not activated");
                          return;
                        }
                      /* We should set the sigma every time, since we may have several different sensors to correct the kalman filter(e.g. vo + opti, laser + baro) */

                      if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                        {
                          /* correction */
                          VectorXd measure_sigma(1); measure_sigma << baro_noise_sigma_;
                          VectorXd meas(1); meas <<  baro_pos_z_ + (baro_bias_kf_->getEstimateState())(0);
                          vector<double> params = {kf_plugin::POS};
                          kf->correction(meas, measure_sigma, -1, params);

                        }

                      /* set the state */
                      // VectorXd state = kf->getEstimateState();
                      // estimator_->setState(State::Z_BASE, mode, 0, state(0));
                      // estimator_->setState(State::Z_BASE, mode, 1, state(1));
                      //alt_state_.states[0].state[1].x = state(0);
                      //alt_state_.states[0].state[1].y = state(1);
                      alt_state_.states[0].state[1].z = (baro_bias_kf_->getEstimateState())(0);

                    }
                }
            }
          break;
        case WITHOUT_BARO_MODE:
          {
            baro_bias_kf_->prediction(VectorXd::Zero(1), stamp.toSec());
            VectorXd meas(1);  meas << (estimator_->getState(State::Z_BASE, 0))[0] - baro_pos_z_;
            baro_bias_kf_->correction(meas, VectorXd::Zero(1), -1);
          }
          break;
        case WITH_BARO_MODE:
          //TODO: this is another part, maybe we have to use another package:
          //http://wiki.ros.org/ethzasl_sensor_fusion
          break;
        default:
          break;
        }
    }

    /* force to change the estimate mode */
    void altEstimateModeCallback(const std_msgs::UInt8ConstPtr & mode_msg)
    {
      alt_estimate_mode_ = mode_msg->data;
      ROS_INFO("change the height estimate mode: %d", alt_estimate_mode_);
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      /* range sensor */
      getParam<double>("range_noise_sigma", range_noise_sigma_, 0.005);
      getParam<int>("initialize_cnt_max", initialize_cnt_max_, 50);
      initialize_cnt_ = initialize_cnt_max_;

      /* first ascending process: check range */
      getParam<double>("ascending_check_range", ascending_check_range_, 0.1); // [m]
      getParam<double>("virtual_min_range", min_range_, -1); // if no set, the min_range is kept as -1, and will be updated by sensor topic
      getParam<double>("virtual_max_range", max_range_, -1); // if no set, the min_range is kept as -1, and will be updated by sensor topic

      /* barometer */
      getParam<std::string>("barometer_sub_name", barometer_sub_name_, string("/baro"));
      getParam<double>("baro_noise_sigma", baro_noise_sigma_, 0.05 );
      getParam<double>("baro_bias_noise_sigma", baro_bias_noise_sigma_, 0.001 );
      getParam<double>("sample_freq", sample_freq_, 100.0 );
      getParam<double>("cutoff_freq", cutoff_freq_, 10.0 );
      getParam<double>("high_cutoff_freq", high_cutoff_freq_, 1.0 );
    }

  };

};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Alt, sensor_plugin::SensorBase);




