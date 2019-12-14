/*
******************************************************************************
* File Name          : attitude_control.h
* Description        : attitude control interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "flight_control/attitude/attitude_control.h"

#ifdef SIMULATION
AttitudeController::AttitudeController(): DELTA_T(0) {}

void AttitudeController::init(ros::NodeHandle* nh)
{
  nh_ = nh;

  pwms_pub_ = nh_->advertise<spinal::Pwms>("/motor_pwms", 1);
  control_term_pub_ = nh_->advertise<spinal::RollPitchYawTerms>("/control_terms", 1);
  anti_gyro_pub_ = nh_->advertise<std_msgs::Float32MultiArray>("/gyro_moment_compensation", 1);
  four_axis_cmd_sub_ = nh_->subscribe("/aerial_robot_control_four_axis", 1, &AttitudeController::fourAxisCommandCallback, this);
  pwm_info_sub_ = nh_->subscribe("/motor_info", 1, &AttitudeController::pwmInfoCallback, this);
  rpy_gain_sub_ = nh_->subscribe("/rpy_gain", 1, &AttitudeController::rpyGainCallback, this);
  p_matrix_pseudo_inverse_inertia_sub_ = nh_->subscribe("/p_matrix_pseudo_inverse_inertia", 1, &AttitudeController::pMatrixInertiaCallback, this);
  pwm_test_sub_ = nh_->subscribe("/pwm_test", 1, &AttitudeController::pwmTestCallback, this);
  att_control_srv_ = nh_->advertiseService("/set_attitude_control", &AttitudeController::setAttitudeControlCallback, this);
  torque_allocation_matrix_inv_sub_ = nh_->subscribe("/torque_allocation_matrix_inv", 1, &AttitudeController::torqueAllocationMatrixInvCallback, this);
  attitude_gains_srv_ = nh_->advertiseService("/set_attitude_gains", &AttitudeController::setAttitudeGainsCallback, this);
  baseInit();
}

#else

AttitudeController::AttitudeController():
  pwms_pub_("/motor_pwms", &pwms_msg_),
  control_term_pub_("/control_terms", &control_term_msg_),
  four_axis_cmd_sub_("/aerial_robot_control_four_axis", &AttitudeController::fourAxisCommandCallback, this ),
  pwm_info_sub_("/motor_info", &AttitudeController::pwmInfoCallback, this),
  rpy_gain_sub_("/rpy_gain", &AttitudeController::rpyGainCallback, this),
  p_matrix_pseudo_inverse_inertia_sub_("/p_matrix_pseudo_inverse_inertia", &AttitudeController::pMatrixInertiaCallback, this),
  pwm_test_sub_("/pwm_test", &AttitudeController::pwmTestCallback, this ),
  att_control_srv_("/set_attitude_control", &AttitudeController::setAttitudeControlCallback, this),
  torque_allocation_matrix_inv_sub_("/torque_allocation_matrix_inv", &AttitudeController::torqueAllocationMatrixInvCallback, this),
  attitude_gains_srv_("/set_attitude_gains", &AttitudeController::setAttitudeGainsCallback, this)
{
}

void AttitudeController::init(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2, StateEstimate* estimator, BatteryStatus* bat, ros::NodeHandle* nh)
{

  pwm_htim1_ = htim1;
  pwm_htim2_ = htim2;
  nh_ = nh;
  estimator_ = estimator;
  bat_ = bat;

  HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_4);

  nh_->advertise(pwms_pub_);
  nh_->advertise(control_term_pub_);

  nh_->subscribe< ros::Subscriber<spinal::FourAxisCommand, AttitudeController> >(four_axis_cmd_sub_);
  nh_->subscribe< ros::Subscriber<spinal::PwmInfo, AttitudeController> >(pwm_info_sub_);
  nh_->subscribe< ros::Subscriber<spinal::RollPitchYawTerms, AttitudeController> >(rpy_gain_sub_);
  nh_->subscribe< ros::Subscriber<std_msgs::Float32, AttitudeController> >(pwm_test_sub_);
  nh_->subscribe< ros::Subscriber<spinal::PMatrixPseudoInverseWithInertia, AttitudeController> >(p_matrix_pseudo_inverse_inertia_sub_);
  nh_->subscribe< ros::Subscriber<spinal::TorqueAllocationMatrixInv, AttitudeController> >(torque_allocation_matrix_inv_sub_);

  nh_->advertiseService(att_control_srv_);
  nh_->advertiseService(attitude_gains_srv_);

  baseInit();
}
#endif

void AttitudeController::baseInit()
{
  // base param for uav model
  motor_number_ = 0;
  uav_model_ = -1;

  start_control_flag_ = false;
  pwm_test_flag_ = false;
  lqi_mode_ = false;
  force_landing_flag_ = false;
  attitude_flag_ = true;
  attitude_gain_receive_flag_ = false;

  //pwm init
  pwm_conversion_mode_ = -1;
  min_duty_ = IDLE_DUTY;
  max_duty_ = min_duty_; //should assign right value from PC(ros)
  force_landing_thrust_ = 0;
  // voltage
  v_factor_ = 1;
  motor_ref_index_ = 0;
  voltage_update_last_time_ = 0;

  reset();
}

void AttitudeController::pwmsControl(void)
{
  static uint32_t ros_pub_last_time = HAL_GetTick();

  for(int i = 0; i < motor_number_; i++)
    {
      float target_thrust = target_thrust_[i];
      /* for dragon, we use dual rotor, so devide into two */
      if(uav_model_ == spinal::UavInfo::DRAGON) target_thrust /= 2;

      if(start_control_flag_)
        {
          target_pwm_[i] = pwmConversion(target_thrust);

          /* constraint */
          if(target_pwm_[i] < min_duty_) target_pwm_[i]  = min_duty_;
          else if(target_pwm_[i]  > max_duty_) target_pwm_[i]  = max_duty_;

          /* motor pwm test */
          if(pwm_test_flag_) target_pwm_[i] = pwm_test_value_;
        }
      /* for ros */
      pwms_msg_.motor_value[i] = (target_pwm_[i] * 2000);
    }

#ifdef SIMULATION
  /* control result publish */
  if(HAL_GetTick() - ros_pub_last_time > CONTROL_PUB_INTERVAL)
    {
      ros_pub_last_time = HAL_GetTick();
      pwms_pub_.publish(pwms_msg_);
      control_term_pub_.publish(control_term_msg_);
    }

#else
  /* control result publish */
  if(HAL_GetTick() - ros_pub_last_time > CONTROL_PUB_INTERVAL)
    {
      ros_pub_last_time = HAL_GetTick();
      pwms_pub_.publish(&pwms_msg_);
      control_term_pub_.publish(&control_term_msg_);
    }


  /* nerve comm type */
#if NERVE_COMM
  for(int i = 0; i < motor_number_; i++) {
#if MOTOR_TEST

    if (i == (HAL_GetTick() / 2000) % motor_number_)
      Spine::setMotorPwm(200, i);
    else
      Spine::setMotorPwm(0, i);
#else
    Spine::setMotorPwm(target_pwm_[i] * 2000 - 1000, i);
#endif
  }
  return;
#endif

  /* direct pwm type */
  pwm_htim1_->Instance->CCR1 = (uint32_t)(target_pwm_[0] * MAX_PWM);
  pwm_htim1_->Instance->CCR2 = (uint32_t)(target_pwm_[1] * MAX_PWM);
  pwm_htim1_->Instance->CCR3 = (uint32_t)(target_pwm_[2] * MAX_PWM);
  pwm_htim1_->Instance->CCR4 = (uint32_t)(target_pwm_[3] * MAX_PWM);

  if(motor_number_ > 4)
    {
      pwm_htim2_->Instance->CCR1 =   (uint32_t)(target_pwm_[4] * MAX_PWM);
      pwm_htim2_->Instance->CCR2 =  (uint32_t)(target_pwm_[5] * MAX_PWM);
    }
  if(motor_number_ > 6)
    {
      pwm_htim2_->Instance->CCR3 = (uint32_t)(target_pwm_[6] * MAX_PWM);
      pwm_htim2_->Instance->CCR4 =  (uint32_t)(target_pwm_[7] * MAX_PWM);
    }

#endif
}

void AttitudeController::update(void)
{
  if(start_control_flag_)
    {
      /* failsafe 1: check the timeout of the flight command receive process */
      if(failsafe_ && !force_landing_flag_ && attitude_flag_ &&
         (int32_t)(HAL_GetTick() - flight_command_last_stamp_) > FLIGHT_COMMAND_TIMEOUT)
        {
          /* timeout => start force landing */
#ifdef SIMULATION
          ROS_ERROR("failsafe1, time now: %d, time last stamp: %d", HAL_GetTick(), flight_command_last_stamp_);
#else
          nh_->logerror("failsafe1");
#endif
          setForceLandingFlag(true);
        }

      /* should be virutal coord */
#ifdef SIMULATION
      Vector3f angles = angles_;
      Vector3f vel = vel_;
      static double prev_time = ros::Time::now().toSec();
      DELTA_T = ros::Time::now().toSec() - prev_time;
      prev_time = ros::Time::now().toSec();
#else
      Vector3f angles = estimator_->getAttEstimator()->getAttitude(Frame::VIRTUAL);
      Vector3f vel = estimator_->getAttEstimator()->getAngular(Frame::VIRTUAL);
#endif

      /* failsafe 3: too large tile angle */
      if(!force_landing_flag_  && (fabs(angles[X]) > MAX_TILT_ANGLE || fabs(angles[Y]) > MAX_TILT_ANGLE))
        {
#ifdef SIMULATION
          ROS_ERROR("failsafe3");
#else
          nh_->logerror("failsafe3");
#endif
          setForceLandingFlag(true);
          error_angle_i_[X] = 0;
          error_angle_i_[Y] = 0;
        }

      /* Force Landing Flag */
      if(force_landing_flag_)
        {
          target_angle_[X] = 0;
          target_angle_[Y] = 0;
          target_angle_[Z] = 0;
          target_cog_force_[X] = 0;
          target_cog_force_[Y] = 0;
        }

      /* LQI Method */
      if(lqi_mode_)
        {
          float lqi_p_term = 0;
          float lqi_i_term = 0;
          float lqi_d_term = 0;

          /* gyro moment */
          Vector3f gyro_moment = vel % (inertia_ * vel);
#ifdef SIMULATION
          std_msgs::Float32MultiArray anti_gyro_msg;
#endif

          for(int i = 0; i < motor_number_; i++)
            {
              motor_rpy_force_[i] = 0; //[N]

              for(int axis = 0; axis < 3; axis++)
                {
                  if(axis < 2)
                    {
                      float error_angle = target_angle_[axis] - angles[axis];
                      /* error_angle_i */
                      if(i == 0 && integrate_flag_ == true)
                        error_angle_i_[axis] += error_angle * DELTA_T;

                      lqi_p_term = -error_angle * p_lqi_gain_[i][axis];
                      lqi_i_term = (error_angle_i_[axis] * i_lqi_gain_[i][axis]);
                    }
                  else
                    { lqi_p_term = 0; lqi_i_term = 0; }

                  //lqi_d_term = vel[axis] * d_lqi_gain[i][axis];
                  lqi_d_term = limit(vel[axis] * d_lqi_gain_[i][axis], 4.5); //(old meesage: can be more strict), why?
                  motor_rpy_force_[i] += (lqi_p_term + lqi_i_term + lqi_d_term); //[N]

                  if(axis == X)
                    {
                      control_term_msg_.motors[i].roll_p = lqi_p_term * 1000;
                      control_term_msg_.motors[i].roll_i= lqi_i_term * 1000;
                      control_term_msg_.motors[i].roll_d = lqi_d_term * 1000;
                    }
                  if(axis == Y)
                    {
                      control_term_msg_.motors[i].pitch_p = lqi_p_term * 1000;
                      control_term_msg_.motors[i].pitch_i = lqi_i_term * 1000;
                      control_term_msg_.motors[i].pitch_d = lqi_d_term * 1000;
                    }
                  if(axis == Z)
                    control_term_msg_.motors[i].yaw_d = lqi_d_term * 1000;//lqi_d_term;
                }
              /* gyro moment compensation */
              float gyro_moment_compensate =
                p_matrix_pseudo_inverse_[i][0] * gyro_moment.x +
                p_matrix_pseudo_inverse_[i][1] * gyro_moment.y +
                p_matrix_pseudo_inverse_[i][2] * gyro_moment.z;
#ifdef SIMULATION
              anti_gyro_msg.data.push_back(gyro_moment_compensate);
#endif
              target_thrust_[i] = base_throttle_term_[i];
              if(attitude_flag_) target_thrust_[i] += (motor_rpy_force_[i] + gyro_moment_compensate);
            }

#ifdef SIMULATION
          anti_gyro_pub_.publish(anti_gyro_msg);
#endif
          if(force_landing_flag_)
            {
              float total_thrust = 0;
              /* sum */
              for(int i = 0; i < motor_number_; i++) total_thrust += target_thrust_[i];
              /* average */
              float average_thrust = total_thrust / motor_number_;

              if(average_thrust > force_landing_thrust_)
                {
                  for(int i = 0; i < motor_number_; i++)
                    base_throttle_term_[i] -= (target_thrust_[i] / average_thrust * FORCE_LANDING_INTEGRAL);
                }
            }
        }
      else
        {
    	  /* Dynamics Inversion method */
    	  if (!attitude_gain_receive_flag_) return;

    	  float error_angle = 0;
    	  float p_term[3], i_term[3], d_term[3];
    	  for(int axis = 0; axis < 3; axis++)
    	  {

    		  if(axis < Z) //roll and pitch
    		  {
    			  //P term
    			  error_angle = target_angle_[axis] - angles[axis];
    			  p_term[axis] = limit(error_angle * attitude_p_gain_[axis], attitude_p_term_limit_[axis]);
    			  //I term
    			  if(integrate_flag_) error_angle_i_[axis] += (error_angle * DELTA_T);
    			  i_term[axis] = limit(error_angle_i_[axis] * attitude_i_gain_[axis], attitude_i_term_limit_[axis]);
    		  }
    		  else //yaw
    		  {
    			  p_term[axis]  = attitude_yaw_p_i_term_;
    			  i_term[axis] = 0;
    		  }

    		  //D term
    		  d_term[axis] = limit(-vel[axis] * attitude_d_gain_[axis], attitude_d_term_limit_[axis]);

    		  //total
    		  target_cog_angular_acc_[axis] = p_term[axis] + i_term[axis] + d_term[axis];
            }

          control_term_msg_.motors[0].roll_p = p_term[X] * 1000;
          control_term_msg_.motors[0].roll_i = i_term[X] * 1000;
          control_term_msg_.motors[0].roll_d = d_term[X] * 1000;
          control_term_msg_.motors[0].pitch_p = p_term[Y] * 1000;
          control_term_msg_.motors[0].pitch_i = i_term[Y] * 1000;
          control_term_msg_.motors[0].pitch_d = d_term[Y] * 1000;
          control_term_msg_.motors[0].yaw_d = d_term[Z] * 1000;

          if(force_landing_flag_)
          {
        	  float total_thrust = 0;
        	  /* sum */
        	  for(int i = 0; i < motor_number_; i++) total_thrust += target_thrust_[i];
        	  /* average */
        	  float average_thrust = total_thrust / motor_number_;

        	  if(average_thrust > force_landing_thrust_)
        	  {
        		  for(int i = 0; i < motor_number_; i++)
        			  base_throttle_term_[i] -= (target_thrust_[i] / average_thrust * FORCE_LANDING_INTEGRAL);
        	  }
          }

          inversionMapping();
        }
    }
  /* force -> pwm */
  pwmsControl();
}

void AttitudeController::inversionMapping(void)
{
	switch (uav_model_)
	  {
    	case spinal::UavInfo::DRONE:
    	  {
    	    auto underActuatedInversion = [this](float x, float y, float z) -> float {
    	    	return target_cog_force_[Z] + target_cog_torque_[X] * x  + target_cog_torque_[Y] * y + target_cog_torque_[Z] * z;
    	    };

    		/* under actuated system */
    		if(motor_number_ == 4)
    		  {
    			target_thrust_[0] = underActuatedInversion(-1,+1,+1); //REAR_R
    			target_thrust_[1] = underActuatedInversion(-1,-1,-1); //FRONT_R
    			target_thrust_[2] = underActuatedInversion(+1,-1,+1); //FRONT_L
    			target_thrust_[3] = underActuatedInversion(+1,+1,-1); //REAR_L
    		  }
    		if(motor_number_ == 6)
    		  {
    			target_thrust_[0] = underActuatedInversion(-0.5 ,0.866, +1);  //REAR_R
    			target_thrust_[1] = underActuatedInversion(-1, 0, -1);        //MIDDLE_R
    			target_thrust_[2] = underActuatedInversion(-0.5, -0.866, +1); //FRONT_R
    			target_thrust_[3] = underActuatedInversion(+0.5 ,-0.866, -1); //FRONT_L
    			target_thrust_[4] = underActuatedInversion(+1, 0, +1);        //MIDDLE_L
    			target_thrust_[5] = underActuatedInversion(+0.5, 0.866,-1);   //REAR_L
    		  }
    		break;
    	  }
    	case spinal::UavInfo::HYDRUS_XI:
    	  {
    		  auto fullyActuatedInversion = [this](int index) -> float {
    			  return base_throttle_term_[index] +
    					 target_cog_angular_acc_[X] * torque_allocation_matrix_inv_[index][X] +
						 target_cog_angular_acc_[Y] * torque_allocation_matrix_inv_[index][Y] +
						 target_cog_angular_acc_[Z] * torque_allocation_matrix_inv_[index][Z];
    		  };
    		  for (unsigned int i = 0; i < motor_number_; i++)
    			  target_thrust_[i] = fullyActuatedInversion(i);
    		  break;
    	  }
	  }
}

void AttitudeController::reset(void)
{
  for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
    {
      target_thrust_[i] = 0;
      target_pwm_[i] = IDLE_DUTY;
    }

  for(int i = 0; i < 3; i++)
    {
      target_angle_[i] = 0;
      target_cog_force_[i] = 0;
      target_cog_torque_[i] = 0;
      target_cog_angular_acc_[i] = 0;
      error_angle_i_[i] = 0;


    }
  //LQI mode
  for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
    {
      base_throttle_term_[i] = 0;
      motor_rpy_force_[i] = 0;
    }
  for (int i = 0; i < MAX_MOTOR_NUMBER; i++)
    {
	  for (int j = 0; j < 3; j++)
	    {
		  torque_allocation_matrix_inv_[i][j] = 0.0;
	    }
    }

  attitude_yaw_p_i_term_ = 0.0;
  integrate_flag_ = false;

  /* failsafe */
  failsafe_ = false;
  flight_command_last_stamp_ = HAL_GetTick();
}

void AttitudeController::fourAxisCommandCallback( const spinal::FourAxisCommand &cmd_msg)
{
  if(!start_control_flag_ || force_landing_flag_) return; //do not receive command

  /* start failsafe func if not activate */
  if(!failsafe_) failsafe_ = true;
  flight_command_last_stamp_ = HAL_GetTick();

  target_angle_[X] = cmd_msg.angles[0];
  target_angle_[Y] = cmd_msg.angles[1];

  /* failsafe2-1: if the pitch and roll angle is too big, start force landing */
  if(fabs(target_angle_[X]) > MAX_TILT_ANGLE || fabs(target_angle_[Y]) > MAX_TILT_ANGLE )
    {
      setForceLandingFlag(true);
#ifdef SIMULATION
      ROS_ERROR("failsafe2");
#else
      nh_->logerror("failsafe2");
#endif
      return;
    }

  /* LQI mode */
  if(lqi_mode_)
    {
      /* check the number of motor which should be equal to the ros throttle */
#ifdef SIMULATION
      if(cmd_msg.base_throttle.size() != motor_number_)
        {
          ROS_ERROR("fource axis commnd: motor number is not identical between fc and pc");
          return;
        }
#else
      if(cmd_msg.base_throttle_length != motor_number_)
    	{
    	  nh_->logerror("fource axis commnd: motor number is not identical between fc and pc");
    	  return;
    	 }
#endif

      for(int i = 0; i < motor_number_; i++)
        base_throttle_term_[i] = cmd_msg.base_throttle[i];
    }
  else
    { /* dynamics inversion */
      switch (uav_model_)
        {
        case spinal::UavInfo::DRONE:
          {
            target_angle_[Z] = cmd_msg.angles[Z];
            target_thrust_[Z] = cmd_msg.base_throttle[0]; //no good name
            break;
          }
        case spinal::UavInfo::HYDRUS_XI:
          {
              /* check the number of motor which should be equal to the ros throttle */
#ifdef SIMULATION
        	  if(cmd_msg.base_throttle.size() != motor_number_)
        	  {
        		  ROS_ERROR("fource axis commnd: motor number is not identical between fc and pc");
        		  return;
        	  }
#else
        	  if(cmd_msg.base_throttle_length != motor_number_) return;
#endif

        	  for(int i = 0; i < motor_number_; i++)
        		  base_throttle_term_[i] = cmd_msg.base_throttle[i];
        	  attitude_yaw_p_i_term_ = cmd_msg.angles[Z]; //P and I term of yaw target acc
        	  break;
          }
        default:
          {
          break;
          }
        }
    }
}

void AttitudeController::pwmInfoCallback( const spinal::PwmInfo &info_msg)
{
  force_landing_thrust_ = info_msg.force_landing_thrust;

  min_duty_ = info_msg.min_pwm;
  max_duty_ = info_msg.max_pwm;
  pwm_conversion_mode_ = info_msg.pwm_conversion_mode;

  motor_info_.resize(0);
#ifdef SIMULATION
  for(int i = 0; i < info_msg.motor_info.size(); i++)
#else
    for(int i = 0; i < info_msg.motor_info_length; i++)
#endif
      {
        motor_info_.push_back(info_msg.motor_info[i]);
      }

}

void AttitudeController::rpyGainCallback( const spinal::RollPitchYawTerms &gain_msg)
{
  if(motor_number_ == 0) return; //not be activated

  /* check the number of motor which should be equal to the ros throttle */
#ifdef SIMULATION
  if(gain_msg.motors.size() != motor_number_)
    {
      if(motor_number_ > 0)
      {
    	  ROS_ERROR("rpy gain: motor number is not identical between fc:%d and pc:%d", motor_number_, (int)gain_msg.motors.size());
    	  return;
      }
    }
#else
  if(gain_msg.motors_length != motor_number_)
	  {
	  	  nh_->logerror("rpy gain: motor number is not identical between fc and pc");
	  	  return;
	  }
#endif

  for(int i = 0; i < motor_number_; i++)
    {
      p_lqi_gain_[i][X] = gain_msg.motors[i].roll_p / 1000.0f;
      i_lqi_gain_[i][X] = gain_msg.motors[i].roll_i / 1000.0f;
      d_lqi_gain_[i][X] = gain_msg.motors[i].roll_d / 1000.0f;
      p_lqi_gain_[i][Y] = gain_msg.motors[i].pitch_p / 1000.0f;
      i_lqi_gain_[i][Y] = gain_msg.motors[i].pitch_i / 1000.0f;
      d_lqi_gain_[i][Y] = gain_msg.motors[i].pitch_d / 1000.0f;
      d_lqi_gain_[i][Z] = gain_msg.motors[i].yaw_d / 1000.0f;
    }
}

void AttitudeController::pwmTestCallback(const std_msgs::Float32& pwm_msg)
{
  pwm_test_flag_ = true;
  start_control_flag_ = true;
  pwm_test_value_ = pwm_msg.data; //2000ms
}

void AttitudeController::pMatrixInertiaCallback(const spinal::PMatrixPseudoInverseWithInertia& msg)
{
#ifdef SIMULATION
  if(msg.pseudo_inverse.size() != motor_number_)
    {
      if(motor_number_ > 0) ROS_ERROR("p matrix pseudo inverse and inertia commnd: motor number is not identical between fc and pc");
      return;
    }
#else
  if(msg.pseudo_inverse_length != motor_number_)
	  {
	  	  nh_->logerror("p matrix pseudo inverse and inertia commnd: motor number is not identical between fc and pc");
	  	  return;
	  }
#endif

  for(int i = 0; i < motor_number_; i ++)
    {
      p_matrix_pseudo_inverse_[i][0] = msg.pseudo_inverse[i].r / 1000.0f;
      p_matrix_pseudo_inverse_[i][1] = msg.pseudo_inverse[i].p / 1000.0f;
      p_matrix_pseudo_inverse_[i][2] = msg.pseudo_inverse[i].y / 1000.0f;
    }

  /* inertia */
  inertia_ = Matrix3f(msg.inertia[0] * 0.001f, msg.inertia[3] * 0.001f, msg.inertia[5] * 0.001f,
                      msg.inertia[3] * 0.001f, msg.inertia[1] * 0.001f, msg.inertia[4] * 0.001f,
                      msg.inertia[5] * 0.001f, msg.inertia[4] * 0.001f, msg.inertia[2] * 0.001f);
}

void AttitudeController::torqueAllocationMatrixInvCallback(const spinal::TorqueAllocationMatrixInv& msg)
{
#ifdef SIMULATION
  if(msg.rows.size() != motor_number_)
    {
      if(motor_number_ > 0) ROS_ERROR("torqueAllocationMatrixInvCallback: motor number is not identical between fc(%d) and pc(%ld)", motor_number_, msg.rows.size());
      return;
    }
#else
  if(msg.rows_length != motor_number_) return;
#endif

  for (int i = 0; i < motor_number_; i++)
    {
	  torque_allocation_matrix_inv_[i][X] = msg.rows[i].x * 0.001f;
	  torque_allocation_matrix_inv_[i][Y] = msg.rows[i].y * 0.001f;
	  torque_allocation_matrix_inv_[i][Z] = msg.rows[i].z * 0.001f;
    }
}

void AttitudeController::setStartControlFlag(bool start_control_flag)
{
  start_control_flag_ = start_control_flag;

  if(!start_control_flag_) reset();
}

void AttitudeController::setMotorNumber(uint8_t motor_number)
{
  /* check the motor number which has spine system */
  if(motor_number_ > 0)
    {
      if(motor_number_ != motor_number)
        {
          motor_number_ = 0;
#ifdef SIMULATION
          ROS_ERROR("ATTENTION: motor number is 0");
#else
          nh_->logerror("ATTENTION: motor number is 0");
#endif
        }
    }
  else
    {
	  size_t control_term_msg_size;
	  if(uav_model_ == spinal::UavInfo::HYDRUS || uav_model_ == spinal::UavInfo::DRAGON)
		  control_term_msg_size = motor_number;
	  else
		  control_term_msg_size = 1;

#ifdef SIMULATION
      pwms_msg_.motor_value.resize(motor_number);
      control_term_msg_.motors.resize(control_term_msg_size);
#else
      pwms_msg_.motor_value_length = motor_number;
      control_term_msg_.motors_length = control_term_msg_size;
      pwms_msg_.motor_value = new uint16_t[motor_number];
      control_term_msg_.motors = new spinal::RollPitchYawTerm[control_term_msg_size];
#endif
      for(int i = 0; i < motor_number; i++) pwms_msg_.motor_value[i] = 0;

      /* the initialize order is important */
      motor_number_ = motor_number ;
    }
}

void  AttitudeController::setUavModel(int8_t uav_model)
{
  /* check the uav model which has spine system */
  if(uav_model_ != -1)
    {
      if(uav_model_ != uav_model)
        {
          uav_model_ = -1;
#ifdef SIMULATION
          ROS_ERROR("ATTENTION: UAV model is not set");
#else
          nh_->logerror("ATTENTION: UAV model is not set");
#endif
        }
    }
  else
    {
      uav_model_ = uav_model;

      if(uav_model_ == spinal::UavInfo::HYDRUS ||
         uav_model_ == spinal::UavInfo::DRAGON)
        lqi_mode_ = true;
    }
}

#ifdef SIMULATION
bool AttitudeController::setAttitudeControlCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
#else
void AttitudeController::setAttitudeControlCallback(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
#endif
{
	attitude_flag_ = req.data;
}

#ifdef SIMULATION
bool AttitudeController::setAttitudeGainsCallback(spinal::SetAttitudeGains::Request& req, spinal::SetAttitudeGains::Response& res)
#else
void AttitudeController::setAttitudeGainsCallback(const spinal::SetAttitudeGains::Request& req, spinal::SetAttitudeGains::Response& res)
#endif
{
	attitude_p_gain_[X] = req.roll_pitch_p;
	attitude_i_gain_[X] = req.roll_pitch_i;
	attitude_d_gain_[X] = req.roll_pitch_d;
	attitude_p_gain_[Y] = req.roll_pitch_p;
	attitude_i_gain_[Y] = req.roll_pitch_i;
	attitude_d_gain_[Y] = req.roll_pitch_d;
	attitude_p_gain_[Z] = 0.0;
	attitude_i_gain_[Z] = 0.0;
	attitude_d_gain_[Z] = req.yaw_d;
	attitude_term_limit_[X] = req.roll_pitch_limit;
	attitude_term_limit_[Y] = req.roll_pitch_limit;
	attitude_p_term_limit_[X] = req.roll_pitch_p_limit;
	attitude_p_term_limit_[Y] = req.roll_pitch_p_limit;
	attitude_i_term_limit_[X] = req.roll_pitch_i_limit;
	attitude_i_term_limit_[Y] = req.roll_pitch_i_limit;
	attitude_d_term_limit_[X] = req.roll_pitch_d_limit;
	attitude_d_term_limit_[Y] = req.roll_pitch_d_limit;
	attitude_d_term_limit_[Z] = req.yaw_d_limit;

	res.success = true;
	attitude_gain_receive_flag_ = true;
#ifdef SIMULATION
	return true;
#endif
}

bool AttitudeController::activated()
{
  /* uav model check and motor property */
  if(motor_number_ > 0 && uav_model_ >= spinal::UavInfo::DRONE && max_duty_ > min_duty_) return true;
  else return false;
}

float AttitudeController::pwmConversion(float thrust)
{
  if(HAL_GetTick() - voltage_update_last_time_ > 500) //[500ms = 0.5s]
    {
#ifdef SIMULATION
      float voltage = motor_info_[0].voltage;
#else
      float voltage = bat_->getVoltage();
#endif
#ifdef SIMULATION
      //voltage = 25.2; //test
#endif

      /* find the best reference */
      float min_voltage_diff = 1e6;
      for(int i = 0; i < motor_info_.size(); i++)
        {
          float voltage_diff = fabs(voltage - motor_info_[i].voltage);
          if(min_voltage_diff > voltage_diff)
            {
              motor_ref_index_ = i;
              min_voltage_diff = voltage_diff;
            }
        }

      switch(pwm_conversion_mode_)
        {
        case spinal::MotorInfo::SQRT_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^2 f] */
            v_factor_ = (motor_info_[motor_ref_index_].voltage / voltage) *  (motor_info_[motor_ref_index_].voltage / voltage) ;
            break;
          }
        case spinal::MotorInfo::POLYNOMINAL_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^1.5 f] */
            v_factor_ = motor_info_[motor_ref_index_].voltage / voltage * inv_sqrt(voltage / motor_info_[motor_ref_index_].voltage);
            break;
          }
        default:
          {
            break;
          }
        }

      voltage_update_last_time_ = HAL_GetTick();
    }

  float target_pwm = IDLE_DUTY;
  switch(pwm_conversion_mode_)
    {
    case spinal::MotorInfo::SQRT_MODE:
      {
        /* pwm = F_inv[(V_ref / V)^2 f] */
        float sqrt_tmp = motor_info_[motor_ref_index_].polynominal[1] * motor_info_[motor_ref_index_].polynominal[1] - 4 * 10 * motor_info_[motor_ref_index_].polynominal[2] * (motor_info_[motor_ref_index_].polynominal[0] - v_factor_ * thrust); //special decimal order shift (x10)
        target_pwm = (-motor_info_[motor_ref_index_].polynominal[1] + sqrt_tmp * inv_sqrt(sqrt_tmp)) / (2 * motor_info_[motor_ref_index_].polynominal[2]);
        break;
      }
    case spinal::MotorInfo::POLYNOMINAL_MODE:
      {
        /* pwm = F_inv[(V_ref / V)^1.5 f] */
        float v_factor_thrust = v_factor_ * thrust > motor_info_[motor_ref_index_].max_thrust?motor_info_[motor_ref_index_].max_thrust: v_factor_ * thrust;
        float v_factor_thrust_decimal = v_factor_thrust * 0.1f; //special decimal order shift (x0.1)
        /* hardcode: 4 dimensional */
        int max_dimenstional = 4;
        target_pwm = motor_info_[motor_ref_index_].polynominal[max_dimenstional];
        for (int j = max_dimenstional - 1; j >= 0; j--)
          {
            target_pwm = target_pwm * v_factor_thrust_decimal + motor_info_[motor_ref_index_].polynominal[j];
          }
        break;
      }
    default:
      {
        break;
      }
    }
  return target_pwm / 100; // target_pwm is [%] -> decimal
}
