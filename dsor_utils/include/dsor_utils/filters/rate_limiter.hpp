// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software;
// License:  WPILib BSD license (https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/md__home_runner_work_allwpilib_allwpilib__l_i_c_e_n_s_e.html)
// This code was based on the WPILib library. Visit the following link for more information:
// https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/_slew_rate_limiter_8h_source.html
 
#pragma once

#include <ros/ros.h> 
#include <dsor_utils/rotations.hpp>

/**
 * A class that limits the rate of change of an input value. Limits the
 * first derivative of the signal passing through it. The output changes
 * no faster than the specified limit.
 */
class RateLimiter {
 public:
 
  /**
   * Creates a new RateLimiter.
   *
   */
  RateLimiter() {}
 

  /**
   * Creates a new RateLimiter with the given rate limit and initial value.
   *
   * @param initialValue The initial value of the input.
   */
  RateLimiter(double initialValue, bool using_circular_units=false)
      : m_prevVal_{initialValue}, using_circular_units_{using_circular_units},
        m_prevTime_{ros::Time::now()} {}
 
  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew
   * rate.
   */
  double Calculate(double input) {
    ros::Time current_time = ros::Time::now();
    double elapsedTime = (current_time - m_prevTime_).toSec();
    
    if (using_circular_units_) {
      
      m_prevVal_ += std::clamp(DSOR::angleDiffDegrees<double>(input, m_prevVal_), -m_rateLimit_ * elapsedTime,
                            m_rateLimit_ * elapsedTime);
    }
    else {
      m_prevVal_ += std::clamp(input - m_prevVal_, -m_rateLimit_ * elapsedTime,
                            m_rateLimit_ * elapsedTime);
    }
    

    m_prevTime_ = current_time;

    return m_prevVal_;
  }

  /**
   * @brief Set the New Rate Limit value
   * 
   * @param rate_limit 
   */
  void setNewRateLimit(double rate_limit) {
    m_rateLimit_ = rate_limit;
  }

  /**
   * @brief Get the current Rate Limit value
   */
  double getRateLimit() {
    return m_rateLimit_;
  }

  /**
   * @brief Get the current previous value
   * 
   */
  double getPreviousValue() {
    return m_prevVal_;
  }
 
  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit
   * when doing so.
   *
   * @param value The value to reset to.
   */
  void Reset(double value) {
    m_prevVal_ = value;
    m_prevTime_ = ros::Time::now();
  }
 
 private:
  bool using_circular_units_;
  double m_rateLimit_{0.0};
  double m_prevVal_;
  ros::Time m_prevTime_;
};
