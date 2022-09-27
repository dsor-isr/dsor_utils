// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software;
// This code was based on the WPILib library. Visit the following link for more information:
// https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/_slew_rate_limiter_8h_source.html
 
#pragma once

#include <ros/ros.h> 
 
/**
 * A class that limits the rate of change of an input value. Limits the
 * first derivative of the signal passing through it. The output changes
 * no faster than the specified limit.
 */
class SlewRateLimiter {
 public:
 
  /**
   * Creates a new SlewRateLimiter with the given rate limit and initial value.
   *
   * @param rateLimit The rate-of-change limit.
   * @param initialValue The initial value of the input.
   */
  explicit SlewRateLimiter(double rateLimit, double initialValue)
      : m_rateLimit{rateLimit},
        m_prevVal_{initialValue},
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

    m_prevVal_ += std::clamp(input - m_prevVal_, -m_rateLimit * elapsedTime,
                            m_rateLimit * elapsedTime);

    m_prevTime_ = current_time;

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
  double m_rateLimit;
  double m_prevVal_;
  ros::Time m_prevTime_;
};
