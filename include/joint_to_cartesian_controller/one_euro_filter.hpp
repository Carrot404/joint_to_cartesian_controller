// Copyright (c) 2025
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * One Euro Filter Implementation
 * Reference: https://gery.casiez.net/1euro/
 */

#ifndef NANO25_CONTROLLERS__ONE_EURO_FILTER_HPP_
#define NANO25_CONTROLLERS__ONE_EURO_FILTER_HPP_

#include <cmath>
#include <limits>

namespace nano_controllers
{

/**
 * @brief Low-pass filter using exponential smoothing
 */
class LowPassFilter
{
public:
  LowPassFilter() : initialized_(false), y_prev_(0.0) {}

  double filter(double x, double alpha)
  {
    if (!initialized_)
    {
      initialized_ = true;
      y_prev_ = x;
      return x;
    }

    double y = alpha * x + (1.0 - alpha) * y_prev_;
    y_prev_ = y;
    return y;
  }

  void reset()
  {
    initialized_ = false;
    y_prev_ = 0.0;
  }

private:
  bool initialized_;
  double y_prev_;
};

/**
 * @brief One Euro Filter for signal smoothing
 *
 * Adaptive low-pass filter that adjusts cutoff frequency based on signal velocity.
 * Good for smoothing noisy signals while maintaining responsiveness to rapid changes.
 *
 * Parameters:
 *   - min_cutoff: Minimum cutoff frequency (Hz). Lower values = more smoothing
 *   - beta: Speed coefficient. Higher values = more responsive to velocity changes
 *   - d_cutoff: Derivative cutoff frequency (Hz). Used for smoothing velocity estimate
 */
class OneEuroFilter
{
public:
  /**
   * @brief Default constructor (for use in containers)
   */
  OneEuroFilter()
      : freq_(125.0),
        min_cutoff_(1.0),
        beta_(0.0),
        d_cutoff_(1.0),
        initialized_(false),
        x_prev_(0.0),
        dx_prev_(0.0)
  {
  }

  /**
   * @brief Constructor
   * @param freq Update frequency in Hz
   * @param min_cutoff Minimum cutoff frequency in Hz (default: 1.0)
   * @param beta Speed coefficient (default: 0.0)
   * @param d_cutoff Derivative cutoff frequency in Hz (default: 1.0)
   */
  OneEuroFilter(double freq, double min_cutoff = 1.0, double beta = 0.0, double d_cutoff = 1.0)
      : freq_(freq),
        min_cutoff_(min_cutoff),
        beta_(beta),
        d_cutoff_(d_cutoff),
        initialized_(false),
        x_prev_(0.0),
        dx_prev_(0.0)
  {
  }

  /**
   * @brief Update filter parameters
   * @param freq Update frequency in Hz
   * @param min_cutoff Minimum cutoff frequency in Hz
   * @param beta Speed coefficient
   * @param d_cutoff Derivative cutoff frequency in Hz
   */
  void setParameters(double freq, double min_cutoff, double beta, double d_cutoff)
  {
    freq_ = freq;
    min_cutoff_ = min_cutoff;
    beta_ = beta;
    d_cutoff_ = d_cutoff;
  }

  /**
   * @brief Filter a new value
   * @param x Input value
   * @return Filtered output
   */
  double filter(double x)
  {
    if (!initialized_)
    {
      initialized_ = true;
      x_prev_ = x;
      dx_prev_ = 0.0;
      return x;
    }

    // Estimate derivative
    double dx = (x - x_prev_) * freq_;

    // Smooth the derivative
    double alpha_d = computeAlpha(d_cutoff_);
    double dx_smoothed = dx_filter_.filter(dx, alpha_d);

    // Adaptive cutoff frequency based on velocity
    double cutoff = min_cutoff_ + beta_ * std::abs(dx_smoothed);

    // Smooth the signal
    double alpha = computeAlpha(cutoff);
    double y = x_filter_.filter(x, alpha);

    // Update previous values
    x_prev_ = x;
    dx_prev_ = dx_smoothed;

    return y;
  }

  /**
   * @brief Reset filter state
   */
  void reset()
  {
    initialized_ = false;
    x_prev_ = 0.0;
    dx_prev_ = 0.0;
    x_filter_.reset();
    dx_filter_.reset();
  }

private:
  /**
   * @brief Compute smoothing factor alpha from cutoff frequency
   * @param cutoff Cutoff frequency in Hz
   * @return Alpha value [0, 1]
   */
  double computeAlpha(double cutoff) const
  {
    double tau = 1.0 / (2.0 * M_PI * cutoff);
    double te = 1.0 / freq_;
    return 1.0 / (1.0 + tau / te);
  }

  double freq_;        // Update frequency in Hz
  double min_cutoff_;  // Minimum cutoff frequency
  double beta_;        // Speed coefficient
  double d_cutoff_;    // Derivative cutoff frequency

  bool initialized_;
  double x_prev_;      // Previous input value
  double dx_prev_;     // Previous derivative

  LowPassFilter x_filter_;   // Filter for signal
  LowPassFilter dx_filter_;  // Filter for derivative
};

} // namespace nano_controllers

#endif // NANO25_CONTROLLERS__ONE_EURO_FILTER_HPP_
