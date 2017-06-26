/*
 * Copyright (c) 2017 Lucas Walter
 * June 2017
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OPEN_SIMPLEX_NOISE_ROS_OPEN_SIMPLEX_NOISE_H
#define OPEN_SIMPLEX_NOISE_ROS_OPEN_SIMPLEX_NOISE_H

#include <dynamic_reconfigure/server.h>
#include <open_simplex_noise_ros/open-simplex-noise.h>
#include <open_simplex_noise_ros/OpenSimplexNoiseConfig.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace open_simplex_noise_ros
{

class OpenSimplexNoise : public nodelet::Nodelet
{
  ros::Publisher pub_;
  open_simplex_noise_ros::OpenSimplexNoiseConfig config_;
  typedef dynamic_reconfigure::Server<open_simplex_noise_ros::OpenSimplexNoiseConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(open_simplex_noise_ros::OpenSimplexNoiseConfig& config,
      uint32_t level);

  boost::recursive_mutex dr_mutex_;

  void update(const ros::TimerEvent& e);

  ros::Timer timer_;

  struct osn_context *ctx_;
public:
  virtual void onInit();
  OpenSimplexNoise();
  ~OpenSimplexNoise();
};

}  // namespace open_simplex_noise_ros

#endif  // OPEN_SIMPLEX_NOISE_ROS_OPEN_SIMPLEX_NOISE_H
