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

#include <cv_bridge/cv_bridge.h>
#include <open_simplex_noise_ros/open_simplex_noise.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


namespace open_simplex_noise_ros
{

OpenSimplexNoise::OpenSimplexNoise() :
  ctx_(NULL)
{
}

OpenSimplexNoise::~OpenSimplexNoise()
{
  if (ctx_)
    open_simplex_noise_free(ctx_);
}


void OpenSimplexNoise::callback(
    open_simplex_noise_ros::OpenSimplexNoiseConfig& config,
    uint32_t level)
{
  config_ = config;
}

void OpenSimplexNoise::update(const ros::TimerEvent& e)
{
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = "mono8";
  cv_image.image = cv::Mat(cv::Size(config_.width, config_.height), CV_8UC1);

  for (size_t y = 0; y < config_.height; ++y)
  {
    for (size_t x = 0; x < config_.width; ++x)
    {
      const double value = open_simplex_noise3(ctx_,
          static_cast<double>(x) / config_.feature_size_x,
          static_cast<double>(y) / config_.feature_size_y,
          cv_image.header.stamp.toSec() / config_.feature_size_t);
      cv_image.image.at<unsigned char>(y, x) = value * config_.scale + config_.offset;
    }
  }

  sensor_msgs::ImagePtr msg = cv_image.toImageMsg();
  pub_.publish(msg);
}

void OpenSimplexNoise::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image", 5);

  server_.reset(new ReconfigureServer(dr_mutex_, getPrivateNodeHandle()));
  dynamic_reconfigure::Server<open_simplex_noise_ros::OpenSimplexNoiseConfig>::CallbackType cbt =
      boost::bind(&OpenSimplexNoise::callback, this, _1, _2);
  server_->setCallback(cbt);

  int seed = 77374;
  getPrivateNodeHandle().getParam("seed", seed);
  open_simplex_noise(seed, &ctx_);

  // TODO(lucasw) move to update function
  const float period = 1.0 / config_.frame_rate;
  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(period),
      &OpenSimplexNoise::update, this);
}

};  // namespace open_simplex_noise_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(open_simplex_noise_ros::OpenSimplexNoise, nodelet::Nodelet)
