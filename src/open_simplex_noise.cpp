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

  const double ti = cv_image.header.stamp.toSec();
  const double wd = config_.width;
  const double ht = config_.width;

  std::vector<double> scale;
  scale.push_back(config_.scale0);
  scale.push_back(config_.scale1);
  scale.push_back(config_.scale2);
  scale.push_back(config_.scale3);

  std::vector<double> offset;
  offset.push_back(config_.offset0);
  offset.push_back(config_.offset1);
  offset.push_back(config_.offset2);
  offset.push_back(config_.offset3);

  std::vector<double> feature_size_x;
  feature_size_x.push_back(config_.feature_size_x0);
  feature_size_x.push_back(config_.feature_size_x1);
  feature_size_x.push_back(config_.feature_size_x2);
  feature_size_x.push_back(config_.feature_size_x3);

  std::vector<double> feature_size_y;
  feature_size_y.push_back(config_.feature_size_y0);
  feature_size_y.push_back(config_.feature_size_y1);
  feature_size_y.push_back(config_.feature_size_y2);
  feature_size_y.push_back(config_.feature_size_y3);

  std::vector<double> feature_size_t;
  feature_size_y.push_back(config_.feature_size_t0);
  feature_size_y.push_back(config_.feature_size_t1);
  feature_size_y.push_back(config_.feature_size_t2);
  feature_size_y.push_back(config_.feature_size_t3);

  std::vector<double> offset_x;
  offset_x.push_back(config_.offset_x0);
  offset_x.push_back(config_.offset_x1);
  offset_x.push_back(config_.offset_x2);
  offset_x.push_back(config_.offset_x3);

  std::vector<double> offset_y;
  offset_y.push_back(config_.offset_y0);
  offset_y.push_back(config_.offset_y1);
  offset_y.push_back(config_.offset_y2);
  offset_y.push_back(config_.offset_y3);

  for (size_t y = 0; y < config_.height; ++y)
  {
    for (size_t x = 0; x < config_.width; ++x)
    {
      // TODO(lucasw) get 4 from cfg
      double value = 0;
      const double xd = static_cast<double>(x);
      const double yd = static_cast<double>(y);

      // it looks like the dynamic reconfigure c++ generation doesn't make
      // this easier- or is there a string access method?
      for (size_t i = 0; i < scale.size(); ++i)
      {
        if (scale[i] != 0.0)
        {
          const double vi = open_simplex_noise3(ctx_,
            (xd + offset_x[i] * wd) / feature_size_x[i],
            (yd + offset_y[i] * ht) / feature_size_y[i],
            ti / config_.feature_size_t0);
          value += vi * scale[i];
        }
        value += offset[i];
      }

      if (config_.clip)
      {
        if (value > 255) value = 255;
        if (value < 0) value = 0;
      }
      cv_image.image.at<unsigned char>(y, x) = value;
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
