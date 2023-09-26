/*
 * Copyright (c) 2023, DFKI GmbH and contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <ros/ros.h>
#include <deque>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <vision_msgs/Detection3DArray.h>
#include <daa_color_classification/ColorClassificationResults.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <multi_hyp_tracker/MultiHypTracker.h>

namespace daa_ros
{
class TrackerNode
{
public:
  TrackerNode();

private:
  daa::multi_hyp_tracker::MultiHypTracker::Ptr tracker_;
  //   std::deque<daa::multi_hyp_tracker::observables::Scan> scan_buffer_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // this needs to be a member variable to take advantage of the internal caches
  image_geometry::PinholeCameraModel camera_model_;
  std::string odom_frame_;

  tf::TransformListener tf_;
  std::shared_ptr<ros::Publisher> track_pub_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_rgb_pub_;
  image_transport::SubscriberFilter image_rgb_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<vision_msgs::Detection3DArray> detected_objs_sub_;
  message_filters::Subscriber<daa_color_classification::ColorClassificationResults> color_classif_sub_;

  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                                            vision_msgs::Detection3DArray>;

  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ApproximateSync> sync_;
  void image_callback(const sensor_msgs::ImageConstPtr& image_rgb, const sensor_msgs::CameraInfoConstPtr& cam_info,
                      const vision_msgs::Detection3DArrayConstPtr& detected_objs);

  using ApproximateColorPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                      vision_msgs::Detection3DArray,
                                                      daa_color_classification::ColorClassificationResults>;

  using ApproximateColorSync = message_filters::Synchronizer<ApproximateColorPolicy>;
  std::shared_ptr<ApproximateColorSync> sync_with_color_;
  void image_color_callback(const sensor_msgs::ImageConstPtr& image_rgb,
                            const sensor_msgs::CameraInfoConstPtr& cam_info,
                            const vision_msgs::Detection3DArrayConstPtr& detected_objs,
                            const daa_color_classification::ColorClassificationResultsConstPtr& color_classifications);
};

}  // namespace daa_ros
