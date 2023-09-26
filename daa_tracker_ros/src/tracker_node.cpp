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

#include <glog/logging.h>
#include <ros/package.h>

#include "tracker_node.h"

#include <multi_hyp_tracker/MultiHypTracker.h>
#include <daa_tracker_ros/TrackArray.h>

namespace daa_ros
{
TrackerNode::TrackerNode() : tracker_(daa::multi_hyp_tracker::MultiHypTracker::create()), nh_priv_("~"), it_(nh_priv_)
{
  // Params
  bool use_color_classification;
  nh_priv_.param<bool>("use_color_classification", use_color_classification, false);
  nh_priv_.param<std::string>("odom_frame", odom_frame_, "odom");

  // TODO: use ROS parameters instead of config file
  //  The best way to do this is probably to make a subclass of Client and override get_param, get_prior,
  //  get_class_name and get_behavioral_model with calls to ros::NodeHandle::getParamCached()?
  string config_filename;
  if (!nh_priv_.getParam("config_filename", config_filename))
  {
    ROS_FATAL("Parameter config_filename not found!");
    ros::shutdown();
    return;
  }
  daa::utils::TrackerConfig::instance().load_config(config_filename);

  // Publisher
  track_pub_ = std::make_shared<ros::Publisher>(nh_priv_.advertise<daa_tracker_ros::TrackArray>("tracks", 1000));
  image_rgb_pub_ = it_.advertise("anchoring_results/image", 1);
  // Subscribers
  image_rgb_sub_.subscribe(it_, "image_rect_color", 100);
  camera_info_sub_.subscribe(nh_priv_, "camera_info", 100);
  detected_objs_sub_.subscribe(nh_priv_, "detected_objects", 100);

  if (use_color_classification)
  {
    color_classif_sub_.subscribe(nh_priv_, "color_classification", 100);
    sync_with_color_ = std::make_shared<ApproximateColorSync>(ApproximateColorPolicy(100), image_rgb_sub_,
                                                              camera_info_sub_, detected_objs_sub_, color_classif_sub_);
    sync_with_color_->registerCallback(
        boost::bind(&TrackerNode::TrackerNode::image_color_callback, this, _1, _2, _3, _4));
  }
  else
  {
    sync_ =
        std::make_shared<ApproximateSync>(ApproximatePolicy(100), image_rgb_sub_, camera_info_sub_, detected_objs_sub_);
    sync_->registerCallback(boost::bind(&TrackerNode::TrackerNode::image_callback, this, _1, _2, _3));
  }
}

void TrackerNode::image_callback(const sensor_msgs::ImageConstPtr& image_rgb,
                                 const sensor_msgs::CameraInfoConstPtr& cam_info,
                                 const vision_msgs::Detection3DArrayConstPtr& detected_objs)
{
  image_color_callback(image_rgb, cam_info, detected_objs, nullptr);
}

void TrackerNode::image_color_callback(
    const sensor_msgs::ImageConstPtr& image_rgb, const sensor_msgs::CameraInfoConstPtr& cam_info,
    const vision_msgs::Detection3DArrayConstPtr& detected_objs,
    const daa_color_classification::ColorClassificationResultsConstPtr& color_classifications)
{
  camera_model_.fromCameraInfo(cam_info);
  // convert image to OpenCV
  cv_bridge::CvImageConstPtr const cv_ptr_rgb = cv_bridge::toCvShare(image_rgb, image_rgb->encoding);
  cv::Mat image_color_bgr;
  if (image_rgb->encoding == sensor_msgs::image_encodings::RGB8)
  {
    cv::cvtColor(cv_ptr_rgb->image, image_color_bgr, CV_RGB2BGR);
  }
  else if (image_rgb->encoding == sensor_msgs::image_encodings::MONO8)
  {
    cv::cvtColor(cv_ptr_rgb->image, image_color_bgr, CV_GRAY2BGR);
  }
  else if (image_rgb->encoding == sensor_msgs::image_encodings::BGR8)
  {
    // image is already in BGR encoding, no conversion necessary
    image_color_bgr = cv_ptr_rgb->image;
  }
  else
  {
    ROS_ERROR("Unknown encoding: %s", image_rgb->encoding.c_str());
    return;
  }

  // construct scan
  auto scan = daa::multi_hyp_tracker::observables::Scan(detected_objs->header.stamp.toSec());
  scan.set_image(image_color_bgr);
  assert(color_classifications == nullptr || color_classifications->colors.size() == detected_objs->detections.size());
  size_t det_id = 0;
  for (auto det : detected_objs->detections)
  {
    // TODO: use object detection instead of bbox center?
    auto bbox_center_3d = det.bbox.center.position;
    auto bbox_center_uv =
        camera_model_.project3dToPixel(cv::Point3d(bbox_center_3d.x, bbox_center_3d.y, bbox_center_3d.z));
    auto bbox_orientation = det.bbox.center.orientation;
    auto class_id = det.results.front().id;
    auto pose = det.results.front().pose.pose;
    auto position = pose.position;
    auto orientation = pose.orientation;

    // transform the pos to map coordinates
    std::string const target_frame = odom_frame_;
    std::string const source_frame = detected_objs->header.frame_id;
    auto stamp = detected_objs->header.stamp;
    tf::Point const tf_point(bbox_center_3d.x, bbox_center_3d.y, bbox_center_3d.z);
    tf::Stamped<tf::Point> const stamped_tf_point_in(tf_point, stamp, source_frame);
    tf::Stamped<tf::Point> stamped_tf_point_out;
    geometry_msgs::QuaternionStamped stamped_tf_quat_in;
    geometry_msgs::QuaternionStamped stamped_tf_quat_out;
    stamped_tf_quat_in.header = detected_objs->header;
    stamped_tf_quat_in.quaternion = orientation;
    tf_.waitForTransform(target_frame, source_frame, stamp, ros::Duration().fromSec(0.5));
    try
    {
      tf_.transformPoint(target_frame, stamped_tf_point_in, stamped_tf_point_out);
      tf_.transformQuaternion(target_frame, stamped_tf_quat_in, stamped_tf_quat_out);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      continue;
    }

    // construct observation
    auto pos = Eigen::Vector3d(stamped_tf_point_out.getX(), stamped_tf_point_out.getY(), stamped_tf_point_out.getZ());
    auto quat = Eigen::Vector4d(stamped_tf_quat_out.quaternion.x, stamped_tf_quat_out.quaternion.y,
                                stamped_tf_quat_out.quaternion.z, stamped_tf_quat_out.quaternion.w);
    auto pos_2d = Eigen::Vector2d(bbox_center_uv.x, bbox_center_uv.y);

    daa::multi_hyp_tracker::observables::Observation obs;
    obs.add_observable(daa::multi_hyp_tracker::observables::Position(pos));
    obs.add_observable(daa::multi_hyp_tracker::observables::ImageUV(pos_2d));
    obs.add_observable(daa::multi_hyp_tracker::observables::ClassID(class_id));
    obs.add_observable(daa::multi_hyp_tracker::observables::Orientation(quat));
    if (color_classifications != nullptr)
    {
      // convert from vector<float> to vector<double>
      auto intensities = color_classifications->colors[det_id].intensities;
      std::vector<double> color_hist(intensities.begin(), intensities.end());

      obs.add_observable(daa::multi_hyp_tracker::observables::ColorHistogram(color_hist));
    }
    scan.add_observation(obs);
    det_id++;
  }

  // process scan
  bool processed = tracker_->process_scan(scan);
  if (processed)
  {
    auto tracks = tracker_->get_tracks();
    if (!tracks.empty())
    {
      daa_tracker_ros::TrackArray track_array_msg;
      track_array_msg.header = detected_objs->header;
      for (auto t : tracks)
      {
        daa_tracker_ros::Track track_msg;
        track_msg.track_id = t->item()->track_id();
        track_msg.percepts = t->item()->to_dict().dump();
        track_array_msg.tracks.push_back(track_msg);
      }
      track_pub_->publish(track_array_msg);
    }
  }
}

} /* namespace daa_ros */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "daa_tracker_node");

  FLAGS_alsologtostderr = true;
  FLAGS_v = 0;
  // google::EnableLogCleaner(1);
  google::InitGoogleLogging(argv[0]);
  daa_ros::TrackerNode const tracker_node;
  ros::spin();

  return 0;
}
