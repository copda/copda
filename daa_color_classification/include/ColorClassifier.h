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

/**
 * @file ColorClassifier.h
 * @brief Definition of the ColorClassification class
 * @author Zongyao Yi
 */

#pragma once

#include <memory>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <vision_msgs/Detection3DArray.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include <daa_color_classification/ColorClassificationConfig.h>
#include <daa_color_classification/ColorClassificationResults.h>

using namespace sensor_msgs;
using namespace vision_msgs;
using namespace message_filters;
using namespace std;

namespace Color
{
enum Type
{
  TRAFFIC_RED = 0,
  TRAFFIC_YELLOW = 24,
  YELLOW_GREEN = 57,
  TRAFFIC_GREEN = 78,
  BLUE = 118,
  PURPLE = 148,
  TELEMAGENTA = 163,
  UNDEFINED = 255
};
static const Type All[] = { TRAFFIC_RED, TRAFFIC_YELLOW, YELLOW_GREEN, TRAFFIC_GREEN,
                            BLUE,        PURPLE,         TELEMAGENTA,  UNDEFINED };
static string colorTypeToString(Type color)
{
  string color_str;

  switch (color)
  {
    case TRAFFIC_RED:
      color_str = "Traffic Red";
      break;
    case TRAFFIC_YELLOW:
      color_str = "Traffic Yellow";
      break;
    case YELLOW_GREEN:
      color_str = "Yellow Green";
      break;
    case TRAFFIC_GREEN:
      color_str = "Traffic Green";
      break;
    case BLUE:
      color_str = "Blue";
      break;
    case PURPLE:
      color_str = "Purple";
      break;
    case TELEMAGENTA:
      color_str = "Telemagenta";
      break;
    default:
      color_str = "Undefined";
      break;
  }

  return color_str;
}

}  // namespace Color

class ColorClassifier
{
public:
  explicit ColorClassifier(ros::NodeHandle* pNh);
  ~ColorClassifier() = default;
  inline void spin()
  {
    pSpinner_->start();
  }

private:
  /* callback functions */
  void image_callback(const ImageConstPtr& seg_image, const ImageConstPtr& rgb_image,
                      const Detection3DArrayConstPtr& detected_objs, const CameraInfoConstPtr& cam_info);
  void parameter_callback(daa_color_classification::ColorClassificationConfig& config, uint32_t level);

  const string CV_WINDOW_RGB = "Color Classiflier RGB";
  const string CV_WINDOW_MASK = "Color Classiflier Mask";

  ros::CallbackQueue callback_queue_;  // todo: not used
  shared_ptr<ros::AsyncSpinner> pSpinner_;
  shared_ptr<ros::NodeHandle> pNh_;

  image_transport::ImageTransport it_;

  /* image subscribers and time synchronizer */
  shared_ptr<message_filters::Subscriber<Image>> pSeg_image_sub_;
  shared_ptr<message_filters::Subscriber<Image>> pRgb_image_sub_;
  shared_ptr<message_filters::Subscriber<Detection3DArray>> pObj_detect_sub_;
  shared_ptr<message_filters::Subscriber<CameraInfo>> pCam_info_sub_;
  shared_ptr<message_filters::TimeSynchronizer<Image, Image, Detection3DArray, CameraInfo>> pSync_;

  /* publishers */
  shared_ptr<ros::Publisher> pResult_pub_;
  image_transport::Publisher image_pub_;

  /* parameter server, for dynamic parameter reconfiguration */
  shared_ptr<dynamic_reconfigure::Server<daa_color_classification::ColorClassificationConfig>> pParam_server_;

  /* a look up table for the histogram */
  vector<Color::Type> color_table_;

  /* manager of all parameters */
  daa_color_classification::ColorClassificationConfig config_;

  /* functions for image processing */
  void preProcessImage(const cv::Mat& imageIn, cv::Mat& imageOut) const;

  static void extractPixels(const cv::Mat& image, vector<cv::Mat>& vecResult, vector<cv::Rect>& bboxes,
                            const cv::Mat& mask, size_t obj_num);

  void createHistograms(const vector<cv::Mat>& vecImages, vector<cv::Mat>& vecHist);

  void createHistogram(const cv::Mat& image, cv::Mat& hist, const cv::Mat& mask = cv::Mat());

  void classifyColor(const vector<cv::Mat>& histograms, vector<vector<Color::Type>>& all_object_colors,
                     vector<vector<float>>& all_object_intensities);

  /* function for publishing the classification results */
  void publishResults(const cv_bridge::CvImageConstPtr& rgb_image_ptr,
                      const vector<vector<Color::Type>>& all_object_colors,
                      const vector<vector<float>>& all_object_intensities, const vector<cv::Mat>& obj_imgs_hsv,
                      const vector<cv::Rect>& bboxes, const vector<cv::Mat>& histograms);

  /* auxiliary functions */
  void visualizeHist(const vector<cv::Mat>& vecHist, int scale = 10);

  void visualizeHist(const cv::Mat& hist, int scale = 10, const string& window_name = "Histogram") const;

  void createBlackPixelMask(const cv::Mat& hsv_imageIn, cv::Mat& maskOut) const;

  void constructColorTable();

  static cv::Scalar convertScalarHSV2BGR(uint8_t H, uint8_t S, uint8_t V);
};
