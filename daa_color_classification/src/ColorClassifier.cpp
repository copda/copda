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
 * @file ColorClassifier.cpp
 * @brief Implementation of the ColorClassification class
 * @author Zongyao Yi
 */

#include <vision_msgs/BoundingBox2D.h>
#include <daa_color_classification/Colors.h>
#include "ColorClassifier.h"

/**
 * @brief constructor: initialize message subscribers, time sychronizer,
 * parameter dynamic reconfigure server, and publisher; Construct the color look up table;
 * @param pNh nodehandle pointer provided by the process that invokes the class
 */
ColorClassifier::ColorClassifier(ros::NodeHandle* pNh) : pNh_(pNh), it_(*pNh)
{
  pSeg_image_sub_ = make_shared<message_filters::Subscriber<Image>>(*pNh_, "/mobipick/dope/instance_seg_image", 1);
  pRgb_image_sub_ =
      make_shared<message_filters::Subscriber<Image>>(*pNh_, "/mobipick/eef_main_cam/rgb/image_rect_color", 1);
  pObj_detect_sub_ =
      make_shared<message_filters::Subscriber<Detection3DArray>>(*pNh_, "/mobipick/dope/detected_objects", 1);
  pCam_info_sub_ =
      make_shared<message_filters::Subscriber<CameraInfo>>(*pNh_, "/mobipick/eef_main_cam/rgb/camera_info", 1);

  pSync_ = std::make_shared<message_filters::TimeSynchronizer<Image, Image, Detection3DArray, CameraInfo>>(
      *pSeg_image_sub_, *pRgb_image_sub_, *pObj_detect_sub_, *pCam_info_sub_, 10);
  pSync_->registerCallback(boost::bind(&ColorClassifier::image_callback, this, _1, _2, _3, _4));

  pResult_pub_ = make_shared<ros::Publisher>(
      pNh->advertise<daa_color_classification::ColorClassificationResults>("/mobipick/color_classification", 1));
  image_pub_ = it_.advertise("debug_image", 10);

  pParam_server_ = std::make_shared<dynamic_reconfigure::Server<daa_color_classification::ColorClassificationConfig>>();
  pParam_server_->setCallback(boost::bind(&ColorClassifier::parameter_callback, this, _1, _2));

  constructColorTable();

  pSpinner_ = std::make_shared<ros::AsyncSpinner>(1);
}

/**
 * @brief Image callback function: all processings happen here
 */
void ColorClassifier::image_callback(const ImageConstPtr& seg_image, const ImageConstPtr& rgb_image,
                                     const Detection3DArrayConstPtr& detected_objs, const CameraInfoConstPtr& cam_info)
{
  cv_bridge::CvImagePtr seg_cv_ptr;
  cv_bridge::CvImagePtr rgb_cv_ptr;
  cv::Mat hsv_image;
  vector<cv::Mat> vecObj;
  vector<cv::Rect> bboxes;
  vector<cv::Mat> vecHist;
  vector<vector<Color::Type>> all_object_colors;
  vector<vector<float>> all_object_intensities;
  try
  {
    seg_cv_ptr = cv_bridge::toCvCopy(seg_image);
    rgb_cv_ptr = cv_bridge::toCvCopy(rgb_image);

    // align rgb image with the segmentation image size
    cv::resize(rgb_cv_ptr->image, rgb_cv_ptr->image, cv::Size(seg_cv_ptr->image.cols, seg_cv_ptr->image.rows));

    preProcessImage(rgb_cv_ptr->image, hsv_image);

    extractPixels(hsv_image, vecObj, bboxes, seg_cv_ptr->image, detected_objs->detections.size());

    createHistograms(vecObj, vecHist);

    classifyColor(vecHist, all_object_colors, all_object_intensities);

    visualizeHist(vecHist, 10);

    publishResults(rgb_cv_ptr, all_object_colors, all_object_intensities, vecObj, bboxes, vecHist);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

/**
 * @brief Dynamic parameter reconfigure callback
 */
void ColorClassifier::parameter_callback(daa_color_classification::ColorClassificationConfig& config, uint32_t level)
{
  config_ = config;

  // level flag "1" is set: hBins or color_tolerance have changed
  if ((level & 1) != 0)
  {
    constructColorTable();
  }
}

/**
 * @brief Remove overexposed, underexposed and blue pixels
 */
void ColorClassifier::preProcessImage(const cv::Mat& imageIn, cv::Mat& imageOut) const
{
  cv::Mat hsv_image;
  cv::cvtColor(imageIn, hsv_image, cv::COLOR_RGB2HSV);

  cv::Mat mask = cv::Mat::ones(hsv_image.rows, hsv_image.cols, CV_8U) * 255;

  // create masks
  {
    cv::Mat mask_V;
    auto min_V = cv::Scalar(0, 0, config_.min_V);
    auto max_V = cv::Scalar(255, 255, config_.max_V);
    cv::inRange(hsv_image, min_V, max_V, mask_V);
    cv::bitwise_and(mask_V, mask, mask);
  }

  {
    cv::Mat mask_S;
    auto min_S = cv::Scalar(0, config_.min_S, 0);
    auto max_S = cv::Scalar(255, 255, 255);
    cv::inRange(hsv_image, min_S, max_S, mask_S);
    cv::bitwise_and(mask_S, mask, mask);
  }

  {
    cv::Mat mask_blue;
    auto lower_blue = cv::Scalar(100, 50, 0);
    auto upper_blue = cv::Scalar(122, 255, 255);
    cv::inRange(hsv_image, lower_blue, upper_blue, mask_blue);
    cv::bitwise_not(mask_blue, mask_blue);
    cv::bitwise_and(mask_blue, mask, mask);
  }

  // apply mask
  cv::bitwise_and(hsv_image, hsv_image, imageOut, mask);
}

/**
 * @brief Extract pixels belonging to the same instance, according to the segmentation mask
 * @param mask[In] the segmentation mask
 * @param vecResult[Out]
 */
void ColorClassifier::extractPixels(const cv::Mat& image, vector<cv::Mat>& vecResult, vector<cv::Rect>& bboxes,
                                    const cv::Mat& mask, const size_t obj_num)
{
  vecResult.clear();
  bboxes.clear();

  // extract pixels belonging to the same object instance
  for (size_t obj_id = 0; obj_id < obj_num; obj_id++)
  {
    cv::Mat obj_mask;
    obj_mask.create(mask.rows, mask.cols, mask.type());
    // iterate over pixels and extract individual object mask
    for (int row = 0; row < mask.rows; row++)
    {
      for (int col = 0; col < mask.cols; col++)
      {
        if (mask.at<uchar>(row, col) == (uchar)obj_id)
        {
          obj_mask.at<uchar>(row, col) = 255;
        }
        else
        {
          obj_mask.at<uchar>(row, col) = 0;
        }
      }
    }

    cv::Mat result;

    // calculate bbox
    cv::Rect bbox = cv::boundingRect(obj_mask);
    if (bbox.empty())
    {
      // this can happen when an erroneous object detection completely occludes a different object
      ROS_WARN("bbox is empty!");
      bbox.height = 1;
      bbox.width = 1;
      result = cv::Mat::zeros(1, 1, image.type());
    }
    else
    {
      // apply the mask on the image
      cv::bitwise_and(image(bbox), image(bbox), result, obj_mask(bbox));
    }

    vecResult.push_back(result);
    bboxes.push_back(bbox);
  }
}

void ColorClassifier::createHistograms(const vector<cv::Mat>& vecImages, vector<cv::Mat>& vecHist)
{
  vecHist.clear();
  for (const cv::Mat& image : vecImages)
  {
    cv::Mat hist;
    createHistogram(image, hist);
    vecHist.push_back(hist.clone());
  }
}

void ColorClassifier::createHistogram(const cv::Mat& image, cv::Mat& hist, const cv::Mat& mask)
{
  int hBins = config_.hBins, sBins = config_.sBins;
  int histSize[] = { hBins, sBins };
  float hRange[] = { 0, 180 };
  float sRange[] = { 0, 256 };
  const float* ranges[] = { hRange, sRange };
  int channels[] = { 0, 1 };

  cv::Mat mask2;
  if (mask.empty())
    createBlackPixelMask(image, mask2);
  else
    mask2 = mask;

  cv::calcHist(&image, 1, channels, mask2, hist, 2, histSize, ranges, true, false);
  cv::threshold(hist, hist, config_.max_hits_per_bin, 0, cv::THRESH_TRUNC);
  hist = cv::max(hist - config_.min_hits_per_bin, 0) + config_.min_hits_per_bin;
  cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
}

/**
 * @brief Classify all object instances' colors
 */
void ColorClassifier::classifyColor(const vector<cv::Mat>& histograms, vector<vector<Color::Type>>& all_object_colors,
                                    vector<vector<float>>& all_object_intensities)
{
  int i = 0;
  for (const cv::Mat& hist : histograms)
  {
    // vector<Color::Type> single_object_colors;
    // cv::Point max_point;
    // cv::minMaxLoc(hist, nullptr, nullptr, nullptr, &max_point);
    // std::cerr << i << " " << max_point << std::endl;
    // i++;
    //
    // uint8_t hVal = max_point.y;
    // Color::Type color = color_table_[hVal];
    // single_object_colors.push_back(color);

    std::map<Color::Type, float> single_object_colors_map;
    for (const auto& color : Color::All)
    {
      single_object_colors_map.emplace(color, 0.0);
    }

    for (int y = 0; y < hist.rows; y++)
      for (int x = 0; x < hist.cols; x++)
      {
        uint8_t hVal = y;
        Color::Type color = color_table_[hVal];
        single_object_colors_map[color] += hist.at<float>(y, x);
      }

    std::vector<Color::Type> single_object_colors;
    std::vector<float> single_object_intensities;
    for (const auto& color : Color::All)
    {
      float& intensity = single_object_colors_map[color];
      if (color != Color::Type::UNDEFINED)
      {
        if (intensity < config_.min_intensity)
          intensity = 0.0;
        single_object_colors.push_back(color);
        single_object_intensities.push_back(intensity);
      }
    }
    all_object_colors.push_back(single_object_colors);
    all_object_intensities.push_back(single_object_intensities);

    //! only for debugging, visualize the point with maximum value
    // int hBins = config_.hBins;
    // int sBins = config_.sBins;
    // cv::Mat histImg = cv::Mat::zeros(sBins*10, hBins*10, CV_8UC1);
    // int h = max_point.x;
    // int s = max_point.y;
    // int scale = 10;
    // cv::rectangle( histImg, cv::Point(h*scale, s*scale),
    //             cv::Point( (h+1)*scale - 1, (s+1)*scale - 1),
    //             cv::Scalar::all(255),
    //             -1 );
    // cv::imshow("hist", histImg);
    // cv::waitKey(30);

    // cv::Mat hsv_img = vecObjectImg_[i];
    // cv::Mat bgr;
    // cv::cvtColor(hsv_img, bgr, cv::COLOR_HSV2BGR);
    // cv::imshow("bgr", bgr);
    // cv::waitKey();
    // i++;
  }
}

/**
 * @brief Visualize a vector of histograms
 * ? not sure how to visualize multiple histograms
 */
void ColorClassifier::visualizeHist(const vector<cv::Mat>& vecHist, int scale)
{
  int window_id = 0;
  for (auto& hist : vecHist)
  {
    visualizeHist(hist, scale, "Histogram" + to_string(window_id));
    window_id++;
  }
}

void ColorClassifier::visualizeHist(const cv::Mat& hist, int scale, const string& window_name) const
{
  double maxVal = 0;
  int hBins = config_.hBins;
  int sBins = config_.sBins;
  cv::minMaxLoc(hist, nullptr, &maxVal);
  cv::Mat histImg = cv::Mat::zeros(sBins * scale, hBins * scale, CV_8UC3);
  for (int h = 0; h < hBins; h++)
    for (int s = 0; s < sBins; s++)
    {
      float binVal = hist.at<float>(h, s);
      int intensity = cvRound(binVal * 255 / maxVal);
      cv::rectangle(histImg, cv::Point(h * scale, s * scale), cv::Point((h + 1) * scale - 1, (s + 1) * scale - 1),
                    cv::Scalar::all(intensity), -1);
      const Color::Type& type = color_table_[h];
      if (type != Color::Type::UNDEFINED)
      {
        auto hue = (uint8_t)((h + 0.5) * 180 / hBins);
        auto saturation = (uint8_t)((s + 1.0) * 255 / sBins);
        uint8_t value = 255;
        const cv::Scalar color_bgr = convertScalarHSV2BGR(hue, saturation, value);

        cv::circle(histImg, cv::Point(h * scale + scale / 2, s * scale + scale / 2), scale / 4, color_bgr, cv::FILLED);
      }
    }
  cv::imshow(window_name, histImg);
  cv::waitKey(30);
}

/**
 * @brief Block black pixels while creating histogram. Because of the previous masking
 * process, images contain lots of black pixels, which should not be taken into account
 * while calculating histograms.
 */
void ColorClassifier::createBlackPixelMask(const cv::Mat& hsv_imageIn, cv::Mat& maskOut) const
{
  if (hsv_imageIn.empty())
  {
    ROS_WARN("createBlackPixelMask: input image is empty!");
    return;
  }
  auto lower_black = cv::Scalar(0, 0, 0);
  auto upper_black = cv::Scalar(255, 255, config_.upper_black_V);
  cv::inRange(hsv_imageIn, lower_black, upper_black, maskOut);

  cv::bitwise_not(maskOut, maskOut);
}

/**
 * @brief Construct a look up table to ease the classification. everytime bin sizes
 * are changed, it should be reconstructed
 */
void ColorClassifier::constructColorTable()
{
  map<uint8_t, Color::Type> table;
  for (auto color : Color::All)
  {
    uint8_t scaled_h = cvRound(color * config_.hBins / 180.);
    table.emplace(scaled_h, color);
  }

  auto it = table.begin();
  uint8_t scaled_tolerance = cvRound(config_.color_tolerance * config_.hBins / 180.);
  color_table_.clear();
  for (int i = 0; i < config_.hBins; i++)
  {
    uint8_t scaled_h = it->first;
    // hue wraps around to 0 at 180 (scaled hue wraps around at config_.hBins)
    int hue_diff = (i - scaled_h + config_.hBins) % config_.hBins;
    if (hue_diff > config_.hBins / 2)
      hue_diff -= config_.hBins;
    if (abs(hue_diff) <= scaled_tolerance)
    {
      color_table_.push_back(it->second);

      if (i == scaled_h + scaled_tolerance)
        if (it != table.end())  // always fulfilled here, just for safety
          it++;
      if (it->second == Color::Type::UNDEFINED)
      {
        // wrap back to beginning
        it = table.begin();
      }
    }
    else
    {
      color_table_.push_back(Color::Type::UNDEFINED);
    }
  }
}

cv::Scalar ColorClassifier::convertScalarHSV2BGR(uint8_t H, uint8_t S, uint8_t V)
{
  cv::Mat bgr;
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(H, S, V));
  cv::cvtColor(hsv, bgr, CV_HSV2BGR);
  return cv::Scalar(bgr.data[0], bgr.data[1], bgr.data[2]);
}

/**
 * @param rgb_image_ptr
 * @param all_object_colors vector of the classification results (one vector of color names per object)
 * @param obj_imgs_hsv vector of the segmented and cropped object images
 * @param histograms
 */
void ColorClassifier::publishResults(const cv_bridge::CvImageConstPtr& rgb_image_ptr,
                                     const vector<vector<Color::Type>>& all_object_colors,
                                     const vector<vector<float>>& all_object_intensities,
                                     const vector<cv::Mat>& obj_imgs_hsv, const vector<cv::Rect>& bboxes,
                                     const vector<cv::Mat>& histograms)
{
  // convert object images to rgb
  vector<cv_bridge::CvImagePtr> obj_img_rgb_ptrs;
  for (const cv::Mat& obj_img_hsv : obj_imgs_hsv)
  {
    cv_bridge::CvImagePtr obj_img_rgb_ptr = boost::make_shared<cv_bridge::CvImage>();
    obj_img_rgb_ptr->header = rgb_image_ptr->header;
    obj_img_rgb_ptr->encoding = rgb_image_ptr->encoding;
    // TODO: ensure that the rgb_image encoding really is "rgb8" or compatible, otherwise use cv::COLOR_HSV2BGR etc.
    cv::cvtColor(obj_img_hsv, obj_img_rgb_ptr->image, cv::COLOR_HSV2RGB);
    obj_img_rgb_ptrs.push_back(obj_img_rgb_ptr);
  }

  // publish ColorClassificationResults
  daa_color_classification::ColorClassificationResults msg;
  msg.header = rgb_image_ptr->header;

  /* pack color names */
  assert(all_object_colors.size() == all_object_intensities.size());
  for (size_t i = 0; i < all_object_colors.size(); i++)
  {
    const auto& single_object_colors = all_object_colors[i];
    const auto& single_object_intensities = all_object_intensities[i];
    daa_color_classification::Colors colors_msg;
    for (const Color::Type& color : single_object_colors)
    {
      colors_msg.colors.push_back(Color::colorTypeToString(color));
    }
    colors_msg.intensities = single_object_intensities;
    msg.colors.push_back(colors_msg);
  }

  /* pack object segment images */
  for (const auto& obj_img_rgb_ptr : obj_img_rgb_ptrs)
  {
    msg.object_images.push_back(*obj_img_rgb_ptr->toImageMsg());
  }

  /* pack histograms */
  for (const cv::Mat& hist : histograms)
  {
    cv_bridge::CvImagePtr hist_cv_ptr(new cv_bridge::CvImage);
    hist_cv_ptr->header = rgb_image_ptr->header;
    hist_cv_ptr->encoding = "mono8";
    hist_cv_ptr->image = hist.clone();
    msg.histograms.push_back(*hist_cv_ptr->toImageMsg());
  }

  /* pack bboxes */
  for (const auto& bbox : bboxes)
  {
    vision_msgs::BoundingBox2D bbox2d;
    bbox2d.center.x = bbox.x + bbox.width / 2.0;
    bbox2d.center.y = bbox.y + bbox.height / 2.0;
    bbox2d.size_x = bbox.width;
    bbox2d.size_y = bbox.height;
    msg.bboxes.push_back(bbox2d);
  }

  pResult_pub_->publish(msg);

  /* publish debug image */
  cv_bridge::CvImagePtr debug_img_ptr = boost::make_shared<cv_bridge::CvImage>();
  debug_img_ptr->header = rgb_image_ptr->header;
  debug_img_ptr->encoding = rgb_image_ptr->encoding;
  // debug_img_ptr->image = cv::Mat::zeros(rgb_image_ptr->image.size(), cv_bridge::getCvType(rgb_image_ptr->encoding));
  debug_img_ptr->image =
      cv::Mat(rgb_image_ptr->image.size(), cv_bridge::getCvType(rgb_image_ptr->encoding), cv::Scalar(255, 255, 255));

  assert(obj_img_rgb_ptrs.size() == bboxes.size());
  for (size_t i = 0; i < obj_img_rgb_ptrs.size(); i++)
  {
    const cv_bridge::CvImagePtr& obj_img_rgb_ptr = obj_img_rgb_ptrs[i];
    const cv::Rect& bbox = bboxes[i];
    cv::Mat mask;
    cv::cvtColor(obj_img_rgb_ptr->image, mask, cv::COLOR_RGB2GRAY);
    obj_img_rgb_ptr->image.copyTo(debug_img_ptr->image(bbox), mask);

    // draw bounding box
    cv::rectangle(debug_img_ptr->image, bbox.tl(), bbox.br(), cv::Scalar(255, 0, 0));

    // TODO: put text (color classification results)
  }

  image_pub_.publish(debug_img_ptr->toImageMsg());
}
