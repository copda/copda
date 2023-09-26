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
 * @file observables.h
 * @author Zongyao Yi (zongyao.yi@dfki.de)
 * @brief Define observables
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include <map>
#include <unordered_map>
#include <memory>
#include <boost/any.hpp>
#include <boost/variant.hpp>
#include <eigen3/Eigen/Eigen>
#include <typeinfo>
#include <opencv2/core.hpp>

#include "utils/types.h"
#include "utils/tracker_config.h"
#include "problib/pdfs/PMF.h"

#include <glog/logging.h>

using namespace std;
namespace daa
{
namespace multi_hyp_tracker
{
namespace observables
{
namespace types = ::daa::utils::types;
using Type = types::ObservableType;

/**
 * @brief Base class for other observables
 */
class Observable
{
public:
  typedef shared_ptr<Observable> Ptr;
  typedef shared_ptr<const Observable> constPtr;

public:
  Type type;

  template <typename T>
  Observable(T val) : val_(val)
  {
  }
  virtual ~Observable()
  {
  }
  boost::any raw_val()
  {
    return val_;
  }
  template <typename T>
  void get_val(T& val) const
  {
    if (typeid(T) == val_.type())
      val = boost::any_cast<T>(val_);
  }

  pbl::PDF::Ptr to_pdf()
  {
    pbl::PDF::Ptr pdf;
    if (type == Type::POSITION)
    {
      auto pos = to_val<types::Point3D>();
      auto cov = utils::TrackerConfig::instance().get_prior("obvervation_noise") *
                 arma::eye(3, 3);  // TODO: observation uncertainty
      pdf = make_shared<pbl::Gaussian>(pbl::Vector3(pos), cov);
    }
    else if (type == Type::CLASS_ID)
    {
      size_t class_id = to_val<size_t>();
      pbl::PMF pmf(1);  // contains only one domain
      string class_name = utils::TrackerConfig::instance().get_class_name(class_id);
      pmf.setExact(class_name);
      pdf = make_shared<pbl::PMF>(pmf);
    }
    else if (type == Type::COLOR_HISTOGRAM)
    {
      auto values = to_val<types::Histogram>();
      pbl::PMF pmf(values.size());
      for (auto i = 0; i < values.size(); i++)
      {
        // if (values[i])
        pmf.setProbability(to_string(i), values[i]);
      }
      if (pmf.getMaxProbability())
        pmf.normalize();
      pdf = make_shared<pbl::PMF>(pmf);
    }
    else
    {
      pdf = nullptr;
    }
    return pdf;
  }

protected:
  boost::any val_;
  template <typename T>
  T to_val() const
  {
    if (typeid(T) == val_.type())
      return boost::any_cast<T>(val_);
  }
};
/**
 * @brief For position
 */
class Position : public Observable
{
public:
  Position() = delete;
  Position(types::Point3D pos) : Observable(pos)
  {
    type = Type::POSITION;
  }
};
/**
 * @brief For orientation
 *
 */
class Orientation : public Observable
{
public:
  Orientation() = delete;
  Orientation(types::Quaternion quat) : Observable(quat)
  {
    type = Type::ORIENTATION;
  }
};
/**
 * @brief Class id
 */
class ClassID : public Observable
{
public:
  ClassID() = delete;
  ClassID(size_t class_id) : Observable(class_id)
  {
    type = Type::CLASS_ID;
  }
};
/**
 * @brief Observation in image coordinates
 */
class ImageUV : public Observable
{
public:
  ImageUV() = delete;
  ImageUV(types::Point2D uv) : Observable(uv)
  {
    type = Type::IMAGE_UV;
  }
};
/**
 * @brief For storing image, currently not used
 */
class Image : public Observable
{
public:
  Image() : Observable(types::Image())
  {
  }
  Image(types::Image img) : Observable(img.clone())
  {
    type = Type::IMAGE;
  }
};

class ColorHistogram : public Observable
{
public:
  ColorHistogram(types::Histogram values) : Observable(values)
  {
    type = Type::COLOR_HISTOGRAM;
  }
};
/**
 * @brief Contains observations of different attributes, e.g., position, color, etc.
 */
class Observation
{
  friend class Scan;
  friend class AnnotatedScan;

public:
  template <typename T>
  void add_observable(const T& obs)
  {
    observable_map_.emplace(obs.type, make_shared<T>(obs));
  }
  const unordered_map<Type, shared_ptr<Observable>>& get_observables() const
  {
    return observable_map_;
  }
  Observable::Ptr operator[](Type key)
  {
    auto it_obv = observable_map_.find(key);
    if (it_obv != observable_map_.end())
    {
      return it_obv->second;
    }
    else
      return nullptr;
  }
  const int id() const
  {
    return id_;
  }

protected:
  int id_;

private:
  unordered_map<Type, shared_ptr<Observable>> observable_map_;
};

/**
 * @brief This class serves as placeholder for dummy observation
 */
class EmptyObservation : public Observation
{
public:
  EmptyObservation()
  {
    id_ = 0;
  }
};
/**
 * @brief Input for the tracker
 */
class Scan
{
public:
  Scan(types::Time timestamp) : timestamp_(timestamp)
  {
  }
  const types::Time timestamp() const
  {
    return timestamp_;
  }
  void add_observation(const Observation& obs)
  {
    auto obs_ = obs;
    obs_.id_ = observations_.size() + 1;
    observations_.push_back(obs_);
  }
  void set_image(const cv::Mat& img)
  {
    img_ = img.clone();
  }
  const cv::Mat& get_image() const
  {
    return img_;
  }
  cv::Mat& get_image()
  {
    return img_;
  }
  bool empty() const
  {
    return observations_.empty();
  }
  vector<Observation> get_observations()
  {
    return observations_;
  }

protected:
  types::Time timestamp_;
  vector<Observation> observations_;
  cv::Mat img_;
};
/**
 * @brief Output of the mht, used to visualize results (for debugging)
 * TODO: Remove this class
 */
class AnnotatedScan : public Scan
{
public:
  AnnotatedScan(const Scan& orig) : Scan::Scan(orig)
  {
  }
  AnnotatedScan(const AnnotatedScan&) = default;
  vector<Observation> get_observations() const
  {
    auto ret = observations_;
    ret.insert(ret.end(), dummy_obs_.begin(), dummy_obs_.end());
    return ret;
  }
  void add_annotation(const size_t& obs_id, const string& object_id)
  {
    if (obs_id == 0)
    {
      LOG(WARNING) << "Empty obs cannot be annotated";
      return;
    }
    size_t real_id = obs_id - 1;
    if (obs_id > observations_.size())
    {
      LOG(WARNING) << "obs_id ecceeds";
      return;
    }
    auto it = target_names_.find(real_id);
    if (it != target_names_.end())
    {
      LOG(WARNING) << "This observation is already annotated, overwritting";
      it->second = object_id;
    }
    else
      target_names_.emplace(real_id, object_id);
  }
  void add_annotation(const Observation& obs, const string& obj_id)
  {
    Observation d_obs(obs);
    d_obs.id_ = -dummy_obs_.size() - 1;
    dummy_obs_.push_back(d_obs);
    target_names_.emplace(d_obs.id(), obj_id);
  }
  bool get_annotation(const int& obs_id, string& obj_id)
  {
    int real_id;
    if (obs_id > 0)
    {
      real_id = obs_id - 1;
    }
    else
    {
      real_id = obs_id;
    }
    auto it = target_names_.find(real_id);
    if (it != target_names_.end())
    {
      obj_id = it->second;
      return true;
    }
    return false;
  }

private:
  vector<Observation> dummy_obs_;
  map<int, string> target_names_;
};

}  // namespace observables

}  // namespace multi_hyp_tracker
}  // namespace daa
