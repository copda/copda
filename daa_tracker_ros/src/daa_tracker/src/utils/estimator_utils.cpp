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
 * @file estimator_utils.cpp
 * @author Zongyao Yi (zongyao.yi@dfki.de)
 * @brief Impementation
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "utils/estimator_utils.h"
#include "utils/tracker_config.h"

namespace daa
{
namespace utils
{
namespace estimator
{
/**
 * @brief Method to create estimators
 *
 * @param est_type
 * @param pdf
 * @return Estimator::Ptr
 */
Estimator::Ptr create(const string& est_type, pbl::PDF::Ptr pdf)
{
  Estimator::Ptr est;
  if (est_type == "KalmanFilter")
  {
    KalmanFilter kf(pdf->dimensions());
    auto gaussian = pbl::PDFtoGaussian(*pdf);
    assert(gaussian);
    kf.init(*gaussian);
    double a_max;
    daa::utils::TrackerConfig::instance().get_param("a_max", a_max);
    kf.setMaxAcceleration(a_max);
    est = make_unique<KalmanFilter>(kf);
  }
  else if (est_type == "Fixed")
  {
    est = make_unique<FixedEstimator>(*pdf);
  }
  else if (est_type == "HistogramEstimator")
  {
    est = make_unique<HistogramEstimator>(*pdf);
  }
  else
  {
    est = nullptr;
  }
  return est;
}
/**
 * @brief Convert estimator to pdf
 *
 * @param est
 * @return pbl::PDF::Ptr
 */
pbl::PDF::Ptr to_pdf(const Estimator::Ptr& est)
{
  switch (est->type())
  {
    case Estimator::Type::KalmanFilter:
    {
      KalmanFilter kf = *(static_cast<KalmanFilter*>(est.get()));
      return make_shared<pbl::Gaussian>(kf.getGaussian());
    }
    case Estimator::Type::Fixed:
    {
      FixedEstimator fixed_est = *(static_cast<FixedEstimator*>(est.get()));
      if (fixed_est.get_pdf().type() == pbl::PDF::DISCRETE)
      {
        const pbl::PMF* pmf = static_cast<const pbl::PMF*>(&fixed_est.get_pdf());
        return make_shared<pbl::PMF>(*pmf);
      }
    }
    case Estimator::Type::HistogramEstimator:
    {
      HistogramEstimator hist_est = *(static_cast<HistogramEstimator*>(est.get()));
      return make_shared<pbl::PMF>(hist_est.get_pdf());
    }
    default:
      return nullptr;
  }
}
bool is_potential_match(const types::DictKey& type, pbl::PDF::constPtr pdf_1, pbl::PDF::constPtr pdf_2)
{
  bool ret = false;
  if (type == types::ObservableType::POSITION)
  {
    auto gaussian_1 = pbl::PDFtoGaussian(pdf_1);
    auto gaussian_2 = pbl::PDFtoGaussian(pdf_2);
    assert(gaussian_1 && gaussian_2);
    double d =
        ::daa::utils::math::mahalanobis_dist(gaussian_1->getMean(), gaussian_1->getCovariance(), gaussian_2->getMean());
    double gate_area;
    if (!utils::TrackerConfig::instance().get_param("gate_area", gate_area))
      gate_area = 1.5;
    ret = (d <= gate_area);  // TODO: get_prior
  }
  else if (type == types::ObservableType::CLASS_ID)
  {
    double likelihood = pdf_1->getLikelihood(*pdf_2);
    ret = likelihood > 0.99;  // TODO: get_prior
  }
  else if (type == types::ObservableType::COLOR_HISTOGRAM)
  {
    double likelihood = pdf_1->getLikelihood(*pdf_2);
    // ret = (likelihood > 0);
    ret = true;
  }
  else
  {
    std::cout << "undefined potential match" << std::endl;  // TODO: logging
  }
  return ret;
}
}  // namespace estimator
}  // namespace utils

}  // namespace daa
