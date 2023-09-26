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
 * @file FixedEstimator.h
 * @author Zongyao Yi (zongyao.yi@dfki.de)
 * @brief Define FixedEstimator class
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "utils/types.h"
#include <memory>
#include "problib/pdfs/PDF.h"
#include "Estimator.h"

using namespace daa::utils;
using namespace std;
namespace daa
{
namespace utils
{
/**
 * @brief Placeholder estimator for fixed properties
 *
 */
class FixedEstimator : public Estimator
{
public:
  Estimator::Ptr clone() const override
  {
    return make_unique<FixedEstimator>(*this);
  }
  FixedEstimator(const pbl::PDF& pdf);
  FixedEstimator(const FixedEstimator& orig);

  virtual void propagate(const types::Time&) override;
  virtual void update(const pbl::PDF& z) override;
  virtual double get_likelihood(const pbl::PDF& z) const override;
  const pbl::PDF& get_pdf() const;

private:
  pbl::PDF::Ptr pdf_;
};

}  // namespace utils
}  // namespace daa
