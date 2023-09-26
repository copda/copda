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
 * @file types.h
 * @author Zongyao Yi (zongyao.yi@dfki.de)
 * @brief Define types used in the package
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <memory>
#include <eigen3/Eigen/Eigen>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include "problib/datatypes.h"
#include "problib/pdfs/PDF.h"
#include "problib/pdfs/Gaussian.h"
namespace daa
{
namespace utils
{
namespace types
{
using json = nlohmann::json;
typedef double Time;
typedef double Duration;
typedef Eigen::Vector2d Point2D;
typedef Eigen::Vector3d Point3D;
typedef Eigen::Vector4d Quaternion;
typedef std::vector<double> Histogram;
typedef cv::Mat Image;
typedef json Dict;
typedef json DictKey;
enum class ObservableType
{
  POSITION,
  IMAGE_UV,  // position in image coordinates
  CLASS_ID,
  IMAGE,
  COLOR_HISTOGRAM,
  ORIENTATION,
};
enum class Attribute
{
  NAME,
  TYPE,
  PERCEPTS,
  TIMESTAMP,
};
namespace query
{
enum class Field
{
  HEADER,
  ARGS,
};
NLOHMANN_JSON_SERIALIZE_ENUM(Field, {
                                        { Field::HEADER, "header" },
                                        { Field::ARGS, "args" },
                                    })
const json HEADER = Field::HEADER;
const json ARGS = Field::ARGS;

namespace args
{
enum class Name
{
  SYMBOL,
  PERCEPTS,
};
NLOHMANN_JSON_SERIALIZE_ENUM(Name, {
                                       { Name::SYMBOL, "symbol" },
                                       { Name::PERCEPTS, "percepts" },
                                   })
const json SYMBOL = Name::SYMBOL;
const json PERCEPTS = Name::PERCEPTS;
}  // namespace args
}  // namespace query

NLOHMANN_JSON_SERIALIZE_ENUM(ObservableType, {
                                                 { ObservableType::POSITION, "position" },
                                                 { ObservableType::IMAGE_UV, "image_uv" },
                                                 { ObservableType::CLASS_ID, "class_id" },
                                                 { ObservableType::IMAGE, "image" },
                                                 { ObservableType::COLOR_HISTOGRAM, "color_histogram" },
                                                 { ObservableType::ORIENTATION, "orientation" },
                                             })
NLOHMANN_JSON_SERIALIZE_ENUM(Attribute,
                             {
                                 { Attribute::NAME, "name" },
                                 { Attribute::TYPE, "type" },
                                 { Attribute::PERCEPTS, "percepts" },
                                 { Attribute::TIMESTAMP, "timestamp" },
                             }

)

}  // namespace types

}  // namespace utils
}  // namespace daa
