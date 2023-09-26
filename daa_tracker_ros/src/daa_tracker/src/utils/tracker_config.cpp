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

#include "utils/tracker_config.h"
namespace daa
{
namespace utils
{
TrackerConfig& TrackerConfig::instance()
{
  return instance_;
}

TrackerConfig::TrackerConfig() : initialized_(false)
{
}
TrackerConfig::~TrackerConfig()
{
}
bool TrackerConfig::is_initialized()
{
  return initialized_;
}
void TrackerConfig::load_config(const string& filename)
{
  load_config(YAML::LoadFile(filename));
}

void TrackerConfig::load_config(const YAML::Node& config)
{
  params_ = config["params"];
  behavioral_model_ = config["behavioral_model"];
  class_name_map_ = config["class_name_map"];
  priors_ = config["priors"];
}

double TrackerConfig::get_prior(string key)
{
  if (priors_[key])
    return priors_[key].as<double>();
  else  // TODO
  {
    return 0;
  }
}
string TrackerConfig::get_behavioral_model(const types::ObservableType& type) const
{
  // auto it = behavioral_model_table_.find(type);
  // if(it == behavioral_model_table_.end())
  // {
  //   throw runtime_error("undefined behavoiral model");
  // }
  const string& key = types::json(type);
  string val;
  if (behavioral_model_[key])
    val = behavioral_model_[key].as<string>();
  else
    val = "None";
  return val;
}

string TrackerConfig::get_class_name(size_t class_id)
{
  if (class_name_map_[class_id])
    return class_name_map_[class_id].as<string>();
  else
  {
    LOG(WARNING) << "Trying to query unknown class, please update 'class_name_map' in the config file";
    return "unknown_" + to_string(class_id);
  }
}

TrackerConfig TrackerConfig::instance_;
// TODO: get rid of this table
std::unordered_map<types::ObservableType, std::string> const TrackerConfig::behavioral_model_table_ = {
  { types::ObservableType::POSITION, "position" },
  { types::ObservableType::CLASS_ID, "class_id" },
  { types::ObservableType::IMAGE_UV, "image_uv" },
  { types::ObservableType::IMAGE, "image" },
};
}  // namespace utils

}  // namespace daa
