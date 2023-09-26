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
#include "utils/types.h"

#include <string>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

using namespace std;
namespace daa
{
namespace utils
{
class TrackerConfig
{
public:
  static TrackerConfig& instance();
  TrackerConfig(TrackerConfig&) = delete;
  TrackerConfig& operator=(TrackerConfig&) = delete;
  ~TrackerConfig();
  bool is_initialized();
  void load_config(const string& filename);
  void load_config(const YAML::Node& config);

  template <typename T>
  bool set_param(const string& key, T& val)
  {
    bool ret = false;
    if (params_[key])
    {
      params_[key] = val;
      ret = true;
    }
    return ret;
  }
  template <typename T>
  bool get_param(const string& key, T& val)
  {
    bool ret = false;
    if (params_[key])
    {
      val = params_[key].as<T>();
      ret = true;
    }
    return ret;
  }
  double get_prior(const string key);
  string get_class_name(const size_t class_id);
  string get_behavioral_model(const types::ObservableType&) const;

private:
  TrackerConfig();

  bool initialized_;
  YAML::Node params_;
  YAML::Node behavioral_model_;
  YAML::Node class_name_map_;
  YAML::Node priors_;
  static std::unordered_map<types::ObservableType, std::string> const behavioral_model_table_;

  static TrackerConfig instance_;
};
}  // namespace utils
}  // namespace daa
