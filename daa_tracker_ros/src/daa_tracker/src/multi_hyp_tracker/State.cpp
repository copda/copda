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
 * @file State.cpp
 * @author Zongyao Yi (zongyao.yi@dfki.de)
 * @brief Implementation of State class
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "multi_hyp_tracker/State.h"

using namespace daa::multi_hyp_tracker;

/**************class State************/
/*************************************/
/**
 * @brief
 *
 * @param timestamp
 * @param obs Observation
 */
State::Ptr State::create(const types::Time& timestamp, const observables::Observation& obs)
{
  // if(prev_state)
  //     return make_shared<State>(timestamp, prev_state, obs);
  // else
  return make_shared<State>(timestamp, obs);
}
/**
 * @brief Clone method
 *
 * @return State::Ptr
 */
State::Ptr State::clone() const
{
  return make_shared<State>(*this);
}
/**
 * @brief Return const ptr
 *
 * @return State::constPtr
 */
State::constPtr State::const_ptr()
{
  return const_pointer_cast<const State>(shared_from_this());
}
/**
 * @brief Construct a new State:: State object
 *
 * @param timestamp
 * @param obs
 */
State::State(const types::Time& timestamp, const observables::Observation& obs)
  : timestamp_(timestamp), obs_id_(obs.id())
{
  log_prob_ = 0;
  track_id_ = State::get_next_id();
  for (const auto typ_obv : obs.get_observables())
  {
    auto obv = typ_obv.second;
    auto prop = Property::create(obv);
    if (prop)
    {
      if (!prop->estimator_)
      {
        storage_.emplace(obv->type, obv->raw_val());
      }
      properties_.emplace(obv->type, std::move(prop));
    }
  }
}
/**
 * @brief Copy constructor
 *
 * @param other
 */
State::State(const State& other)
  : timestamp_(other.timestamp_)
  , track_id_(other.track_id_)
  , log_prob_(other.log_prob_)
  , obs_id_(other.obs_id_)
  , storage_(other.storage_)
  , debug_info_(other.debug_info_)
{
  if (!other.symbol_.empty())
    symbol_ = other.symbol_;
  for (auto it_prop = other.properties_.begin(); it_prop != other.properties_.end(); it_prop++)
  {
    properties_.emplace(it_prop->first, it_prop->second->clone());
  }
}
void State::set_symbol(const string symbol)
{
  symbol_ = symbol;
}

const string State::symbol() const
{
  return symbol_;
}

const uint32_t State::track_id() const
{
  return track_id_;
}
/**
 * @brief Return the log probability of the state
 *
 * @return const double
 */
const double State::log_prob() const
{
  return log_prob_;
}
/**
 * @brief The observation id the state contains
 *
 * @return const size_t
 */
const size_t State::obs_id() const
{
  return obs_id_;
}
/**
 * @brief Return timestamp
 *
 * @return const types::Time
 */
const types::Time State::timestamp() const
{
  return timestamp_;
}
/**
 * @brief Properties of the state
 *
 * @return Map of properties
 */
const map<State::Type, State::Property::Ptr>& State::properties() const
{
  return properties_;
}
/**
 * @brief Set log prob, used after n_scan pruning
 *
 * @param log_prob
 */
void State::set_log_prob(const double& log_prob)
{
  log_prob_ = log_prob;
}
/**
 * @brief Propagte the state based on the behavioral model
 *
 * @param timestamp
 */
void State::propagate(const types::Time& timestamp)
{
  auto dt = timestamp - timestamp_;
  for (auto it_prop = properties_.begin(); it_prop != properties_.end(); it_prop++)
    it_prop->second->propagate(dt);
}
/**
 * @brief Update with observation
 *
 * @param timestamp
 * @param obs
 */
void State::update(const types::Time& timestamp, const observables::Observation& obs)
{
  last_timestamp_ = timestamp_;
  timestamp_ = timestamp;
  auto dt = timestamp_ - last_timestamp_;
  PCHECK(dt > 0) << "Negative dt";
  obs_id_ = obs.id();
  if (obs.id())
  {
    double likelihood = max(get_likelihood(obs, dt), 1e-100);
    log_prob_ = log(likelihood);
    for (const auto typ_obv : obs.get_observables())
    {
      auto type = typ_obv.first;
      auto obv = typ_obv.second;
      auto it_prob = properties_.find(type);
      if (it_prob != properties_.end())
      {
        it_prob->second->update(obv);
      }
      else
      {
        LOG(ERROR) << "property type doesn't exist";
      }
      auto it_storage = storage_.find(type);

      if (it_storage != storage_.end())
      {
        it_storage->second = obv->raw_val();
      }
    }
    log_prob_ -= log(utils::TrackerConfig::instance().get_prior("prob_false_alarm"));  // using log likelihood ratio
  }
  else
  {
    log_prob_ = log(1 - utils::TrackerConfig::instance().get_prior("prob_detection"));
  }
  auto status_dict = to_dict();
  debug_info_ = status_dict.dump(4);
}
/**
 * @brief Merge with another state, used after the n_scan pruning
 *
 * @param state
 * @param ref_state
 */
void State::merge(State::Ptr& state, State::constPtr ref_state)
{
  double acc_log_prob = state->log_prob() + ref_state->log_prob();
  auto symbol = state->symbol();
  state = ref_state->clone();
  state->set_symbol(symbol);
  state->log_prob_ = acc_log_prob;
}

/**
 * @brief Check if the observation is a potential match
 *
 * @param obs
 * @return true if the observation is a potential match
 */
bool State::is_potential_match(const observables::Observation& obs)
{
  PCHECK(!properties_.empty()) << "State has no properties.";
  for (const auto typ_obv : obs.get_observables())
  {
    types::DictKey type = typ_obv.first;
    auto obv = typ_obv.second;
    auto it_prob = properties_.find(type);
    PCHECK(it_prob != properties_.end());
    if (it_prob->second->estimator_)
    {
      auto est_pdf = it_prob->second->get_pdf();
      auto obv_pdf = obv->to_pdf();
      auto is_match = ::daa::utils::estimator::is_potential_match(type, est_pdf, obv_pdf);
      if (!is_match)
        return false;
    }
  }
  return true;
}
/**
 * @brief Serialize the state into dict
 *
 * @return types::Dict
 */
types::Dict State::to_dict() const
{
  types::Dict state_dict;
  state_dict["timestamp"] = timestamp_;
  state_dict["obs_id"] = obs_id();
  for (auto it_prop = properties_.begin(); it_prop != properties_.end(); it_prop++)
  {
    // get vals from properties
    types::DictKey type = it_prop->first;
    types::Dict pdf_dict;

    const auto prop = it_prop->second;

    // TODO: move to a centralized mapping function

    if (type == Type::POSITION)
    {
      auto gaussian = pbl::PDFtoGaussian(prop->get_pdf());
      PCHECK(gaussian);
      pdf_dict["type"] = "gaussian";
      pdf_dict["mean"] = gaussian->getMean();
      // std::stringstream s;
      // gaussian->getCovariance().raw_print(s);
      // cout<<s.str();
      pdf_dict["cov"] = gaussian->getCovariance();
      state_dict[type] = pdf_dict;
    }
    else if (type == Type::CLASS_ID)
    {
      auto pmf = pbl::PDFtoPMF(prop->get_pdf());
      PCHECK(pmf);
      pdf_dict["type"] = "discrete";
      pdf_dict["domains"] = pmf->getDomainSize();
      types::Dict prob_field;
      vector<string> vals;
      vector<double> probs;
      pmf->getValues(vals);
      pmf->getProbabilities(probs);
      PCHECK(vals.size() == probs.size());
      for (int i = 0; i < vals.size(); i++)
      {
        prob_field[vals[i]] = probs[i];
      }
      pdf_dict["values"] = prob_field;
      state_dict[type] = pdf_dict;
    }
    else if (type == Type::IMAGE_UV || type == Type::IMAGE)
    {
      // TODO
    }
    else if (type == Type::COLOR_HISTOGRAM)
    {
      auto pmf = pbl::PDFtoPMF(prop->get_pdf());
      PCHECK(pmf);
      pdf_dict["type"] = "discrete";
      pdf_dict["domains"] = pmf->getDomainSize();
      types::Dict prob_field;
      vector<string> vals;
      vector<double> probs;
      pmf->getValues(vals);
      pmf->getProbabilities(probs);
      PCHECK(vals.size() == probs.size());
      for (int i = 0; i < vals.size(); i++)
      {
        prob_field[vals[i]] = probs[i];
      }
      pdf_dict["values"] = prob_field;
      state_dict[type] = pdf_dict;
    }
    else
    {
    }
  }
  for (auto it = storage_.begin(); it != storage_.end(); it++)
  {
    types::DictKey type = it->first;
    auto value = it->second;
    if (type == Type::ORIENTATION)
    {
      auto quat_mean = boost::any_cast<types::Quaternion>(value);
      auto quat = pbl::Vector4(quat_mean[0], quat_mean[1], quat_mean[2], quat_mean[3]);
      // stringstream ss;
      // ss << quat;
      state_dict[type] = quat;
    }
    else if (type == Type::IMAGE_UV)
    {
      // TODO
    }
  }
  return state_dict;
}
/**
 * @brief Get likelihood of the observation given the state
 *
 * @param obs
 * @param dt zero by default
 * @return double
 */
double State::get_likelihood(const observables::Observation& obs, const types::Time dt) const
{
  double likelihood = 1;
  double log_likelihood = 0;
  LOG_IF(FATAL, properties_.empty()) << "State has no properties";
  for (const auto typ_obv : obs.get_observables())
  {
    auto type = typ_obv.first;
    auto obv = typ_obv.second;
    auto it_prob = properties_.find(type);
    if (it_prob != properties_.end())
    {
      auto prop_pdf = it_prob->second->get_pdf();
      auto obv_pdf = obv->to_pdf();
      if (prop_pdf)
      {
        auto prop_like = prop_pdf->getLikelihood(*obv_pdf);
        string type_name = types::json(type);
        auto prob_model = daa::utils::TrackerConfig::instance().get_prior("prob_" + type_name);
        auto decay_factor = daa::utils::TrackerConfig::instance().get_prior("decay_factor_" + type_name);
        prob_model *= exp(-decay_factor * dt);
        prop_like = max(0.0001, prop_like);
        CHECK(prop_like > 0);
        likelihood *= (prob_model * prop_like);

      }  // active property
    }
    else
    {
      LOG(ERROR) << "Property type doesn't exist";
    }
  }
  return likelihood;
}
/**
 * @brief Get likelihood of of a state given current state
 *
 * @param state
 * @return double
 */
double State::get_likelihood(State::Ptr state) const
{
  const auto reference_props = state->properties();
  double likelihood = 1;
  PCHECK(!properties_.empty()) << "State has no properties.";
  PCHECK(!state->properties().empty()) << "Reference state has no properties.";
  PCHECK(state->properties().size() == properties_.size()) << "State property sizes don't match.";
  for (auto it_prop = reference_props.begin(); it_prop != reference_props.end(); it_prop++)
  {
    const auto type = it_prop->first;
    const auto ref_prop = it_prop->second;

    const auto it = properties_.find(type);
    if (it != properties_.end())
    {
      auto prop_pdf = it->second->get_pdf();
      auto ref_pdf = ref_prop->get_pdf();
      if (prop_pdf && ref_pdf)
        likelihood *= prop_pdf->getLikelihood(*ref_pdf);
    }
  }
  return max(likelihood, 1e-100);
}

uint32_t State::next_id_ = 0;

/********class State::Property********/

/**
 * @brief Create new pointer
 *
 * @param obv Observable, e.g. position, color, class id, etc.
 * @return State::Property::Ptr
 */
State::Property::Ptr State::Property::create(observables::Observable::Ptr obv)
{
  string est_type = daa::utils::TrackerConfig::instance().get_behavioral_model(obv->type);
  return make_shared<State::Property>(utils::estimator::create(est_type, obv->to_pdf()));
}
/**
 * @brief Clone
 *
 * @return State::Property::Ptr
 */
State::Property::Ptr State::Property::clone()
{
  // State::Property prop_clone(estimator_->clone());
  return make_shared<State::Property>(*this);
}
/**
 * @brief Copy constructor
 *
 * @param prop another object
 */
State::Property::Property(const Property& prop)
  : estimator_(prop.estimator_ ? std::move(prop.estimator_->clone()) : nullptr)
{
}
/**
 * @brief Construct with an estimator
 *
 * @param est the estimator such as Kalman filter
 */
State::Property::Property(utils::Estimator::Ptr est) : estimator_(std::move(est))
{
}
/**
 * @brief Propagate with the estimator
 *
 * @param dt
 */
void State::Property::propagate(types::Time dt)
{
  if (estimator_)
    estimator_->propagate(dt);
}
/**
 * @brief Update step
 *
 * @param obv
 */
void State::Property::update(observables::Observable::Ptr obv)
{
  if (estimator_)
  {
    auto obv_pdf = obv->to_pdf();
    PCHECK(obv_pdf) << "Cannot convert this obverable to pdf";
    estimator_->update(*obv_pdf);
  }
}
/**
 * @brief Get pdf
 *
 * @return pbl::PDF::Ptr
 */
pbl::PDF::Ptr State::Property::get_pdf() const
{
  if (estimator_)
    return ::daa::utils::estimator::to_pdf(estimator_);
  else
    return nullptr;
}
/**
 * @brief Debug method
 *
 * @param root
 */
void daa::multi_hyp_tracker::show_tree(StateNode::Ptr root, string file_name)
{
  function<string(State::Ptr)> label_func = [](State::Ptr state) {
    const auto props = state->properties();
    string label;
    if (!state->symbol().empty())
      label += "name: " + state->symbol() + "\n";
    label += "time: " + to_string(state->timestamp()) + "\n";
    label += "obs id: " + to_string(state->obs_id()) + "\n";
    label += "log prob: " + to_string(state->log_prob()) + "\n";
    for (auto it_prop = props.begin(); it_prop != props.end(); it_prop++)
    {
      const auto type = it_prop->first;
      const auto prop = it_prop->second;

      if (type == observables::Type::POSITION)
      {
        const auto pdf = prop->get_pdf();
        auto gauss_pdf = pbl::PDFtoGaussian(*pdf);
        auto pos = gauss_pdf->getMean();
        std::ostringstream s;
        s.precision(3);
        pos.st().raw_print(s);
        string pos_str = s.str();
        label += ("pos: " + pos_str);
      }
    }

    return label;
  };
  function<double(State::Ptr)> weight_func = [](State::Ptr state) { return 0.; };
  string output_path;
  daa::utils::TrackerConfig::instance().get_param("output_path", output_path);
  if (file_name.empty())
    file_name = "state_tree.dot";
  StateTreeUtils::Viz::generate_graph(root, output_path + file_name, label_func, weight_func);
}
