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
 * @file State.h
 * @author Zongyao Yi (zongyao.yi@dfki.de)
 * @brief Define State class
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <vector>
#include <map>
#include <boost/any.hpp>
#include "data_structures/tree.h"
#include "multi_hyp_tracker/observables.h"
#include "utils/types.h"
#include "utils/estimator_utils.h"
#include "utils/tracker_config.h"

#include <glog/logging.h>

using namespace std;
using namespace daa::utils;
namespace daa
{
namespace multi_hyp_tracker
{
namespace types = ::daa::utils::types;
/**
 * @brief Stores state of different properties, node of the multi-hypotheses tree
 */
struct State : public enable_shared_from_this<State>
{
public:
  // data types
  typedef observables::Type Type;
  typedef shared_ptr<State> Ptr;
  typedef shared_ptr<const State> constPtr;
  /**
   * @brief State property, e.g., position, class id, color etc. Different properties have different estimators (e.g.,
   * KalmanFilter)
   */
  struct Property
  {
    friend class State;

    typedef shared_ptr<Property> Ptr;
    static Ptr create(observables::Observable::Ptr);
    Ptr clone();
    Property(const Property&);
    Property(utils::Estimator::Ptr);
    void propagate(types::Time);
    void update(observables::Observable::Ptr);
    pbl::PDF::Ptr get_pdf() const;

  private:
    utils::Estimator::Ptr estimator_;
  };
  // functions
  static Ptr create(const types::Time& timestamp, const observables::Observation& obs);
  static void merge(Ptr&, constPtr);
  Ptr clone() const;
  constPtr const_ptr();
  State() = delete;
  State(const types::Time&, const observables::Observation& obs);
  State(const State&);
  void propagate(const types::Time&);
  void update(const types::Time&, const observables::Observation&);
  void set_log_prob(const double& log_prob);
  void set_symbol(const string symbol);
  // query functions
  bool is_potential_match(const observables::Observation&);
  types::Dict to_dict() const;
  double get_likelihood(const observables::Observation&, const types::Time dt = 0) const;
  double get_likelihood(State::Ptr) const;
  const map<Type, Property::Ptr>& properties() const;
  const types::Time timestamp() const;
  const double log_prob() const;
  const string symbol() const;
  const uint32_t track_id() const;
  const size_t obs_id() const;

private:
  static uint32_t get_next_id()
  {
    return ++next_id_;
  }
  // data
  string symbol_;
  uint32_t track_id_;
  types::Time last_timestamp_;
  types::Time timestamp_;
  double log_prob_;
  size_t obs_id_;
  map<Type, Property::Ptr> properties_;
  map<Type, boost::any> storage_;
  string debug_info_;

  static uint32_t next_id_;
};

typedef ::daa::data_structures::TreeNode<State::Ptr> StateNode;
typedef ::daa::data_structures::TreeUtils<State::Ptr> StateTreeUtils;
typedef StateNode::wPtr Track;

void show_tree(StateNode::Ptr root, string file_name = "");
}  // namespace multi_hyp_tracker
}  // namespace daa
