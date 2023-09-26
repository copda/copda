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
 * @file MultiHypTracker.h
 * @author Zongyao Yi (zongyao.yi@dfki.de)
 * @brief Define MultiHypTracker class
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include "multi_hyp_tracker/State.h"
#include "multi_hyp_tracker/observables.h"
#include "multi_hyp_tracker/Hypothesis.h"

#include "utils/tracker_config.h"
#include "utils/types.h"
#include <vector>
#include <map>
#include <deque>
#include <memory>
#include <list>
#include <glog/logging.h>

using namespace std;

namespace daa
{
namespace multi_hyp_tracker
{
class MultiHypTracker
{
public:
  // data types
  typedef shared_ptr<MultiHypTracker> Ptr;
  // public methods
  static Ptr create()
  {
    return make_shared<MultiHypTracker>();
  }
  MultiHypTracker() = default;
  bool process_scan(const observables::Scan& scan);
  vector<StateNode::Ptr> get_tracks();

private:
  void propagate_nodes(types::Time timestamp);
  void expand_nodes();
  void add_nodes();
  void pre_pruning();
  void form_global_hyp();
  void post_pruning();
  void purge();

private:
  // data
  vector<StateNode::wPtr> selected_roots_;
  vector<StateNode::wPtr> mature_roots_;
  list<Hypothesis> global_hyp_;
  deque<observables::Scan> input_queue_;
  types::Time timestamp_;
  vector<StateNode::Ptr> hyp_trees_;  // all trees
};
}  // namespace multi_hyp_tracker
}  // namespace daa
