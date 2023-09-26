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
 * @file MultiHypTracker.cpp
 * @author Zongyao Yi (zongyao.yi@dfki.de)
 * @brief Implementation of MultiHypTracker
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "multi_hyp_tracker/MultiHypTracker.h"

using namespace daa::multi_hyp_tracker;

#ifdef __cplusplus
extern "C" {
#endif

#include "cliquer.h"

#ifdef __cplusplus
}
#endif

/**
 * @brief Interface method
 *
 * @param scan Input of the algorithm
 */
bool MultiHypTracker::process_scan(const observables::Scan& scan)
{
  if (scan.empty())
  {
    LOG(INFO) << "Propagating empty scan...";
    // return false;
  }
  timestamp_ = scan.timestamp();
  input_queue_.push_back(scan);
  propagate_nodes(scan.timestamp());
  expand_nodes();
  add_nodes();
  pre_pruning();
  form_global_hyp();
  post_pruning();
  purge();
  int n_scan;
  daa::utils::TrackerConfig::instance().get_param("n_scan", n_scan);
  if (input_queue_.size() > n_scan)
    input_queue_.pop_front();
  return true;
}

/**
 * @brief Run propagation step
 *
 * @param timestamp
 */
void MultiHypTracker::propagate_nodes(types::Time timestamp)
{
  for (auto it = hyp_trees_.begin(); it != hyp_trees_.end(); it++)
  {
    auto root = *it;
    auto leaves = StateTreeUtils::get_leaves(root);
    for (auto node : leaves)
    {
      node->item()->propagate(timestamp);
    }
  }
}

/**
 * @brief Expand existing nodes with current observations, also update tree ids
 * @param None
 */
void MultiHypTracker::expand_nodes()
{
  auto scan = input_queue_.back();
  auto timestamp = scan.timestamp();
  for (auto root : hyp_trees_)
  {
    auto leaf_nodes = StateTreeUtils::get_leaves(root);
    for (auto node : leaf_nodes)
    {
      for (auto obs : scan.get_observations())
      {
        // expand with obs
        if (node->item()->is_potential_match(obs))
        {
          auto cur_state = node->item();
          auto next_state = cur_state->clone();
          next_state->update(timestamp, obs);
          node->expand(next_state);
          // candidate_hyp_.push_back(Hypothesis(new_node));
        }
      }
      // expand with empty obs
      auto next_state = node->item()->clone();
      next_state->update(timestamp, observables::EmptyObservation());
      node->expand(next_state);
    }
  }
}

/**
 * @brief Create a new root nodes for each observation, representing the posibility that
 * the observation comes from a new target
 * @param None
 */
void MultiHypTracker::add_nodes()
{
  auto scan = input_queue_.back();
  auto timestamp = scan.timestamp();
  // vector<StateNode::Ptr> new_nodes;
  for (const auto& obs : scan.get_observations())
  {
    auto new_root = StateNode::create(State::create(timestamp, obs));
    // candidate_hyp_.push_back(new_root);
    hyp_trees_.push_back(new_root);
  }
}
/**
 * @brief Perform first pruning
 *
 */
void MultiHypTracker::pre_pruning()
{
  // pruning params
  int n_scan;
  int b_th;  // prune tree when it has more than b_th branches
  double k_best;
  double min_score;
  daa::utils::TrackerConfig::instance().get_param("min_score", min_score);
  daa::utils::TrackerConfig::instance().get_param("n_scan", n_scan);
  daa::utils::TrackerConfig::instance().get_param("k_best", k_best);  // select the top k_best% best tracks
  daa::utils::TrackerConfig::instance().get_param("b_th", b_th);
  int n_miss = n_scan;
  for (auto root : hyp_trees_)
  {
    vector<Hypothesis> hypotheses;
    vector<double> scores;
    StateTreeUtils::invalidate_branch(root);
    auto leaves = StateTreeUtils::get_leaves(root);
    Hypothesis::ValidCriteria criteria = 0;
    Hypothesis::ValidationParam valid_params;
    for (auto node : leaves)
    {
      auto hyp = Hypothesis(node);
      hypotheses.push_back(hyp);
      scores.push_back(hyp.score);
    }
    // if branch width exceeds threshold, do threshold trimming
    if (leaves.size() > b_th)
    {
      criteria |= Hypothesis::ValidCriterion::THRESHOLD;
      // find min score
      if (k_best < 1.)
      {
        sort(scores.begin(), scores.end(), [](double a, double b) { return (a > b); });
        min_score = scores[k_best * scores.size()];
        // TODO: negative scores should be allowed
        min_score = max(min_score, 0.);  // make sure that it is larger than 0
      }
      else
        min_score = 0.;
      valid_params.threshold = min_score;
    }
    // always add this criterion
    criteria |= Hypothesis::ValidCriterion::DUMMY_OBS;
    valid_params.max_num_dum_scan = n_miss;
    // verify hypothese
    Hypothesis::verify(hypotheses, criteria, valid_params);
    // prune invalid branches
    for (auto hyp : hypotheses)
    {
      if (hyp.is_valid)
        StateTreeUtils::validate_branch(hyp.node());
    }
    StateTreeUtils::prune_invalid(root);
  }
}
/**
 * @brief Get MAP solution
 *
 */
void MultiHypTracker::form_global_hyp()
{
  vector<Hypothesis> candidate_hyp;
  auto timestamp = input_queue_.back().timestamp();
  for (auto tree : hyp_trees_)
  {
    auto leaves = StateTreeUtils::get_leaves(tree);
    for (auto node : leaves)
    {
      if (node->item()->timestamp() == timestamp)  // hypothesis has to be aligned
        candidate_hyp.push_back(Hypothesis(node));
    }
  }
  if (candidate_hyp.empty())
    return;
  // TODO: normalize hyp scores
  auto it_max = max_element(candidate_hyp.begin(), candidate_hyp.end(),
                            [](const Hypothesis& a, const Hypothesis& b) { return a.score < b.score; });
  auto it_min = min_element(candidate_hyp.begin(), candidate_hyp.end(),
                            [](const Hypothesis& a, const Hypothesis& b) { return a.score < b.score; });
  {
    double max_score = it_max->score;
    double min_score = it_min->score;
    if (max_score - min_score)
    {
      double new_max = 1000.;
      double new_min = 1.;
      double rescale_factor = (new_max - new_min) / (max_score - min_score);
      for (auto& hyp : candidate_hyp)
      {
        hyp.score = (hyp.score - min_score) * rescale_factor + new_min;
      }
    }
  }
  /*construct graph*/
  graph_t* g;
  set_t s;
  if (candidate_hyp.empty())
  {
    VLOG(1) << "No candidate hypotheses";
    return;
  }
  g = graph_new(candidate_hyp.size());
  if (g == NULL)
  {
    LOG(ERROR) << "Graph is null";
    return;
  }
  /*add vertecies and define their weights*/
  for (size_t i = 0; i < candidate_hyp.size(); i++)
  {
    auto weight = max((int)round(candidate_hyp[i].score), 1);
    g->weights[i] = weight;  // weights are ints
  }
  /*add edges between candidate_hyp that are compatible*/
  for (size_t i = 0; i < candidate_hyp.size() - 1; i++)
  {
    for (size_t j = i + 1; j < candidate_hyp.size(); j++)
    {
      if (Hypothesis::is_compatible(candidate_hyp[i], candidate_hyp[j]))
      {
        GRAPH_ADD_EDGE(g, i, j);
      }
    }
  }
  /*solve MWIS*/
  // ASSERT(graph_test(g, stderr));  // TODO: suppress the output stream

  clique_options clique_options_struct = {
    reorder_by_default, NULL, NULL, NULL, NULL, NULL, NULL, 0
  };  // clique_print_time set to NULL

  s = clique_find_single(g, 0, 0, FALSE, &clique_options_struct);
  assert(SET_MAX_SIZE(s) == candidate_hyp.size());  // TODO: add error handling
  global_hyp_.clear();
  for (int i = 0; i < SET_MAX_SIZE(s); i++)
  {
    if (SET_CONTAINS(s, i))
    {
      // push to global hyp
      global_hyp_.push_back(candidate_hyp[i]);
    }
  }
}
/**
 * @brief Perform n scan pruning
 *
 */
void MultiHypTracker::post_pruning()
{
  int n_scan;
  daa::utils::TrackerConfig::instance().get_param("n_scan", n_scan);
  n_scan = max(n_scan, 2);
  mature_roots_.clear();
  selected_roots_.clear();
  for (auto& root : hyp_trees_)
  {
    bool selected = false;
    auto it =
        find_if(global_hyp_.begin(), global_hyp_.end(), [&root](const Hypothesis& hyp) { return hyp.root() == root; });
    StateNode::Ptr new_root;
    if (it != global_hyp_.end())  // tree contains global hypothesis
    {
      new_root = StateTreeUtils::n_scan_pruning(it->node(), n_scan);
      selected = true;
    }
    else  // tree doesn't contain global hypothesis
    {
      // get the most possible hypothesis
      auto leaves = StateTreeUtils::get_leaves(root);
      vector<Hypothesis> hypotheses;
      for (auto node : leaves)
      {
        hypotheses.push_back(Hypothesis(node));
      }
      auto it_max = max_element(hypotheses.begin(), hypotheses.end(),
                                [](const Hypothesis& a, const Hypothesis& b) { return a.score < b.score; });

      new_root = StateTreeUtils::n_scan_pruning(it_max->node(), n_scan);
    }
    if (new_root)
    {
      StateTreeUtils::merge_roots(root, new_root);
      State::merge(root->item(), new_root->item()->const_ptr());
      new_root = nullptr;
      mature_roots_.push_back(root);
      if (selected)
        selected_roots_.push_back(root);
    }
  }
}
/**
 * @brief Purge trees containing no global hypotheses
 *
 */
void MultiHypTracker::purge()
{
  for (auto it_root = hyp_trees_.begin(); it_root != hyp_trees_.end();)
  {
    auto it = find_if(global_hyp_.begin(), global_hyp_.end(),
                      [it_root](const Hypothesis& hyp) { return hyp.root() == *it_root; });
    if (it != global_hyp_.end())
    {
      it_root++;
    }
    else
    {
      it_root = hyp_trees_.erase(it_root);
    }
  }
}

vector<StateNode::Ptr> MultiHypTracker::get_tracks()
{
  vector<StateNode::Ptr> roots;
  for (auto wRoot : selected_roots_)
  {
    roots.push_back(wRoot.lock());
  }
  return roots;
}
