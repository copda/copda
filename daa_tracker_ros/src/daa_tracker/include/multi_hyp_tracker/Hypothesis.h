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
 * @file Hypothesis.h
 * @author Zongyao Yi (zongyao.yi@dfki.de)
 * @brief Define Hypothesis class
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <vector>
#include "State.h"

using namespace std;

namespace daa
{
namespace multi_hyp_tracker
{
/**
 * @brief Hypothesis class, assignment of two states
 *
 */
struct Hypothesis
{
public:
  typedef int ValidCriteria;
  enum ValidCriterion
  {
    THRESHOLD = 1 << 0,
    DUMMY_OBS = 1 << 1
  };
  struct ValidationParam
  {
    double threshold = 50;
    int max_num_dum_scan = 5;
  };
  /**
   * @brief Construct from a state node
   *
   * @param node
   */
  Hypothesis(StateNode::Ptr node) : node_(node), is_valid(false)
  {
    // TODO: accumulate scores, get obs_ids
    score = 0;
    auto n = node;
    root_ = node;
    while (n)
    {
      obs_ids.push_back(n->item()->obs_id());
      score += n->item()->log_prob();
      n = n->parent();
      root_ = n ? n : root_;
    }
    assert(node->depth() + 1 == obs_ids.size());
  }
  /**
   * @brief Get root
   *
   * @return const StateNode::Ptr
   */
  const StateNode::Ptr root() const
  {
    return root_.lock();
  }
  /**
   * @brief Get node
   *
   * @return const StateNode::Ptr
   */
  const StateNode::Ptr node() const
  {
    return node_.lock();
  }
  vector<size_t> obs_ids;  // squence of obs ids, front: latest, back: oldest
  double score;
  bool is_valid;

private:
  StateNode::wPtr node_;
  StateNode::wPtr root_;

public:
  /**
   * @brief Mark hypotheses according to the provided criteria
   *
   * @param hyps
   * @param criterion
   * @param param
   */
  static void verify(vector<Hypothesis>& hyps, const ValidCriteria criterion, const ValidationParam param)
  {
    bool with_thres = criterion & (1 << 0);
    bool with_dum_obs = criterion & (1 << 1);

    for (auto& hyp : hyps)
    {
      hyp.is_valid = true;
      if (with_thres)
      {
        if (hyp.score < param.threshold)
          hyp.is_valid = false;
      }
      if (with_dum_obs)
      {
        int num_dum_obs = 0;
        for (auto it_obs_id = hyp.obs_ids.rbegin(); it_obs_id != hyp.obs_ids.rend(); it_obs_id++)
        {
          if (*it_obs_id == 0)
            num_dum_obs++;
          if (num_dum_obs == param.max_num_dum_scan)
          {
            hyp.is_valid = false;
            break;
          }
        }
      }
    }
  }
  /**
   * @brief Check if two tracks are compatible
   *
   * @param track_a
   * @param track_b
   * @return true
   * @return false
   */
  static bool is_compatible(Hypothesis track_a, Hypothesis track_b)
  {
    if (track_a.root() == track_b.root())  // TODO
      return false;
    for (auto it_a = track_a.obs_ids.begin(), it_b = track_b.obs_ids.begin();
         (it_a != track_a.obs_ids.end()) && (it_b != track_b.obs_ids.end()); it_a++, it_b++)
    {
      if ((*it_a) == (*it_b) && (*it_a != 0))  // not including dummy obs
        return false;
    }
    return true;
  }
};

}  // namespace multi_hyp_tracker
}  // namespace daa
