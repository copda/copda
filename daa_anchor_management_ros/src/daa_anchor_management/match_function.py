# Copyright (c) 2023, DFKI GmbH and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# from scipy.stats import norm
import math
import numpy as np
from daa_knowledge_base import KBClient, KBQueryType


class MatchFunction:
    def __init__(self, kb_client: KBClient) -> None:
        self.kb_client = kb_client

    def state_prior(self, class_id: str):
        """
        Get prior distribution of an object.

        Args:
        ----
            class_id (str): class name

        Return:
        ------
            float: state prior (should be a distribution)

        """
        # TODO: get total number of existing instances for this class
        # TODO: get places where new instance can appear
        # TODO: construct uniform distribution for those places
        # TODO: return the prior as a function of attributes e.g. position
        instance_count = self.kb_client.query(type=KBQueryType.GET_INSTANCE_COUNT, data=class_id)
        table_size = 1.4 * 0.7  # TODO: hard coded table size
        table_num = int(3)
        current_count = instance_count.get('current_count')
        max_count = instance_count.get('max_count')
        if current_count >= 0 and max_count >= 0:
            prob_new_instance = 0.1 if current_count < max_count else 0.0
            prior = prob_new_instance * (1.0 / (table_num * table_size))
        else:  # no prior knowledge for maximum count
            prior = 1.0 / table_size
        return prior

    def new_score(self, obs: dict):
        # TODO: invoke state_prior(obs)
        prior = self.state_prior(obs['class_id'])
        score = math.log(prior) if prior > 0 else -1000.0
        return score

    def fitness(self, state: dict, obs: dict):
        """
        Calculate the likelihood given the state.

        TODO: implement the calculation

        Args:
        ----
            state (dict): Object's state
            obs (dict): Object's observation

        Return:
        ------
            float: Likelihood

        """
        fitness = 0
        for key, value in state.items():
            # calculate log likelihood for position
            if key == "position_distribution":
                obs_mean = np.array(obs[key]['mean'])
                # obs_cov = np.array(obs[key]['cov']).reshape((3, 3))
                state_mean = np.array(value['mean'])
                state_cov = np.array(value['cov']).reshape((3, 3))
                diff = state_mean - obs_mean
                # inv_cov = np.linalg.inv(state_cov + obs_cov)
                inv_cov = np.linalg.inv(state_cov)
                mahalanobis_dist_sq = np.dot(inv_cov @ diff, diff)
                # det = np.linalg.det(state_cov + obs_cov)
                det = np.linalg.det(state_cov)
                pi2_pow = math.pow(2 * np.pi, 3)
                pos_sqrt_pow = 1 / math.sqrt(pi2_pow * det)
                fitness += -0.5 * mahalanobis_dist_sq + math.log(pos_sqrt_pow)
        return fitness

    def topo_score(self, assignments, tracks):
        """
        Calculate topological score of a set of assignments.

        This is done with the help of the KB module, which gives the relational
        graphs in the form of matrices before and after the assignments of concern.

        Args:
        ----
            assignments (list): List of assignments to be evaluated
            tracks (list): Tracks which are referred to by the track ids in the
            assignments

        Returns
        -------
            float: Topological score

        """
        relation_matrices = self.kb_client.query(type=KBQueryType.GET_RELATION_MATRICES, data=None)
        num_relation_pairs = 0
        for _, mat in relation_matrices.items():
            num_relation_pairs += mat.size
        query_data = []
        for assign in assignments:
            query_data.append({"symbol": assign.symbol, "percepts": tracks[assign.track_id]})
        ret = self.kb_client.query(type=KBQueryType.INFER_RELATION_MATRICES, data=query_data)
        if ret is None:
            return 0

        inferred_matrices, reference_matrices = ret[0], ret[1]
        num_relation_pairs_inferred = 0
        topo_cost = 0
        for rel_name in inferred_matrices.keys():
            matrix_infer = inferred_matrices[rel_name]
            matrix_prev = reference_matrices[rel_name]
            if matrix_infer.size and matrix_prev.size:
                diff = matrix_infer - matrix_prev
                if np.linalg.matrix_rank(diff):
                    topo_cost += np.sum(np.abs(diff)).item()
                num_relation_pairs_inferred += matrix_infer.size

        # num_relation_pairs_diff = abs(num_relation_pairs - num_relation_pairs_inferred)
        # return -topo_cost - num_relation_pairs_diff
        return -topo_cost
