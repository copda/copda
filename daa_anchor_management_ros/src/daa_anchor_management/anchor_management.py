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

import logging
import numpy as np
import networkx as nx
from daa_knowledge_base import KBClient, KBQueryType, KBUpdateType
from daa_anchor_management.anchor import Anchor
from daa_anchor_management.match_function import MatchFunction


class Assignment:
    def __init__(
        self,
        symbol: str,
        track_id: np.uint32,
    ) -> None:
        """
        Initialize the Assignment object.

        Args:
        ----
                symbol: The symbol of the instance.
                track_id: The track's id.


        """
        self.symbol: str = symbol
        self.track_id: np.uint32 = track_id
        self.score: np.double = np.double(0.0)

    @staticmethod
    def is_compatible(ass_a, ass_b):
        """
        Check if two assignments are compatible. Compatible means that they don't have the same symbol or track_id.

        Args:
        ----
                ass_a: Assignment
                ass_b: Assignment

        """
        # Check if two assignments have undefined symbols.
        if (ass_a.symbol == "") or (ass_b.symbol == ""):
            if ass_a.track_id == ass_b.track_id:
                return False
            else:
                return True
        if (ass_a.symbol == ass_b.symbol) or (ass_a.track_id == ass_b.track_id):
            return False
        else:
            return True


class AnchorManagement:
    def __init__(self, kb_client: KBClient = None) -> None:
        """
        Initialize the AnchorManagement. An instance of KBClient is provided to communicate with the knowledge base.

        Args:
        ----
                kb_client: An KBClient instance


        """
        self.dangling_tracks = {}
        self.track_buffer = []
        self.anchors: list(Anchor) = []
        self.track2anchor: dict = {}
        self.symbol2anchor: dict = {}
        self.match_function = MatchFunction(kb_client)
        self.timestamp = 0.0
        # TODO
        self.config: dict = {}
        self.kb_client: KBClient = kb_client

    def add_tracks(self, tracks: list):
        """
        Add tracks to the buffer.

        Args:
        ----
                tracks: list of Track objects

        """
        self.track_buffer.append(tracks)
        logging.debug(f"track buffer size: {len(self.track_buffer)}")

    def run_once(self, timestamp):
        """Run one iteration of the algorithm. This is the main entry point for the algorithm."""
        self.timestamp = timestamp
        self._detect_inactive_anchors()
        self._update_anchor_percepts()
        self._infer_anchor_percepts()
        assignments = self._generate_assignments()
        anchor_updated = False
        # Evaluate the assignments and apply the best assignments to the current
        # anchors.
        if len(assignments) > 0:
            self._eval_assignments(assignments)
            best_assignments = self._find_best_assignments(assignments)
            self._apply_assignments(best_assignments)
            anchor_updated = True

        self._push_updates()
        # Update the anchor mappings if the anchor_updated flag is set.
        if anchor_updated:
            self._update_anchor_mappings()
        self.dangling_tracks.clear()

    def _detect_inactive_anchors(self):
        """Deactivate anchors which is linked to a deactivated instance in the knowledge base."""
        all_ins = self.kb_client.query(type=KBQueryType.GET_ALL_INSTANCES, data=None)
        for anchor in self.anchors:
            if not anchor.is_deactivated() and anchor.symbol not in all_ins:
                anchor.deactivate()
            if anchor.symbol == Anchor.UNDEFINED_SYMBOL:
                logging.warning("undefined anchor")
                anchor.deactivate()

    def _update_anchor_percepts(self):
        """Update percepts for active anchors and find dangling tracks for the following stages."""
        tracks = self.track_buffer.pop(0)
        # Mark all anchors as lost.
        for anchor in self.anchors:
            anchor.mark_lost()
        # Update active anchors whose track ids can be found in the track input.
        # Otherwise mark them as dangling tracks.
        for t in tracks:
            anchor = self.track2anchor.get(t["track_id"])
            # Update the anchor's percepts.
            if anchor is not None:
                updated = anchor.update_percepts(self._decode_percepts(t["percepts"]))
                # Mark the anchor as lost if the percept update is rejected.
                if not updated:
                    anchor.mark_lost()
                    self.dangling_tracks.update({t["track_id"]: self._decode_percepts(t["percepts"])})
            else:
                self.dangling_tracks.update({t["track_id"]: self._decode_percepts(t["percepts"])})

    def _apply_assignments(self, assignments):
        """
        Apply assignments to the anchors. This is called after we have the best assignments.

        Args:
        ----
                assignments: list of Assignments

        """
        # Add anchor to the anchor list of assignments
        for assign in assignments:
            percepts = self.dangling_tracks.get(assign.track_id)
            # Add an anchor to the list of anchors.
            if assign.symbol == "":
                # create new anchor
                self.anchors.append(Anchor(symbol=Anchor.UNDEFINED_SYMBOL, track_id=assign.track_id, percepts=percepts))
            else:  # update anchor
                anchor = self.symbol2anchor.get(assign.symbol)
                assert anchor is not None
                anchor_holding_track = self.track2anchor.get(assign.track_id)
                if anchor is not anchor_holding_track and anchor_holding_track is not None:

                    anchor_holding_track.set_to_prev_track_id()
                    self.track2anchor.update({anchor_holding_track.prev_track_id: anchor_holding_track})
                anchor.set_track_id(assign.track_id)
                anchor.update_percepts(percepts)

    def _infer_anchor_percepts(self):
        """Get the inferred anchor percepts from the KB."""
        for anchor in self.anchors:
            if anchor.is_lost():
                # TODO: get inferred percepts from KB
                pass

    def _generate_assignments(self):
        """
        Enumerate all possible combinations and generate candidate assignments for evaluation.

        Returns
        -------
                list of Assignment

        """
        assignments = []
        # Returns a list of assignments for each dangling track.
        for track_id in self.dangling_tracks.keys():
            # Assignments representing new anchors
            if self.dangling_tracks[track_id]['obs_id'] > 0:
                assignments.append(Assignment(symbol="", track_id=track_id))
            # Assignments representing reacquiring
            # Add a assignment to the assignments list.
            for anchor in self.anchors:
                # If anchor is lost add a assignment to assignments list.
                if anchor.is_lost() and not anchor.is_deactivated():
                    res = self.kb_client.query(
                        type=KBQueryType.IS_POTENTIAL_MATCH, symbol=anchor.symbol, data=self.dangling_tracks[track_id]
                    )
                    # Add a new assignment to the assignment list.
                    if res:
                        assignments.append(Assignment(symbol=anchor.symbol, track_id=track_id))
        return assignments

    def _eval_assignments(self, assignments):
        """
        Evaluate the score of each assignment.

        Assignments are evaluated by the match function at the perceptual level.

        Args:
        ----
                assignments: list of Assignment

        """
        # Calculates the score of the assignment score for each assignment.
        for assign in assignments:
            anchor = self.symbol2anchor.get(assign.symbol)
            # fitness of the match function
            if anchor is not None:
                assign.score = self.match_function.fitness(anchor.percepts, self.dangling_tracks[assign.track_id])
            else:
                assign.score = self.match_function.new_score(self.dangling_tracks[assign.track_id])

    def _find_best_assignments(self, assignment_list):
        """
        Find the best assignments in the input list.

        This is done by forming a graph and finding all the cliques and evaluating them. The best
        assignment set is the one that has highest score, which is the weighted
        sum of perceptual and topological scores

        Args:
        ----
                assignment_list: list of assignments to be evaluated

        Returns
        -------
                a list of Assignment

        """
        G = nx.Graph()
        # Add ass_id to the graph.
        for ass_id in range(len(assignment_list)):
            G.add_node(ass_id)
        # Add edges between the two assignments.
        for ass_a_id in range(len(assignment_list) - 1):
            # Add edges between the two assignments.
            for ass_b_id in range(ass_a_id + 1, len(assignment_list)):
                ass_a = assignment_list[ass_a_id]
                ass_b = assignment_list[ass_b_id]
                # Add an edge between ass_a and ass_b to the graph.
                if Assignment.is_compatible(ass_a, ass_b):
                    G.add_edge(ass_a_id, ass_b_id)

        # Find all cliques and evaluate
        assignment_sets = []
        # Find cliques in the graph G.
        for clq in nx.clique.find_cliques(G):
            assignment_set = {}
            assignment_set["assignments"] = []
            # Add assignment to assignment_set.
            for idx in clq:
                assignment_set["assignments"].append(assignment_list[idx])
            assignment_sets.append(assignment_set)

        weights = {
            "topo_score": 10.0,
            "percept_score": 1.0,
        }
        # Apply the topo_score percept_score to each assignment set.
        for assignment_set in assignment_sets:
            percept_score = sum(assignment.score for assignment in assignment_set["assignments"])

            topo_score = self.match_function.topo_score(assignment_set["assignments"], self.dangling_tracks)
            assignment_set["score"] = weights["topo_score"] * topo_score + weights["percept_score"] * percept_score
            # for assignment in assignment_set:
            #     assignment.score += weights["topo_score"] * topo_score

        best_assignment_set = max(assignment_sets, key=lambda x: x["score"])
        return best_assignment_set["assignments"]

    def _update_anchor_mappings(self):
        """Update symbol-anchor and track-anchor mappings."""
        self.symbol2anchor.clear()
        self.track2anchor.clear()
        # Update all the anchor objects in the anchor list
        for anchor in self.anchors:
            self.symbol2anchor.update({anchor.symbol: anchor})
            self.track2anchor.update({anchor.track_id: anchor})

    def _push_updates(self):
        """Push updates to KB. The updates can be: 1. new anchor was created; 2. update percepts of existing anchors."""
        for anchor in self.anchors:
            # New anchor was created and a new symbol is generated by the KB
            if anchor.symbol == Anchor.UNDEFINED_SYMBOL:
                try:
                    new_symbol = self.kb_client.update(type=KBUpdateType.CREATE_INSTANCE, data=anchor.percepts)  # TODO
                except Exception as e:  # TODO: create exception class for this
                    logging.error(e)
                anchor.symbol = new_symbol
                logging.debug(f"create new anchor for: {new_symbol}")
            # Update the active anchor
            elif not anchor.is_lost() and not anchor.is_deactivated():
                self.kb_client.update(type=KBUpdateType.UPDATE_INSTANCE, data=anchor.percepts, symbol=anchor.symbol)
        res = self.kb_client.update(type=KBUpdateType.COMMIT_UPDATES, data=str(self.timestamp))
        self._merge_matches(res.get("matches"))

    def _merge_matches(self, matches: dict):
        for symbol_to_keep, symbol_to_discard in matches.items():
            # TODO
            self._update_anchor_mappings()
            anchor_to_keep = self.symbol2anchor.get(symbol_to_keep)
            anchor_to_discard = self.symbol2anchor.get(symbol_to_discard)
            anchor_to_keep.merge(anchor_to_discard)
            self.anchors.remove(anchor_to_discard)

    def _decode_percepts(self, msg: dict) -> dict:
        """
        Decode percepts from parsed from the json message.

        TODO: this will be removed in the future.

        Args:
        ----
                msg: A dictionary

        Returns
        -------
                A dictionary containing the following keys: [
                "timestamp",
                "orientation",
                "position",
                "position_distribution",
                "class_id",
                "colors",
                "color_histogram",
            ]

        """
        percepts = {}
        percepts["timestamp"] = msg.get("timestamp")
        percepts["obs_id"] = msg.get("obs_id")
        percepts["orientation"] = msg.get("orientation")
        percepts["position"] = msg.get("position")["mean"]
        percepts["position_distribution"] = msg.get("position")
        percepts["class_id"] = list(msg.get("class_id")["values"].keys())[0]
        percepts["colors"] = []
        percepts["color_histogram"] = {}
        # Update colors in percepts colors.
        if msg.get("color_histogram") and msg.get("color_histogram")["values"]:
            # Update colors in the color histogram.
            for key, value in msg.get("color_histogram")["values"].items():
                color_name = self._colors[key]
                percepts["color_histogram"].update({color_name: value})
                if value > 0:
                    percepts["colors"].append(color_name)
        return percepts
