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

from owlready2 import DataProperty, FunctionalProperty, get_ontology, Thing
import numpy as np
from ast import literal_eval
import sys
from collections import deque

# These methods will be automatically imported when the onto is loaded

onto_uri = None
for arg in sys.argv:
    if arg == "--onto_uri":
        idx = sys.argv.index(arg)
        sys.argv.pop(idx)
        onto_uri = sys.argv[idx]
        sys.argv.pop(idx)
if not onto_uri:
    raise RuntimeError("[copda.methods]: failed to load ontology uri")
onto = get_ontology(onto_uri)
with onto:

    class position(DataProperty, FunctionalProperty):
        def from_list(value):
            return np.array2string(np.array(value), formatter={'float_kind': lambda x: "%.3f" % x})

    class orientation(DataProperty, FunctionalProperty):
        def from_list(value):
            return np.array2string(np.array(value), formatter={'float_kind': lambda x: "%.3f" % x})

    class dimensions(DataProperty, FunctionalProperty):
        def from_list(value):
            return np.array2string(np.array(value), formatter={'float_kind': lambda x: "%.3f" % x})

    class color(DataProperty, FunctionalProperty):
        def from_list(value):
            return ",".join(value)

    class confine(DataProperty, FunctionalProperty):
        def from_list(value):
            return np.array2string(np.array(value), formatter={'float_kind': lambda x: "%.3f" % x})

    class Distribution(Thing):
        pass

    class Gaussian(Distribution):
        def from_dict(self, value):
            self.mean = value["mean"]
            self.cov = value["cov"]

    class Surface(Thing):
        def set_confine(self, confine: np.ndarray):
            if len(confine.shape) > 1:
                confine = confine.flatten()
                assert confine.size == 6
            self.confine = onto.confine.from_list(confine)

        def np_confine(self):
            if self.confine:
                confine = np.array(literal_eval(self.confine.replace(' ', ',')))
                return confine.reshape((3, 2))
            else:
                return None

        def is_obj_within_confine(self, obj: onto.Object):
            surface_confine = self.np_confine()
            obj_pos = obj.expected_position()
            return np.all(obj_pos > surface_confine[:, 0]) and np.all(obj_pos < surface_confine[:, 1])

    class Object(Thing):
        def np_position(self):
            if self.position:
                return np.array(literal_eval(self.position.replace(' ', ',')))
            else:
                return None

        def expected_position(self):
            pos = self.np_position()
            return pos

        def expected_distance_to(self, obj: onto.Object):
            pos = self.expected_position()
            ref_pos = obj.expected_position()
            return np.linalg.norm(pos - ref_pos)

        def init(self, percepts):
            self.percept_history = deque(maxlen=5)
            self.created_time = percepts["timestamp"]
            self.deactivated = False
            self.update(percepts)

        def roll_back(self):
            if len(self.percept_history) > 1:
                last_percepts = self.percept_history[-2]
                self._apply_percepts(last_percepts)
                self.percept_history.pop()

        def _apply_percepts(self, percepts):
            for key, value in percepts.items():
                if key == "position":
                    self.position = onto.position.from_list(value)
                if key == "timestamp":
                    self.timestamp = value
                if key == "position_distribution":
                    pass  # TODO gaussian instances are not destroyed
                    # gaussian = Gaussian()
                    # gaussian.from_dict(value)
                    # self.position_distributions.append(gaussian)
                    # if len(self.position_distributions) > 5:
                    #     destroy_entity(self.position_distributions[0])
                if key == "orientation":
                    self.orientation = orientation.from_list(value)
                if key == "colors":
                    self.color = color.from_list(value)

        def merge(self, other):
            if type(self) is type(other):
                assert other.percept_history
                percepts = other.percept_history[-1]
                self.update(percepts)
                for rel in onto.relations_to_infer:
                    # copy also the relations
                    setattr(self, rel.name, getattr(other, rel.name))
            else:
                print("try to merge object of differnt type")

        def update(self, percepts):
            if (not self.timestamp) or self.timestamp < percepts["timestamp"]:
                self._apply_percepts(percepts)
                self.percept_history.append(percepts)

        def deactivate(self):
            self.deactivated = True

        def activate(self):
            self.deactivated = False

        def is_deactivated(self):
            return self.deactivated

    onto.relations_to_infer = [onto.contains, onto.supports]
