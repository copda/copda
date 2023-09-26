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

from collections import deque
import numpy as np
import copy


class Anchor:
    """Define the Anchor class."""

    UNDEFINED_SYMBOL = "undefined_symbol"

    def __init__(
        self,
        symbol: str,
        track_id,
        percepts: dict,
    ):
        self.symbol: str = symbol
        self.prev_track_id = track_id
        self.track_id = track_id
        self.percepts: dict = percepts
        self.timestamp: float = percepts['timestamp']
        self.lost: bool = False
        self.history = deque(maxlen=100)
        self.deactivated: bool = False
        # self.track_history = deque(maxlen=10)

    def is_lost(self):
        return self.lost

    def is_deactivated(self):
        return self.deactivated

    def mark_lost(self):
        self.lost = True

    def deactivate(self):
        self.deactivated = True

    def set_track_id(self, track_id):
        if track_id != self.track_id:
            self.prev_track_id = copy.copy(self.track_id)
            self.track_id = track_id

    def set_to_prev_track_id(self):
        self.track_id = copy.copy(self.prev_track_id)

    def is_update_plausible(self, percepts: dict):
        time_forward = self.timestamp < percepts['timestamp']
        distance = np.array(self.percepts['position']) - np.array(percepts['position'])
        distance = np.linalg.norm(distance)
        dt = percepts['timestamp'] - self.timestamp
        if dt > 0:
            velocity = distance / dt
            velocity_in_bound = velocity < 0.1  # TODO: move to config
        else:
            velocity_in_bound = distance < 0.1  # TODO: move to config
        # Debug
        if not velocity_in_bound:
            print('reject update due to high vel')

        return time_forward and velocity_in_bound

    def update_percepts(self, percepts: dict):
        """
        Update anchor percepts.

        Args:
        ----
            percepts (dict): new percepts

        Returns
        -------
            bool: True if the percepts are updated

        """
        if self.percepts is not None:
            # Only update when the incoming percepts are newer
            if self.is_update_plausible(percepts):
                self.history.append(self.percepts.copy())
                self.percepts = percepts.copy()
                self.timestamp = percepts['timestamp']
                if percepts['obs_id'] == 0:  # dummy observation
                    return False
                else:
                    self.lost = False
                    return True
            else:
                return False
