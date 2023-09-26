#!/usr/bin/env python3

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

import json
import rospy
from daa_anchor_management import AnchorManagement
from daa_kb_client import KBClient_ROS
from daa_tracker_ros.msg import TrackArray
import logging

logging.basicConfig(level=logging.DEBUG)


class AnchorManagementNode:
    def __init__(self):
        """Initialize AnchorManagementNode."""
        rospy.Subscriber("daa_tracker/tracks", TrackArray, self.track_callback, queue_size=100)
        self.anchor_management = AnchorManagement(
            KBClient_ROS(query_service_name='daa_knowledge_base/query', update_service_name='daa_knowledge_base/update')
        )

    def track_callback(self, track_array_msg: TrackArray):
        """
        Define the callback function for TrackArray messages.

        Args:
        ----
            track_array_msg (TrackArray): TrackArray message

        """
        tracks = []
        for track in track_array_msg.tracks:
            track_id = track.track_id
            # Parse percepts from string
            percepts = json.loads(track.percepts)
            tracks.append({'track_id': track_id, 'percepts': percepts})
        # Run the AM once
        self.anchor_management.add_tracks(tracks)
        self.anchor_management.run_once()


if __name__ == '__main__':
    rospy.init_node('daa_anchor_management')
    am = AnchorManagementNode()
    rospy.spin()
