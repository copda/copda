#!/usr/bin/env python

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

import rospy
import message_filters
import copy
import random

from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from daa_color_classification.msg import ColorClassificationResults, Colors

COLORS_TO_PART_NAMES = {
    "Traffic Red": "red_part",
    "Traffic Yellow": "yellow_part",
    "Yellow Green": "bright_green_part",
    "Traffic Green": "dark_green_part",
    "Purple": "purple_part",
    "Telemagenta": "magenta_part",
}


class ColorObjectPublisherNode:
    def __init__(self):
        random.seed()

        # parameters
        self._class_ids = rospy.get_param('~class_ids')

        sub_listener = rospy.SubscribeListener()
        sub_listener.peer_subscribe = self.subscription_cb
        sub_listener.peer_unsubscribe = self.unsubscription_cb

        # publishers
        self._pub_detected_objs = rospy.Publisher(
            'detected_objects_plus_colorparts', Detection3DArray, queue_size=10, subscriber_listener=sub_listener
        )

        # Subscriber variables
        self.detected_objs_sub = None
        self.color_classification_sub = None
        self.ts = None

    def subscription_cb(self, topic_name, topic_publish, peer_publish):
        if self.detected_objs_sub is None and self.color_classification_sub is None and self.ts is None:
            self.detected_objs_sub = message_filters.Subscriber('dope/detected_objects', Detection3DArray)
            self.color_classification_sub = message_filters.Subscriber(
                'color_classification', ColorClassificationResults
            )
            self.ts = message_filters.TimeSynchronizer(
                [self.detected_objs_sub, self.color_classification_sub], queue_size=100
            )
            self.ts.registerCallback(self._msg_callback)

    def unsubscription_cb(self, topic_name, num_peers):
        if num_peers == 0:
            self.detected_objs_sub.sub.unregister()
            self.detected_objs_sub = None
            self.color_classification_sub.sub.unregister()
            self.color_classification_sub = None
            self.ts = None

    def _msg_callback(self, detection_array: Detection3DArray, colors: ColorClassificationResults):
        output_msg = copy.deepcopy(detection_array)
        for det, colors in zip(detection_array.detections, colors.colors):
            if len(det.results) != 1:
                rospy.logerr('Expected exactly 1 object hypothesis per detection, got %d', len(det.results))
                continue
            if det.results[0].id != self._class_ids['klt']:
                continue
            output_msg.detections.extend(self._make_part_detections(det, colors))

        self._pub_detected_objs.publish(output_msg)

    def _make_part_detections(self, det: Detection3D, colors: Colors):
        dets = []
        for color, intensity in zip(colors.colors, colors.intensities):
            if intensity == 0.0:
                continue
            if color not in COLORS_TO_PART_NAMES:  # should only be "Blue"
                continue

            result = ObjectHypothesisWithPose()
            result.id = self._class_ids[COLORS_TO_PART_NAMES[color]]
            result.score = intensity
            result.pose = copy.deepcopy(det.results[0].pose)

            det_out = Detection3D()
            det_out.header = copy.deepcopy(det.header)
            det_out.results = [result]
            det_out.bbox = copy.deepcopy(det.bbox)
            det_out.bbox.size.x /= 2.0
            det_out.bbox.size.y /= 2.0
            det_out.bbox.size.z /= 2.0

            # just for better visualization:
            det_out.bbox.center.position.x += (random.random() - 0.5) * 0.10
            det_out.bbox.center.position.y += (random.random() - 0.5) * 0.10
            det_out.bbox.center.position.z += (random.random() - 0.5) * 0.10

            dets.append(det_out)
        return dets


def main():
    rospy.init_node('color_object_publisher')
    ColorObjectPublisherNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
