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

import message_filters
import rospy
from itertools import zip_longest
from daa_color_classification.msg import ColorClassificationResults, Colors
from object_pose_msgs.msg import ObjectList, ObjectPose
from vision_msgs.msg import Detection3DArray, Detection3D

COLORS_TO_INSTANCE_IDS = {
    "Traffic Red": 1,  # red_part
    "Traffic Yellow": 2,  # yellow_part
    "Yellow Green": 3,  # bright_green_part
    "Traffic Green": 4,  # dark_green_part
    "Purple": 5,  # purple_part
    "Telemagenta": 6,  # magenta_part
}


class FakeAnchoringNode:
    def __init__(self):
        # parameters
        self._class_ids = rospy.get_param('~class_ids')

        sub_listener = rospy.SubscribeListener()
        sub_listener.peer_subscribe = self.subscription_cb
        sub_listener.peer_unsubscribe = self.unsubscription_cb

        # publishers
        self._pub_object_list = rospy.Publisher(
            'anchored_objects', ObjectList, queue_size=10, subscriber_listener=sub_listener
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
        output_msg = ObjectList()
        output_msg.header = detection_array.header
        for det, colors in zip_longest(detection_array.detections, colors.colors):
            output_pose = self._make_output_pose(det, colors)
            if output_pose is not None:
                output_msg.objects.append(output_pose)

        self._pub_object_list.publish(output_msg)

    def _make_output_pose(self, det: Detection3D, colors: Colors):
        output_pose = ObjectPose()

        if len(det.results) != 1:
            rospy.logerr('Expected exactly 1 object hypothesis per detection, got %d', len(det.results))
            return None
        object_hypothesis = det.results[0]

        output_pose.pose = object_hypothesis.pose.pose
        output_pose.size = det.bbox.size
        # output_pose.min and output_pose.max left empty (not used)
        class_id_to_name = {class_id: name for name, class_id in self._class_ids.items()}
        output_pose.class_id = class_id_to_name[object_hypothesis.id]
        output_pose.instance_id = 1

        # "fake anchoring": for KLTs, determine the instance ID based on the color of the contents;
        # for everything else, leave id = 1
        if det.results[0].id == self._class_ids['klt'] and colors is not None:
            # find maximum color
            max_color = None
            max_intensity = float('-inf')
            for color, intensity in zip(colors.colors, colors.intensities):
                if intensity == 0.0:
                    continue
                if color not in COLORS_TO_INSTANCE_IDS:  # should only be "Blue"
                    continue
                if max_intensity < intensity:
                    max_intensity = intensity
                    max_color = color

            # "fake anchoring": assign instance ID based on maximum color of contents
            if max_color is None:
                # output_pose.instance_id = 0

                # In the current demo, we never have empty KLTs, so any empty KLT is actually a KLT where we didn't
                # recognize the color of the contents. This is never good and only provokes collisions with the real
                # KLT once it's detected. Thus, filter out empty KLTs here.
                return None
            else:
                output_pose.instance_id = COLORS_TO_INSTANCE_IDS[max_color]

        return output_pose


def main():
    rospy.init_node('color_object_publisher')
    FakeAnchoringNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
