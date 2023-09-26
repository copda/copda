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

import numpy as np
import rospy
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class Detection3DToMarkersNode(object):
    def __init__(self):
        self.prev_num_detections = 0

        # parameters
        self.text_scale = rospy.get_param('~text_scale', 0.05)
        self.mesh_scales = {}
        self.mesh_urls = {}
        self.draw_colors = {}
        self.class_ids = rospy.get_param("~class_ids")
        for model in self.class_ids.keys():
            try:
                self.mesh_scales[model] = rospy.get_param('~mesh_scales')[model]
            except KeyError:
                self.mesh_scales[model] = 1.0

            try:
                self.mesh_urls[model] = rospy.get_param('~meshes')[model]
            except KeyError:
                pass

            try:
                self.draw_colors[model] = tuple(rospy.get_param("~draw_colors")[model])
            except KeyError:
                self.draw_colors[model] = (
                    np.random.randint(0, 255),
                    np.random.randint(0, 255),
                    np.random.randint(0, 255),
                )

        # publishers
        self.pub_markers = rospy.Publisher('markers', MarkerArray, queue_size=10)

        # subscribers
        self.sub_det3d = rospy.Subscriber("detected_objects", Detection3DArray, self.detections_callback)

    def detections_callback(self, detections_msg):
        self.publish_markers(detections_msg, self.prev_num_detections)
        self.prev_num_detections = len(detections_msg.detections)

    def publish_markers(self, detection_array, prev_num_detections, color=None, namespace_prefix=''):
        """
        Publish markers.

        :param Detection3DArray detection_array: The detections to visualize
        :param int prev_num_detections: The previous number of detections
        :param ColorRGBA color: The color to use for all markers. If 'None' (default), use separate color
                                from config for each object class.
        :param str namespace_prefix: Optional namespace prefix for the markers
        :return: nothing
        """
        # Object markers
        class_id_to_name = {class_id: name for name, class_id in self.class_ids.items()}
        markers = MarkerArray()
        for i, det in enumerate(detection_array.detections):
            if len(det.results) != 1:
                rospy.logerr('Expected exactly 1 object hypothesis per detection, got %d', len(det.results))
                continue
            name = class_id_to_name[det.results[0].id]
            if color:
                color_rgba = color
            else:
                color_rgba = ColorRGBA()
                draw_color = self.draw_colors[name]
                color_rgba.r = draw_color[0] / 255.0
                color_rgba.g = draw_color[1] / 255.0
                color_rgba.b = draw_color[2] / 255.0
                color_rgba.a = 1.0

            # cube marker
            marker = Marker()
            marker.header = detection_array.header
            marker.action = Marker.ADD
            marker.pose = det.bbox.center
            marker.color = color_rgba
            marker.color.a = 0.4
            marker.ns = namespace_prefix + "bboxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.scale = det.bbox.size
            markers.markers.append(marker)

            # text marker
            marker = Marker()
            marker.header = detection_array.header
            marker.action = Marker.ADD
            marker.pose = det.bbox.center
            marker.color = color_rgba
            marker.color.a = 1.0
            marker.id = i
            marker.ns = namespace_prefix + "texts"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.scale.x = self.text_scale
            marker.scale.y = self.text_scale
            marker.scale.z = self.text_scale
            marker.text = '{} ({:.2f})'.format(name, det.results[0].score)
            markers.markers.append(marker)

            # mesh marker
            try:
                marker = Marker()
                marker.header = detection_array.header
                marker.action = Marker.ADD
                marker.pose = det.bbox.center
                marker.color = color_rgba
                marker.color.a = 0.7
                marker.ns = namespace_prefix + "meshes"
                marker.id = i
                marker.type = Marker.MESH_RESOURCE
                marker.scale.x = self.mesh_scales[name]
                marker.scale.y = self.mesh_scales[name]
                marker.scale.z = self.mesh_scales[name]
                marker.mesh_resource = self.mesh_urls[name]
                markers.markers.append(marker)
            except KeyError:
                # user didn't specify self.meshes[name], so don't publish marker
                pass

        for i in range(len(detection_array.detections), prev_num_detections):
            for ns in [namespace_prefix + s for s in ["bboxes", "texts", "meshes"]]:
                marker = Marker()
                marker.action = Marker.DELETE
                marker.ns = ns
                marker.id = i
                markers.markers.append(marker)

        self.pub_markers.publish(markers)


def main():
    rospy.init_node('detection3d_to_markers')
    Detection3DToMarkersNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
