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

import math
import colorsys
import copy
import random

import rospy
from daa_msgs.msg import AnchoredObjectArray
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class AnchoredObjectsToMarkersNode(object):
    def __init__(self):
        self.prev_num_anchored_objects = 0

        # parameters
        self.text_scale = rospy.get_param('~text_scale', 0.05)
        self.text_color = ColorRGBA(r=249, g=44, b=0)
        self.mesh_scales = {}
        self.mesh_urls = {}
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
        # maximum number of objects (only relevant for colors)
        self.max_objects = rospy.get_param('~max_objects', 40)
        self.colors = random_colors(self.max_objects)

        # publishers
        self.pub_markers = rospy.Publisher('markers', MarkerArray, queue_size=10)

        # subscribers
        self.sub_anchored_objects = rospy.Subscriber(
            "anchored_objects", AnchoredObjectArray, self.anchored_objects_callback
        )

    def anchored_objects_callback(self, anchored_objects_msg: AnchoredObjectArray):
        self.publish_markers(anchored_objects_msg, self.prev_num_anchored_objects)
        self.prev_num_anchored_objects = len(anchored_objects_msg.objects)

    def publish_markers(self, anchored_objects_array, prev_num_anchored_objects, namespace_prefix=''):
        """
        Publish markers.

        :param AnchoredObjectArray anchored_objects_array: The anchored objects to visualize
        :param int prev_num_anchored_objects: The previous number of anchored objects
        :param str namespace_prefix: Optional namespace prefix for the markers
        :return: nothing
        """
        # Object markers
        class_id_to_name = {class_id: name for name, class_id in self.class_ids.items()}
        markers = MarkerArray()
        for i, obj in enumerate(anchored_objects_array.objects):
            class_name = class_id_to_name[obj.classification.id]
            color_rgba = copy.deepcopy(self.colors[obj.instance_id % self.max_objects])

            # cube marker
            marker = Marker()
            marker.header = anchored_objects_array.header
            marker.action = Marker.ADD
            marker.pose = obj.bbox.center
            marker.color = color_rgba
            marker.color.a = 0.4
            marker.ns = namespace_prefix + "bboxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.scale = obj.bbox.size
            markers.markers.append(marker)

            # text marker
            marker = Marker()
            marker.header = anchored_objects_array.header
            marker.action = Marker.ADD
            marker.pose = obj.bbox.center
            marker.color = self.text_color
            marker.color.a = 1.0
            marker.id = i
            marker.ns = namespace_prefix + "texts"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.scale.x = self.text_scale
            marker.scale.y = self.text_scale
            marker.scale.z = self.text_scale
            marker.text = obj.name
            markers.markers.append(marker)

            # mesh marker
            try:
                marker = Marker()
                marker.header = anchored_objects_array.header
                marker.action = Marker.ADD
                marker.pose = obj.classification.pose.pose
                marker.color = color_rgba
                marker.color.a = 0.7
                marker.ns = namespace_prefix + "meshes"
                marker.id = i
                marker.type = Marker.MESH_RESOURCE
                marker.scale.x = self.mesh_scales[class_name]
                marker.scale.y = self.mesh_scales[class_name]
                marker.scale.z = self.mesh_scales[class_name]
                marker.mesh_resource = self.mesh_urls[class_name]
                markers.markers.append(marker)
            except KeyError:
                # user didn't specify self.meshes[name], so don't publish marker
                pass

            # covariance marker for position
            marker = Marker()
            marker.header = anchored_objects_array.header
            marker.action = Marker.ADD
            marker.pose = obj.classification.pose.pose
            marker.color = color_rgba
            marker.color.a = 0.4
            marker.ns = namespace_prefix + "covariances"
            marker.id = i
            marker.type = Marker.SPHERE
            # display 95% interval: μ ± 2𝜎
            # - the (x, x), (y, y) and (z, z) entries in the covariance matrix are 𝜎², therefore sqrt
            # - the marker scale is 2 * radius, therefore 4 and not 2.
            marker.scale.x = 4 * math.sqrt(obj.classification.pose.covariance[0])
            marker.scale.y = 4 * math.sqrt(obj.classification.pose.covariance[7])
            marker.scale.z = 4 * math.sqrt(obj.classification.pose.covariance[14])

            markers.markers.append(marker)

        for i in range(len(anchored_objects_array.objects), prev_num_anchored_objects):
            for ns in [namespace_prefix + s for s in ["bboxes", "texts", "meshes", "covariances"]]:
                marker = Marker()
                marker.action = Marker.DELETE
                marker.ns = ns
                marker.id = i
                markers.markers.append(marker)

        self.pub_markers.publish(markers)


def random_colors(N, bright=True):
    """
    Generate random colors.

    To get visually distinct colors, generate them in HSV space then convert to RGB.

    :param N: The number of colors to generate.
    :type N: int
    :param bright: Whether to generate bright colors. Default is True.
    :type bright: bool
    :return: A list of ColorRGBA objects representing the generated random colors.
    :rtype: list
    """
    # NOTE: This function could be replaced later by something more sophisticated such as distinctipy. This would
    # provide a generator to always return the next most distinctive color compared to the ones already in use,
    # so we would not require N (= max_objects) anymore.
    brightness = 1.0 if bright else 0.7
    hsv = [(i / N, 1, brightness) for i in range(N)]
    colors = list(map(lambda c: colorsys.hsv_to_rgb(*c), hsv))
    random.shuffle(colors)
    colors_rgba = [ColorRGBA(r=r, g=g, b=b, a=1.0) for (r, g, b) in colors]
    return colors_rgba


def main():
    rospy.init_node('anchored_objects_to_markers')
    AnchoredObjectsToMarkersNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
