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

import cv2

import message_filters
import rospy
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3DArray


class Detection3DToImgNode(object):
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.camera_model = PinholeCameraModel()

        # parameters
        self.class_ids = rospy.get_param('~class_ids')
        self.draw_colors = rospy.get_param('~draw_colors')

        # publishers
        self.pub_rgb_dope_points = rospy.Publisher('~detected_objects_img', Image, queue_size=10)
        self.pub_camera_info = rospy.Publisher('~camera_info', CameraInfo, queue_size=10)

        # subscribers
        # NOTE: input image must be rectified
        image_sub = message_filters.Subscriber('~image_rect_in', Image)
        info_sub = message_filters.Subscriber('~camera_info_in', CameraInfo)
        det_sub = message_filters.Subscriber('~detected_objects_in', Detection3DArray)
        ts = message_filters.TimeSynchronizer([det_sub, image_sub, info_sub], queue_size=100)
        ts.registerCallback(self.image_callback)

    def image_callback(self, detections_msg, image_msg, camera_info_msg):
        self.camera_model.fromCameraInfo(camera_info_msg)
        img = self.cv_bridge.imgmsg_to_cv2(image_msg, 'rgb8')

        class_id_to_name = {class_id: name for name, class_id in self.class_ids.items()}
        for det in detections_msg.detections:
            if len(det.results) != 1:
                rospy.logerr('Expected exactly 1 object hypothesis per detection, got %d', len(det.results))
                continue
            class_name = class_id_to_name[det.results[0].id]

            # project bounding box center to 2D pixel coordinates
            bbox_center_3d = det.bbox.center.position
            bbox_center_uv = self.camera_model.project3dToPixel((bbox_center_3d.x, bbox_center_3d.y, bbox_center_3d.z))

            # paint small circle + text onto image at bbox center (for debugging)
            color = self.draw_colors[class_name]
            center = (int(bbox_center_uv[0]), int(bbox_center_uv[1]))
            cv2.circle(img, center=center, radius=10, color=color, thickness=(-1))
            cv2.putText(
                img,
                text=class_name,
                org=center,
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1,
                color=color,
                thickness=2,
            )

        # Publish the image with results overlaid
        rgb_points_img = CvBridge().cv2_to_imgmsg(img, 'rgb8')
        rgb_points_img.header = camera_info_msg.header
        self.pub_rgb_dope_points.publish(rgb_points_img)
        self.pub_camera_info.publish(camera_info_msg)


def main():
    rospy.init_node('detection3d_to_img')
    Detection3DToImgNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
