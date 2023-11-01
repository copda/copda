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

import rospy
import json
import numpy as np
from object_pose_msgs.msg import ObjectList, ObjectPose
from daa_msgs.msg import AnchoredObjectArray, AnchoredObject
from daa_knowledge_base_ros.srv import Query, QueryRequest
from std_srvs.srv import Trigger
import tf.transformations


class KnowledgeBasePublisher:
    def __init__(self, config) -> None:
        self._config = config
        query_service_name = 'daa_knowledge_base/query'
        rospy.wait_for_service(query_service_name)
        self._query_service_proxy = rospy.ServiceProxy(query_service_name, Query)

        clear_service_name = '/pick_pose_selector_node/pose_selector_clear'
        try:
            rospy.wait_for_service(clear_service_name, timeout=10.0)
            self._clear_service_proxy = rospy.ServiceProxy(clear_service_name, Trigger)
        except rospy.ROSException:
            rospy.logwarn("Timeout while waiting for service %s, continuing without clearing!", clear_service_name)
            self._clear_service_proxy = None

        self._pub_object_list = rospy.Publisher('object_list', ObjectList, queue_size=10)
        self._pub_anchored_objects = rospy.Publisher('anchored_objects', AnchoredObjectArray, queue_size=10)
        self._dimensions = self._make_dimensions()

    def _make_dimensions(self):
        class DefaultDictWithArgs(dict):
            def __init__(self, factory):
                super().__init__()
                self.factory = factory

            def __missing__(self, key):
                self[key] = self.factory(key)
                return self[key]

        def get_model_transform(model):
            try:
                M = np.array(rospy.get_param('~model_transforms')[model], dtype='float64')
                return tf.transformations.quaternion_from_matrix(M)
            except KeyError:
                return np.array([0.0, 0.0, 0.0, 1.0], dtype='float64')

        def rotate_vector(vector, quaternion):
            q_conj = tf.transformations.quaternion_conjugate(quaternion)
            vector = np.array(vector, dtype='float64')
            vector = np.append(vector, [0.0])
            vector = tf.transformations.quaternion_multiply(q_conj, vector)
            vector = tf.transformations.quaternion_multiply(vector, quaternion)
            return vector[:3]

        def get_dimensions(model):
            dims = tuple(rospy.get_param("~onto_config/dimensions")[model])

            # rotate bbox dimensions if necessary
            # (this only works properly if model_transform is in 90 degree angles)
            dims = rotate_vector(vector=dims, quaternion=get_model_transform(model))
            dims = np.absolute(dims)

            # scale to meters
            CONVERT_SCALE_CM_TO_METERS = 100
            dims /= CONVERT_SCALE_CM_TO_METERS
            dims = tuple(dims)

            return dims

        return DefaultDictWithArgs(get_dimensions)

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self._publish()
            rate.sleep()

    def _publish(self):
        try:
            req = self._query_service_proxy(header=QueryRequest.GET_ALL_INSTANCES, data=None)
            if req.data:
                all_instances = json.loads(req.data)
            else:
                return
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return
        try:
            if self._clear_service_proxy:
                self._clear_service_proxy()
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
        anchored_objects_msg = AnchoredObjectArray()
        anchored_objects_msg.header.frame_id = self._config["global_frame"]
        pose_selector_objects_msgs = ObjectList()
        for symbol, percepts in all_instances.items():
            obj_msg = AnchoredObject()
            obj_msg.header.frame_id = anchored_objects_msg.header.frame_id
            obj_msg.header.stamp = rospy.Time.from_sec(percepts['timestamp'])
            if anchored_objects_msg.header.stamp < obj_msg.header.stamp:
                anchored_objects_msg.header.stamp = obj_msg.header.stamp

            obj_msg.instance_id = int(symbol.rsplit(sep='_', maxsplit=1)[1])
            obj_msg.name = symbol

            latest_percept = percepts
            obj_msg.classification.id = self._config["class_ids"][latest_percept['class_id']]
            # TODO: obj_msg.classification.score
            obj_msg.classification.score = 1.0
            obj_msg.classification.pose.pose.position.x = latest_percept['position'][0]
            obj_msg.classification.pose.pose.position.y = latest_percept['position'][1]
            obj_msg.classification.pose.pose.position.z = latest_percept['position'][2]
            # TODO: obj_msg.classification.pose.pose.orientation
            obj_msg.classification.pose.pose.orientation.w = latest_percept['orientation'][3]
            obj_msg.classification.pose.pose.orientation.x = latest_percept['orientation'][0]
            obj_msg.classification.pose.pose.orientation.y = latest_percept['orientation'][1]
            obj_msg.classification.pose.pose.orientation.z = latest_percept['orientation'][2]
            # position covariance: from 3x3 to 6x6 matrix
            obj_msg.classification.pose.covariance[0:3] = latest_percept['position_distribution']['cov'][0:3]
            obj_msg.classification.pose.covariance[6:9] = latest_percept['position_distribution']['cov'][3:6]
            obj_msg.classification.pose.covariance[12:15] = latest_percept['position_distribution']['cov'][6:9]
            # TODO: orientation covariance
            obj_msg.classification.pose.covariance[21] = 1.0
            obj_msg.classification.pose.covariance[28] = 1.0
            obj_msg.classification.pose.covariance[35] = 1.0

            obj_msg.bbox.center = obj_msg.classification.pose.pose

            model = symbol.rsplit(sep='_', maxsplit=1)[0]
            dims = self._dimensions[model]
            obj_msg.bbox.size.x = dims[0]
            obj_msg.bbox.size.y = dims[1]
            obj_msg.bbox.size.z = dims[2]

            anchored_objects_msg.objects.append(obj_msg)

            obj_pose_msg = ObjectPose()
            obj_pose_msg.class_id = latest_percept['class_id']
            # Hack for the different naming for now
            if obj_pose_msg.class_id == 'power_drill':
                obj_pose_msg.class_id = 'power_drill_with_grip'
            obj_pose_msg.instance_id = obj_msg.instance_id
            obj_pose_msg.pose = obj_msg.classification.pose.pose
            obj_pose_msg.size = obj_msg.bbox.size
            pose_selector_objects_msgs.objects.append(obj_pose_msg)

        pose_selector_objects_msgs.header = anchored_objects_msg.header
        self._pub_anchored_objects.publish(anchored_objects_msg)
        self._pub_object_list.publish(pose_selector_objects_msgs)


if __name__ == "__main__":
    rospy.init_node('knowledge_base_publisher')
    config = rospy.get_param('~')
    kb_publisher = KnowledgeBasePublisher(config)
    kb_publisher.run()
