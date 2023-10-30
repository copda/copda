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
import rospkg
import json
from collections import defaultdict
from object_pose_msgs.msg import ObjectList, ObjectPose
from daa_msgs.msg import AnchoredObjectArray, AnchoredObject
from daa_knowledge_base_ros.srv import Query, QueryRequest

rospack = rospkg.RosPack()


class KnowledgeBasePublisher:
    def __init__(self, config) -> None:
        self._config = config
        query_service_name = 'daa_knowledge_base/query'
        self._query_service_name = query_service_name
        rospy.wait_for_service(query_service_name)
        self._query_service_proxy = rospy.ServiceProxy(query_service_name, Query)
        self._pub_object_list = rospy.Publisher('object_list', ObjectList, queue_size=10)
        self._pub_anchored_objects = rospy.Publisher('anchored_objects', AnchoredObjectArray, queue_size=10)

        def next_instance_id():
            return len(self._instance_ids)

        self._instance_ids = defaultdict(next_instance_id)

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
        anchored_objects_msg = AnchoredObjectArray()
        anchored_objects_msg.header.frame_id = self._config["global_frame"]
        pose_selector_objects_msgs = ObjectList()
        for symbol, percepts in all_instances.items():
            obj_msg = AnchoredObject()
            obj_msg.header.frame_id = anchored_objects_msg.header.frame_id
            obj_msg.header.stamp = rospy.Time.from_sec(percepts['timestamp'])
            if anchored_objects_msg.header.stamp < obj_msg.header.stamp:
                anchored_objects_msg.header.stamp = obj_msg.header.stamp

            obj_msg.instance_id = self._instance_ids[symbol]
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

            # TODO: get mesh
            obj_msg.bbox.size.x = 0.2
            obj_msg.bbox.size.y = 0.2
            obj_msg.bbox.size.z = 0.2

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
