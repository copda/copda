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
from daa_knowledge_base import KBClient, KBQueryType, KBUpdateType
from daa_knowledge_base_ros.srv import Query, QueryRequest, QueryResponse
from daa_knowledge_base_ros.srv import Update, UpdateRequest


class KBClient_ROS(KBClient):
    def __init__(self, query_service_name, update_service_name, **kwargs):
        super().__init__(**kwargs)
        rospy.wait_for_service(query_service_name)
        rospy.wait_for_service(update_service_name)
        self.query_service_proxy = rospy.ServiceProxy(query_service_name, Query)
        self.update_service_proxy = rospy.ServiceProxy(update_service_name, Update)

    def query(self, type: KBQueryType, data, **kwargs):
        header = None
        if type == KBQueryType.IS_POTENTIAL_MATCH:
            header = QueryRequest.IS_POTENTIAL_MATCH
            req_data = {'symbol': kwargs['symbol'], 'percepts': data}
            req_data = json.dumps(req_data)
        elif type == KBQueryType.GET_RELATION_MATRICES:
            header = QueryRequest.GET_RELATION_MATRICES
            req_data = None
        elif type == KBQueryType.INFER_RELATION_MATRICES:
            header = QueryRequest.INFER_RELATION_MATRICES
            req_data = json.dumps(data)
        elif type == KBQueryType.GET_ALL_INSTANCES:
            header = QueryRequest.GET_ALL_INSTANCES
            req_data = None
        elif type == KBQueryType.GET_INSTANCE_COUNT:
            header = QueryRequest.GET_INSTANCE_COUNT
            req_data = data
        else:
            raise rospy.ServiceException("query request type not defined")

        try:
            res = self.query_service_proxy(header=header, data=req_data)
            return self._decode_query_response(res)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return None

    def update(self, type: KBUpdateType, data, **kwargs):
        header = None
        if type == KBUpdateType.CREATE_INSTANCE:
            header = UpdateRequest.CREATE_INSTANCE
            req_data = self._encode_percepts(data)
        elif type == KBUpdateType.UPDATE_INSTANCE:
            header = UpdateRequest.UPDATE_INSTANCE
            req_data = {'symbol': kwargs['symbol'], 'percepts': data}
            req_data = json.dumps(req_data)
        elif type == KBUpdateType.COMMIT_UPDATES:
            header = UpdateRequest.COMMIT_UPDATES
            req_data = data
        else:
            raise rospy.ServiceException("update request type not defined")
        try:
            res = self.update_service_proxy(header=header, data=req_data)
            return self._decode_update_response(res)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return None

    def _decode_query_response(self, res):
        if res.data is None:
            raise rospy.ServiceException("response data is None.")
        if res.header == QueryResponse.GET_RELATION_MATRICES:

            matrices = json.loads(res.data)
            for key, mat in matrices.items():
                matrices[key] = np.array(mat, dtype=int)
            return matrices
        elif res.header == QueryResponse.INFER_RELATION_MATRICES:
            matrices = json.loads(res.data)
            inferred = matrices['inferred']
            reference = matrices['reference']
            for key, mat in inferred.items():
                inferred[key] = np.array(mat, dtype=int)
            for key, mat in reference.items():
                reference[key] = np.array(mat, dtype=int)
            return inferred, reference
        elif res.header == QueryResponse.GET_ALL_INSTANCES:
            all_instances = {}
            if res.data:
                all_instances = json.loads(res.data)

            return all_instances
        elif res.header == QueryResponse.GET_INSTANCE_COUNT:
            if res.data:
                count = json.loads(res.data)
            else:
                count = {}
            return count
        elif res.header == QueryResponse.IS_POTENTIAL_MATCH:
            return eval(res.data)

        else:
            raise rospy.ServiceException("response type not defined.")

    # TODO
    def _decode_update_response(self, res):
        return res.data

    def _encode_percepts(self, percepts):
        encoded = json.dumps(percepts)
        return encoded
