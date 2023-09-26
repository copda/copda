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
import os
import json
from daa_knowledge_base_ros.srv import Query, QueryRequest, QueryResponse
from daa_knowledge_base_ros.srv import Update, UpdateRequest, UpdateResponse

from daa_knowledge_base import KBHandle

rospack = rospkg.RosPack()


class KnowledgeBaseServer:
    def __init__(self, config) -> None:
        config.update(
            {
                "working_dir": os.path.join(rospack.get_path("daa_knowledge_base_ros"), "kb_temp"),
                "prolog_path": os.path.join(rospack.get_path("daa_knowledge_base_ros"), "src/prolog"),
            }
        )
        self.kb_handle = KBHandle(config)
        self.query_service = rospy.Service('~query', Query, self._handle_query_req)
        self.update_service = rospy.Service('~update', Update, self._handle_update_req)

        # self.stop = False
        rospy.spin()

    def _handle_update_req(self, req):
        try:
            if req.header == UpdateRequest.CREATE_INSTANCE:
                percepts = self._decode_update_req(req)
                new_symbol = self.kb_handle.handle_create_instance(percepts)
                rospy.loginfo(f"create instance: {new_symbol}")
                return UpdateResponse(data=new_symbol)
            elif req.header == UpdateRequest.UPDATE_INSTANCE:
                symbol, percepts = self._decode_update_req(req)
                self.kb_handle.handle_update_instance(symbol, percepts)
                rospy.loginfo(f"update instance: {symbol}")
                return UpdateResponse()
            elif req.header == UpdateRequest.COMMIT_UPDATES:
                self.kb_handle.handle_commit_updates()
                return UpdateResponse()

        except Exception as e:  # TODO
            rospy.logerr(e)
            return UpdateResponse()

    def _handle_query_req(self, req):
        try:
            if req.header == QueryRequest.GET_RELATION_MATRICES:
                res = self.kb_handle.get_relation_matrices()
                res = json.dumps(res)
                return QueryResponse(header=QueryResponse.GET_RELATION_MATRICES, data=res)

            elif req.header == QueryRequest.INFER_RELATION_MATRICES:
                assignments = json.loads(req.data)
                res = self.kb_handle.infer_relation_matrices(assignments)
                res = json.dumps(res)
                return QueryResponse(header=QueryResponse.INFER_RELATION_MATRICES, data=res)

            elif req.header == QueryRequest.IS_POTENTIAL_MATCH:
                symbol, percepts = self._decode_query_req(req)
                res = self.kb_handle.is_potential_match(symbol=symbol, percepts=percepts)
                return QueryResponse(header=QueryResponse.IS_POTENTIAL_MATCH, data=str(res))

            elif req.header == QueryRequest.GET_ALL_INSTANCES:
                all_instances = self.kb_handle.get_all_individuals()
                all_instances = json.dumps(all_instances)
                return QueryResponse(header=QueryResponse.GET_ALL_INSTANCES, data=all_instances)

            elif req.header == QueryRequest.GET_INSTANCE_COUNT:
                class_id = req.data
                count = self.kb_handle.get_instance_count(class_id)
                res = json.dumps(count)
                return QueryResponse(header=QueryResponse.GET_INSTANCE_COUNT, data=res)

            elif req.header == QueryRequest.PROLOG_QUERY:
                if not req.data:
                    return QueryResponse(header=QueryRequest.FAILED, data='')
                answer = self.kb_handle.handle_prolog_query(req.data)
                answer = json.dumps(answer)
                return QueryResponse(header=QueryRequest.PROLOG_QUERY, data=answer)
            else:
                raise rospy.ServiceException('query type not defined')
        except Exception as e:  # TODO
            rospy.logerr(e)
            return QueryResponse(
                header=QueryResponse.FAILED,
            )

    def _decode_query_req(self, req):
        if req.header == QueryRequest.GET_RELATION_MATRICES:
            raise NotImplementedError
        elif req.header == QueryRequest.IS_POTENTIAL_MATCH:
            decoded = json.loads(req.data)
            return decoded.get('symbol'), decoded.get('percepts')

    def _encode_query_res(self, res):
        # TODO
        encoded_res = QueryResponse()
        encoded_res.data = res
        return encoded_res

    def _decode_update_req(sef, req):
        if req.header == UpdateRequest.CREATE_INSTANCE:
            percepts = json.loads(req.data)
            return percepts
        elif req.header == UpdateRequest.UPDATE_INSTANCE:
            decoded = json.loads(req.data)
            return decoded.get('symbol'), decoded.get('percepts')

    def _encode_update_res(self, res):
        encoded_res = UpdateResponse()
        encoded_res.data = res
        return encoded_res


if __name__ == "__main__":
    rospy.init_node('knowledge_base_server')
    config = rospy.get_param('~')

    kb_server = KnowledgeBaseServer(config)
