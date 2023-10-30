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

import sys
import os
import importlib
import shutil
import numpy as np
import logging
from itertools import count
from threading import RLock
from daa_knowledge_base.prolog_mt import PrologMT
from owlready2 import destroy_entity, World, FunctionalProperty, InverseFunctionalProperty


from daa_knowledge_base.kb_ontology import KB_Ontology


class KBHandle:
    _ids = count(0)

    def __init__(self, config) -> None:
        self.id = "world_" + str(next(self._ids))
        self._updated = False

        self._world_dir = os.path.join(config["working_dir"], self.id)
        onto_config = config["onto_config"]
        onto_config.update(
            {
                "owl_path": self._world_dir,
                "onto_path": self._world_dir,
                "onto_name": self.id,
            }
        )
        if not os.path.isdir(self._world_dir):
            os.makedirs(self._world_dir)
        for f in config["prolog_files"]:
            src = os.path.join(config["prolog_path"], f)
            dst = os.path.join(self._world_dir, f)
            shutil.copy(src, dst)
        self._rule_file = os.path.join(self._world_dir, config["rule_file"])
        self._reload_file = os.path.join(self._world_dir, config["reload_file"])

        # Create ontology
        scenario = config["scenario"]
        kb_onto = KB_Ontology(onto_config)
        kb_onto.create(scenario)
        self._colors = config.get("colors") or {}
        self._owl_file = kb_onto.owl_file
        # Load ontology
        self.onto_lock = RLock()
        sys.argv.append("--onto_uri")
        sys.argv.append(kb_onto.onto_uri)  # the loaded module needs to know this
        self._my_world = World(filename=kb_onto.onto_file, exclusive=False, enable_thread_parallelism=True)
        self.onto = self._my_world.get_ontology(kb_onto.onto_uri).load()  # this loads the python module

        sys.argv.append("--onto")
        sys.argv.append(self.onto)
        importlib.import_module('daa_knowledge_base.prolog_foreign')

        self.prolog = PrologMT()  # https://swi-prolog.discourse.group/t/threaded-queries-rulebase-independence/1236/15
        self.prolog.consult(self._rule_file)

        # TODO: get rid of this
        #  self.class_dict = kb_onto.class_dict doesn't work, because kb_onto.class_dict was created
        #  before closing the db
        if scenario == "maker_space":
            self.class_dict = {
                'klt': self.onto.KLT,
                'power_drill_with_grip': self.onto.Powerdrill,
                'mustard': self.onto.Mustard,
                'sugar': self.onto.Sugar,
                'cracker': self.onto.Cracker,
                'soup': self.onto.Soup,
                'multimeter': self.onto.Multimeter,
                'relay': self.onto.Relay,
                'screwdriver': self.onto.Screwdriver,
                'materialbox': self.onto.MaterialBox,
                'hot_glue_gun': self.onto.HotGlueGun,
            }
        elif scenario == "marina":
            self.class_dict = {
                'boat': self.onto.Boat,
            }
        else:
            logging.warning("Unknown scenario!")
            raise RuntimeError
        self.object_count = {}
        self.object_max_count = config.get("max_count")
        # Quick hack for setting up the tables, this should be removed
        # TODO: Init table locations in odom frame, since tracker outputs
        # results in the odom frame
        if config.get("table_locations") and config.get("table_sizes"):
            table_locations = config.get("table_locations")
            table_sizes = config.get("table_sizes")
            location_offset = config.get("location_offset")
            location_offset = np.array(location_offset)
            for table_name, location in table_locations.items():
                table_size = np.array(table_sizes[table_name])
                location = -np.array(location)
                location += location_offset
                confine = np.stack([-table_size / 2 + location, table_size / 2 + location], axis=1)
                confine[2] = np.array([table_size[2], table_size[2] + 0.5])
                table_instance = self.onto.Table(table_name)
                table_instance.set_confine(confine)

    def mark_updated(self):
        self._updated = True

    def get_all_individuals(self):
        ret = {}
        with self.onto_lock:
            for onto_object in self.onto.Object.instances():
                ret.update({onto_object.name: onto_object.percept_history[-1]})
        return ret

    def get_instance_count(self, class_id: str):
        try:
            return {
                "current_count": len(list(self.class_dict.get(class_id).instances())),
                "max_count": self.object_max_count.get(class_id) if class_id in self.object_max_count else int(0),
            }
        except AttributeError:
            logging.error(f"{class_id} not exists in the ontology")
            return {"current_count": int(-1), "max_count": int(-1)}

    def handle_commit_updates(self):
        with self.onto_lock:
            self._commit_updates()

    def handle_create_instance(self, percepts: dict):
        class_id = percepts['class_id']
        if self.class_dict.get(class_id):
            obj_count = self.object_count.get(class_id)
            if obj_count:
                self.object_count[class_id] += 1
            else:
                self.object_count[class_id] = 1
            new_symbol = class_id + "_" + str(self.object_count[class_id])

            with self.onto_lock:
                with self.onto:
                    self._create_instance(new_symbol, percepts)
            self.mark_updated()
            return new_symbol
        else:
            raise Exception(
                "undefined class '{}'. Instance won't be created. Please update the ontology".format(class_id)
            )

    def handle_update_instance(self, symbol: str, percepts: dict):
        if self.onto[symbol] is not None:
            self.onto[symbol].update(percepts)
            self.mark_updated()
        else:
            raise Exception(f'symbol: {symbol} does not exist')

    def handle_prolog_query(self, q_str: str):
        q_str = q_str.replace('onto:', self.onto.get_base_iri())
        answers = list(self.prolog.query(q_str))
        ret = []
        for answer in answers:
            answer_dict = {}
            for k, v in answer.items():
                if len(self.onto.search(iri=v)) > 0:
                    onto_instance = self.onto.search(iri=v)[0]
                    answer_dict.update({k: onto_instance.name})
            if len(answer_dict) == len(answer):
                ret.append(answer_dict)
        return ret

    def is_potential_match(self, symbol: str, percepts: dict):
        if not self.onto:
            raise Exception("onto is None")
        instance = None
        ref_instance = None
        ret = None
        with self.onto_lock:
            with self.onto:
                instance = self._create_instance(symbol=None, percepts=percepts)
            if self.onto[symbol]:
                ref_instance = self.onto[symbol]
            self._synchronize_prolog()
            if ref_instance:
                q = "is_potential_match('{}','{}')".format(ref_instance.iri, instance.iri)
                answer = list(self.prolog.query(q))
                ret = True if answer else False
            else:
                q = "is_potential_match(Obj,'{}')".format(instance.iri)
                answers = list(self.prolog.query(q))
                ret = []
                for answer in answers:
                    if answer['Obj'] == instance.iri:
                        continue
                    ref_obj = self.onto.search(iri=answer['Obj'])[0]
                    assert ref_obj
                    ret.append(ref_obj.name)
            if instance:
                with self.onto:
                    destroy_entity(instance)
                self._synchronize_prolog()

        return ret

    def _create_instance(self, symbol: str, percepts: dict):
        class_id = percepts['class_id']
        if symbol:
            instance = self.class_dict.get(class_id)(symbol)
        else:
            instance = self.class_dict.get(class_id)()
        instance.init(percepts)
        return instance

    def infer_relation_matrices(self, assignments):
        """
        Create temporary instances under the ontology and perform inference with prolog.

        TODO: Create instances under inference ontology and do the inference
        there?
        """
        temp_instances = []
        related_instances = []
        with self.onto_lock:
            for assign in assignments:
                try:
                    symbol = assign["symbol"]
                    percepts = assign["percepts"]
                    if symbol == "":
                        with self.onto:
                            new_instance = self._create_instance(symbol=None, percepts=percepts)
                        temp_instances.append(new_instance)
                    else:
                        with self.onto:
                            self.onto[symbol].update(percepts)
                        related_instances.append(self.onto[symbol])
                except AttributeError as e:
                    logging.error(f"{symbol}: {e}")
                    continue
            # save new instances in owl first, then load them into prolog
            self._synchronize_prolog()
            # calculate matrix
            inferred_matrices = self._infer_relation_matrices()
            ref_matrices = self._get_relation_matrices()
            # clear changes, very important!
            for ins in related_instances:
                ins.roll_back()
            for ins in temp_instances:
                destroy_entity(ins)

            self._synchronize_prolog()
        matrices_bundle = {'inferred': inferred_matrices, 'reference': ref_matrices}
        return matrices_bundle

    def _infer_relation_matrices(self):
        matrices = {}
        for rel in self.onto.relations_to_infer:
            q1 = "infer_{}(ObjA, ObjB)".format(rel.name)
            res1 = list(self.prolog.query(q1))
            rows = {}
            cols = {}
            domain_keys = []
            range_keys = []
            for o_domain, idx in zip(rel.domain[0].instances(), range(len(rel.domain[0].instances()))):
                rows[o_domain.iri] = idx
                domain_keys.append(o_domain.iri)
            for o_range, idx in zip(rel.range[0].instances(), range(len(rel.range[0].instances()))):
                cols[o_range.iri] = idx
                range_keys.append(o_range.iri)
            matrix_infer = np.zeros((len(rows), len(cols)), dtype=int)
            for r in res1:
                i = rows[r["ObjA"]]
                j = cols[r["ObjB"]]
                matrix_infer[i][j] += 1
            matrices.update(
                {
                    rel.name: matrix_infer.tolist(),
                }
            )
        return matrices

    def _get_relation_matrices(self):
        matrices = {}

        for rel in self.onto.relations_to_infer:
            q2 = "{}(ObjA, ObjB)".format(rel.name)
            res2 = list(self.prolog.query(q2))
            rows = {}
            cols = {}
            domain_keys = []
            range_keys = []
            for o_domain, idx in zip(rel.domain[0].instances(), range(len(rel.domain[0].instances()))):
                rows[o_domain.iri] = idx
                domain_keys.append(o_domain.iri)
            for o_range, idx in zip(rel.range[0].instances(), range(len(rel.range[0].instances()))):
                cols[o_range.iri] = idx
                range_keys.append(o_range.iri)
            matrix_prev = np.zeros((len(rows), len(cols)), dtype=int)
            for r in res2:
                i = rows[r["ObjA"]]
                j = cols[r["ObjB"]]
                matrix_prev[i][j] += 1

            matrices.update(
                {
                    rel.name: matrix_prev.tolist(),
                }
            )
        return matrices

    def get_relation_matrices(self):
        if self._updated:
            self._commit_updates()
        matrices = self._get_relation_matrices()
        return matrices

    def _infer(self, relation: str, obj_a: str = None, obj_b: str = None) -> list:

        obj_a = "'{}'".format(obj_a) if obj_a else "ObjA"
        obj_b = "'{}'".format(obj_b) if obj_b else "ObjB"
        q = "infer_{}({}, {})".format(relation, obj_a, obj_b)
        return list(self.prolog.query(q))

    def _infer_relations(self):
        for rel in self.onto.relations_to_infer:
            q1 = "infer_{}(ObjA, ObjB)".format(rel.name)
            q2 = "{}(ObjA, ObjB)".format(rel.name)
            res1 = list(self.prolog.query(q1))
            res2 = list(self.prolog.query(q2))
            rows = {}
            cols = {}
            domain_keys = []
            range_keys = []
            for o_domain, idx in zip(rel.domain[0].instances(), range(len(rel.domain[0].instances()))):
                rows[o_domain.iri] = idx
                domain_keys.append(o_domain.iri)
            for o_range, idx in zip(rel.range[0].instances(), range(len(rel.range[0].instances()))):
                cols[o_range.iri] = idx
                range_keys.append(o_range.iri)

            matrix_infer = np.zeros((len(rows), len(cols)), dtype=int)
            matrix_prev = np.zeros((len(rows), len(cols)), dtype=int)
            for r in res1:
                i = rows[r["ObjA"]]
                j = cols[r["ObjB"]]
                matrix_infer[i][j] += 1

            for r in res2:
                i = rows[r["ObjA"]]
                j = cols[r["ObjB"]]
                matrix_prev[i][j] += 1

            if len(rows) and len(cols):
                diff = matrix_infer - matrix_prev
                if np.linalg.matrix_rank(diff):
                    indices = np.where(diff == 1)
                    for i, j in zip(indices[0], indices[1]):
                        obj_a = self.onto.search(iri=domain_keys[i])[0]
                        obj_b = self.onto.search(iri=range_keys[j])[0]
                        logging.info(
                            "new relation will be inserted: '{} {} {}'".format(obj_a.name, rel.name, obj_b.name)
                        )
                        with self.onto:
                            if issubclass(rel, FunctionalProperty):
                                setattr(obj_a, rel.name, obj_b)
                            elif issubclass(rel, InverseFunctionalProperty):
                                inv_rel = rel.get_inverse_property()  # This is a functional prop
                                prev_obj_a = getattr(
                                    obj_b, inv_rel.name
                                )  # This obj holds relation with obj_b, need to cut this relation first
                                if prev_obj_a:
                                    getattr(prev_obj_a, rel.name).remove(obj_b)
                                getattr(obj_a, rel.name).append(obj_b)
                            else:
                                getattr(obj_a, rel.name).append(obj_b)
                    indices = np.where(diff == -1)
                    for i, j in zip(indices[0], indices[1]):
                        obj_a = self.onto.search(iri=domain_keys[i])[0]
                        obj_b = self.onto.search(iri=range_keys[j])[0]
                        if issubclass(rel, FunctionalProperty):
                            re = self._infer(relation="not_" + rel.name, obj_a=domain_keys[i], obj_b=range_keys[j])
                            if re:
                                logging.info(
                                    "old relation will be deleted: '{} {} {}'".format(obj_a.name, rel.name, obj_b.name)
                                )
                                with self.onto:
                                    setattr(obj_a, rel.name, None)

    def _merge_matches(self):
        q = "is_match(ObjA, ObjB)"
        res = list(self.prolog.query(q))
        matches = {}
        for r in res:
            obj_a = r["ObjA"]
            obj_b = r["ObjB"]
            if obj_a == obj_b:
                continue
            obj_a = self.onto.search(iri=obj_a)[0]
            obj_b = self.onto.search(iri=obj_b)[0]
            assert obj_a and obj_b
            if (
                obj_a in matches.keys()
                or obj_a in matches.values()
                or obj_b in matches.keys()
                or obj_b in matches.values()
            ):
                pass
            else:
                matches.update(
                    {
                        obj_a: obj_b,
                    }
                )
        if matches:
            logging.info("found matches: ", matches)
            disposal_objs = []
            ret = {}
            for obj_a, obj_b in matches.items():
                if obj_a.created_time <= obj_b.created_time:
                    with self.onto:
                        obj_a.merge(obj_b)
                        if obj_b not in disposal_objs:
                            disposal_objs.append(obj_b)
                            ret.update({obj_a.name: obj_b.name})
                else:
                    with self.onto:
                        obj_b.merge(obj_a)
                        if obj_a not in disposal_objs:
                            disposal_objs.append(obj_a)
                            ret.update({obj_b.name: obj_a.name})
            for obj in disposal_objs:
                with self.onto:
                    logging.debug("destroy {}".format(obj.name))
                    destroy_entity(obj)
            return ret
        else:
            return None

    def _commit_updates(self):
        self._synchronize_prolog()
        self._infer_relations()
        # self._synchronize_prolog()
        # matches = self._merge_matches()
        # self._synchronize_prolog()
        # self._infer_relations()
        self._my_world.save()
        self._my_world.as_rdflib_graph().serialize(destination=self._owl_file, format="xml")
        self._updated = False
        # if matches:
        #     print(matches)
        # return matches

    def _synchronize_prolog(self):
        self._my_world.as_rdflib_graph().serialize(destination=self._owl_file, format="xml")
        self.prolog.consult(self._reload_file)
