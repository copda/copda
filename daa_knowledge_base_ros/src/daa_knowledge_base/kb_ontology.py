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

from owlready2 import (
    AnnotationProperty,
    World,
    close_world,
    DataProperty,
    default_world,
    FunctionalProperty,
    InverseFunctionalProperty,
    ObjectProperty,
    sync_reasoner_pellet,
    Thing,
)
import os


class KB_Ontology:
    def __init__(self, config):
        self.class_dict = {}
        # private
        self.config = config
        self.onto_filename = "ontology.sqlite3"
        self.owl_filename = "knowledge_graph.owl"
        self.owl_file = os.path.join(config["owl_path"], self.owl_filename)
        self.onto_file = os.path.join(config["onto_path"], self.onto_filename)
        # self.onto_uri = os.path.join(config["onto_uri"], config["onto_name"])
        self.onto_uri = config["onto_uri"]

    def create(self, scenario="maker_space"):
        if os.path.isfile(self.onto_file):
            os.remove(self.onto_file)
        if os.path.isfile(self.owl_file):
            os.remove(self.owl_file)
        world = World(filename=self.onto_file, exclusive=False)

        onto = world.get_ontology(self.onto_uri)
        # onto = get_ontology(self.onto_uri)
        with onto:
            if scenario == "maker_space":

                class python_module(AnnotationProperty):
                    pass

                # object classes
                class Object(Thing):
                    pass

                class Surface(Thing):
                    pass

                class Table(Surface):
                    pass

                class Container(Object):
                    pass

                class SmallItem(Object):
                    pass

                class KLT(Container):
                    pass

                class Soup(SmallItem):
                    pass

                class Cracker(SmallItem):
                    pass

                class Powerdrill(SmallItem):
                    pass

                class Mustard(SmallItem):
                    pass

                class Sugar(SmallItem):
                    pass

                class Multimeter(SmallItem):
                    pass

                class Relay(SmallItem):
                    pass

                class Screwdriver(SmallItem):
                    pass

                class MaterialBox(Container):
                    pass

                class HotGlueGun(SmallItem):
                    pass

                # relations
                class contains(ObjectProperty, FunctionalProperty):
                    domain = [Container]
                    range = [SmallItem]
                    python_name = "contains"

                class is_inside_of(ObjectProperty, FunctionalProperty):
                    domain = [SmallItem]
                    range = [Container]
                    inverse_property = contains
                    python_name = "is_inside_of"

                class supports(ObjectProperty, InverseFunctionalProperty):
                    domain = [Surface]
                    range = [Object]
                    python_name = "supports"

                class is_on_top_of(ObjectProperty, FunctionalProperty):
                    domain = [Object]
                    range = [Surface]
                    inverse_property = supports
                    python_name = "is_on_top_of"

                # properties
                class timestamp(DataProperty, FunctionalProperty):
                    domain = [Object]
                    range = [float]
                    python_name = "timestamp"

                class position(DataProperty, FunctionalProperty):
                    domain = [Object]
                    range = [str]
                    python_name = "position"

                class orientation(DataProperty, FunctionalProperty):
                    domain = [Object]
                    range = [str]
                    python_name = "orientation"

                class color(DataProperty, FunctionalProperty):
                    domain = [Object]
                    range = [str]
                    python_name = "color"

                class max_number(DataProperty, FunctionalProperty):
                    namespace = onto
                    domain = [Object]
                    range = [int]
                    python_name = "max_number"

                class dimensions(DataProperty, FunctionalProperty):
                    namespace = onto
                    domain = [Object]
                    range = [str]
                    python_name = "dimensions"

                class confine(DataProperty, FunctionalProperty):
                    namespace = onto
                    domain = [Surface]
                    range = [str]
                    python_name = "confine"

                self.class_dict.update(
                    {
                        'klt': KLT,
                        'power_drill_with_grip': Powerdrill,
                        'mustard': Mustard,
                        'sugar': Sugar,
                        'cracker': Cracker,
                        'soup': Soup,
                        'multimeter': Multimeter,
                        'relay': Relay,
                        'screwdriver': Screwdriver,
                        'materialbox': MaterialBox,
                        'hot_glue_gun': HotGlueGun,
                    }
                )
            elif scenario == "marina":

                class python_module(AnnotationProperty):
                    pass

                # object classes
                class Object(Thing):
                    pass

                class Container(Object):
                    pass

                class SmallItem(Object):
                    pass

                class Boat(Container):
                    pass

                # relations
                class contains(ObjectProperty, FunctionalProperty):
                    domain = [Container]
                    range = [SmallItem]
                    python_name = "contains"

                class is_inside_of(ObjectProperty, FunctionalProperty):
                    domain = [SmallItem]
                    range = [Container]
                    inverse_property = contains
                    python_name = "is_inside_of"

                # properties
                class timestamp(DataProperty, FunctionalProperty):
                    domain = [Object]
                    range = [float]
                    python_name = "timestamp"

                class position(DataProperty, FunctionalProperty):
                    domain = [Object]
                    range = [str]
                    python_name = "position"

                class color(DataProperty):
                    domain = [Object]
                    range = [str]
                    python_name = "color"

                class max_number(DataProperty, FunctionalProperty):
                    namespace = onto
                    domain = [Object]
                    range = [int]
                    python_name = "max_number"

                class dimensions(DataProperty, FunctionalProperty):
                    namespace = onto
                    domain = [Object]
                    range = [str]
                    python_name = "dimensions"

                self.class_dict.update(
                    {
                        'boat': Boat,
                    }
                )
            elif scenario == "table":
                raise NotImplementedError
            else:
                print("Unknown scenario. Cannot create ontology.")
                raise NotImplementedError
        close_world(onto)
        onto.python_module = "daa_knowledge_base.onto_methods"  # add annotation
        with onto:
            try:
                sync_reasoner_pellet(infer_property_values=True, infer_data_property_values=True)
            except Exception:
                print(list(default_world.inconsistent_classes()))
        graph = world.as_rdflib_graph()
        graph.serialize(destination=self.owl_file, format="xml")
        world.save()
        world.close()
