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

from pyswip.easy import Atom, registerForeign
import numpy as np
import sys

onto = None

for arg in sys.argv:
    if arg == "--onto":
        idx = sys.argv.index(arg)
        sys.argv.pop(idx)
        onto = sys.argv[idx]
        sys.argv.pop(idx)
if not onto:
    raise RuntimeError("[daa_knowledge_base.prolog_foreign]: failed to load ontology")


# def comp_have_same_color(ObjA, ObjB):
#     if (not isinstance(ObjA, Atom)) or (not isinstance(ObjB, Atom)):
#         raise RuntimeError
#     obj_a = onto.search(iri=ObjA.value)
#     obj_b = onto.search(iri=ObjB.value)
#     if not obj_a or not obj_b:
#         raise RuntimeError
#     obj_a = obj_a[0]
#     obj_b = obj_b[0]
#     if (not obj_a.color) and (not obj_b.color):
#         return True
#     if obj_a and obj_b:
#         return bool(set(obj_b.color) & set(obj_a.color))
#     else:
#         return True


def comp_far_enough(t1, t2):
    if not isinstance(t1, Atom) or not isinstance(t2, Atom):
        return False
    obj_a = onto.search(iri=t1.value)
    obj_b = onto.search(iri=t2.value)
    if not obj_a or not obj_b:
        return False
    obj_a = obj_a[0]
    obj_b = obj_b[0]
    if obj_a.expected_distance_to(obj_b) > 0.2:
        return True
    else:
        return False


def comp_close_enough(t1, t2):
    if not isinstance(t1, Atom) or not isinstance(t2, Atom):
        return False
    obj_a = onto.search(iri=t1.value)
    obj_b = onto.search(iri=t2.value)
    if not obj_a or not obj_b:
        return False
    obj_a = obj_a[0]
    obj_b = obj_b[0]
    if obj_a.expected_distance_to(obj_b) < 0.15:
        return True
    else:
        return False


def comp_is_overlapping(t1, t2):
    if not isinstance(t1, Atom) or not isinstance(t2, Atom):
        return False
    obj_a = onto.search(iri=t1.value)
    obj_b = onto.search(iri=t2.value)
    if not obj_a or not obj_b:
        return False
    obj_a = obj_a[0]
    obj_b = obj_b[0]
    # TODO: assert overlapping
    d = obj_a.expected_distance_to(obj_b)
    dim_a = np.max(obj_a.np_dimensions())
    dim_b = np.max(obj_b.np_dimensions())
    if d < 0.5 * (dim_a + dim_b):
        return True
    else:
        return False


def comp_confine(t1, t2):
    if not isinstance(t1, Atom) or not isinstance(t2, Atom):
        return False
    obj_a = onto.search(iri=t1.value)
    obj_b = onto.search(iri=t2.value)
    if not obj_a or not obj_b:
        return False
    obj_a = obj_a[0]
    obj_b = obj_b[0]
    if obj_a.is_obj_within_confine(obj_b):
        return True
    else:
        return False


def comp_not_confine(t1, t2):
    return not comp_confine(t1, t2)


registerForeign(comp_close_enough, arity=2)
registerForeign(comp_far_enough, arity=2)
registerForeign(comp_confine, arity=2)
registerForeign(comp_is_overlapping, arity=2)
# registerForeign(comp_have_same_color, arity=2)

# registerForeign(is_a, arity=2, flags=PL_FA_NONDETERMINISTIC)
# def is_a(Obj, Type, context):
#     def unify(obj, type, id):
#         cls = getattr(onto, type.value)
#         if id >= len(cls.instances()):
#             return 0  # stop
#         if isinstance(type, Atom) and isinstance(obj, Variable):
#             Obj.unify(Atom(cls.instances()[context].iri))
#             return 2  # next
#         if isinstance(type, Atom) and isinstance(obj, Atom):
#             o = onto.search(iri=obj.value)[0]
#             if isinstance(o, cls):
#                 return 1
#             else:
#                 return 0

#     control = PL_foreign_control(context)
#     context = PL_foreign_context(context)

#     if control == PL_FIRST_CALL:
#         context = 0
#         r = unify(Obj, Type, context)
#         if not r:
#             return False
#         elif r == 1:
#             return True
#         elif r == 2:
#             context += 1
#             return PL_retry(context)
#     elif control == PL_REDO:
#         if not unify(Obj, Type, context):
#             return False
#         context += 1
#         return PL_retry(context)
#     elif control == PL_PRUNED:
#         pass
