
:- use_module(library(semweb/rdf_db)).
:- use_module(library(semweb/rdf_portray)).
:- use_module(library(semweb/rdfs)).

:- rdf_register_prefix(onto, 'http://dfki.ipr.de/onto#').
:- rdf_load("knowledge_graph.owl").

np_string_to_vector(Str, Vector) :-
  split_string(Str, " "," []", List),
  maplist(number_string, Vector, List).


is_at(O, P) :-
  rdf(O, onto:position,literal(type(_, Pstr))),
  np_string_to_vector(Pstr, P).

timestamp(O, T) :-
  rdf(O, onto:timestamp,literal(type(_, T))).


object_dimensions(O, D) :-
  rdf_has(O, onto:dimensions,literal(type(_, Dstr))),
  np_string_to_vector(Dstr, D).

contains(Container, Item) :-
  rdf_has(Container, onto:contains, Item).


is_inside_of(Item, Container) :-
  rdf_has(Item, onto:is_inside_of, Container).

%depend on foreign functions
infer_contains(Container, Item):-
  is_a(Container, 'Container'),
  is_a(Item, 'SmallItem'),
  comp_distance(Container, Item, Dist),
  =<(Dist, 0.15).
% has(Container, onto:'Color', Color),
% has(Item, onto:'Color', Color).

infer_not_contains(Container, Item):-
  comp_distance(Container, Item, Dist),
  >=(Dist, 0.20).

infer_match(ObjA, ObjB) :-
  infer_contains(ObjA, Item),
  contains(ObjB, Item).
