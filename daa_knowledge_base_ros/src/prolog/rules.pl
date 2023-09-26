
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
:- rdf_meta
  is_at(r,r).
timestamp(O, T) :-
  rdf_has(O, onto:timestamp,literal(type(_, T_atom))),
  atom_number(T_atom, T).
:- rdf_meta
  timestamp(r, r).
close_in_time(ObjA, ObjB) :-
  timestamp(ObjA, T1),
  timestamp(ObjB, T2),
  abs(T1 - T2) =< 1.0.
:- rdf_meta
  close_in_time(r, r).
object_dimensions(O, D) :-
  rdf_has(O, onto:dimensions,literal(type(_, Dstr))),
  np_string_to_vector(Dstr, D).

has_colors(Obj, Colors):-
  rdf_has(Obj,onto:color, literal(type(_, Cstr))),
  split_string(Cstr, ",","",Colors).
:- rdf_meta
  has_colors(r,r).

contains(Container, Item) :-
  rdf_has(Container, onto:contains, Item).
:- rdf_meta
  contains(r,r).
is_inside_of(Item, Container) :-
  rdf_has(Item, onto:is_inside_of, Container).
:- rdf_meta
  is_inside_of(r,r).
is_a(Obj, Class) :-
  rdf_has(Obj, rdf:type, T),
  rdf_reachable(T ,rdfs:subClassOf, Class).
:- rdf_meta
  is_a(r, r).

supports(Surface, Object):-
  rdf_has(Surface, onto:supports, Object).
:- rdf_meta
  supports(r,r).
is_on_top_of(Object, Surface):-
  rdf_has(Object, onto:is_on_top_of, Surface).

is_on_top_of(Object, Surface):-
  contains(Object, Item),
  rdf_has(Item, onto:is_on_top_of, Surface).

is_on_top_of(Object, Surface):-
  contains(Container, Object),
  rdf_has(Container, onto:is_on_top_of, Surface).

:- rdf_meta
  is_on_top_of(r,r).
%depend on foreign functions
infer_contains(Container, Item):-
  is_a(Container, onto:'Container'),
  is_a(Item, onto:'SmallItem'),
  close_in_time(Container, Item),
  comp_close_enough(Container, Item),
  have_common_colors(Container, Item).

infer_supports(Surface, Object):-
  is_a(Surface, onto:'Surface'),
  is_a(Object, onto:'Object'),
  comp_confine(Surface, Object).
% both have no colors
have_common_colors(ObjA, ObjB):-
  has_colors(ObjA, Color1),
  has_colors(ObjB, Color2),
  intersection(Color1, Color2,CommonColors),
  CommonColors == [""].

% both have colors
have_common_colors(ObjA, ObjB):-
  has_colors(ObjA, Color1),
  has_colors(ObjB, Color2),
  intersection(Color1, Color2,CommonColors),
  CommonColors \== [""].
  % member(_, CommonColors). %because of [""] this will be true
:- rdf_meta
  have_common_colors(r,r).

have_no_common_colors(ObjA, ObjB) :-
  has_colors(ObjA, Color1),
  has_colors(ObjB, Color2),
  intersection(Color1, Color2,CommonColors),
  not(member(_, CommonColors)).

infer_not_contains(Container, Item):-
  close_in_time(Container, Item),
  have_no_common_colors(Container, Item),
  comp_far_enough(Container, Item).

is_same_type(ObjA, ObjB):-
  rdf_has(ObjA, rdf:type, T),
  rdf_has(ObjB, rdf:type, T),
  rdf_reachable(T ,rdfs:subClassOf, onto:'Object').
  % not(rdf_equal(ObjA, ObjB)).
:- rdf_meta
  is_same_type(r, r).

% for small items
is_potential_match(Ref_Obj, Obj):-
  is_same_type(Ref_Obj, Obj),
  not(is_a(Ref_Obj, onto:'Container')).
% for containers
is_potential_match(Ref_Obj, Obj):-
  is_a(Ref_Obj, onto:'Container'),
  is_a(Obj, onto:'Container'),
  contains(Ref_Obj, Item),
  infer_contains_or_empty(Obj,Item).
% for empty containers
is_potential_match(Ref_Obj, Obj):-
  is_a(Ref_Obj, onto:'Container'),
  is_a(Obj, onto:'Container'),
  not(contains(Ref_Obj, _)),
  % is_on_top_of(Ref_Obj, Surface),
  % is_on_top_of(Obj, Surface).
  have_common_colors(Ref_Obj, Obj).

infer_contains_or_empty(Obj, Item) :-
  infer_contains(Obj,Item);
  not(infer_contains(Obj,_)).

contains_or_empty(Obj, Item) :-
  contains(Obj, Item);
  not(contains(Obj, _)).
:- rdf_meta
  contains_or_empty(r,r).
  % (
  %   is_a(Ref_Obj, onto:'Container') ->
  %   writeln('they are both containers'),
  %   (
  %     infer_contains(Obj, Item), contains(Ref_Obj, Item) ->
  %     writeln('they contains the same item')
  %     ;
  %     writeln('they contains the differnt items')
  %   )
  %   ;
  %   writeln('they are not containers')
  % ).

% for matching empty containers assuming each table has only one container
is_match(ObjA, ObjB) :-
  is_a(ObjA, onto:'Container'),
  is_a(ObjB, onto:'Container'),
  ObjA \== ObjB,
  not(contains(ObjA, _)),
  not(contains(ObjB, _)),
  have_common_colors(ObjA, ObjB),
  is_on_top_of(ObjA, Surface),
  is_on_top_of(ObjB, Surface).
  % comp_close_enough(ObjA, ObjB).

is_match(ObjA, ObjB) :-
  % ObjA \== ObjB,
  contains(ObjA, Item),
  contains(ObjB, Item),
  ObjA \== ObjB,
  have_common_colors(ObjA, ObjB).
is_match(ObjA, ObjB) :-
  is_inside_of(ObjA, Container),
  is_inside_of(ObjB, Container),
  ObjA \== ObjB.

infer_match(ObjA, ObjB) :-
  ObjA \== ObjB,
  infer_contains(ObjA, Item),
  contains(ObjB, Item).
