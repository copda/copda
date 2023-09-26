
:- use_module(library(semweb/rdf_db)).
:- use_module(library(semweb/rdf_portray)).
:- use_module(library(semweb/rdfs)).

:- rdf_register_prefix(onto, 'http://dfki.ipr.de/onto#').
:- rdf_load("knowledge_graph.owl").


switch(X, [Val:Goal|Cases]) :-
  ( X=Val ->
      call(Goal)
  ;
      switch(X, Cases)
  ).

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

object_distance(A,B,Distance) :-
	% ground(A),
	% ground(B),
  % ground(Distance),
  % is_at(A, P1),
  % is_at(B, P2),
  % distance(P1, P2, Distance).
	is_at(A, [AX,AY,AZ]),
	is_at(B, [BX,BY,BZ]),
	DX is AX - BX,
	DY is AY - BY,
	DZ is AZ - BZ,
	Distance is sqrt(((DX*DX) + (DY*DY)) + (DZ*DZ)).

is_a2(O, T) :-
  rdf_has(O, rdf:type, Td),
  % rdf_reachable(Td,rdfs:subClassOf, onto:'Object'),
  rdf_reachable(Td,rdfs:subClassOf, T).


is_container(O) :-
  % ground(O),
  rdf_has(O, rdf:type, T),
  rdf_reachable(T,rdfs:subClassOf, onto:'Container'),
  not(rdf_equal(O, onto:'Container')).
  % rdfs:rdfs_individual_of(O, onto:'Container').

is_small_item(S) :-
  % rdfs_individual_of(S, onto:'SmallItem').
  rdf_has(S, rdf:type, T),
  rdf_reachable(T,rdfs:subClassOf,onto:'SmallItem'),
  not(rdf_equal(S, onto:'SmallItem')).

check(Fact) :-
    call(Fact), !,
    write('Fact Exists');
    write('Fact Doesen\'t exists'), fail.

contains(Container, Item) :-
  % check(rdf_has(Container, onto:contains, Item)),
  rdf_has(Container, onto:contains, Item).


is_inside_of(Item, Container) :-
  rdf_has(Item, onto:is_inside_of, Container).

% infer_distance(ObjA, ObjB, Dist) :-
is_functional_prop(Prop) :-
  rdf_has(Prop, rdf:type ,owl:'FunctionalProperty').


infer_contains(Container, Item):-
  is_a(Container, 'Container'),
  is_a(Item, 'SmallItem'),
  comp_distance(Container, Item, Dist),
  =<(Dist, 0.15).
% has(Container, onto:'Color', Color),
% has(Item, onto:'Color', Color).
infer_not_contains(Container, Item):-
  % \+ is_a(Container, 'Container'),
  % \+ is_a(Item, 'SmallItem'),
  comp_distance(Container, Item, Dist),
  >=(Dist, 0.20).

infer_match(ObjA, ObjB) :-
  infer_contains(ObjA, Item),
  contains(ObjB, Item).


% contain_same(ObjA, ObjB) :-
%   (
%   contains(ObjA, Item), writeln(Item)
%   -> contains(ObjB, Item2), writeln(Item2)
%   ; false
%   ).

is_potential_match(ObjA, ObjB) :-
  is_same_class(ObjA, ObjB).
  % comp_topo_consistent(ObjA, ObjB).
  % (
  % contains(ObjA, Item)
  % ->
  % ( contains(ObjA, Item),contains(ObjB, Item))
  % ; true
  % ).
  % TODO: try use this instead
  % is_a(ObjA, Class),
  % is_a(ObjB, Class),

of_same_class(ObjA, ObjB) :-
  is_a2(ObjA, Cls),
  is_a2(ObjB, Cls),
  ObjA \== ObjB.
% infer_match(ObjA, ObjB) :-
%   infer_contains(ObjA, Item),
%   infer_contains(ObjB, Item),
%   ObjA \== ObjB.

% infer_match(ObjA, ObjB) :-
%   contains(ObjA, Item),
%   infer_contains(ObjB, Item).
  % (
  %   is_container(ObjA)
  %   ->
  %   infer_contains(ObjA, Item),
  %   contains(ObjB, Item),
  %   ObjA \== ObjB
  % ).

find_match(ObjA, ObjB) :-
  (
    is_container(ObjA)
    ->
    contains(ObjA, Item),
    contains(ObjB, Item),
    ObjA \== ObjB
    ; is_small_item(ObjA)
    ->
    is_inside_of(ObjA, Container),
    is_inside_of(ObjB, Container),
    ObjA \== ObjB
  ).

% find_conflict(ObjA, ObjB) :-
%   switch(ObjA, [
%       a : writeln(case1),
%       b : writeln(case2),
%       c : writeln(case3)
%   ]).

contains(C, O, MinD) :-
  ground(MinD),
  is_container(C),
  is_small_item(O),
  % is_at(C,[CX, CY, CZ]),
  % is_at(O,[OX, OY, OZ]),
  not(rdf_equal(C, O)),
  object_distance(C, O, Dist),
  =<(Dist, MinD).
  % object_dimensions(C, [CD, CW, CH]),
  % object_dimensions(O, [OD, OW, OH]),
  % InnerObj is contained by OuterObj if (center_i+0.5*dim_i)<=(center_o+0.5*dim_o)
  % for all dimensions (x, y, z)
  % >=( (CX - 0.5*CD), (OX - 0.5*OD)-0.05 ),
  % =<( (CX + 0.5*CD), (OX + 0.5*OD)+0.05 ),
  % >=( (CY - 0.5*CW), (OY - 0.5*OW)-0.05 ),
  % =<( (CY + 0.5*CW), (OY + 0.5*OW)+0.05 ),
  % >=( (CZ - 0.5*CH), (OZ - 0.5*OH)-0.05 ),
  % =<( (CZ + 0.5*CH), (OZ + 0.5*OH)+0.05 ).


% find_match(X, M) :-
%   % is_match(X,M).
%   MinD is 0.1,
%   (
%     % rdf(X, subclasspf, container),
%     is_container(X)
%     ->
%     % X \== M,
%     contains(X, Y, MinD),
%     contains(M, Y, MinD),
%     X \== M
%     ; is_small_item(X),  writeln(X)
%     ->
%     contains(C, X, MinD),
%     contains(C, M, MinD),
%     X \== M
%   ).
