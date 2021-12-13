;Header and description

(define (domain hiprl_mini_multi)

;remove requirements that are not needed
(:requirements 
    :strips
    :typing
    :disjunctive-preconditions
    :negative-preconditions
)


;:strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)
(:types 
    agent
    location
    receptacle
    obj
    otype
    direction
)

; un-comment following line if constants are needed

(:predicates ;todo: define predicates here
    (planning_agent ?a - agent)
    (at_location ?a - agent ?l - location)
    (receptacle_at_location ?r - receptacle ?l - location)
    (object_at_location ?o - obj ?l - location)
    (openable ?r - receptacle)
    (opened ?r - receptacle)
    (closed ?r - receptacle)
    (in_receptacle ?o - obj ?r - receptacle)
    (out_receptacle ?o - obj)
    (checked ?r - receptacle)
    (object_type ?o - obj ?t - otype)
    (holds ?a - agent ?o - obj)
    (holdsAny ?a - agent)
    (handsfree ?a - agent)
    (full ?r - receptacle)
    (empty ?r - receptacle)
    (conn ?v0 - location ?v1 - location ?v2 - direction)
    (clear ?v0 - location)
)



;define actions here
;; agent move one grid ... is this appropriate? not goes to obstacle?
(:action move-robot
    :parameters (?a - agent ?lStart - location ?lEnd - location ?dir - direction)
    :precondition (and (planning_agent ?a)
                       (at_location ?a ?lStart)
                       (conn ?lStart ?lEnd ?dir)
                       (at_location ?a ?lStart)
                       (clear ?lEnd))
    :effect (and 
        (at_location ?a ?lEnd)
        (not (at_location ?a ?lStart))
        (not (clear ?lEnd))
        (clear ?lStart)
    )
)
;;agent opens receptacle
(:action OpenObject
    :parameters (?a - agent ?al - location ?r - receptacle ?rl - location ?dir - direction)
    :precondition (and (planning_agent ?a)
        (at_location ?a ?al)
        (receptacle_at_location ?r ?rl)
        (openable ?r)
        (closed ?r)
        (conn ?al ?rl ?dir)
    )
    :effect (and
        (opened ?r)
        (not (closed ?r))
        (checked ?r)
    )
)

;; agent closes receptacle
(:action CloseObject
    :parameters (?a - agent ?al - location ?r - receptacle ?rl - location ?dir - direction)
    :precondition (and 
        (planning_agent ?a)
        (at_location ?a ?al)
        (receptacle_at_location ?r ?rl)
        (openable ?r)
        (opened ?r)
        (conn ?al ?rl ?dir)
    )
    :effect (and
        (not (opened ?r))
        (closed ?r)
    )
)

;; agent picks up obj in a receptacle
(:action PickupObjectInReceptacle
    :parameters (?a - agent ?al - location ?o - obj ?ol - location ?r - receptacle ?dir - direction)
    :precondition (and 
        (planning_agent ?a)
        (at_location ?a ?al)
        (object_at_location ?o ?ol)
        (conn ?al ?ol ?dir)
        (opened ?r)
        (in_receptacle ?o ?r)
        (handsfree ?a)
    )
    :effect (and
        (not (in_receptacle ?o ?r))
        (out_receptacle ?o)
        (holds ?a ?o)
        (holdsAny ?a)
        (not (handsfree ?a))
        (not (object_at_location ?o ?ol))
        (not ( full ?r ))
        (empty ?r) 
    )
)

;; agent puts down obj in a receptacle
(:action PutObjectInReceptacle
    :parameters (?a - agent ?al - location ?ot - otype ?o - obj ?ol - location ?r - receptacle ?dir - direction)
    :precondition (and 
        (planning_agent ?a)
        (at_location ?a ?al)
        (receptacle_at_location ?r ?ol)
        (opened ?r)
        (empty ?r)
        (object_type ?o ?ot)
        (holds ?a ?o)
        (conn ?al ?ol ?dir)
    )
    :effect (and
        (in_receptacle ?o ?r)
        (object_at_location ?o ?ol)
        (full ?r)
        (not (empty ?r))
        (not (holds ?a ?o))
        (not (holdsAny ?a))
        (not (out_receptacle ?o))
        (handsfree ?a)
    )
)

;; agent picks up obj
(:action PickupObject
    :parameters (?a - agent ?al - location ?o - obj ?ol - location ?dir - direction)
    :precondition (and 
        (planning_agent ?a)
        (at_location ?a ?al)
        (object_at_location ?o ?ol)
        (out_receptacle ?o)
        (conn ?al ?ol ?dir)
        (handsfree ?a)
    )
    :effect (and
        (not (object_at_location ?o ?ol))
        (not (handsfree ?a))
        (holds ?a ?o)
        (holdsAny ?a)
        (clear ?ol)
    )
)

;; agent puts down obj
(:action PutObject
    :parameters (?a - agent ?al - location ?ot - otype ?o - obj ?ol - location ?dir - direction)
    :precondition (and 
        (planning_agent ?a)
        (at_location ?a ?al)
        (clear ?ol)
        (object_type ?o ?ot)
        (conn ?al ?ol ?dir)
        (holds ?a ?o)
    )
    :effect (and
        (object_at_location ?o ?ol)
        (not (clear ?ol))
        (not (holds ?a ?o))
        (not (holdsAny ?a))
        (handsfree ?a)
        (out_receptacle ?o)
    )
)
)
