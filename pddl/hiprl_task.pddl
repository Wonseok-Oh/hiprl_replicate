;Header and description

(define (domain hiprl_task)

;remove requirements that are not needed
(:requirements 
    :adl
    :fluents
)


;:strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)
(:types 
    agent
    location
    receptacle
    object
    rtype
    otype
    direction
)

; un-comment following line if constants are needed


(:predicates ;todo: define predicates here
    (at_location ?a - agent ?l - location)
    (receptacle_at_location ?r - receptacle ?l - location)
    (object_at_location ?o - object ?l - location)
    (openable ?r - receptacle)
    (opened ?r - receptacle)
    (in_receptacle ?o - object ?r - receptacle)
    (checked ?r - receptacle)
    (object_type ?o - object ?t - otype)
    (holds ?a - agent ?o - object)
    (holdsAny ?a - agent)
    (full ?r - receptacle)
    (conn ?v0 - location ?v1 - location ?v2 - direction)
    (clear ?v0 - location)
)

(:functions 
    (total_cost)
)


;define actions here
;; agent move one grid ... is this appropriate? not goes to obstacle?
(:action move-robot
    :parameters (?a - agent ?lStart - location ?lEnd - location ?dir - direction)
    :precondition (and (at_location ?a ?lStart)
                       (conn ?lStart ?lEnd ?dir)
                       (at_location ?a ?lStart)
                       (clear ?lEnd))
    :effect (and 
        (at_location ?a ?lEnd)
        (not (at_location ?a ?lStart))
        (not (clear ?lEnd))
        (clear ?lStart)
        (increase (total_cost) 1)

    )
)
;;agent opens receptacle
(:action OpenObject
    :parameters (?a - agent ?al - location ?r - receptacle ?rl - location ?dir - direction)
    :precondition (and
        (at_location ?a ?al)
        (receptacle_at_location ?r ?rl)
        (openable ?r)
        (not (opened ?r))
        (conn ?al ?rl ?dir)
    )
    :effect (and
        (opened ?r)
        (checked ?r)
        (increase (total_cost) 1)
    )
)

;; agent closes receptacle
(:action CloseObject
    :parameters (?a - agent ?al - location ?r - receptacle ?rl - location ?dir - direction)
    :precondition (and
        (at_location ?a ?al)
        (receptacle_at_location ?r ?rl)
        (openable ?r)
        (opened ?r)
        (conn ?al ?rl ?dir)
    )
    :effect (and
        (not (opened ?r))
        (increase (total_cost) 1)
    )
)

;; agent picks up object in a receptacle
(:action PickupObjectInReceptacle
    :parameters (?a - agent ?al - location ?o - object ?ol - location ?r - receptacle)
    :precondition (and 
        (at_location ?a ?al)
        (object_at_location ?o ?ol)
        (receptacle_at_location ?r ?ol)
        (exists (?dir - direction) (conn ?al ?ol ?dir))
        (opened ?r)
        (in_receptacle ?o ?r)
        (not (holdsAny ?a))
    )
    :effect (and
        (not (in_receptacle ?o ?r))
        (holds ?a ?o)
        (holdsAny ?a)
        (not (object_at_location ?o ?ol))
        (not ( full ?r )) 
        (increase (total_cost) 1)
    )
)

;; agent puts down object in a receptacle
(:action PutObjectInReceptacle
    :parameters (?a - agent ?al - location ?ot - otype ?o - object ?ol - location ?r - receptacle)
    :precondition (and 
        (at_location ?a ?al)
        (receptacle_at_location ?r ?ol)
        (opened ?r)
        (not (full ?r))
        (object_type ?o ?ot)
        (holds ?a ?o)
        (exists (?dir - direction) (conn ?al ?ol ?dir))
    )
    :effect (and
        (in_receptacle ?o ?r)
        (object_at_location ?o ?ol)
        (full ?r)
        (not (holds ?a ?o))
        (not (holdsAny ?a))
        (increase (total_cost) 1)
    )
)

;; agent picks up object
(:action PickupObject
    :parameters (?a - agent ?al - location ?o - object ?ol - location)
    :precondition (and 
        (at_location ?a ?al)
        (object_at_location ?o ?ol)
        (exists (?dir - direction) (conn ?al ?ol ?dir))
        (not (holdsAny ?a))
    )
    :effect (and
        (not (object_at_location ?o ?ol))
        (holds ?a ?o)
        (holdsAny ?a)
        (clear ?ol)
        (increase (total_cost) 1)
    )
)

;; agent puts down object
(:action PutObject
    :parameters (?a - agent ?al - location ?ot - otype ?o - object ?ol - location)
    :precondition (and 
        (at_location ?a ?al)
        (clear ?ol)
        (object_type ?o ?ot)
        (exists (?dir - direction) (conn ?al ?ol ?dir))
        (holds ?a ?o)
    )
    :effect (and
        (object_at_location ?o ?ol)
        (not (clear ?ol))
        (not (holds ?a ?o))
        (not (holdsAny ?a))
        (increase (total_cost) 1)
    )
)
)
