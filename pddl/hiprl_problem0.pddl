(define (problem hiprl_problem_mini)
    (:domain hiprl_mini)
    (:objects 
        f0-0f f0-1f f0-2f f0-3f f0-4f f0-5f f0-6f f0-7f f0-8f f0-9f f1-0f f1-1f f1-2f f1-3f f1-4f f1-5f f1-6f f1-7f f1-8f f1-9f f2-0f f2-1f f2-2f f2-3f f2-4f f2-5f f2-6f f2-7f f2-8f f2-9f f3-0f f3-1f f3-2f f3-3f f3-4f f3-5f f3-6f f3-7f f3-8f f3-9f f4-0f f4-1f f4-2f f4-3f f4-4f f4-5f f4-6f f4-7f f4-8f f4-9f f5-0f f5-1f f5-2f f5-3f f5-4f f5-5f f5-6f f5-7f f5-8f f5-9f f6-0f f6-1f f6-2f f6-3f f6-4f f6-5f f6-6f f6-7f f6-8f f6-9f f7-0f f7-1f f7-2f f7-3f f7-4f f7-5f f7-6f f7-7f f7-8f f7-9f f8-0f f8-1f f8-2f f8-3f f8-4f f8-5f f8-6f f8-7f f8-8f f8-9f f9-0f f9-1f f9-2f f9-3f f9-4f f9-5f f9-6f f9-7f f9-8f f9-9f - location
        robot - agent
		box0 - receptacle
		ball0 - obj
        ball_type - otype
		box_type - rtype
		down left right up - direction
    )
    (:init
		(at_location robot f0-0f)
		(handsFree robot)
        (object_type ball0 ball_type)

       ; 0,0 block
    	(conn f0-0f f0-1f right)
	    (conn f0-0f f1-0f down)
	    (conn f0-1f f0-0f left)
	    (conn f0-1f f0-2f right)
	    (conn f0-1f f1-1f down)
	    (conn f0-2f f0-1f left)
	    (conn f0-2f f0-3f right)
	    (conn f0-2f f1-2f down)
	    (conn f0-3f f0-2f left)
	    (conn f0-3f f0-4f right)
	    (conn f0-3f f1-3f down)
	    (conn f0-4f f0-3f left)
    	(conn f0-4f f1-4f down)
	    (conn f1-0f f0-0f up)
    	(conn f1-0f f1-1f right)
	    (conn f1-0f f2-0f down)
	    (conn f1-1f f0-1f up)
	    (conn f1-1f f1-0f left)
	    (conn f1-1f f1-2f right)
	    (conn f1-1f f2-1f down)
	    (conn f1-2f f0-2f up)
    	(conn f1-2f f1-1f left)
	    (conn f1-2f f1-3f right)
	    (conn f1-2f f2-2f down)
	    (conn f1-3f f0-3f up)
	    (conn f1-3f f1-2f left)
	    (conn f1-3f f1-4f right)
	    (conn f1-3f f2-3f down)
	    (conn f1-4f f0-4f up)
	    (conn f1-4f f1-3f left)
	    (conn f1-4f f2-4f down)
	    (conn f2-0f f1-0f up)
	    (conn f2-0f f2-1f right)
	    (conn f2-0f f3-0f down)
	    (conn f2-1f f1-1f up)
	    (conn f2-1f f2-0f left)
	    (conn f2-1f f2-2f right)
	    (conn f2-1f f3-1f down)
	    (conn f2-2f f1-2f up)
	    (conn f2-2f f2-1f left)
	    (conn f2-2f f2-3f right)
	    (conn f2-2f f3-2f down)
	    (conn f2-3f f1-3f up)
	    (conn f2-3f f2-2f left)
	    (conn f2-3f f2-4f right)
	    (conn f2-3f f3-3f down)
	    (conn f2-4f f1-4f up)
	    (conn f2-4f f2-3f left)
	    (conn f2-4f f2-5f right)
	    (conn f2-4f f3-4f down)
	    (conn f3-0f f2-0f up)
	    (conn f3-0f f3-1f right)
	    (conn f3-0f f4-0f down)
	    (conn f3-1f f2-1f up)
	    (conn f3-1f f3-0f left)
	    (conn f3-1f f3-2f right)
	    (conn f3-1f f4-1f down)
	    (conn f3-2f f2-2f up)
	    (conn f3-2f f3-1f left)
	    (conn f3-2f f3-3f right)
	    (conn f3-2f f4-2f down)
	    (conn f3-3f f2-3f up)
	    (conn f3-3f f3-2f left)
	    (conn f3-3f f3-4f right)
	    (conn f3-3f f4-3f down)
	    (conn f3-4f f2-4f up)
	    (conn f3-4f f3-3f left)
	    (conn f3-4f f4-4f down)
	    (conn f4-0f f3-0f up)
	    (conn f4-0f f4-1f right)
	    (conn f4-0f f5-0f down)
	    (conn f4-1f f3-1f up)
	    (conn f4-1f f4-0f left)
	    (conn f4-1f f4-2f right)
	    (conn f4-2f f3-2f up)
	    (conn f4-2f f4-1f left)
	    (conn f4-2f f4-3f right)
	    (conn f4-3f f3-3f up)
	    (conn f4-3f f4-2f left)
	    (conn f4-3f f4-4f right)
	    (conn f4-4f f3-4f up)
	    (conn f4-4f f4-3f left)

        ;; connect 0,0 1,0 block
        (conn f4-0f f5-0f down)
        (conn f5-0f f4-0f up)
        (conn f4-1f f5-1f down)
        (conn f5-1f f4-1f up)
        (conn f4-2f f5-2f down)
        (conn f5-2f f4-2f up)
        (conn f4-3f f5-3f down)
        (conn f5-3f f4-3f up)
        (conn f4-4f f5-4f down)
        (conn f5-4f f4-4f up)

        ; 1,0 block
        (conn f5-0f f5-1f right)
	    (conn f5-0f f6-0f down)
	    (conn f5-1f f5-0f left)
	    (conn f5-1f f5-2f right)
	    (conn f5-1f f6-1f down)
	    (conn f5-2f f5-1f left)
	    (conn f5-2f f5-3f right)
	    (conn f5-2f f6-2f down)
	    (conn f5-3f f5-2f left)
	    (conn f5-3f f5-4f right)
	    (conn f5-3f f6-3f down)
	    (conn f5-4f f5-3f left)
    	(conn f5-4f f6-4f down)
	    (conn f6-0f f5-0f up)
    	(conn f6-0f f6-1f right)
	    (conn f6-0f f7-0f down)
	    (conn f6-1f f5-1f up)
	    (conn f6-1f f6-0f left)
	    (conn f6-1f f6-2f right)
	    (conn f6-1f f7-1f down)
	    (conn f6-2f f5-2f up)
    	(conn f6-2f f6-1f left)
	    (conn f6-2f f6-3f right)
	    (conn f6-2f f7-2f down)
	    (conn f6-3f f5-3f up)
	    (conn f6-3f f6-2f left)
	    (conn f6-3f f6-4f right)
	    (conn f6-3f f7-3f down)
	    (conn f6-4f f5-4f up)
	    (conn f6-4f f6-3f left)
	    (conn f6-4f f7-4f down)
	    (conn f7-0f f6-0f up)
	    (conn f7-0f f7-1f right)
	    (conn f7-0f f8-0f down)
	    (conn f7-1f f6-1f up)
	    (conn f7-1f f7-0f left)
	    (conn f7-1f f7-2f right)
	    (conn f7-1f f8-1f down)
	    (conn f7-2f f6-2f up)
	    (conn f7-2f f7-1f left)
	    (conn f7-2f f7-3f right)
	    (conn f7-2f f8-2f down)
	    (conn f7-3f f6-3f up)
	    (conn f7-3f f7-2f left)
	    (conn f7-3f f7-4f right)
	    (conn f7-3f f8-3f down)
	    (conn f7-4f f6-4f up)
	    (conn f7-4f f7-3f left)
	    (conn f7-4f f8-4f down)
	    (conn f8-0f f7-0f up)
	    (conn f8-0f f8-1f right)
	    (conn f8-0f f9-0f down)
	    (conn f8-1f f7-1f up)
	    (conn f8-1f f8-0f left)
	    (conn f8-1f f8-2f right)
	    (conn f8-1f f9-1f down)
	    (conn f8-2f f7-2f up)
	    (conn f8-2f f8-1f left)
	    (conn f8-2f f8-3f right)
	    (conn f8-2f f9-2f down)
	    (conn f8-3f f7-3f up)
	    (conn f8-3f f8-2f left)
	    (conn f8-3f f8-4f right)
	    (conn f8-3f f9-3f down)
	    (conn f8-4f f7-4f up)
	    (conn f8-4f f8-3f left)
	    (conn f8-4f f9-4f down)
	    (conn f9-0f f8-0f up)
	    (conn f9-0f f9-1f right)
	    (conn f9-0f f5-0f down)
	    (conn f9-1f f8-1f up)
	    (conn f9-1f f9-0f left)
	    (conn f9-1f f9-2f right)
	    (conn f9-2f f8-2f up)
	    (conn f9-2f f9-1f left)
	    (conn f9-2f f9-3f right)
	    (conn f9-3f f8-3f up)
	    (conn f9-3f f9-2f left)
	    (conn f9-3f f9-4f right)
	    (conn f9-4f f8-4f up)
	    (conn f9-4f f9-3f left)

        ;; connect 0,0 0,1 block
        (conn f0-4f f0-5f right)
        (conn f0-5f f0-4f left)
        (conn f1-4f f1-5f right)
        (conn f1-5f f1-4f left)
        (conn f2-4f f2-5f right)
        (conn f2-5f f2-4f left)
        (conn f3-4f f3-5f right)
        (conn f3-5f f3-4f left)
        (conn f4-4f f4-5f right)
        (conn f4-5f f4-4f left)

        ; 0,1 block
        (conn f0-5f f0-6f right)
	    (conn f0-5f f1-5f down)
	    (conn f0-6f f0-5f left)
	    (conn f0-6f f0-7f right)
	    (conn f0-6f f1-6f down)
	    (conn f0-7f f0-6f left)
	    (conn f0-7f f0-8f right)
	    (conn f0-7f f1-7f down)
	    (conn f0-8f f0-7f left)
	    (conn f0-8f f0-9f right)
	    (conn f0-8f f1-8f down)
	    (conn f0-9f f0-8f left)
    	(conn f0-9f f1-9f down)
	    (conn f1-5f f0-5f up)
    	(conn f1-5f f1-6f right)
	    (conn f1-5f f2-5f down)
	    (conn f1-6f f0-6f up)
	    (conn f1-6f f1-5f left)
	    (conn f1-6f f1-7f right)
	    (conn f1-6f f2-6f down)
	    (conn f1-7f f0-7f up)
    	(conn f1-7f f1-6f left)
	    (conn f1-7f f1-8f right)
	    (conn f1-7f f2-7f down)
	    (conn f1-8f f0-8f up)
	    (conn f1-8f f1-7f left)
	    (conn f1-8f f1-9f right)
	    (conn f1-8f f2-8f down)
	    (conn f1-9f f0-9f up)
	    (conn f1-9f f1-8f left)
	    (conn f1-9f f2-9f down)
	    (conn f2-5f f1-5f up)
	    (conn f2-5f f2-6f right)
	    (conn f2-5f f3-5f down)
	    (conn f2-6f f1-6f up)
	    (conn f2-6f f2-5f left)
	    (conn f2-6f f2-7f right)
	    (conn f2-6f f3-6f down)
	    (conn f2-7f f1-7f up)
	    (conn f2-7f f2-6f left)
	    (conn f2-7f f2-8f right)
	    (conn f2-7f f3-7f down)
	    (conn f2-8f f1-8f up)
	    (conn f2-8f f2-7f left)
	    (conn f2-8f f2-9f right)
	    (conn f2-8f f3-8f down)
	    (conn f2-9f f1-9f up)
	    (conn f2-9f f2-8f left)
	    (conn f2-9f f3-9f down)
	    (conn f3-5f f2-5f up)
	    (conn f3-5f f3-6f right)
	    (conn f3-5f f4-5f down)
	    (conn f3-6f f2-6f up)
	    (conn f3-6f f3-5f left)
	    (conn f3-6f f3-7f right)
	    (conn f3-6f f4-6f down)
	    (conn f3-7f f2-7f up)
	    (conn f3-7f f3-6f left)
	    (conn f3-7f f3-8f right)
	    (conn f3-7f f4-7f down)
	    (conn f3-8f f2-8f up)
	    (conn f3-8f f3-7f left)
	    (conn f3-8f f3-9f right)
	    (conn f3-8f f4-8f down)
	    (conn f3-9f f2-9f up)
	    (conn f3-9f f3-8f left)
	    (conn f3-9f f3-5f right)
	    (conn f3-9f f4-9f down)
	    (conn f4-5f f3-5f up)
	    (conn f4-5f f4-6f right)
	    (conn f4-5f f5-5f down)
	    (conn f4-6f f3-6f up)
	    (conn f4-6f f4-5f left)
	    (conn f4-6f f4-7f right)
	    (conn f4-7f f3-7f up)
	    (conn f4-7f f4-6f left)
	    (conn f4-7f f4-8f right)
	    (conn f4-8f f3-8f up)
	    (conn f4-8f f4-7f left)
	    (conn f4-8f f4-9f right)
	    (conn f4-9f f3-9f up)
	    (conn f4-9f f4-8f left)

        ;; connect 1,0 1,1 block
        (conn f5-4f f5-5f right)
        (conn f5-5f f5-4f left)
        (conn f6-4f f6-5f right)
        (conn f6-5f f6-4f left)
        (conn f7-4f f7-5f right)
        (conn f7-5f f7-4f left)
        (conn f8-4f f8-5f right)
        (conn f8-5f f8-4f left)
        (conn f9-4f f9-5f right)
        (conn f9-5f f9-4f left)

        ;; connect 0,1 1,1 block
        (conn f4-5f f5-5f down)
        (conn f5-5f f4-5f up)
        (conn f4-6f f5-6f down)
        (conn f5-6f f4-6f up)
        (conn f4-7f f5-7f down)
        (conn f5-7f f4-7f up)
        (conn f4-8f f5-8f down)
        (conn f5-8f f4-8f up)
        (conn f4-9f f5-9f down)
        (conn f5-9f f4-9f up)

        ; 1,1 blocks
        (conn f5-5f f5-6f right)
	    (conn f5-5f f6-5f down)
	    (conn f5-6f f5-5f left)
	    (conn f5-6f f5-7f right)
	    (conn f5-6f f6-6f down)
	    (conn f5-7f f5-6f left)
	    (conn f5-7f f5-8f right)
	    (conn f5-7f f6-7f down)
	    (conn f5-8f f5-7f left)
	    (conn f5-8f f5-9f right)
	    (conn f5-8f f6-8f down)
	    (conn f5-9f f5-8f left)
    	(conn f5-9f f6-9f down)
	    (conn f6-5f f5-5f up)
    	(conn f6-5f f6-6f right)
	    (conn f6-5f f7-5f down)
	    (conn f6-6f f5-6f up)
	    (conn f6-6f f6-5f left)
	    (conn f6-6f f6-7f right)
	    (conn f6-6f f7-6f down)
	    (conn f6-7f f5-7f up)
    	(conn f6-7f f6-6f left)
	    (conn f6-7f f6-8f right)
	    (conn f6-7f f7-7f down)
	    (conn f6-8f f5-8f up)
	    (conn f6-8f f6-7f left)
	    (conn f6-8f f6-9f right)
	    (conn f6-8f f7-8f down)
	    (conn f6-9f f5-9f up)
	    (conn f6-9f f6-8f left)
	    (conn f6-9f f7-9f down)
	    (conn f7-5f f6-5f up)
	    (conn f7-5f f7-6f right)
	    (conn f7-5f f8-5f down)
	    (conn f7-6f f6-6f up)
	    (conn f7-6f f7-5f left)
	    (conn f7-6f f7-7f right)
	    (conn f7-6f f8-6f down)
	    (conn f7-7f f6-7f up)
	    (conn f7-7f f7-6f left)
	    (conn f7-7f f7-8f right)
	    (conn f7-7f f8-7f down)
	    (conn f7-8f f6-8f up)
	    (conn f7-8f f7-7f left)
	    (conn f7-8f f7-9f right)
	    (conn f7-8f f8-8f down)
	    (conn f7-9f f6-9f up)
	    (conn f7-9f f7-8f left)
	    (conn f7-9f f8-9f down)
	    (conn f8-5f f7-5f up)
	    (conn f8-5f f8-6f right)
	    (conn f8-5f f9-5f down)
	    (conn f8-6f f7-6f up)
	    (conn f8-6f f8-5f left)
	    (conn f8-6f f8-7f right)
	    (conn f8-6f f9-6f down)
	    (conn f8-7f f7-7f up)
	    (conn f8-7f f8-6f left)
	    (conn f8-7f f8-8f right)
	    (conn f8-7f f9-7f down)
	    (conn f8-8f f7-8f up)
	    (conn f8-8f f8-7f left)
	    (conn f8-8f f8-9f right)
	    (conn f8-8f f9-8f down)
	    (conn f8-9f f7-9f up)
	    (conn f8-9f f8-8f left)
	    (conn f8-9f f9-9f down)
	    (conn f9-5f f8-5f up)
	    (conn f9-5f f9-6f right)
	    (conn f9-5f f5-0f down)
	    (conn f9-6f f8-6f up)
	    (conn f9-6f f9-5f left)
	    (conn f9-6f f9-7f right)
	    (conn f9-7f f8-7f up)
	    (conn f9-7f f9-6f left)
	    (conn f9-7f f9-8f right)
	    (conn f9-8f f8-8f up)
	    (conn f9-8f f9-7f left)
	    (conn f9-8f f9-9f right)
	    (conn f9-9f f8-9f up)
	    (conn f9-9f f9-8f left)
    )
    
    (:goal 
		(object_at_location ball0 f0-0f)
    )

	;(:metric minimize (total_cost))

)