(define (problem task)
(:domain hiprl_mini)
(:objects
    robot - agent
    box0 - receptacle
    ball0 - obj    
    f0-0f f0-1f f1-0f f1-1f - location
    ball_type - otype
    down left right up - direction
)
(:init
    ; (not (at_location robot f0-0f))
    (at_location robot f1-2f)

    (receptacle_at_location box0 f1-3f)
    (object_at_location ball0 f2-3f)

    (openable box0)


    (closed box0)


    (checked box0)

    (object_type ball0 ball_type)





    (empty box0)

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

    (clear f3-1f)
    (clear f2-1f)
    (clear f1-1f)
    (clear f2-2f)
    (clear f1-2f)
    (clear f3-3f)
    (clear f2-3f)

)
(:goal (and
    (in_receptacle ball0 box0)
))
)
