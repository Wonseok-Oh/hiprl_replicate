(define (problem task)
(:domain hiprl_mini)
(:objects
    robot - agent
    f0-0f f0-1f f0-2f f0-3f f0-4f f0-5f f0-6f f0-7f f0-8f f0-9f f1-0f f1-1f f1-2f f1-3f f1-4f f1-5f f1-6f f1-7f f1-8f f1-9f f2-0f f2-1f f2-2f f2-3f f2-4f f2-5f f2-6f f2-7f f2-8f f2-9f f3-0f f3-1f f3-2f f3-3f f3-4f f3-5f f3-6f f3-7f f3-8f f3-9f f4-0f f4-1f f4-2f f4-3f f4-4f f4-5f f4-6f f4-7f f4-8f f4-9f f5-0f f5-1f f5-2f f5-3f f5-4f f5-5f f5-6f f5-7f f5-8f f5-9f f6-0f f6-1f f6-2f f6-3f f6-4f f6-5f f6-6f f6-7f f6-8f f6-9f f7-0f f7-1f f7-2f f7-3f f7-4f f7-5f f7-6f f7-7f f7-8f f7-9f f8-0f f8-1f f8-2f f8-3f f8-4f f8-5f f8-6f f8-7f f8-8f f8-9f f9-0f f9-1f f9-2f f9-3f f9-4f f9-5f f9-6f f9-7f f9-8f f9-9f - location
    box0 box1 box2 - receptacle
    ball0 - obj
    ball_type - otype
    down left right up - direction
)
(:init
    (at_location robot f5-4f)

    (receptacle_at_location box1 f7-5f)
    (receptacle_at_location box0 f4-4f)
    (receptacle_at_location box2 f1-1f)

    (object_at_location ball0 f5-4f)

    (openable box1)
    (openable box0)
    (openable box2)

    (opened box0)
    (opened box2)
    (opened box1)



    (out_receptacle ball0)

    (checked box0)
    (checked box2)
    (checked box1)

    (object_type ball0 ball_type)

    (holds robot ball0)

    (holdsany robot)



    (empty box2)
    (empty box1)
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

    (clear f4-2f)
    (clear f5-2f)
    (clear f6-2f)
    (clear f4-3f)
    (clear f5-3f)
    (clear f3-4f)
    (clear f5-6f)
    (clear f5-5f)
    (clear f6-6f)
    (clear f6-5f)
    (clear f7-6f)
    (clear f8-6f)
    (clear f8-5f)
    (clear f8-4f)
    (clear f8-3f)
    (clear f8-2f)
    (clear f5-7f)
    (clear f6-7f)
    (clear f7-7f)
    (clear f8-7f)
    (clear f5-8f)
    (clear f6-8f)
    (clear f7-8f)
    (clear f8-8f)
    (clear f3-6f)
    (clear f4-6f)
    (clear f3-7f)
    (clear f4-7f)
    (clear f3-8f)
    (clear f4-8f)
    (clear f6-3f)
    (clear f3-5f)
    (clear f4-5f)
    (clear f2-5f)
    (clear f2-6f)
    (clear f2-7f)
    (clear f2-8f)
    (clear f2-4f)
    (clear f1-4f)
    (clear f1-5f)
    (clear f1-6f)
    (clear f1-7f)
    (clear f1-8f)
    (clear f1-3f)
    (clear f2-3f)
    (clear f2-2f)
    (clear f1-2f)
    (clear f3-3f)
    (clear f3-2f)
    (clear f2-1f)
    (clear f8-1f)
    (clear f3-1f)
    (clear f4-1f)
    (clear f5-1f)
    (clear f6-1f)
    (clear f7-1f)
    (clear f7-2f)
    (clear f7-3f)
    (clear f7-4f)
    (clear f6-4f)

)
(:goal (and
    (object_at_location ball0 f3-8f)
))
)
