uint32[] type_id_list
uint32[] object_id_list
uint32[] object_pos_list
uint32[] object_state_list
uint32 agent_dir


# type id
uint32 EMPTY = 1
uint32 WALL = 2
uint32 BALL = 6
uint32 BOX = 7
uint32 GOAL = 8

# state for box
uint32 CLOSE = 0
uint32 OPEN_NOTHING = 1
uint32 OPEN_CONTAIN = 2
