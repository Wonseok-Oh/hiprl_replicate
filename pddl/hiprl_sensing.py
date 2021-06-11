from geometry_msgs.msg import PoseStamped
resolution = 3.0
map_grid_size = 10

def at_location(msg, params):
    ret_value = []
    # Get current values of robot at to find current robot location
    attributes = get_kb_attribute("at_location")
    for a in attributes:
        if not a.is_negative:
            prev_location = a.values[1].value
            break
    
    for robot in params[0]:
        print('robot: {}'.format(robot))
        x = int((msg.pose.position.x - resolution / 2.0) / 3)
        y = int(map_grid_size - (msg.pose.position.y + resolution / 2.0) / 3)
        print('x,y: {},{}'.format(x, y))
        curr_location = 'f' + str(x) + '-' + str(y) + 'f'
        print_str = 'prev_location: {prev}, curr_location: {curr}'
        print(print_str.format(prev = prev_location, curr = curr_location))
        if prev_location != curr_location:
            ret_value.append((robot + ':' + prev_location, False))
            ret_value.append((robot + ':' + curr_location, True))
    
    return ret_value