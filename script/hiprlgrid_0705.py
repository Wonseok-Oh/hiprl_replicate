from gym_minigrid.minigrid import *
from gym_minigrid.register import register
from gym_minigrid.envs.empty import EmptyEnv
import numpy as np
from SpatialMap import SpatialMap, ObjectMap, BinaryMap
import rospy
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.srv import DispatchService
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf import transformations
import time, copy
import roslaunch
from rosparam import upload_params
from yaml import load

from find_frontier.srv import InitPos, InitPosResponse
from hiprl_replicate.msg import Obs
from hiprl_replicate.srv import *
from rosplan_dispatch_msgs.msg import CompletePlan
from rosplan_knowledge_msgs.msg import processReset

#import tf_conversions
#import tf2_ros
from math import pi
#from builtins import None

class HiPRLGridV0(MiniGridEnv):
    """
    Environment similar to kitchen.
    This environment has goals and rewards.
    """

    # Enumeration of possible actions
    class MetaActions(IntEnum):
        # explore, scan, plan
        explore = 0
        scan = 1
        plan = 2
        
        # stop this episode
        stop = 3
    

    
    def __init__(self, grid_size_ = 10, max_steps_ = 100, agent_view_size_ = 5, num_objects=1, num_boxes = 3, process_num = 0):
        self.process_num = process_num
        print(self.process_num)
        data_path = '/home/morin/catkin_ws/src/hiprl_replicate/pddl/'
        domain_path = data_path + 'hiprl_mini.pddl'
        problem0_path = data_path + 'hiprl_problem0.pddl'
        problem_path = data_path + 'problem.pddl'
        planner_command = '/home/morin/catkin_ws/src/rosplan/rosplan_planning_system/common/bin/popf2 DOMAIN PROBLEM /home/morin/catkin_ws/src/hiprl_replicate/pddl/plan0.pddl'
        
        # objmap_to_image converter launch
        file0_package = 'find_frontier'
        file0_executable = 'objmap_to_image_converter'
        node0 = roslaunch.core.Node(file0_package, file0_executable, name = 'objmap_to_image_converter' + str(process_num), args = str(process_num), output='screen')
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        self.process0 = self.launch.launch(node0)
        
        # hiprl_explore launch
        file1_package = 'find_frontier'
        file1_executable = 'find_frontier_node'
        node1 = roslaunch.core.Node(file1_package, file1_executable, name = 'hiprl_explore' + str(process_num), args = str(process_num), output='screen')
        f = open('/home/morin/catkin_ws/src/find_frontier/param/global_costmap' + str(process_num) + '.yaml', 'r')
        yaml_file = load(f)
        f.close()
        upload_params('/hiprl_explore' + str(process_num) + '/', yaml_file)
        self.process1 = self.launch.launch(node1)
        rospy.set_param('use_sim_time', False)
        
        # knowledge manager launch
        file2_package = 'rosplan_knowledge_base'
        file2_executable = 'knowledgeBase'
        node2 = roslaunch.core.Node(file2_package, file2_executable, name = 'rosplan_knowledge_base' + str(process_num), args = str(process_num), output='screen')
        rospy.set_param('rosplan_knowledge_base' + str(process_num) + '/domain_path', domain_path)
        rospy.set_param('rosplan_knowledge_base' + str(process_num) + '/problem_path', problem0_path)
        rospy.set_param('rosplan_knowledge_base' + str(process_num) + '/use_unknowns', False)
        self.process2 = self.launch.launch(node2)
        
        file3_package = 'rosplan_planning_system'
        file3_executable = 'problemInterface'
        node3 = roslaunch.core.Node(file3_package, file3_executable, name = 'rosplan_problem_interface' + str(process_num), args = str(process_num), output='screen')
        rospy.set_param('rosplan_problem_interface' + str(process_num) + '/knowledge_base', 'rosplan_knowledge_base' + str(process_num))
        rospy.set_param('rosplan_problem_interface' + str(process_num) + '/domain_path', domain_path)
        rospy.set_param('rosplan_problem_interface' + str(process_num) + '/problem_path', problem_path)
        rospy.set_param('rosplan_problem_interface' + str(process_num) + '/problem_topic', 'problem_instance')
        self.process3 = self.launch.launch(node3)
        
        file4_package = 'hiprl_replicate'
        file4_executable = 'knowledge_update_node'
        node4 = roslaunch.core.Node(file4_package, file4_executable, name = 'rosplan_knowledge_update_node' + str(process_num), args = str(process_num), output='screen')
        rospy.set_param('rosplan_knowledge_update_node' + str(process_num) + '/knowledge_base', 'rosplan_knowledge_base' + str(process_num))
        self.process4 = self.launch.launch(node4)
        
        # planner launch
        file5_package = 'rosplan_planning_system'
        file5_executable = 'popf_planner_interface'
        node5 = roslaunch.core.Node(file5_package, file5_executable, name = 'rosplan_planner_interface' + str(process_num), args = str(process_num), output='screen')
        rospy.set_param('rosplan_planner_interface' + str(process_num) + '/use_problem_topic', True)
        rospy.set_param('rosplan_planner_interface' + str(process_num) + '/problem_topic', 'rosplan_problem_interface' + str(process_num) + '/problem_instance')
        rospy.set_param('rosplan_planner_interface' + str(process_num) + '/planner_topic', 'planner_output')
        rospy.set_param('rosplan_planner_interface' + str(process_num) + '/domain_path', domain_path)
        rospy.set_param('rosplan_planner_interface' + str(process_num) + '/problem_path', problem_path)
        rospy.set_param('rosplan_planner_interface' + str(process_num) + '/data_path', data_path)
        rospy.set_param('rosplan_planner_interface' + str(process_num) + '/planner_interface', 'popf_planner_interface')
        rospy.set_param('rosplan_planner_interface' + str(process_num) + '/planner_command', planner_command)
        self.process5 = self.launch.launch(node5)
                
        file6_package = 'rosplan_planning_system'
        file6_executable = 'pddl_simple_plan_parser'
        node6 = roslaunch.core.Node(file6_package, file6_executable, name = 'rosplan_parsing_interface' + str(process_num), args = str(process_num), output='screen')
        rospy.set_param('rosplan_parsing_interface' + str(process_num) + '/knowledge_base', 'rosplan_knowledge_base' + str(process_num))
        rospy.set_param('rosplan_parsing_interface' + str(process_num) + '/planner_topic', 'rosplan_planner_interface' + str(process_num) + '/planner_output')
        rospy.set_param('rosplan_parsing_interface' + str(process_num) + '/plan_topic', 'complete_plan')
        self.process6 = self.launch.launch(node6)
        
        print('gym_env' + str(process_num))
        if (process_num == 0):
            self.init_node = rospy.init_node('gym_env' + str(process_num), anonymous=True)
        self.spatial_map_pub = rospy.Publisher("spatial_map" + str(process_num), OccupancyGrid, queue_size = 1, latch=True)
        self.object_map_pub = rospy.Publisher("object_map" + str(process_num), OccupancyGrid, queue_size = 1)
        self.agent_pos_pub = rospy.Publisher("pose" + str(process_num), PoseStamped, queue_size = 1, latch=True)
        self.goal_pos_pub = rospy.Publisher("goal_pose" + str(process_num), PoseStamped, queue_size = 1, latch=True)
        self.agent_init_pos_pub = rospy.Publisher("initial_pose" + str(process_num), PoseStamped, queue_size = 1, latch=True)
        self.navigation_map_pub = rospy.Publisher("navigation_map" + str(process_num), OccupancyGrid, queue_size = 1, latch=True)
        self.explore_action_sub = rospy.Subscriber("action_plan" + str(process_num), Int32MultiArray, self.explore_plan_cb)
        self.observation_pub = rospy.Publisher("observation" + str(process_num), Obs, queue_size = 1)
        self.reset_pub = rospy.Publisher("rosplan_knowledge_base" + str(process_num)+ "/reset", processReset, queue_size = 1, latch=True)
        #self.action_service = rospy.Service("action_execution", ActionExecution, self.execute_action)
        self.complete_plan_sub = rospy.Subscriber("/rosplan_parsing_interface"+ str(process_num) + "/complete_plan" , CompletePlan, self.complete_plan_cb)
        # temporal abstraction: 5 timestep here (param)
        self.temp_abstr_lev = 5
        self.coverage_reward_coeff = 0.002
        self.open_reward_coeff = 0.1
        self.carry_reward_coeff = 0.005
        self.spatial_map = SpatialMap(grid_size_, grid_size_)
        self.object_map = ObjectMap(grid_size_, grid_size_)
        
        self.floor_map_one_hot = BinaryMap(grid_size_, grid_size_)
        self.goal_map_one_hot = BinaryMap(grid_size_, grid_size_)
        self.wall_map_one_hot = BinaryMap(grid_size_, grid_size_)
        self.box_map_one_hot = BinaryMap(grid_size_, grid_size_)
        self.checked_box_map_one_hot = BinaryMap(grid_size_, grid_size_)
        self.ball_map_one_hot = BinaryMap(grid_size_, grid_size_)
        self.unknown_map_one_hot = BinaryMap(grid_size_, grid_size_)
        
        self.prev_agent_pos = None
        self.num_objects = num_objects
        self.meta_actions = HiPRLGridV0.MetaActions
        self.num_boxes = num_boxes
        self.width = grid_size_
        self.height = grid_size_
        self.render_counter = 0
        self.explore_action_plan = []
        self.explore_action_set = False
        self.complete_plan = []
        self.complete_plan_flag = False
        self.goal_pos = None
        #self.planner_action_set = False
        #self.planner_action_plan = []
        self.planner_action_num = 0
        #self.dispatch_plan_end = True
        #self.action_execution_flag = False

        #self.br = tf2_ros.TransformBroadcaster()
        
        
        obs = super().__init__(grid_size = grid_size_, max_steps= max_steps_, agent_view_size = agent_view_size_)
        #print(obs)
        #self.update_maps(obs, None)
        #print("agent_pos: %d, %d" %(self.agent_pos[0], self.agent_pos[1]))
        #print("agent_dir: %d" % self.agent_dir)
        #print("map_size: %d x %d" %(self.spatial_map.map.info.width, self.spatial_map.map.info.height))
#        for i in range(5):
#            self.spatial_map_pub.publish(self.spatial_map.map)
#            self.object_map_pub.publish(self.object_map.map)
#            self.spatial_map.rate.sleep()

    def __del__(self):
        self.process0.stop()
        self.process1.stop()
        self.process2.stop()
        self.process3.stop()
        self.process4.stop()
        self.process5.stop()
        self.process6.stop()
        self.launch.stop()
        rospy.signal_shutdown("End class")

    def complete_plan_cb(self, msg):
        self.complete_plan = msg.plan
        self.complete_plan_flag = True
        self.dispatch_plan_end = False

    def generate_actions_from_complete_plan(self):
        actions = []
        self.planner_tracking_dir = self.agent_dir
        for item in self.complete_plan:
            item_actions = []
            dir_key_value = item.parameters[-1]
        
            # the last parameter should set key as dir
            assert(dir_key_value.key == 'dir')
        
            # make robot to face given direction
            if dir_key_value.value == 'left':
                goal_dir = 2
            elif dir_key_value.value == 'up':
                goal_dir = 3
            elif dir_key_value.value == 'right':
                goal_dir = 0
            elif dir_key_value.value == 'down':
                goal_dir = 1

            dth = self.planner_tracking_dir - goal_dir
            if dth < 0:
                dth += 4
            if dth == 3:
                item_actions.append(self.Actions.right)
            
            else:
                for i in range(dth):
                    item_actions.append(self.Actions.left)
            
            self.planner_tracking_dir = goal_dir
            if item.name == "move-robot":
                item_actions.append(self.Actions.forward)
                
    
            elif item.name == "openobject":
                item_actions.append(self.Actions.open)
                
            elif item.name == "pickupobjectinreceptacle" or item.name == "pickupobject":
                item_actions.append(self.Actions.pickup)
                
            elif item.name == "putobjectinreceptacle" or item.name == "putobject":
                item_actions.append(self.Actions.drop)
                
            elif item.name == "closeobject":
                item_actions.append(self.Actions.close)
            
            actions.append((item, item_actions))
        return actions
        
    def explore_plan_cb(self, msg):
        self.explore_action_plan  = [0] * len(msg.data)
        for i in range(len(msg.data)):
            self.explore_action_plan[i] = msg.data[i]
        self.explore_action_set = True
        print("Explore_plan_cb called")
        
        # generate actions from the index_plan

    def _gen_grid(self, width, height):
        # Create the grid
        self.grid = Grid(width, height)

        # Generate the surrounding walls
        self.grid.horz_wall(0, 0)
        self.grid.horz_wall(0, height-1)
        self.grid.vert_wall(0, 0)
        self.grid.vert_wall(width-1, 0)

        # Types and colors of objects we can generate
        types = ['key', 'ball']

        objs = []
        objPos = []
        boxes = []
        boxPos = []

        def near_obj(env, p1):
            for p2 in objPos:
                dx = p1[0] - p2[0]
                dy = p1[1] - p2[1]
                if abs(dx) <= 1 and abs(dy) <= 1:
                    return True
            return False
        # initially, put a box containing a ball
        boxColor = self._rand_elem(COLOR_NAMES)
        objColor = self._rand_elem(COLOR_NAMES)
        obj = Box(len(boxes), boxColor, Ball(len(objs), objColor) )
        boxes.append(boxColor)
        objs.append(('ball', objColor))
        pos = self.place_obj(obj, reject_fn=near_obj)
        objPos.append(pos)
        boxPos.append(pos)
        # Until we have generated all the objects
        while len(objs) < self.num_objects:
            objType = self._rand_elem(types)
            objColor = self._rand_elem(COLOR_NAMES)

            # If this object already exists, try again
            if (objType, objColor) in objs:
                continue

            if objType == 'key':
                obj = Key(len(objs), objColor)
            elif objType == 'ball':
                obj = Ball(len(objs), objColor)
                
            pos = self.place_obj(obj, reject_fn=near_obj)

            objs.append((objType, objColor))
            objPos.append(pos)

        while len(boxes) < self.num_boxes:
            boxColor = self._rand_elem(COLOR_NAMES)
            
            # If this object already exists, try again
            if boxColor in boxes:
                continue
            
            box = Box(len(boxes), boxColor)
            #print("box.isOpen: %d" % box.isOpen)
            pos = self.place_obj(box, reject_fn=near_obj)
            boxes.append(boxColor)
            boxPos.append(pos)
        
        # place a goal    
        obj = Goal()
        objColor = self._rand_elem(COLOR_NAMES)
        self.goal_pos = self.place_obj(obj, reject_fn = near_obj)
        
        # publish the goal position to update the knowledge base
        self.publish_ros_goal_pos(self.goal_pos)
        
        # Randomize the agent start position and orientation
        self.place_agent()

        # Choose a random object to be moved
        self.objIdx = self._rand_int(0, len(objs))
        self.move_type, self.moveColor = objs[self.objIdx]
        self.move_pos = objPos[self.objIdx]

        # Choose a target object (to put the first object into)
        self.target_pos = self.goal_pos

        self.mission = 'put the %s %s into the goal' % (
            self.moveColor,
            self.move_type,
        )
        
    def reset(self):
        #print("reset is called")
        self.opened_receptacles = set()
        self.closed_receptacles = set()
        self.seen_obj = set()
        self.seen_box = set()
        self.checked_receptacles = set()
        self.visited_locations = set()
        self.can_end = False
        self.object_map = ObjectMap(self.width, self.height)
        self.spatial_map = SpatialMap(self.width, self.height)
        
        self.floor_map_one_hot = BinaryMap(self.width, self.height)
        self.goal_map_one_hot = BinaryMap(self.width, self.height)
        self.wall_map_one_hot = BinaryMap(self.width, self.height)
        self.box_map_one_hot = BinaryMap(self.width, self.height)
        self.checked_box_map_one_hot = BinaryMap(self.width, self.height)
        self.ball_map_one_hot = BinaryMap(self.width, self.height)
        self.unknown_map_one_hot = BinaryMap(self.width, self.height)

        
        self.new_coverage = 0
        self.prev_agent_pos = None
        self.render_counter = 0
        self.explore_action_plan = []
        self.explore_action_set = False
        #self.planner_action_set = False
        #self.planner_action_plan = []
        self.planner_action_num = 0
        #self.dispatch_plan_end = True
        self.action_execution_flag = False
        self.complete_plan = []
        self.complete_plan_flag = False
        reset_msg = processReset()
        reset_msg.domain_path = "/home/morin/catkin_ws/src/hiprl_replicate/pddl/hiprl_mini.pddl"
        reset_msg.problem_path = "/home/morin/catkin_ws/src/hiprl_replicate/pddl/hiprl_problem0.pddl"
        self.reset_pub.publish(reset_msg)
        obs = super().reset()
        self.update_maps(obs, None)
        self.spatial_map_pub.publish(self.spatial_map.map)
        self.object_map_pub.publish(self.object_map.map)
        self.agent_init_pos = self.publish_ros_agent_pos()
        self.agent_init_dir = self.agent_dir
        self.agent_init_pos_pub.publish(self.agent_init_pos)
        self.publish_observation(obs['image'])
        return self.generate_network_input_one_hot(obs)
        #return np.reshape(self.object_map.map.data, (self.width*self.object_map.factor, self.height*self.object_map.factor))
    
    def step(self, action):
        
        preCarrying = self.carrying
        #print("agent_pos: %d, %d" %(self.agent_pos[0], self.agent_pos[1]))
        #print("agent_dir: %d" % self.agent_dir)
        obs, reward, done, info = MiniGridEnv.step(self, action)
        
        reward = -0.01 # constant time penalty

        # update spatial map & object map
        # self.update_maps(obs, action)
        #self.spatial_map_pub.publish(self.spatial_map.map)
        #self.object_map_pub.publish(self.object_map.map)
        #self.spatial_map.rate.sleep()
        self.update_maps(obs, action)
        self.spatial_map_pub.publish(self.spatial_map.map)
        self.object_map_pub.publish(self.object_map.map)
        self.publish_ros_agent_pos()
        #self.broadcast_tf()

        #self.spatial_map.rate.sleep()
        
        # publish observation information to update knowledgebase
        self.publish_observation(obs['image'])
        #print(obs['image'])
        
        # reward for open/close action
        if info is not None:
            if info.can_toggle() and action == self.actions.open:
                if info.objectId in self.closed_receptacles: # if the receptacle was closed
                    self.opened_receptacles.add(info.objectId) # add to the opened_receptacles list
                    self.closed_receptacles.discard(info.objectId)
                    if info.objectId in self.checked_receptacles: # if the receptacle was checked before, penalize
                        reward += -1.0 * self.open_reward_coeff
                    else:                                    # else, if it was not checked, give reward
                        self.checked_receptacles.add(info.objectId)
                        reward += 1.0 * self.open_reward_coeff
                
                elif info.objectId in self.opened_receptacles: # penalty for open opened_receptacles
                    reward += -1.0 * self.open_reward_coeff
            elif info.can_toggle() and action == self.actions.close:            
                if info.objectId in self.opened_receptacles: # if the receptacle was opened
                    self.closed_receptacles.add(info.objectId) # add to the closed_receptacles list
                    self.opened_receptacles.discard(info.objectId)
                    
                elif info.objectId in self.closed_receptacles:
                    reward += -1.0 * self.open_reward_coeff # penalty for close closed_receptacles    
        
        
        # reward(penalize) for carrying (non)target object or 
        # just finish the episode
        if self.carrying != None:
            if self.objIdx == self.carrying.objectId:
                reward += 1.0 * self.carry_reward_coeff
            else:
                reward += -1.0 * self.carry_reward_coeff

        # If successfully dropping an object into the target
        u, v = self.dir_vec
        ox, oy = (self.agent_pos[0] + u, self.agent_pos[1] + v)
        tx, ty = self.target_pos
        if action == self.actions.drop and preCarrying:
            front_obj = self.grid.get(ox,oy)
            if front_obj.type is 'goal':
                if abs(ox - tx) == 0 and abs(oy - ty) == 0:
                    done = True
                    reward += 0.5
        
        # coverage reward
        reward += self.new_coverage * self.coverage_reward_coeff
        
        # if step num exceed 200, done
 
        self.prev_agent_pos = self.agent_pos
        self.prev_agent_dir = self.agent_dir
        self.new_coverage = 0
        #print("obs: ")
        #print(obs)
        
        #print(self.opened_receptacles)
        #print(self.closed_receptacles)
        #print(self.checked_receptacles)

        return obs, reward, done, info

    def publish_observation(self, image):
        ros_msg = Obs()
    
        for i in range(self.agent_view_size):
            for j in range(self.agent_view_size):
#                print("{}'th column, {}'th row".format(i,j))
                abs_i, abs_j = self.get_world_coordinate(i, j)
#                print("{}, {}".format(abs_i, abs_j))
                if image[i][j][0] == OBJECT_TO_IDX['wall']:
                    ros_msg.type_id_list.append(image[i][j][0])
                    ros_msg.object_id_list.append(0)
                    ros_msg.object_pos_list.append(abs_i)
                    ros_msg.object_pos_list.append(abs_j)
                    ros_msg.object_state_list.append(0)
                
                elif image[i][j][0] == OBJECT_TO_IDX['box']:
                    ros_msg.type_id_list.append(image[i][j][0])
                    ros_msg.object_id_list.append(image[i][j][3])
                    ros_msg.object_pos_list.append(abs_i)
                    ros_msg.object_pos_list.append(abs_j)
                    ros_msg.object_state_list.append(image[i][j][2])
                    
                elif image[i][j][0] == OBJECT_TO_IDX['empty']:
                    ros_msg.type_id_list.append(image[i][j][0])
                    ros_msg.object_id_list.append(0)
                    ros_msg.object_pos_list.append(abs_i)
                    ros_msg.object_pos_list.append(abs_j)
                    ros_msg.object_state_list.append(0)
                    
                elif image[i][j][0] == OBJECT_TO_IDX['ball']:
                    ros_msg.type_id_list.append(image[i][j][0])
                    ros_msg.object_id_list.append(image[i][j][3])
                    ros_msg.object_pos_list.append(abs_i)
                    ros_msg.object_pos_list.append(abs_j)
                    ros_msg.object_state_list.append(image[i][j][2])

                elif image[i][j][0] == OBJECT_TO_IDX['goal']:
                    ros_msg.type_id_list.append(image[i][j][0])
                    ros_msg.object_id_list.append(0)
                    ros_msg.object_pos_list.append(abs_i)
                    ros_msg.object_pos_list.append(abs_j)
                    ros_msg.object_state_list.append(0)

        
        ros_msg.agent_dir = self.agent_dir
        self.observation_pub.publish(ros_msg)        
        
        
    def invoke(self, meta_action):
        reward_sum = 0
        if meta_action == self.meta_actions.scan:
            for i in range(4):
                obs, reward, done, info = self.step(self.actions.left)
                print('step=%s, reward=%.2f' % (self.step_count, reward))
                reward_sum += reward
                if done:
                    print('done!')
                    break
                #else:
                    #redraw(obs)
                    #time.sleep(0.1)
            
            if reward_sum == -0.04:
                reward_sum += -0.5
                done = True
                
        if meta_action == self.meta_actions.explore:
            actions = self.explore()
    
            for i in range(len(actions)):
                obs, reward, done, info = self.step(actions[i])
                print('step=%s, reward=%.2f' % (self.step_count, reward))
                reward_sum += reward
                if done:
                    print('done!')
                    break
                #else:
                    #redraw(obs)
                    #time.sleep(0.5)
            
            if len(actions) <= 1 or reward_sum == -0.01:
                obs = self.gen_obs()
                reward_sum += -0.5
                done = True
                
        if meta_action == self.meta_actions.plan:
            actions = self.plan()
    
            if actions == None or len(actions) == 0:
                obs, reward, done, info = self.step(self.Actions.pickup)
                done = True
                reward_sum += -0.5
                print('step=%s, reward=%.2f' % (self.step_count, reward))
                reward_sum += reward
                if done:
                    print('done!')
                #else:
                    #redraw(obs)
                    #time.sleep(0.5)
            
            else:
                len_actions = 1
                while (len_actions != 0):
                    obs, reward, done, info = self.dispatch_plan(actions)
                    len_actions = 0
                    if obs == None: # invalid plan
                        obs = self.gen_obs()
                        reward= 0
                        done = False
                        info = None
                        break
                    for i in range(len(actions)):
                        len_actions = len_actions + len(actions[i][1])
                    print('step=%s, reward=%.2f' % (self.step_count, reward))
                    reward_sum += reward
                    if done:
                        print('done!')
                        break
                    #else:
                        #redraw(obs)
                        #time.sleep(0.5)
                
            #process.stop()
        obs = self.gen_obs()
        # map = self.generate_network_input(obs)
        map = self.generate_network_input_one_hot(obs)
        #print("map: ")
        #print(map)
        #print('%s, Overall reward=%.2f' % (meta_action, reward_sum))
        #print("{}, {}, {}, {}, {}".format(type(obs), type(reward_sum), type(done), type(info), type(map)))
        return obs, reward_sum, done, info, map

    def generate_network_input_one_hot(self, obs):
        temp_ball_map = copy.deepcopy(self.ball_map_one_hot)
        map = np.zeros(shape=(7, 10, 10), dtype=np.uint8)

        obs_grid, _ = Grid.decode(obs['image'])
        object = obs_grid.get(obs_grid.width//2, obs_grid.height-1)
        wx, wy = self.get_world_coordinate(obs_grid.width//2, obs_grid.height-1)
        if np.array_equal(self.agent_pos, np.array([wx,wy])):
            wy = self.spatial_map.map.info.height - wy - 1
            #self.spatial_map.update_cell(np.array([wx,wy]), SpatialMap.OccGridStates.free
            if object is not None and object.type == 'ball':
                temp_ball_map.update_cell(np.array([wx, wy]), 1)
        map[0,:] = copy.deepcopy(np.reshape(temp_ball_map.map.data, (self.width, self.height)))        
        map[1,:] = copy.deepcopy(np.reshape(self.box_map_one_hot.map.data, (self.width, self.height)))
        map[2,:] = copy.deepcopy(np.reshape(self.checked_box_map_one_hot.map.data, (self.width, self.height)))
        map[3,:] = copy.deepcopy(np.reshape(self.floor_map_one_hot.map.data, (self.width, self.height)))
        map[4,:] = copy.deepcopy(np.reshape(self.goal_map_one_hot.map.data, (self.width, self.height)))
        map[5,:] = copy.deepcopy(np.reshape(self.unknown_map_one_hot.map.data, (self.width, self.height)))
        map[6,:] = copy.deepcopy(np.reshape(self.wall_map_one_hot.map.data, (self.width, self.height)))
        return map
        
    def generate_network_input(self, obs):
        map = copy.deepcopy(self.object_map)
        #map = np.reshape(self.object_map.map.data, (self.width*self.object_map.factor, self.height*self.object_map.factor))
        obs_grid, _ = Grid.decode(obs['image'])
        object = obs_grid.get(obs_grid.width//2, obs_grid.height-1)
        wx, wy = self.get_world_coordinate(obs_grid.width//2, obs_grid.height-1)
        if np.array_equal(self.agent_pos, np.array([wx,wy])):
            wy = self.spatial_map.map.info.height - wy - 1
            #self.spatial_map.update_cell(np.array([wx,wy]), SpatialMap.OccGridStates.free
            if object is not None and object.type == 'ball':
                map.update_cell(np.array([wx, wy]), ObjectMap.ObjGridStates.ball, center_only = True)

        return np.reshape(map.map.data, (self.width*self.object_map.factor, self.height*self.object_map.factor))
    
    def render(self, mode = 'human', close = False, highlight = True, tile_size = TILE_PIXELS):
        img = super().render(mode, close, highlight, tile_size)
        obs = self.gen_obs()
        if self.render_counter == 0:
            self.update_maps(obs, None)
            self.spatial_map_pub.publish(self.spatial_map.map)
            self.object_map_pub.publish(self.object_map.map)
            self.agent_init_pos = self.publish_ros_agent_pos()
            self.agent_init_dir = self.agent_dir
            self.agent_init_pos_pub.publish(self.agent_init_pos)
            self.publish_observation(obs['image'])
            #self.broadcast_tf()
        self.render_counter += 1
        return img
    '''
    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = (self.agent_pos[0] + 0.5) * self.spatial_map.map.info.resolution
        t.transform.translation.y = (self.agent_pos[1] + 0.5) * self.spatial_map.map.info.resolution
        t.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.agent_dir*pi/2 + pi/2)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
    '''    
    def publish_ros_agent_pos(self):
        ros_agent_pos = PoseStamped()
        ros_agent_pos.header.frame_id = "map"
        ros_agent_pos.header.stamp = rospy.Time.now()
        print("self.agent_pos: %d, %d" % (self.agent_pos[0], self.agent_pos[1]) )
        ros_agent_pos.pose.position.x = (self.agent_pos[0]+0.5) * self.spatial_map.map.info.resolution 
        ros_agent_pos.pose.position.y = (self.spatial_map.map.info.height - self.agent_pos[1] - 1 + 0.5) * self.spatial_map.map.info.resolution
        ros_agent_pos.pose.position.z = 0
        orientation = self.dir_to_quaternion()
        ros_agent_pos.pose.orientation.x = orientation[0]
        ros_agent_pos.pose.orientation.y = orientation[1]
        ros_agent_pos.pose.orientation.z = orientation[2]
        ros_agent_pos.pose.orientation.w = orientation[3]
        self.agent_pos_pub.publish(ros_agent_pos)
        return ros_agent_pos

    def publish_ros_goal_pos(self, pos):
        goal_pos = PoseStamped()
        goal_pos.header.frame_id = "map"
        goal_pos.header.stamp = rospy.Time.now()
        goal_pos.pose.position.x = pos[0]
        goal_pos.pose.position.y = pos[1]
        self.goal_pos_pub.publish(goal_pos)
        return goal_pos

    def dir_to_quaternion(self):
        dir = self.agent_dir
        ##########
        # 0: >
        # 1: V
        # 2: <
        # 3: ^
        ##########
        if dir == 0:
            yaw = 0
        elif dir == 1:
            yaw = -pi/2
        elif dir == 2:
            yaw = pi
        elif dir == 3:
            yaw = pi/2
        return transformations.quaternion_from_euler(0, 0, yaw)      
    
    # now, spatial map update algorithm is done    
    def update_maps(self, obs, action):
        #print(obs['image'])
        #print(self.agent_dir)
        if np.array_equal(self.prev_agent_pos, self.agent_pos) and np.array_equal(self.prev_agent_dir, self.agent_dir): # no movement
            if action == self.actions.open or action == self.actions.pickup or action == self.actions.drop:
                front = self.front_pos
                fwd_cell = self.grid.get(front[0], front[1])

                front[1] = self.spatial_map.map.info.height - front[1] - 1
                if fwd_cell is None: # empty cell or out of range
                    if 0 <= front[0] and front[0] <= self.spatial_map.map.info.width-1 and 0 <= front[1] and front[1] <= self.spatial_map.map.info.height-1:
                        self.spatial_map.update_cell(front, SpatialMap.OccGridStates.free)
                        self.object_map.update_cell(front, ObjectMap.ObjGridStates.floor)
                        self.floor_map_one_hot.update_cell(front, 1)
                else:
                    
                    # update object map
                    if fwd_cell.type == 'box':
                        print("fwd_cell.objectId: %d" % fwd_cell.objectId)
                        if fwd_cell.isOpen == True:
                            if fwd_cell.contains is not None:
                                if fwd_cell.contains.type == 'ball':
                                    self.object_map.update_cell(front, ObjectMap.ObjGridStates.ball, center_only = True)
                                    self.ball_map_one_hot.update_cell(front, 1)
                                elif fwd_cell.contains.type == 'key':
                                    self.object_map.update_cell(front, ObjectMap.ObjGridStates.key, center_only = True)
                            else:
                                self.object_map.update_cell(front, ObjectMap.ObjGridStates.checked_box)
                                self.checked_box_map_one_hot.update_cell(front, 1)
                        
                        elif fwd_cell.objectId not in self.checked_receptacles:
                            self.object_map.update_cell(front, ObjectMap.ObjGridStates.box)
                            self.box_map_one_hot.update_cell(front, 1)
                    
                    elif fwd_cell.type == 'key':
                        self.object_map.update_cell(front, ObjectMap.ObjGridStates.key, center_only = True)
                        
                    elif fwd_cell.type == 'ball':
                        self.object_map.update_cell(front, ObjectMap.ObjGridStates.ball, center_only = True)
                        self.ball_map_one_hot.update_cell(front, 1)
                    elif fwd_cell.type == 'wall':
                        self.object_map.update_cell(front, ObjectMap.ObjGridStates.wall)
                        self.wall_map_one_hot.update_cell(front, 1)
                    elif fwd_cell.type == 'goal':
                        self.object_map.update_cell(front, ObjectMap.ObjGridStates.goal)
                        self.goal_map_one_hot.update_cell(front, 1)
                    else:
                        print("update_maps: this should not happen. new type")
                
                    # update spatial map
                    if fwd_cell.can_overlap():
                        self.spatial_map.update_cell(front, SpatialMap.OccGridStates.free)
                    else:
                        self.spatial_map.update_cell(front, SpatialMap.OccGridStates.occupied)
    
                        
            else:
                return
        else:
            # for all the cells in the agent's view, update the cells info into the map unless it is not chekced receptacles
            obs_grid, _ = Grid.decode(obs['image'])
            for i in range(obs_grid.width):
                for j in range(obs_grid.height):
                    object = obs_grid.get(i,j)
                    wx, wy = self.get_world_coordinate(i,j)
                    if np.array_equal(self.agent_pos, np.array([wx,wy])):
                        
                        wy = self.spatial_map.map.info.height - wy - 1
                        self.spatial_map.update_cell(np.array([wx,wy]), SpatialMap.OccGridStates.free)
                        continue
                    wy = self.spatial_map.map.info.height - wy - 1 

                    if object is None: # empty cell or out of range
                        if 0 <= wx and wx <= self.spatial_map.map.info.width-1 and 0 <= wy and wy <= self.spatial_map.map.info.height-1:
                            self.spatial_map.update_cell(np.array([wx,wy]), SpatialMap.OccGridStates.free)
                            self.object_map.update_cell(np.array([wx,wy]), ObjectMap.ObjGridStates.floor)
                            self.floor_map_one_hot.update_cell(np.array([wx,wy]), 1)
                    else:
                        # update object map
                        if object.type == 'box':
                            if object.objectId in self.checked_receptacles:
                                pass
                            
                            elif object.isOpen == True:
                                self.opened_receptacles.add(object.objectId)
                                if object.contains is not None:
                                    if object.contains.type == 'ball':
                                        self.object_map.update_cell(np.array([wx,wy]), ObjectMap.ObjGridStates.ball, center_only = True)
                                        self.ball_map_one_hot.update_cell(np.array[wx,wy], 1)
                                    elif object.contains.type == 'key':
                                        self.object_map.update_cell(np.array([wx,wy]), ObjectMap.ObjGridStates.key, center_only = True)
                                else:
                                    self.object_map.update_cell(np.array([wx,wy]), ObjectMap.ObjGridStates.box)
                                    self.box_map_one_hot.update_cell(np.array([wx,wy]), 1)
                            else:
                                self.closed_receptacles.add(object.objectId)
                                self.object_map.update_cell(np.array([wx, wy]), ObjectMap.ObjGridStates.box)
                                self.box_map_one_hot.update_cell(np.array([wx,wy]), 1)
                            
                        elif object.type == 'key':
                            self.object_map.update_cell(np.array([wx,wy]), ObjectMap.ObjGridStates.key, center_only = True)
                            
                        elif object.type == 'ball':
                            self.object_map.update_cell(np.array([wx,wy]), ObjectMap.ObjGridStates.ball, center_only = True)
                            self.ball_map_one_hot.update_cell(np.array([wx,wy]), 1)
                        elif object.type == 'wall':
                            self.object_map.update_cell(np.array([wx, wy]), ObjectMap.ObjGridStates.wall)
                            self.wall_map_one_hot.update_cell(np.array([wx,wy]), 1)
                        elif object.type == 'goal':
                            self.object_map.update_cell(np.array([wx, wy]), ObjectMap.ObjGridStates.goal)
                            self.goal_map_one_hot.update_cell(np.array([wx,wy]), 1)
                        else:
                            print("update_maps: this should not happen. new type")
                            
                        
                        index = self.spatial_map.xy_to_index(wx, wy)
                        if self.spatial_map.map.data[index] == SpatialMap.OccGridStates.unknown:
                            self.new_coverage += 1

                        # update spatial map
                        if object.can_overlap():
                            self.spatial_map.update_cell(np.array([wx,wy]), SpatialMap.OccGridStates.free)
                        else:
                            self.spatial_map.update_cell(np.array([wx,wy]), SpatialMap.OccGridStates.occupied)

                
        self.spatial_map.map.header.stamp = rospy.Time.now()
        self.object_map.map.header.stamp = rospy.Time.now()                   

    def scan(self):
        self.step(self.Actions.left)
        self.step(self.Actions.left)
        self.step(self.Actions.left)
        self.step(self.Actions.left)

    def explore(self):
        rospy.wait_for_service('init_pose_update' + str(self.process_num))
        result = False
        try:
            init_pose_update = rospy.ServiceProxy('init_pose_update' + str(self.process_num), InitPos)
            result = init_pose_update(self.agent_init_pos.pose.position.x, self.agent_init_pos.pose.position.y, self.agent_init_dir)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
        print("init_pose_update: ", result)

        try:
            pose_update = rospy.ServiceProxy('pose_update' + str(self.process_num), InitPos)
            x = (self.agent_pos[0] + 0.5) * self.spatial_map.map.info.resolution
            y = (self.spatial_map.map.info.height - self.agent_pos[1] - 0.5) * self.spatial_map.map.info.resolution
            result = pose_update(x, y, self.agent_dir)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
        
        print("pose_update: ", result)       
    
        # Execute mapConvert in findFrontier, which generates plan
        self.navigation_map_pub.publish(self.spatial_map.map)
        counter = 0
        while(self.explore_action_set is False):
            print("explore_action_set: ", self.explore_action_set)
            counter = counter + 1
            time.sleep(0.1)
            if counter > 10:
                return []
            #block until explore plan is subscribed and updated
        print("plan: ", self.explore_action_plan)
        self.explore_action_set = False
        
        # step using actions extracted from explore_action_plan
        # temporally, set left
        #self.explore_action_plan = [self.Actions.left] * 4
        return self.explore_action_plan
    
    def plan(self):
        print("Generating a Problem")
        rospy.wait_for_service('/rosplan_problem_interface'+ str(self.process_num) + '/problem_generation_server')
        try:
            problem_generation = rospy.ServiceProxy('/rosplan_problem_interface' + str(self.process_num) + '/problem_generation_server', Empty)
            resp = problem_generation()
        except rospy.ServiceException as e:
            print("Problem Generation Service call failed: %s" %e)
            return None

        print("Planning")            
        rospy.wait_for_service('/rosplan_planner_interface' + str(self.process_num) + '/planning_server')
        try:
            run_planner = rospy.ServiceProxy('/rosplan_planner_interface' + str(self.process_num) + '/planning_server', Empty)
            resp = run_planner()

        except rospy.ServiceException as e:
            print("Planning Service call failed: %s" %e)
            return None

        print("Executing the Plan")                        
        rospy.wait_for_service('/rosplan_parsing_interface' + str(self.process_num) + '/parse_plan')
        try:
            parse_plan = rospy.ServiceProxy('/rosplan_parsing_interface' + str(self.process_num) + '/parse_plan', Empty)
            resp = parse_plan()
        except rospy.ServiceException as e:
            print("Plan Parsing Service call failed: %s" %e)
            return None
        #rospy.wait_for_service('/rosplan_parsing_interface/alert_plan_action_num')
        #try:
        #    alert_plan_action_num = rospy.ServiceProxy('/rosplan_parsing_interface/alert_plan_action_num', AlertPlanActionNum)
        #    resp = alert_plan_action_num()
        #    self.planner_action_num = resp.num_actions
        #except rospy.ServiceException as e:
        #    print("Plan Parsing Service call failed: %s" %e)
        counter = 0
        while(self.complete_plan_flag is False):
            time.sleep(0.1)
            counter = counter + 1
            print("complete_plan_flag: ", self.action_execution_flag)
            if counter > 10:
                self.dispatch_plan_action_id = 0
                self.prev_dispatch_plan_action_id = -1
                return None
        print("complete_plan_flag is set to True")
        
        actions = self.generate_actions_from_complete_plan()
        self.dispatch_plan_action_id = 0
        self.prev_dispatch_plan_action_id = -1
        self.complete_plan_flag = False
        return actions
    
    def dispatch_plan(self, actions):
        if self.dispatch_plan_action_id - self.prev_dispatch_plan_action_id > 0 and len(actions) > 0:
            precondition_check = self.check_precondition(actions[self.dispatch_plan_action_id][0])
            if precondition_check == False:
                print("dispatch_plan: {} precondition not achieved".format(actions[self.dispatch_plan_action_id][0].name))
                self.dispatch_plan_action_id = 0
                self.prev_dispatch_plan_action_id = -1
                return None, None, None, None
            
        action = actions[self.dispatch_plan_action_id][1].pop(0)
        #print(actions[self.dispatch_plan_action_id][0])
        
        obs, reward, done, info = self.step(action)
        self.prev_dispatch_plan_action_id = self.dispatch_plan_action_id        

        # if actions for semantic action is done (= actions[dispatch_plan_action_id][1] is empty)
        if not actions[self.dispatch_plan_action_id][1]:
            self.process_action_effect(actions[self.dispatch_plan_action_id][0])
            self.dispatch_plan_action_id += 1

        return obs, reward, done, info
    
    def check_precondition(self, item):
        rospy.wait_for_service('/check_precondition' + str(self.process_num))
        try:
            check_precondition = rospy.ServiceProxy('/check_precondition' + str(self.process_num), ActionExecution)
            req = ActionExecutionRequest()
            req.name = item.name
            req.action_id = item.action_id
            req.parameters = copy.deepcopy(item.parameters)
            resp = check_precondition(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Check Precondition Service call failed: %s" %e)
            return False
    
    def process_action_effect(self, item):
        rospy.wait_for_service('/process_action_effect' + str(self.process_num))
        try:
            check_precondition = rospy.ServiceProxy('/process_action_effect' + str(self.process_num), ActionExecution)
            req = ActionExecutionRequest()
            req.name = item.name
            req.action_id = item.action_id
            req.parameters = copy.deepcopy(item.parameters)
            resp = check_precondition(req)
            return resp.success
        except rospy.ServiceException as e:
            print("process_action_effect service call failed: %s" %e)
            return False   
register(
    id='MiniGrid-HiPRLGrid-v0',
    entry_point='gym_minigrid.envs:HiPRLGridV0'
)

""" For advanced sim
        # Place random objects in the world
        types1 = ['cup', 'apple', 'egg']
        types2 = ['cabinet', 'table', 'refrigerator', 'microwave', 'sink']
        objColor = self._rand_elem(COLOR_NAMES)
        tableObj = Table(objColor)
        self.place_obj(tableObj)
        
        objColor = self._rand_elem(COLOR_NAMES)
        refrigeratorObj = Refrigerator(objColor)
        self.place_obj(refrigeratorObj)
        
        objColor = self._rand_elem(COLOR_NAMES)
        microwaveObj = Microwave(objColor)
        self.place_obj(microwaveObj)
        
        objColor = self._rand_elem(COLOR_NAMES)
        sinkObj = Sink(objColor)
        self.place_obj(sinkObj)
        
        objColor = self._rand_elem(COLOR_NAMES)
        cabinetObj = Cabinet(objColor)
        self.place_obj(cabinetObj)
            
        for i in range(0, 5):
            objType = self._rand_elem(types1)
            objColor = self._rand_elem(COLOR_NAMES)
            if objType == 'cup':
                obj = Cup(i, objColor)
            elif objType == 'apple':
                obj = Apple(i, objColor)
            elif objType == 'egg':
                obj = Egg(i, objColor)         
            self.place_obj(obj)

        # No explicit mission in this environment --> to be modified
        self.mission = '' """