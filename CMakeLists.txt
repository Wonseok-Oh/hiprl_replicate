cmake_minimum_required(VERSION 3.0.2)
project(hiprl_replicate)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rosplan_planning_system
  nav_msgs
  roscpp
  std_msgs
  diagnostic_msgs
  message_generation
)

add_message_files(
   FILES
   Obs.msg
   Obs_multi.msg
)

add_service_files(
   FILES
   ActionExecution.srv
   AlertPlanActionNum.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   diagnostic_msgs   
)

catkin_package(
   CATKIN_DEPENDS nav_msgs roscpp message_runtime
   EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(knowledge_updater
   src/knowledge_updater.cpp
)
add_dependencies(knowledge_updater ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knowledge_updater ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_executable(knowledge_update_node src/knowledge_update_node.cpp)
add_dependencies(knowledge_update_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knowledge_update_node knowledge_updater
   ${catkin_LIBRARIES}
)

add_library(knowledge_updater_multi
   src/knowledge_updater_multi.cpp
)
add_dependencies(knowledge_updater_multi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knowledge_updater_multi ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_executable(knowledge_update_multi_node src/knowledge_update_multi_node.cpp)
add_dependencies(knowledge_update_multi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knowledge_update_multi_node knowledge_updater_multi
   ${catkin_LIBRARIES}
)

add_library(knowledge_updater_multi_individual
   src/knowledge_updater_multi_individual.cpp
)
add_dependencies(knowledge_updater_multi_individual ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knowledge_updater_multi_individual ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_executable(knowledge_update_multi_individual_node src/knowledge_update_multi_individual_node.cpp)
add_dependencies(knowledge_update_multi_individual_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(knowledge_update_multi_individual_node knowledge_updater_multi_individual
   ${catkin_LIBRARIES}
)



add_library(hiprl_action_interface
   src/hiprl_action_interface.cpp
)
add_dependencies(hiprl_action_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hiprl_action_interface ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(hiprl_action_interface_node src/hiprl_action_interface_node.cpp)
add_dependencies(hiprl_action_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hiprl_action_interface_node hiprl_action_interface
   ${catkin_LIBRARIES}
)

