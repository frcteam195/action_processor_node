cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME "action_processor_node")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(${PROJECT_NAME})
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rio_control_node uuid_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Action.msg
  ActionList.msg
  ActionListParallel.msg
  ParallelActionID.msg
  OverallActions.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ActionReq.srv
  ActionUpdate.srv
  CancelOverall.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  uuid_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rio_control_node
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  CATKIN_DEPENDS roscpp message_generation message_runtime rio_control_node
)

include_directories(
  include
 ${catkin_INCLUDE_DIRS}
 ${CMAKE_SOURCE_DIR}/../../../third_party_libs/ROSProtoDef2022/out
)

message("CATKIN INCLUDES:${catkin_INCLUDE_DIRS}")

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

file(GLOB_RECURSE executable_sources CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

file(GLOB_RECURSE test_sources CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/test/src/*.cpp"
)

add_executable(${PROJECT_NAME} ${executable_sources})
target_include_directories(${PROJECT_NAME} PUBLIC "include")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
