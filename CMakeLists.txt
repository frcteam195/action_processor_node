cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME "action_processor_node")
set(LIBRARY_NAME "action_helper")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS roscpp
  std_msgs
  message_generation
  rio_control_node
  uuid_msgs
  lib_uuid_node)

## generate messages in the 'msg' folder
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
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS roscpp message_generation message_runtime rio_control_node lib_uuid_node
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

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

add_library(${LIBRARY_NAME} ${executable_sources})
target_include_directories(${LIBRARY_NAME} PUBLIC "include")
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} Threads::Threads)
add_dependencies(${LIBRARY_NAME} ${${LIBRARY_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${LIBRARY_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
# install(TARGETS standalone_converter
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${LIBRARY_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h"
   #FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  ## Add gtest based cpp test target and link libraries.
  catkin_add_gtest(${LIBRARY_NAME}-test ${test_sources})
  if(TARGET ${LIBRARY_NAME}-test)
    target_link_libraries(${LIBRARY_NAME}-test ${LIBRARY_NAME})
    target_include_directories(${LIBRARY_NAME}-test PUBLIC "include" "test/include")
    add_dependencies(${LIBRARY_NAME}-test ${LIBRARY_NAME} ${${LIBRARY_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()
endif()