cmake_minimum_required(VERSION 3.1.2)
project(leader_follower_controller_plugin)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  rospy
  pluginlib
  geometry_msgs
  dynamic_reconfigure
  mrs_uav_managers
  mrs_msgs
  mrs_lib
  std_msgs
  message_generation
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

generate_dynamic_reconfigure_options(
  cfg/leader_follower_controller.cfg
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# acados
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/acados")
#list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/git/acados")
find_package(acados REQUIRED)
#add_compile_options(-DACADOS_WITH_QPDUNES -DACADOS_WITH_OSQP)
#add_definitions(-DACADOS_WITH_QPDUNES -DACADOS_WITH_OSQP)

get_target_property(acados_INCLUDE_DIRS acados INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(hpipm_INCLUDE_DIRS hpipm INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(blasfeo_INCLUDE_DIRS blasfeo INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(qpoases_INCLUDE_DIRS qpOASES_e INTERFACE_INCLUDE_DIRECTORIES)
#get_target_property(qpdunes_INCLUDE_DIRS qpdunes INTERFACE_INCLUDE_DIRECTORIES)
#get_target_property(osqp_INCLUDE_DIRS osqp INTERFACE_INCLUDE_DIRECTORIES)

#link_directories(${acados_INCLUDE_DIRS}/../lib)

link_directories(/home/josef/acados/lib)
#link_directories(/home/mrs/git/acados/lib)
add_message_files(
  FILES
  LKFState.msg
  Attitude.msg
  ControlAction.msg
  )

set(LIBRARIES
  LeaderFollowerController
  )

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
  )
  
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${acados_INCLUDE_DIRS}
  ${hpipm_INCLUDE_DIRS}
  ${blasfeo_INCLUDE_DIRS}
  )

  # Leader Follower Controller

FILE(GLOB SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(LeaderFollowerController
  ${SOURCE_FILES}
  src/lkf.cpp 
  )

  add_dependencies(LeaderFollowerController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

  target_link_libraries(LeaderFollowerController
  ${catkin_LIBRARIES}
  acados
  hpipm
  blasfeo
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
