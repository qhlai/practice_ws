cmake_minimum_required(VERSION 2.8.3)

project(jaka_control)

find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)
find_package(catkin REQUIRED COMPONENTS

moveit_core 
moveit_ros_planning 
moveit_ros_planning_interface 
moveit_visual_tools
# intera_core_msgs

   geometry_msgs
   roscpp
   rospy
   sensor_msgs
   std_msgs
   message_generation
   std_srvs
   robot_msgs
#    CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime std_srvs
)
generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs std_srvs robot_msgs
)
find_package(Eigen3)
find_package(Boost REQUIRED system thread)
include_directories(
# include
#  include/jaka_jog_panel
#  ${QT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
#   src/libs
)
# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)


add_executable(jaka_control src/client.cpp)
target_link_libraries(jaka_control  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_control robot_msgs)


add_executable(jaka_moveit src/moveit.cpp)
target_link_libraries(jaka_moveit  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_moveit robot_msgs)


add_executable(jaka_test src/test.cpp)
target_link_libraries(jaka_test  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_test robot_msgs)


add_executable(jaka_calib src/calib.cpp)
target_link_libraries(jaka_calib  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_calib robot_msgs)



add_executable(jaka_home src/home.cpp)
target_link_libraries(jaka_home  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_home robot_msgs)

add_executable(jaka_focus src/focus.cpp)
target_link_libraries(jaka_focus  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_focus robot_msgs)


add_executable(jaka_calc src/calc.cpp)
target_link_libraries(jaka_calc  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_calc robot_msgs)


add_executable(jaka_calib_test src/calib_test.cpp)
target_link_libraries(jaka_calib_test  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_calib_test robot_msgs)


add_executable(jaka_calib_test_qua src/calib_test_qua.cpp)
target_link_libraries(jaka_calib_test_qua  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_calib_test_qua robot_msgs)


add_executable(jaka_mapping src/mapping.cpp)
target_link_libraries(jaka_mapping  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_mapping robot_msgs)

add_executable(jaka_inter src/inter.cpp)
target_link_libraries(jaka_inter  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_inter robot_msgs)


add_executable(jaka_show src/show.cpp)
target_link_libraries(jaka_show  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_show robot_msgs)



add_executable(calib_auto src/calib_auto.cpp)
target_link_libraries(calib_auto  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(calib_auto robot_msgs)
