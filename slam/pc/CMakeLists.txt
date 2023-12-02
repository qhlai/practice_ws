cmake_minimum_required(VERSION 2.8.3)

project(slam_tools)

find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)
find_package(PCL 1.8 REQUIRED)
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
   # robot_msgs
#    CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime std_srvs
)

find_package(Eigen3)
find_package(Boost REQUIRED system thread)
find_package(octomap REQUIRED)
# include_directories(${OCTOMAP_INCLUDE_DIRS})
# target_link_libraries(${OCTOMAP_LIBRARIES})

include_directories(
# include
#  include/jaka_jog_panel
#  ${QT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
#   src/libs
)
# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)


# add_executable(jaka_inter src/inter.cpp)
# target_link_libraries(jaka_inter  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
# add_dependencies(jaka_inter robot_msgs)


# add_executable(jaka_show src/show.cpp)
# target_link_libraries(jaka_show  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
# add_dependencies(jaka_show robot_msgs)


add_executable(ply2pcd src/ply2pcd.cpp)
target_link_libraries(ply2pcd  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(pcd2oct src/pcd2oct.cpp)
target_link_libraries(pcd2oct  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})