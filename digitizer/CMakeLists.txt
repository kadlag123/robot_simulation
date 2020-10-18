cmake_minimum_required(VERSION 2.8.3)

project(digitizer)
set(CMAKE_CXX_FLAGS "-fpermissive -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
	#gazebo_ros
	rospy
	dynamic_reconfigure
)

generate_dynamic_reconfigure_options (
  cfg/Digitizer.cfg
)

#find_package(roslaunch)

catkin_package(DEPENDS)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


#install(DIRECTORY launch models worlds
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#foreach(dir launch urdf models)
#	install(DIRECTORY ${dir}/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)

catkin_install_python(PROGRAMS submissions/Suyash/PathFinder.py submissions/Suyash/PathFinderNode.py submissions/Suyash/Movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
