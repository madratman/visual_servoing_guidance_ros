cmake_minimum_required(VERSION 2.8.3)
project(GuidanceRos)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  geometry_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PID_controller Line_detector
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(PID_controller
   src/PID_controller.cpp
 )

add_library(Line_detector
  src/Line_detector.cpp
 )

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

MESSAGE("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if (CMAKE_SIZEOF_VOID_P EQUAL 4)
		MESSAGE("-- 32bit detected")	
		link_directories(${PROJECT_SOURCE_DIR}/lib/x86)
	elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
		MESSAGE("-- 64bit detected")
		link_directories(${PROJECT_SOURCE_DIR}/lib/x64)
	endif ()
else()
	MESSAGE("-- Non-linux platform detected but sorry we do not support :D")
endif ()

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
	MESSAGE("-- " ${CMAKE_SYSTEM_PROCESSOR} " detected")
	link_directories(${PROJECT_SOURCE_DIR}/lib/XU3)
else ()
endif ()

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(GuidanceNode ${PROJECT_SOURCE_DIR}/src/GuidanceNode.cpp ${PROJECT_SOURCE_DIR}/src/DJI_utility.cpp)
add_executable(GuidanceNodeCalibration ${PROJECT_SOURCE_DIR}/src/GuidanceNodeCalibration.cpp ${PROJECT_SOURCE_DIR}/src/DJI_utility.cpp)
add_executable(GuidanceNodeTest ${PROJECT_SOURCE_DIR}/src/GuidanceNodeTest.cpp)
add_executable(PID_node ${PROJECT_SOURCE_DIR}/src/PID_node.cpp)

target_link_libraries(GuidanceNode DJI_guidance usb-1.0 ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(GuidanceNodeCalibration DJI_guidance usb-1.0 ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(GuidanceNodeTest ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(PID_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
