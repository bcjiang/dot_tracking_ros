cmake_minimum_required(VERSION 2.8.11)

#if(POLICY CMP0020)
#	cmake_policy(SET CMP0020 NEW)
#endif()

project(dot_tracking_ros)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# set(CMAKE_LIBRARY_PATH "C:\\Program Files (x86)\\Windows Kits\\8.0\\Lib\\win8\\um\\x64")

# add_definitions(-DNOMINMAX)
# set(CMAKE_BUILD_TYPE Debug)

set(OpenCV_STATIC OFF)


# Find OpenCV
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf)
find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# find_package(Qt5Widgets REQUIRED)
# set(CMAKE_AUTOMOC ON)


# IF(WIN32)
#     set(PTHREADS_INCLUDE_DIR "" CACHE PATH "Pthreads Include Directory")
#   set(PTHREADS_LIB_DIR "" CACHE PATH "Pthreads Lib Directory")
# ELSEIF(UNIX)
#   find_package(Threads)
# ENDIF()

catkin_package()

# Pattern tracker
add_subdirectory(src/libpatterntracker)
add_subdirectory(src/libchessdetector)


include_directories(
		${CMAKE_CURRENT_BINARY_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${CMAKE_CURRENT_SOURCE_DIR}/src/libpatterntracker
		${CMAKE_CURRENT_SOURCE_DIR}/src/libchessdetector
)

file(GLOB HEADER_FILES include/*.h)
file(GLOB CXX_FILES src/*.cpp)

add_executable(dot_tracker
		${CXX_FILES}
		${HEADER_FILES}
		)

target_link_libraries(dot_tracker
		${OpenCV_LIBS}
		${catkin_LIBRARIES}
		libpatterntracker
		)