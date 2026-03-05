cmake_minimum_required(VERSION 2.8.3)
project(ublox)

set(CMAKE_CXX_FLAGS "-std=c++17")
set(Boost_DEBUG ON)
set(Boost_DETAILED_FAILURE_MSG ON)

find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(async_comm QUIET)
find_package(Boost)
if (NOT ${async_comm_FOUND})
	message("Async_comm library not found!")
	add_subdirectory(lib/async_comm)
	set(async_comm_LIBRARIES async_comm)
	set(async_comm_INCLUDE_DIRS lib/async_comm/include;${Boost_INCLUDE_DIRS})
endif()


message("async_comm_INCLUDE_DIRS: ${async_comm_INCLUDE_DIRS}\n")

add_library(UBLOX
    src/ublox.cpp
	src/parsers/ubx_maps.cpp
    src/parsers/ubx.cpp
    src/parsers/rtcm.cpp
    src/parsers/nav.cpp
    src/utctime.cpp
    src/field_remote.cpp
)

message("UBLOX Boost include dirs: ${Boost_INCLUDE_DIRS}")
target_link_libraries(UBLOX ${async_comm_LIBRARIES} ${gnss_utils_LIBRARIES}  ${openSSL_LIBRARIES} -lcrypto)
target_include_directories(UBLOX PUBLIC include ${async_comm_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

option(BUILD_TESTS "Build Tests" ON)
if (${BUILD_TESTS})
	if(NOT TARGET gtest)
		# Download and unpack googletest at configure time
		configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
		execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
		RESULT_VARIABLE result
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
		if(result)
		message(FATAL_ERROR "CMake step for googletest failed: ${result}")
		endif()
		execute_process(COMMAND ${CMAKE_COMMAND} --build .
		RESULT_VARIABLE result
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
		if(result)
		message(FATAL_ERROR "Build step for googletest failed: ${result}")
		endif()

		# Prevent overriding the parent project's compiler/linker
		# settings on Windows
		set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

		# Add googletest directly to our build. This defines
		# the gtest and gtest_main targets.
		add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
						${CMAKE_CURRENT_BINARY_DIR}/googletest-build
						EXCLUDE_FROM_ALL)

		# The gtest/gtest_main targets carry header search path
		# dependencies automatically when using CMake 2.8.11 or
		# later. Otherwise we have to add them here ourselves.
		if (CMAKE_VERSION VERSION_LESS 2.8.11)
		include_directories("${gtest_SOURCE_DIR}/include")
		endif()
	endif()
   	message(STATUS "Building UBLOX parser Tests")
   	find_package(Threads REQUIRED)
   	add_executable(ublox_test
       src/test/test_rtcm.cpp
       src/test/test_time.cpp
       src/test/test_eph.cpp)
   	target_link_libraries(ublox_test gtest gtest_main pthread UBLOX)
endif()

option(BUILD_EXAMPLES "Build Examples" ON)
if (${BUILD_EXAMPLES})
	add_executable(base src/examples/base.cpp)
	target_link_libraries(base UBLOX)

	add_executable(rover src/examples/rover.cpp)
	target_link_libraries(rover UBLOX)

	add_executable(read src/examples/read.cpp)
	target_link_libraries(read UBLOX)
endif()

option(BUILD_ROS "Build ROS Node" ON)
if (${BUILD_ROS})
    message(STATUS "BUILDING UBLOX ROS PACKAGE")
	find_package(catkin REQUIRED COMPONENTS
		rosbag
		roscpp
		sensor_msgs
		message_generation
		shared_messages)

	add_message_files(
		DIRECTORY msg
		FILES
		PosVelEcef.msg
		PositionVelocityTime.msg
		RelPos.msg
		RelPosFlags.msg
		RTCMInput.msg
		Satellite.msg
		SatelliteStatus.msg
		SurveyStatus.msg
		Ephemeris.msg
		GlonassEphemeris.msg
		Observation.msg
		ObsVec.msg
		CfgValGetType.msg)
	
	add_service_files(
		DIRECTORY srv
		FILES
		CfgValGet.srv
		CfgValDel.srv
		CfgValSet.srv
		CfgReset.srv
		CfgValGetAll.srv
		GetVersion.srv
		initModule.srv)
		
	generate_messages(
  		DEPENDENCIES
  		std_msgs)
	catkin_package(
		LIBRARIES UBLOX
		INCLUDE_DIRS include
  		CATKIN_DEPENDS std_msgs message_runtime
	)

	add_executable(ublox_ros src/ros/ublox_ros.cpp 
		src/ros/callback_ros.cpp src/ros/services_ros.cpp src/ros/init_ros.cpp)
	target_link_libraries(ublox_ros UBLOX ${catkin_LIBRARIES}  ${openSSL_LIBRARIES} -lcrypto)
    target_include_directories(ublox_ros PUBLIC ../../devel/include ${catkin_INCLUDE_DIRS})
    add_dependencies(ublox_ros ${catkin_EXPORTED_TARGETS})

endif()
