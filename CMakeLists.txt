cmake_minimum_required(VERSION 3.0.2)
project(swarm_ros_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bridge_node src/bridge_node.cpp)

target_link_libraries(bridge_node
   ${catkin_LIBRARIES}
   zmq
   zmqpp
)

if(CATKIN_ENABLE_TESTING)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_executable(bridge_node_asan_test src/bridge_node.cpp)
    target_compile_options(bridge_node_asan_test PRIVATE -fsanitize=address -fno-omit-frame-pointer)
    set_target_properties(bridge_node_asan_test PROPERTIES LINK_FLAGS "-fsanitize=address")
    target_link_libraries(bridge_node_asan_test
      ${catkin_LIBRARIES}
      zmq
      zmqpp
    )

    add_test(
      NAME bridge_lifecycle_asan
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/bridge_lifecycle_test.py $<TARGET_FILE:bridge_node_asan_test>
    )
    set_tests_properties(bridge_lifecycle_asan PROPERTIES
      TIMEOUT 45
      ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0:abort_on_error=1"
    )
  endif()
endif()

install(TARGETS bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
	scripts/listener.py
	scripts/talker.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
