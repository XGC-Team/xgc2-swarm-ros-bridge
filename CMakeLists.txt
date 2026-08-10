cmake_minimum_required(VERSION 3.0.2)
project(swarm_ros_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  scout_msgs
)

add_library(swarm_ros_bridge_protocol_v2
  src/v2/protocol.cpp
)
target_compile_features(swarm_ros_bridge_protocol_v2 PUBLIC cxx_std_17)
target_compile_options(swarm_ros_bridge_protocol_v2 PRIVATE
  -Wall -Wextra -Wpedantic
)
target_include_directories(swarm_ros_bridge_protocol_v2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(swarm_ros_bridge_ros1_codec_v2
  src/v2/ros1_codec.cpp
)
target_compile_features(swarm_ros_bridge_ros1_codec_v2 PUBLIC cxx_std_17)
target_compile_options(swarm_ros_bridge_ros1_codec_v2 PRIVATE
  -Wall -Wextra -Wpedantic
)
target_include_directories(swarm_ros_bridge_ros1_codec_v2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(swarm_ros_bridge_ros1_codec_v2
  swarm_ros_bridge_protocol_v2
  ${catkin_LIBRARIES}
)
add_dependencies(swarm_ros_bridge_ros1_codec_v2 ${catkin_EXPORTED_TARGETS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES swarm_ros_bridge_protocol_v2 swarm_ros_bridge_ros1_codec_v2
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs scout_msgs
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
  add_executable(swarm_ros_bridge_v2_protocol_test
    test/v2_protocol_test.cpp
  )
  target_compile_features(swarm_ros_bridge_v2_protocol_test PRIVATE cxx_std_17)
  target_compile_options(swarm_ros_bridge_v2_protocol_test PRIVATE
    -Wall -Wextra -Wpedantic
  )
  target_link_libraries(swarm_ros_bridge_v2_protocol_test
    swarm_ros_bridge_protocol_v2
  )
  add_test(
    NAME swarm_ros_bridge_v2_protocol
    COMMAND $<TARGET_FILE:swarm_ros_bridge_v2_protocol_test>
  )
  add_test(
    NAME swarm_ros_bridge_v2_reference_encoder
    COMMAND ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/v2_reference_encoder_test.py
      $<TARGET_FILE:swarm_ros_bridge_v2_protocol_test>
      ${CMAKE_CURRENT_SOURCE_DIR}/test/golden_v2_vectors.txt
  )

  add_executable(swarm_ros_bridge_v2_ros1_codec_test
    test/v2_ros1_codec_test.cpp
  )
  target_compile_features(swarm_ros_bridge_v2_ros1_codec_test PRIVATE cxx_std_17)
  target_compile_options(swarm_ros_bridge_v2_ros1_codec_test PRIVATE
    -Wall -Wextra -Wpedantic
  )
  target_link_libraries(swarm_ros_bridge_v2_ros1_codec_test
    swarm_ros_bridge_ros1_codec_v2
    ${catkin_LIBRARIES}
  )
  add_dependencies(swarm_ros_bridge_v2_ros1_codec_test ${catkin_EXPORTED_TARGETS})
  add_test(
    NAME swarm_ros_bridge_v2_ros1_codec
    COMMAND $<TARGET_FILE:swarm_ros_bridge_v2_ros1_codec_test>
  )

  install(TARGETS
    swarm_ros_bridge_v2_protocol_test
    swarm_ros_bridge_v2_ros1_codec_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/tests
  )

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


install(TARGETS bridge_node swarm_ros_bridge_protocol_v2 swarm_ros_bridge_ros1_codec_v2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/swarm_ros_bridge/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# scripts
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

install(FILES docs/protocol-v2.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
)
