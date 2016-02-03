cmake_minimum_required(VERSION 2.8.3)
project(graph_test)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    graph_map
    triplet_graph
    geolib2
    tue_config
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES graph_test
 CATKIN_DEPENDS triplet_graph tue_config geolib2
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library( graph_simulator
    src/World.cpp
)
target_link_libraries(graph_simulator ${catkin_LIBRARIES})

add_executable(test_localization
    src/test_localization.cpp
    ${HEADER_FILES}
)
target_link_libraries(test_localization graph_simulator ${catkin_LIBRARIES})

add_executable(test_triplet_graph
    src/test_triplet_graph.cpp
)
target_link_libraries(test_triplet_graph ${catkin_LIBRARIES})

add_executable(test_graph_reader_writer
    src/test_graph_reader_writer.cpp
)
target_link_libraries(test_graph_reader_writer ${catkin_LIBRARIES})

add_executable(mapping
    src/mapping.cpp
)
target_link_libraries(mapping ${catkin_LIBRARIES})

add_executable(localization
    src/localization.cpp
)
target_link_libraries(localization ${catkin_LIBRARIES})
