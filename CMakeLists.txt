cmake_minimum_required(VERSION 3.20)
project(awake_sim_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_awake_slam
    cpp/bindings/module.cpp
    cpp/src/slam_core.cpp
    cpp/src/pose_graph.cpp
)
target_include_directories(_awake_slam PRIVATE cpp/include)
target_link_libraries(_awake_slam PRIVATE Eigen3::Eigen)
set_target_properties(_awake_slam PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/src/awake_sim"
)

enable_testing()
add_executable(awake_slam_tests
    cpp/tests/test_slam_core.cpp
    cpp/src/slam_core.cpp
    cpp/src/pose_graph.cpp
)
target_include_directories(awake_slam_tests PRIVATE cpp/include)
target_link_libraries(awake_slam_tests PRIVATE Eigen3::Eigen)
add_test(NAME awake_slam_core COMMAND awake_slam_tests)

add_executable(awake_pose_graph_tests
    cpp/tests/test_pose_graph.cpp
    cpp/src/pose_graph.cpp
)
target_include_directories(awake_pose_graph_tests PRIVATE cpp/include)
target_link_libraries(awake_pose_graph_tests PRIVATE Eigen3::Eigen)
add_test(NAME awake_pose_graph COMMAND awake_pose_graph_tests)
