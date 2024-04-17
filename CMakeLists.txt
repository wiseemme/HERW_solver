cmake_minimum_required(VERSION 3.0.0)
project(HERW_solver VERSION 0.1.0)

set(CMAKE_BUILD_TYPE "release")
# set everything up for c++ 17 features
set(CMAKE_CXX_STANDARD 17)
# Don't add this line if you will try_compile with boost.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED)
find_package(Sophus 1.0 REQUIRED)
include_directories(include)

add_executable(random_robot_world_problem examples/random_robot_world_problem.cpp include/robot_world_sim.hpp src/robot_world_sim.cpp include/wang_solver.hpp src/wang_solver.cpp)
target_include_directories(random_robot_world_problem
  PRIVATE
    ${EIGEN_INCLUDE_DIRS}
    ${SOPHUS_INCLUDE_DIRS}
)
target_link_libraries(random_robot_world_problem
  PRIVATE 
    ${EIGEN_LIBRARIES}
    ${SOPHUS_LIBRARIES}
    Sophus::Sophus
)
