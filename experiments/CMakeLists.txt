project(dalec-experiments CXX C)
cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 17)

message(STATUS "================  EXPERIMENTS  ======================")

# Dependencies
add_subdirectory(batch_runners)

# Declare and configure experiment runner executable
add_executable(run-dist-batch "run-dist-batch.cpp")
target_link_libraries(run-dist-batch PUBLIC DPGO gtsam jrl ${Boost_LIBRARIES} batch_runners)

add_executable(g2o-2-mr-jrl "g2o-2-mr-jrl.cpp")
target_link_libraries(g2o-2-mr-jrl PUBLIC gtsam jrl ${Boost_LIBRARIES} admm)
