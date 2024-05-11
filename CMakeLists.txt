project(mesa CXX C)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)

message(STATUS "================  BUILDING MESA  ======================")

# Dependencies
find_package(GTSAM REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(jrl REQUIRED)

# Build Thirdparty code
add_subdirectory(thirdparty)

# Build the MESA code
add_subdirectory(mesa)

# Build Experiments directory
add_subdirectory(experiments)
