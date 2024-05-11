message(STATUS "================  MESA  ======================")

# The Library
add_library(mesa INTERFACE)
target_link_libraries(mesa INTERFACE gtsam)
target_include_directories(mesa INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include" ${GTSAM_INCLUDE_DIR})
