message(STATUS "====================== DGS ======================")
add_subdirectory(distributed-gauss-sidel)

message(STATUS "====================== MB-ADMM ======================")
add_subdirectory(admm-slam)

message(STATUS "====================== DC2-PGO ======================")
add_subdirectory(dc2-pgo)