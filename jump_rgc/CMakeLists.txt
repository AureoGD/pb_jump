# cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# #============================================================================
# # Initialize the project
# #============================================================================
# project(jump_rgc)

# #============================================================================
# # Configure the project
# #============================================================================
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# #============================================================================
# # Find packages
# #============================================================================
# # If you get an error at this line, you need to install gz-cmake

# find_package(OsqpEigen REQUIRED)
# find_package(pybind11 REQUIRED)
# find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# include_directories(
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# add_library(${PROJECT_NAME}
#     SHARED
# 		src/pred_control.cpp
# 		src/op_wrapper.cpp
# 		src/opt_problem.cpp
# 		src/opt_problem1.cpp
# 		src/opt_problem2.cpp
# 		src/opt_problem3.cpp
# 		src/opt_problem4.cpp
# 		src/opt_problem5.cpp
# 		src/opt_problem6.cpp
# 		src/jump_robot_model.cpp
# )

# target_link_libraries(${PROJECT_NAME}
# 	PUBLIC
# 		OsqpEigen::OsqpEigen
# )

# set(BINDINGS_MODULE_NAME "pybind_opWrapper")

# pybind11_add_module(${BINDINGS_MODULE_NAME} MODULE
# 	"src/jump_robot_model.cpp"
# 	# "src/opt_problem.cpp"
# 	# "src/jump_robot_model.cpp"
#     "src/pybind_opWrapper.cpp"
# )

# target_link_libraries(${BINDINGS_MODULE_NAME}
# 	PRIVATE
# 		OsqpEigen::OsqpEigen
# )

#######

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(jump_rgc)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#============================================================================
# Find packages
#============================================================================
find_package(OsqpEigen REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
    SHARED
    src/pred_control.cpp
    src/op_wrapper.cpp
    src/opt_problem.cpp
    src/opt_problem1.cpp
    src/opt_problem2.cpp
    src/opt_problem3.cpp
    src/opt_problem4.cpp
    src/opt_problem5.cpp
    src/opt_problem6.cpp
    src/jump_robot_model.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        OsqpEigen::OsqpEigen
)

set(BINDINGS_MODULE_NAME "pybind_opWrapper")

# Only source (.cpp) files should be listed here
pybind11_add_module(${BINDINGS_MODULE_NAME} MODULE
    "src/pybind_opWrapper.cpp"
	"src/op_wrapper.cpp"
    "src/jump_robot_model.cpp"
)

target_link_libraries(${BINDINGS_MODULE_NAME}
    PRIVATE
        OsqpEigen::OsqpEigen
		${PROJECT_NAME}
)
