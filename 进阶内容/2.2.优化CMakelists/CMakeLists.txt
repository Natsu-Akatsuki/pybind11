cmake_minimum_required(VERSION 3.13)
project(add_two_num)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

include(cmake/pybind11.cmake)
pybind11_add_module(${PROJECT_NAME}_pyb add_two_num.cpp)

set_target_properties(${PROJECT_NAME}_pyb PROPERTIES PREFIX "")               #  指定前缀
set_target_properties(${PROJECT_NAME}_pyb PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_pyb)   #  指定文件名
set_target_properties(${PROJECT_NAME}_pyb PROPERTIES SUFFIX ".so")            #  指定后缀
set_target_properties(${PROJECT_NAME}_pyb PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})  # 指定库的输出路径
set_target_properties(${PROJECT_NAME}_pyb PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})  # 指定可执行文件的输出路径
target_compile_definitions(${PROJECT_NAME}_pyb PRIVATE EXPORT_PYBIND11_MODULE_NAME=${PROJECT_NAME}_pyb)