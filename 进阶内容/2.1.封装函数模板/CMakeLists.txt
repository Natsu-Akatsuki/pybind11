cmake_minimum_required(VERSION 3.10)
project(test_function_template)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE RELEASE)

find_package(pybind11)

pybind11_add_module(test_function_template_pyb test_function_template.cpp)
set_target_properties(test_function_template_pyb PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})