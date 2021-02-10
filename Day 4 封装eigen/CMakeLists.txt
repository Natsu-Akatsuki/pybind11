cmake_minimum_required(VERSION 3.10)
project(add_plus)

set(CMAKE_CXX_STANDARD 11)

# 使用当前conda环境的python解释器
find_package(PythonInterp)
find_package(PythonLibs)

# 导入eigen库
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIRS} )

find_package(pybind11)
pybind11_add_module(add_plus add_plus.cpp)