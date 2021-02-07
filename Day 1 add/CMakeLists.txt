cmake_minimum_required(VERSION 3.10)
project(add)

set(CMAKE_CXX_STANDARD 11)

# 使用当前conda环境的python解释器
find_package(PythonInterp)
find_package(PythonLibs)

find_package(pybind11)
pybind11_add_module(add add.cpp)