cmake_minimum_required(VERSION 3.10)
project(throw_exception)

find_package(PythonInterp)
find_package(PythonLibs)
find_package(pybind11)
pybind11_add_module(throw_exception throw_exception.cpp)