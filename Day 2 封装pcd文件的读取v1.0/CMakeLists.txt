cmake_minimum_required(VERSION 3.10)
project(PcdIo)


if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif ()

find_package(PCL 1.11 QUIET PATHS /home/helios/local_library/share/pcl-1.11/ NO_DEFAULT_PATH)
find_package(PythonInterp)
find_package(PythonLibs)
find_package(pybind11)

include_directories(${PCL_INCLUDE_DIRS})
pybind11_add_module(load_pcd_file load_pcd_file.cpp)
target_link_libraries(load_pcd_file PRIVATE ${PCL_LIBRARIES})


