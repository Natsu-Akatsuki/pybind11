 

 

# 封装pcd文件的读取（导出为c++可执行文件）

- 本实例为导出可执行文件而非拓展库

- 导出可执行文件是为了可以进行DEBUG

- 只需要在原来的基础上进行一些修改就over了

  

## 代码

1、创建文件 `load_pcd_file.cpp` 

```c++
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <string>

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;
using namespace py::literals;

// 有关读取的点云类型
typedef pcl::PointXYZI PointType;
#define FIELD_NUM 4

// 生成c++可执行文件，用于debug
py::array_t<float> load_pcd_file(std::string &file_name, bool remove_nan) {
    pcl::PointCloud<PointType>::Ptr pointcloud(new pcl::PointCloud<PointType>);
    std::cout << "loading file: " << file_name << std::endl;

    if (pcl::io::loadPCDFile<PointType>(file_name, *pointcloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        // todo 抛出异常
    }
    if (remove_nan) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pointcloud, *pointcloud, indices);
    }

    unsigned int pointcloud_size = pointcloud->width * pointcloud->height;
    py::array_t<float, py::array::c_style> pointcloud_output({(const int) pointcloud_size, FIELD_NUM});

    // 用这种方法可以使用()的赋值方法
    auto pointcloud_proxy = pointcloud_output.mutable_unchecked<2>(); // 2表示dimensions
    for (py::ssize_t i = 0; i < pointcloud_proxy.shape(0); i++) {
        pointcloud_proxy(i, 0) = pointcloud->points[i].x;
        pointcloud_proxy(i, 1) = pointcloud->points[i].y;
        pointcloud_proxy(i, 2) = pointcloud->points[i].z;
        pointcloud_proxy(i, 3) = pointcloud->points[i].intensity;
    }

    return pointcloud_output;
}


int main(int argc, char **argv) {
    py::scoped_interpreter guard{};
    std::string file_name = "/home/helios/calibration/00001.pcd";
    auto pointcloud = load_pcd_file(file_name, false);
}
```



## 代码解构

1. 特殊的部分：包括有头文件、main函数和一个解释器对象

```c++
// header
#include <pybind11/embed.h>

// object
py::scoped_interpreter guard{};
```



## 编译和运行代码

1. 创建CMakeLists.txt文件

```cmake
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
add_executable(load_pcd_file load_pcd_file.cpp)
# 注意此处有 pybind11::embed
target_link_libraries(load_pcd_file PRIVATE ${PCL_LIBRARIES} pybind11::embed)
```

2. 执行可执行文件`./load_pcd_file` 



