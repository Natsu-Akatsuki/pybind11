 

 

# 封装pcd文件的读取

- 重点是构建一个numpy的c++封装类 `py::array_t<float>`来作为c++和python点云数据互通的桥梁

## 代码

1、创建文件 `load_pcd_file.cpp` 

```c++
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <string>

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
    std::cout << "loading file" << file_name << std::endl;
    if (pcl::io::loadPCDFile<PointType>(file_name, *pointcloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        // todo 抛出异常，被python接收
    }
    if (remove_nan) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pointcloud, *pointcloud, indices);
    }

    unsigned int pointcloud_size = pointcloud->width * pointcloud->height;
//    std::cout << pointcloud_size << std::endl;

//    auto pointcloud_output = py::array_t<float, py::array::c_style>(
//            py::array::ShapeContainer({pointcloud_size, FIELD_NUM}));

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

PYBIND11_MODULE(load_pcd_file, m) {
    m.doc() = "读取pcd点云并转换为np格式";
    m.doc() = "pcd2io";

    m.def("load_pcd_file", &load_pcd_file, "读取pcd点云文件",
          "file_name"_a, "remove_nan"_a = true);
};
```



## 代码解构

1. 创建`py::array`数组的方法：

```c++
// 方法一：
auto pointcloud_output = py::array_t<float, py::array::c_style>(
    py::array::ShapeContainer({pointcloud_size, FIELD_NUM})  );

// 方法二：
// 该方法传入前需要将类型转换为只读变量（加上const）否则会报错
// No matching constructor for initialization of 'py::array_t<float, py::array::c_style>'
// 第二个模板参数指的是storage layout是row-major(c_style)还是column-major(f_style)
py::array_t<float, py::array::c_style> pointcloud_output({(const int) pointcloud_size, FIELD_NUM});
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

pybind11_add_module(load_pcd_file load_pcd_file.cpp)
target_link_libraries(load_pcd_file PRIVATE ${PCL_LIBRARIES})
```

PS: 

- [不显示pcl warning](https://github.com/PointCloudLibrary/pcl/issues/3680)

```cmake
# 方法一（使用当前conda环境的python解释器）
find_package(PythonInterp)
find_package(PythonLibs)

# 方法二（可以不依赖于conda环境）
set(PYTHON_EXECUTABLE /home/helios/.conda/envs/pcdet/bin/python)
```

2. 从python调用封装函数

```c++
$ python
>>> import load_pcd_file
>>> load_pcd_file.load_pcd_file('/home/helios/calibration/0001.pcd', False)
loading file/home/helios/calibration/0001.pcd
array([[ 2.4369328,  7.4470015,  2.09954  ,  0.       ],
       [ 2.450845 ,  7.3976755,  2.0881522,  0.       ],
       [ 2.4717412,  7.3744216,  2.0840108,  0.       ],
       ...,
       [ 2.1694767, -2.066687 , -0.8028567,  1.       ],
       [ 2.1427321, -2.0555243, -0.7956098,  1.       ],
       [ 2.1661112, -2.09252  , -0.8069978,  1.       ]], dtype=float32)
>>> 
```

