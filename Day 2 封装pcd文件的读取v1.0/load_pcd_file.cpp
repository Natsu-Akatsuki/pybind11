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

// 生成c++拓展库
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