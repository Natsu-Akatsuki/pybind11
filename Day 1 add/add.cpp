#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

int add(int i, int j) {
    return i + j;
}


// 基本模板
//PYBIND11_MODULE(add, m) {
//    m.doc() = "add two number";
//    m.def("add", &add, "add two number");
//}

// 使用关键词和默认参数
PYBIND11_MODULE(add, m) {
    m.doc() = "add two number";
    m.def("add", &add, "add two numbers",
          py::arg("i") = 1, py::arg("j") = 2);
}