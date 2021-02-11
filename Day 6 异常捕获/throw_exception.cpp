#include <iostream>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace std;
using namespace pybind11::literals;

void throw_exception() {
    throw std::exception();
}

// 基本模板
PYBIND11_MODULE(throw_exception, m) {
    m.doc() = "throw exception";
    m.def("throw_exception", &throw_exception, "throw_exception");
}

