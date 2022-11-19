#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

int add(int i, int j) {
  return i + j;
}

PYBIND11_MODULE(EXPORT_PYBIND11_MODULE_NAME, m) {
  m.doc() = "add two number";
  m.def("add", &add, "add two numbers", py::arg("i") = 1, py::arg("j") = 2);
}