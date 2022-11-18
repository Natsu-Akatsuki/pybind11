#include <pybind11/pybind11.h>
#include <iostream>
namespace py = pybind11;
using namespace pybind11::literals;

template<typename T>
void set(T t) {
  std::cout << t << std::endl;
}

PYBIND11_MODULE(test_function_template_pyb, m) {
  m.doc() = "测试封装函数模板";
  m.def("set", &set<int>);
  m.def("set", &set<std::string>);
}