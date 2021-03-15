#include <iostream>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
//#include <pybind11/stl_bind.h>
namespace py = pybind11;

struct Names {
    int a;
    int b;
    int c;
    int d[10];
};

struct values {
    Names names;
};


// Wrapping code
PYBIND11_MODULE(bind_nested_structures, m) {
    py::class_<Names>(m, "Names")
            .def(py::init<>())
            .def_readwrite("a", &Names::a)
            .def_readwrite("b", &Names::b)
            .def_readwrite("c", &Names::c)
            .def_property_readonly("d", [](py::object& obj) {
        Names &cplus_o = obj.cast<Names &>();
        return py::array{10, cplus_o.d, obj};
    });

    py::class_<values>(m, "values")
            .def(py::init<>())
            .def_readwrite("names", &values::names);
}
