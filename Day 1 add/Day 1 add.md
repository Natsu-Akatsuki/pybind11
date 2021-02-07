 

 

# 封装一个加法函数

### 代码

1、创建文件 `add.cpp` 

```c++
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

int add(int i, int j) {
    return i + j;
}

// level 0 基本模板
//PYBIND11_MODULE(add, m) {
//    m.doc() = "add two number";
//    m.def("add", &add, "add two number");
//}

// level 1 使用关键词和默认参数
PYBIND11_MODULE(add, m) {
    m.doc() = "add two number";
    m.def("add", &add, "add two numbers",
          py::arg("i") = 1, py::arg("j") = 2);
}
```



### 代码解构

1. 封装的代码

```c++
PYBIND11_MODULE(add, m) {
    m.doc() = "add two number";
    m.def("add", &add, "add two numbers",
          py::arg("i") = 1, py::arg("j") = 2);
}

PYBIND11_MODULE(<导出的模块名，无双引号>, m) {
    m.doc() = "模块的描述性说明";
    m.def("导出的函数名", <函数地址>, "函数的描述性说明",
          py::arg("关键词") = 默认值;
}
```

PS：调用`using namespace pybind11::literals`后有简化形式

```
using namespace pybind11::literals;
py::arg("关键词") = 默认值   ->  "关键词"_a = 默认值

e.g.
m.def("add2", &add, "i"_a = 1, "j"_a = 2);
```



### 编译和运行代码

1. 创建CMakeLists.txt文件

```cmake
cmake_minimum_required(VERSION 3.17)
project(add)

set(CMAKE_CXX_STANDARD 11)

# 使用当前conda环境的python解释器
find_package(PythonInterp)
find_package(PythonLibs)

find_package(pybind11)
// 生成add模块（这个模块名要跟源文件  PYBIND11_MODULE(<导出的模块名，无双引号>, m)  写的模块名一致，否则会报错）
pybind11_add_module(add add.cpp)
```

PS: 指定python解释器的方法

```cmake
# 方法一（使用当前conda环境的python解释器）
find_package(PythonInterp)
find_package(PythonLibs)

# 方法二（可以不依赖于conda环境）
set(PYTHON_EXECUTABLE /home/helios/.conda/envs/pcdet/bin/python)
```

2. 调用函数

```
$ python
>>> import add
>>> add.add(3,4) 
7
```

