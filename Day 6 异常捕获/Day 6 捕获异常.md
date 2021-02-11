 

 

# 捕获异常

- （目的）：让c++抛出异常来被python捕获
- 整体不难，知道抛出和捕获异常的语法即可



## 代码

1. 创建文件 `throw_exception.cpp` 

```c++
#include <iostream>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace std;
using namespace pybind11::literals;


void throw_exception() {
    // 抛出异常
    throw std::exception();
}

// 基本模板
PYBIND11_MODULE(throw_exception, m) {
    m.doc() = "throw exception";
    m.def("throw_exception", &throw_exception, "throw_exception");
}
```

2. 创建文件`catch_exception.py`

```python
import throw_exception

if __name__ == '__main__':
    try:
        throw_exception.throw_exception()
     # 捕获异常
    except Exception as e:
        print(repr(e))
```



## 编译和运行代码

1. 创建CMakeLists.txt文件

```cmake
cmake_minimum_required(VERSION 3.10)
project(throw_exception)

find_package(PythonInterp)
find_package(PythonLibs)
find_package(pybind11)
pybind11_add_module(throw_exception throw_exception.cpp)
```

2. 调用`catch_exception.py`（注意动态库的路径）

```
>>>
RuntimeError('std::exception')
```



## 知识点

- 如果c++中抛出了一个异常，则pybind的`c++ exception handler`将会捕获这个c++异常，并将其转换为python中的异常来抛出。最终实现python捕获c++抛出的异常的功能。

- [其中c++抛出的异常将转换为如下的python异常](https://pybind11.readthedocs.io/en/stable/advanced/exceptions.html)：

  | Exception thrown by C++    | Translated to Python exception type                          |
  | -------------------------- | ------------------------------------------------------------ |
  | `std::exception`           | `RuntimeError`                                               |
  | `std::bad_alloc`           | `MemoryError`                                                |
  | `std::domain_error`        | `ValueError`                                                 |
  | `std::invalid_argument`    | `ValueError`                                                 |
  | `std::length_error`        | `ValueError`                                                 |
  | `std::out_of_range`        | `IndexError`                                                 |
  | `std::range_error`         | `ValueError`                                                 |
  | `std::overflow_error`      | `OverflowError`                                              |
  | `pybind11::stop_iteration` | `StopIteration` (used to implement custom iterators)         |
  | `pybind11::index_error`    | `IndexError` (used to indicate out of bounds access in `__getitem__`, `__setitem__`, etc.) |
  | `pybind11::key_error`      | `KeyError` (used to indicate out of bounds access in `__getitem__`, `__setitem__` in dict-like objects, etc.) |
  | `pybind11::value_error`    | `ValueError` (used to indicate wrong value passed in `container.remove(...)`) |
  | `pybind11::type_error`     | `TypeError`                                                  |
  | `pybind11::buffer_error`   | `BufferError`                                                |
  | `pybind11::import_error`   | `import_error`                                               |
  | Any other exception        | `RuntimeError`                                               |

- [如果这些不够用，这可以自定义异常转换函数](https://pybind11.readthedocs.io/en/stable/advanced/exceptions.html#registering-custom-translators)