 

 

# 封装一个加法函数 plus

- 封装一个矩阵加法函数，每个元素均加1
- python传入numpy，c++用eigen接收，并回传



## 代码

1. 创建文件 `add_plus.py` 

```c++
import numpy as np
from add_plus import add_plus

if __name__ == '__main__':
    # create a 2D numpy matrix
    arr_np = np.arange(10000).reshape(200, 50)
    print("处理前：前三个元素分别为：", arr_np[0][0], " ", arr_np[0][1], " ", arr_np[0][2])
    # 对矩阵进行逐元素的+1
    arr_np = add_plus(arr_np)
    print("处理后：前三个元素分别为：", arr_np[0][0], " ", arr_np[0][1], " ", arr_np[0][2])
```

2. 创建文件

```c++
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace pybind11::literals;

#define pass_ref  // pass_value or pass_ref

// 方法一：传值
#ifdef pass_value

Eigen::MatrixXi add_plus(Eigen::MatrixXi arr_np) {

    /*
    for (int i = 0; i < arr_np.rows(); i++) {
        for (int j = 0; j < arr_np.cols(); j++)
            arr_np(i, j) = arr_np(i, j) + 1;
    }
     */

    // Eigen也支持向量化操作
    arr_np.array() += 1;
    return arr_np;
}

#endif

// 方法二：传引入
#ifdef pass_ref

Eigen::MatrixXi add_plus(Eigen::MatrixXi &arr_np) {
    arr_np.array() += 1;
    return arr_np;
}

#endif

// 使用关键词和默认参数
PYBIND11_MODULE(add_plus, m) {
    m.doc() = "给矩阵进行逐元素的+1";
    m.def("add_plus", &add_plus, "给矩阵进行逐元素的+1");
}
```



## 代码解构

1. 方法一：传值，pybind会将numpy的数据进行`拷贝`得到一个临时的Eigen对象，然后再以这个Eigen对象作为形参来调用函数

```c++
 // 方法一：传值
Eigen::MatrixXi add_plus(Eigen::MatrixXi arr_np) {
	...
    // Eigen也支持向量化操作
    arr_np.array() += 1;
    return arr_np;
}
```

- 常规的`dense eigen`数据类型均可以实现这个效果


2. 方法二：

```c++
// 方法二：传引入
Eigen::MatrixXi add_plus(Eigen::MatrixXi &arr_np) {
    arr_np.array() += 1;
    return arr_np;
}
```



PS：

- numpy的`memory layout`默认是`c-major`而eigen则是`f-major`；实测，单纯用上面的代码，输入是`c-major`，返回是`f-major`。当然可以做一些处理让输入和输出数据的`memory layout`保持一致。但尚未清楚这样的不一致会有什么影响，暂时没找到一个切入点，待后续遇到时补充。



## 编译和运行代码

1. 创建CMakeLists.txt文件

```cmake
cmake_minimum_required(VERSION 3.10)
project(add_plus)

set(CMAKE_CXX_STANDARD 11)

# 使用当前conda环境的python解释器
find_package(PythonInterp)
find_package(PythonLibs)

# 导入eigen库
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIRS} )

find_package(pybind11)
pybind11_add_module(add_plus add_plus.cpp)
```

2. 调用函数`add_plus.py`（注意动态库和add_plus.py所在目录要一致）

```
$ python add_plus.py 
处理前：前三个元素分别为： 0   1   2
处理后：前三个元素分别为： 1   2   3
```
