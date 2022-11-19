# Pybind11学习教程

本仓库提供使用`pybind11`封装C++代码的各种函数实现

## 知识点

### 入门

一般来说，通过**Day 1**的实例知道pybind的基本语法，通过**Day 2**和**Day 4**知道怎么沟通numpy数据（解决接口转换问题）便能解决大部分的封装任务（比如封装pcl点云库的函数）

- Day 1：封装一个简单的c++函数来被python调用（pybind版"hello world"）
- Day 2：使用`py::array`对象来搭建c++与python numpy数据互通的桥梁
- Day 3：不同于前面生成的是拓展库，本例将生成c++可执行文件，以便于debug

- Day 4：使用`eigen`对象来搭建c++与python numpy数据互通的桥梁

- Day 5：联立`setup.py`和`CmakeLists`装c++拓展模块

- Day 6：异常捕获 / 异常抛出（不重要）

- Day 9：封装嵌套的结构体类型

### 进阶

- 以下补充在实际应用下，涉及的功能需求

|                 功能                 |                           参考资料                           |
| :----------------------------------: | :----------------------------------------------------------: |
|           2.1 封装函数模板           | [官方文档](https://pybind11.readthedocs.io/en/stable/advanced/functions.html?highlight=template#binding-functions-with-template-parameters) |
| 2.2 设计更具功能性的`CMakeLists.txt` |                              —                               |

## TODO

- [ ] 解读 type conversion
- [ ] 了解row-major和column-major不一致会带来的影响
- [ ] 了解&eigen 和 eigen::ref的区别

## 规范

|            后缀             |       描述       |
| :-------------------------: | :--------------: |
|           <功能>            |  工程名、模板名  |
|         <功能>.cpp          |    c++文件名     |
| <功能>\_pyb.so / <功能>_pyb |    目标文件名    |
|         c_<功能>.py         | 对封装的上层封装 |

## Q&A

（1）`Cython`和`pybind`的区别？

* 相同点：都能生成Python能调用的拓展库，也能生成c++可执行文件
* 不同点：起点不同。`Cython`的起点是`Cython`：需要用基于`Cython`的语法编写`Cython`文件，再用`cython`来编译得到c++的源程序，继续进一步封装得到拓展库；`pybind`的起点则是c++，不需要中间文件（如`Cython`源文件），即能封装得到拓展库。

（2）`pybind`如何DEBUG

- 对于库文件而言，可以尝试先生成c++可执行文件来进行debug

（3）`undefined symbol`如何解决？

- 这个可能有很多种原因，一般都是动态库链接不成功，比如在`pybind11_add_module`生成目标文件后，没进行 `target_link_libraries`  
