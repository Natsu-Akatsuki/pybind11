# pybind 实例

- 本库结合任务，记录pybind封装时的实现细节
- 一般来说，通过**Day 1**的实例知道pybind的基本语法，通过**Day 2**和**Day 4**知道怎么沟通numpy数据（解决接口转换问题）便能解决大部分的封装任务（比如封装pcl点云库的函数）



# 知识点

## 入门

- Day 1：封装一个简单的c++函数来被python调用（pybind版"hello world"）
- Day 2：使用`py::array`对象来搭建c++与python numpy数据互通的桥梁
- Day 3：不同于前面生成的是拓展库，本例将生成c++可执行文件，以便于debug

- Day 4：使用`eigen`对象来搭建c++与python numpy数据互通的桥梁

- Day 5：联立`setup.py`和`CmakeLists`装c++拓展模块

- Day 6：异常捕获 / 异常抛出（不重要）

- Day 9：封装嵌套的结构体类型

# TODO

- [ ] 解读 type conversion
- [ ] 了解row-major和column-major不一致会带来的影响
- [ ] 了解&eigen 和 eigen::ref的区别