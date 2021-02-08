# pybind 实例

本库结合任务，记录pybind封装时的具体实现



# 知识点

- Day 1：封装一个简单的c++函数来被python调用（pybind版"hello world"）
- Day 2：使用`py::array`对象来搭建c++与python numpy数据互通的桥梁
- Day 3：不同于前面生成的是拓展库，本例将生成c++可执行文件，以便于debug

- [ ] Day 4：使用`eigen`对象来搭建c++与python numpy数据互通的桥梁

- Day 5：联立`setup.py`和`CmakeLists`安装c++拓展模块

- [ ] Day 6：异常捕获 / 异常抛出
- [ ] Day 7：一些线索 / 知识点 / DEBUG