# CMakeLists

- 修改目标文件的文件名和输出路径

```cmake
set_target_properties(${PROJECT_NAME}_pyb PROPERTIES PREFIX "")               #  指定前缀
set_target_properties(${PROJECT_NAME}_pyb PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_pyb)   #  指定文件名
set_target_properties(${PROJECT_NAME}_pyb PROPERTIES SUFFIX ".so")            #  指定后缀
set_target_properties(${PROJECT_NAME}_pyb PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})  # 指定库的输出路径
set_target_properties(${PROJECT_NAME}_pyb PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})  # 指定可执行文件的输出路径

```

- 使用宏定义来实现一改全改，避免在`PYBIND11_MODULE(<导出的模块名>)`中，忘记修改模块名

```cmake
target_compile_definitions(${PROJECT_NAME}_pyb PRIVATE EXPORT_PYBIND11_MODULE_NAME=${PROJECT_NAME}_pyb)
```

