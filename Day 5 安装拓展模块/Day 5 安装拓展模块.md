 

 

# 安装拓展模块库

- 为不用把动态库挪到调用文件的目录下才能被调用，可以将拓展模块进行安装

- 一般来说，下面的`setup.py`只需要改动`setup()`中的内容即可

  

## 代码

1. 在`Day 1 add`的文件基础上，在`CmakeLists`所在目录创建`setup.py`

```python
import os
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        # setup.py的绝对路径
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            # 检查是否已安装cmake
            out = subprocess.check_output(['cmake', '--version'])
            # 输出cmake版本
            print(str(out,encoding='utf-8'))
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        # 配置构建(build)类型
        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        # window 系统
        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]
            if sys.maxsize > 2 ** 32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        # linux系统
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())  # 获取下面的版本号

        # 创建build目录
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        # 执行终端命令cmake
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)


setup(
    name='add_project',
    version='0.0.1',
    author='Natsu-Akatsuki',
    author_email='@qq.com',
    description='install the add module',
    long_description='install the add module',
    ext_modules=[CMakeExtension('add')],  # 指定拓展模块的相关信息，如拓展模块名、相关依赖
    cmdclass=dict(build_ext=CMakeBuild),  # 并不意味着在命令行中执行build_ext才重载，隐式调用时也会重载
    zip_safe=False,
)
```



## 代码解构

1. 安装的代码

```python
setup(
    name='add_project',
    version='0.0.1',
    author='Natsu-Akatsuki',
    author_email='@qq.com',
    description='install the add module',
    long_description='install the add module',
    ext_modules=[CMakeExtension('add')],  # 指定拓展模块的相关信息，如拓展模块名、相关依赖
    cmdclass=dict(build_ext=CMakeBuild),  # 并不意味着在命令行中执行build_ext才重载，隐式调用时也会重载
    zip_safe=False,
)
```

对应：

```c++
setup(
    name='工程名',                # 这个对应pip list左边第一列看到的名字
    version='0.0.1',
    author='作者名',
    author_email='邮箱',
    description='工程描述',
    long_description='',
    ext_modules=[CMakeExtension('模块名')],  # 指定拓展模块的相关信息，如拓展模块名、相关依赖
    cmdclass=dict(build_ext=CMakeBuild),  # 并不意味着在命令行中执行build_ext才重载，隐式调用时也会重载
    zip_safe=False,
)
```

PS：

- 前面提到源程序和CMakeLists.txt的模块名需要保持一致（即需要PYBIND11_MODULE(xxx，)中的xxx与CMakeLists中 `pybind11_add_module` 中指定的target名字一致），此处`ext_modules=[CMakeExtension('add')]`可以另起名字



## 编译和运行代码

1. 编译和安装（此处采用develop的安装方式）

   ```bash
   $ python setup.py develop
   ```

2. 调用函数（然后就可以在任意路径调用这个模块）

```
$ python
>>> import add
>>> add.add(3,4) 
7
```

