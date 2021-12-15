#  thermal_camera

此工程为MAGNITY热像仪的驱动代码，此说明文档为临时用，待补全

- 软件包结构

thermal_camera_ws/src中包含了写好的驱动软件包，而整层为驱动的工作空间

- 兼容性问题

其中的软件包中，的CMakeLists.txt将原本的set(CMAKE_CXX_FLAGS "-std=c++11")改成了c++14，OpenCV的版本也由3.1改为4.2，如果出现兼容性问题可以尝试修改。

### 修改日志