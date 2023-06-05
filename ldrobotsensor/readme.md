- [ROS功能包使用方法](#ROS功能包使用方法)
# ROS功能包使用方法
## 1. 系统设置
- 通过CP2102模块连接激光雷达，然后将CP2102模块的USB端接入你的Linux主机,然后给设备-x权限
- 以ssl20l为例,在launch/ssl20l.launch文件中修改`port_name`为接入的设备号,以`/dev/ttyUSB0`为例
```bash
$ sudo chmod 777 /dev/ttyUSB0
```
## 2. 编译方法
- ROS版本：
> 使用catkin编译，在readme文件所在目录下执行如下指令.
```bash
$ cd <sdk软件包根目录>
$ catkin_make
# 若出现缺少包依赖，通过如下指令安装依赖的包
$ rosdep install --from-paths src --ignore-src -r -y
```
## 3. 运行方法
- ROS版本：
```bash
$ cd <sdk软件包根目录>
$ source devel/setup.bash
# 启动SSL-20L设备节点
$ roslaunch ldlidar ssl20l.launch
# or
$ roslaunch ldlidar viewer_ssl20l_noetic.launch #  启动并显示数据在Rviz，适用于ubuntu20.04 noetic
$ roslaunch ldlidar viewer_ssl20l_kinetic_melodic.launch #  启动并显示数据在Rviz，适用于ubuntu16.04 kineitc, ubuntu18.04 melodic
# 启动SSL-20N设备节点
$ roslaunch ldlidar ssl20n.launch
# or
$ roslaunch ldlidar viewer_ssl20n_noetic.launch #  启动并显示数据在Rviz，适用于ubuntu20.04 noetic
$ roslaunch ldlidar viewer_ssl20n_kinetic_melodic.launch #  启动并显示数据在Rviz，适用于ubuntu16.04 kineitc, ubuntu18.04 melodic
# 启动SSL-20P设备节点
$ roslaunch ldlidar ssl20p.launch
# or
$ roslaunch ldlidar viewer_ssl20p_noetic.launch #  启动并显示数据在Rviz，适用于ubuntu20.04 noetic
$ roslaunch ldlidar viewer_ssl20p_kinetic_melodic.launch #  启动并显示数据在Rviz，适用于ubuntu16.04 kineitc, ubuntu18.04 melodic
# 启动SSL-07N设备节点
$ roslaunch ldlidar ld07n.launch
# or
$ roslaunch ldlidar viewer_ld07n_noetic.launch #  启动并显示数据在Rviz，适用于ubuntu20.04 noetic
$ roslaunch ldlidar viewer_ld07n_kinetic_melodic.launch #  启动并显示数据在Rviz，适用于ubuntu16.04 kineitc, ubuntu18.04 melodic
```

## 4. 测试
> 代码支持在ubuntun16.04、ubuntu18.04、ubuntu20.04 ROS版本下测试，使用rviz可视化
> rviz的配置在rviz文件夹下面。
- Fixed frame： base_laser
- Laser Scan topic name: /scan
