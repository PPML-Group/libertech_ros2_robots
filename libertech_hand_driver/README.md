# freehand-controller

[![standard-readme compliant](https://img.shields.io/badge/standard--readme-OK-green.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)

Controller for FreeHand

FreeHand机械手使用说明文档

## Table of Contents

- [Install](#install)
- [Usage](#usage)
- [API](#api)

## Install

在Ubuntu22.04上经过了测试，需要安装

* **ROS2 (Humble / Rolling)**
* Python3
* numpy
* scipy
* kso
* yaml
* rich

关于ros2安装：可以使用鱼香ros安装,版本为rolling
```
wget http://fishros.com/install -O fishros && . fishros 
```
或者找到fishros下载的文件，rename为fishros.sh，然后执行
```
sh ./fishros.sh 
```
然后编译
```
colcon build --symlink-install
```
注意：
* `--symlink-install`对python文件创建链接，更改python节点无需重新编译
* 若在conda创建的虚拟环境中运行，需要在对应环境下编译，此时需要在虚拟环境下安装colcon-common-extensions
```
pip3 install colcon-common-extensions
```
* `source ./install/local_setup.zsh`或者`source ./install/local_setup.zsh`

## Usage

* 基于ROS2 Rviz2的显示

```
ros2 launch freehand_description display.launch.py
```

* 基于ROS2 rqt的拖动控制

```
ros2 launch freehand_controller freehand_ros_driver_gui.launch.py
```

* freehand_controller功能包中的子文件夹freehand_controller基于hand_sdk更改

## API

所有接口都写在`hand_control.py`，通过构造`HandApi`类进行调用，**运动学的部分函数还在更新中**

**调用接口示例**

* 启动线程，运行手的异步函数run()
```python
class HandDriver(Node):
  def __init__(self):
    super().__init__('freehand_ros_driver')
      self.hand = HandAPI(hand_path, actions_path=actions_path)
      self.thread_ = threading.Thread(target=self.run_loop, daemon=True)
      self.thread_.start()

def run_loop(self):
    asyncio.run(self.hand.run())
```
* 然后在主线程中，将命令添加到队列
`self.hand.add_queue(q, finger)`

* 每个手指都有一个队列，异步完成动作的发送

**运动学部分**

参考`HandModelKinematics`

* 四杆机构关系，解算耦合关节
  * `four_bar_forward(q, A, B, C, D, degree1_init, degree2_init)`
  * `four_bar_backward(q, A, B, C, D, degree1_init, degree2_init)`
  * `four_bar_pip2dip(q,name)`:`name`是`index`，`middle`，`fourth`，`little`，`thumb`
  * `four_bar_dip2pip(q,name)`
  * `four_bar_mcp2pip(q,name)`:用于无名指和小拇指
  * `four_bar_pip2mcp(q,name)`:用于无名指和小拇指

* 计算关节空间与工作空间的运动学映射
  * `ik_e2q(pos: np.ndarray, fg_name: str)` 使用sko库的粒子群算法
  * `fk_q2e(q: np.ndarray, fg_name: str)` 使用roboticstoolbox
* 计算驱动空间与关节空间的运动学映射
  * `ik_q2d(q: np.ndarray, fg_name: str)`
  * TODO: `fk_q2d()` 

## Note

**机械手关节转向定义**
* 参考URDF
* 垂直于掌心向里看，右手朝右旋转为正，朝左旋转为负；左手朝左旋转为正，朝右旋转为负

**手指与电机对应关系**
```python
finger_mapping = {
    "index": 1,
    "middle": 2,
    "fourth": 3,
    "little": 4,
    "thumb": 5,
} # 手指顺序
finger_driver_num = {
    "index": 3,
    "middle": 3,
    "fourth": 2,
    "little": 2,
    "thumb": 3,
} # 每个手指的主动自由度
finger_joint_index = {
    "index": 0,
    "middle": 3,
    "fourth": 6,
    "little": 8,
    "thumb": 10,
} # 手指的关节序号
motor_mapping = {
    "index" : [9,8,7],
    "middle": [5,10,6],
    "fourth": [2,4],
    "little": [1,3],
    "thumb" : [0,11,12],
    "all"   : [9,8,7,5,10,6,2,4,1,3,0,11,12],
} # 每个手指对应的电机id
```
* **用于判断接收到的关节反馈属于哪个电机/手指**
* 手指从食指开始编号，这由URDF文件决定，大拇指为4号（从0开始）
* 电机顺序与手指关节顺序不对应

**库函数**
* hand_cfg.py:运动学参数
* hand_model_cal.py:无名指、小拇指逆运动学函数
* ExtCommProtocol.py:通信数据结构定义

## BUG

* 左手不能超过1000的力
* 左右手由于装配不同，kp不一样
* 轨迹规划还可以优化

## Change Log

### version:0.1.1

*October, 2022*

初始版

### version:0.1.2

*December, 2022*

* 使用线程函数进行发送与接收
* 完善手指逆运动学
### version:0.2.1

*July, 2023*

* 适配第三代手

### version:0.3.0

*November, 2023*

* 3.5代手，异步函数