# libertech_humanoid_description

## 命名
* 模块： xxx_macro.urdf
* 关节： xx_(right, left)_(yaw, pitch, roll)_joint
  * 手指关节 hand_(right, left)_fxx_joint

## 坐标系定义
* 见rviz2
* 之后导出时，尽量按照当前坐标系规则导出

## Note

* 注意velocity, effort 的限制，符合物理
* 每个关节的初始位置不要放在极限位置，否则会卡住，尤其是手
* ros2 contorller 的名字一定不要写错
* 运行单手 ignition的关节pub频率为760Hz mujoco为950+Hz
* 运行双手 ignition的关节pub频率为350Hz mujoco为700+Hz
* 运行双手+相机 ignition的关节pub频率更低一些
* 左右leg和waist使用一个contorller
* 尽量不依赖外部包， 目前依赖bota的力传感器功能包
* zed包依赖舍弃，使用粗略的位置确定zedm的中心位置
* realsense 雷达暂时没加入xacro

## TODO

* **肩关节与身体连接处存在问题**
*** 左右leg和waist 后续应该更改为effort controller**
* **由于不同部位使用了不同的ros_control命名空间，是否需要重新一起写？**