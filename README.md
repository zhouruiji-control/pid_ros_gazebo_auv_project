# PID ROS2 Gazebo AUV Depth Control

基于 **ROS2 Humble + Gazebo + C++** 实现的 AUV（Autonomous Underwater Vehicle）定深 PID 控制与仿真项目。

本项目是我学习 ROS2、C++ 控制程序以及 Gazebo 仿真的一个阶段性实践。项目最初只是一个能够独立运行的 PID 定深程序，之后逐步加入 ROS2 通信、Gazebo 模型显示，并最终进一步实现了 **Gazebo 参与运动计算的受力闭环版本**。

整个项目目前保留两套实现：

* **普通版 / 展示版**：PID 与AUV 模型在 ROS2 节点内部完成运动计算，Gazebo 负责显示 AUV 的运动过程。
* **Gazebo 受力版**：PID 只计算控制量，通过 Gazebo 外力接口作用于 AUV，AUV 的实际深度再从 Gazebo 中读取并反馈给 PID，形成闭环。

对我而言，这个项目最重要的意义并不是单纯“让一个模型动起来”，而是通过亲自拆分控制器、被控对象、反馈和执行机构，逐渐理解一个控制算法如何从简单程序一步步接入 ROS2 和 Gazebo。

---

# 1. 项目目标

项目主要完成以下内容：

* C++ 实现 PID 定深控制器
* 建立简化的一维 AUV 垂向运动模型
* 加入测量噪声
* 加入低通滤波
* 加入积分限幅
* 加入控制输出限幅
* 加入控制量变化率限制
* 使用 ROS2 Topic 传递控制与状态信息
* 使用 CSV 保存实验数据
* 使用 Python 绘制实验结果
* 在 Gazebo 中建立简化 AUV 模型
* 实现 ROS2 深度与 Gazebo 位姿同步
* 实现 Gazebo 深度反馈
* 实现 Gazebo 垂向外力控制
* 实现 Gazebo 受力 PID 闭环

---

# 2. 开发环境

目前项目主要在以下环境下开发与测试：

```text
Windows + WSL2
Ubuntu 22.04
ROS2 Humble
Gazebo
C++
Python
CMake
colcon
VS Code
Git / GitHub
```

---

# 3. 项目整体思路

这个项目并不是一开始就直接建立 Gazebo 受力控制，而是按照由简单到复杂的顺序逐步实现。

整个过程可以概括为：

```text
C++ PID
    ↓
一维 AUV 模型
    ↓
ROS2 节点化
    ↓
Gazebo 模型显示
    ↓
Gazebo 位姿同步
    ↓
Gazebo 深度反馈
    ↓
Gazebo 外力控制
    ↓
Gazebo 受力闭环
```



---



# 4. PID 控制器

PID 控制器基本形式为：

```text
error = target_depth - current_depth
```

控制量由：

```text
P + I + D
```

组成。

除了基本 PID，我还逐步加入了几个实际工程中比较常见的处理。

## 6.1 积分限幅

防止误差长时间存在时积分项无限累积。

```text
Integral Windup
```

会导致控制输出过大，因此对积分量进行限制。

---

## 6.2 控制输出限幅

实际执行机构不可能输出无限大的推力，因此控制器输出需要限制在一定范围内。

---

## 6.3 输出变化率限制

即使最终输出没有超过最大值，如果相邻两个控制周期变化过大，也可能导致系统响应过于剧烈。

因此进一步限制控制量变化速度。

---

## 6.4 微分项

微分项主要用于反映误差变化趋势，并在后续 Gazebo 受力控制中帮助抑制振荡。

---

# 7. 测量噪声与低通滤波

为了不让仿真中的反馈信号过于理想，在普通版中加入了测量噪声。

因此程序中同时存在：

```text
真实深度
测量深度
滤波深度
```

基本过程为：

```text
真实深度
   ↓
加入测量噪声
   ↓
测量深度
   ↓
低通滤波
   ↓
滤波深度
```

这样可以观察：

```text
真实状态
测量状态
滤波后的状态
```

之间的差别。

这也是我第一次比较直观地认识到：

> 控制器获得的反馈值，在真实系统中通常并不会像理论仿真中那么理想。

---

# 8. ROS2 Topic

项目通过 ROS2 Topic 发布不同状态量。

项目过程中使用过的主要 Topic 包括：

```text
/auv/current_trueposition
/auv/measure_position
/auv/lvbo_posiition
/auv/velocity
/auv/newposition
/auv/output
/auv/error
/auv/p_term
/auv/i_term
/auv/d_term
```

不同版本中的实际使用情况可能有所区别。

这些 Topic 分别用于观察：

```text
当前深度
测量深度
滤波深度
垂向速度
更新后的深度
控制器输出
深度误差
PID 的 P 项
PID 的 I 项
PID 的 D 项
```

通过 ROS2 Topic 将这些变量独立发布之后，可以使用：

```bash
ros2 topic list
```

查看 Topic。

例如：

```bash
ros2 topic echo /auv/output
```

可以实时查看 PID 控制输出。

---

# 9. Gazebo 坐标系

项目中一个非常重要的小问题是：

```text
Gazebo z 轴：向上为正

AUV 深度：向下为正
```

因此二者之间需要转换：

```text
depth = -z
```

也就是：

```text
depth = 10 m
```

在 Gazebo 中对应：

```text
z = -10 m
```

这个符号问题虽然看起来很简单，但在后续外力控制中非常重要。

如果：

```text
误差方向
控制量方向
Gazebo 外力方向
```

任何一个符号设置错误，都可能出现：

```text
目标要求下潜
↓
控制器却让 AUV 上浮
↓
误差越来越大
↓
控制输出越来越大
↓
系统快速发散
```

因此在建立受力闭环之前，我专门进行了正负外力的开环测试。

---

# 10. 普通版：Gazebo 可视化闭环

第一套 Gazebo 方案中，并没有立即让 Gazebo 负责动力学计算。

其基本结构为：

```text
auv_pid_node
      ↓
一维 AUV 模型
      ↓
计算当前深度
      ↓
gazebo_pose_sync_node
      ↓
Gazebo AUV 位姿
```

也就是说：

> ROS2 中的一维模型负责计算运动，Gazebo 主要负责把运动结果显示出来。

这种方式虽然不能称为真正的 Gazebo 受力仿真，但它有一个很大的优点：

```text
结构简单
容易调试
控制结果稳定
能够快速验证 ROS2 与 Gazebo 的连接
```

这也是我选择先完成普通版，再继续做受力版的主要原因。

---

# 11. Gazebo 受力版

完成普通版之后，我开始思考一个问题：

> 如果 Gazebo 只是显示 ROS2 已经计算好的位置，那么 Gazebo 本身实际上并没有真正参与闭环运动计算。

因此下一步将系统修改成：

```text
PID Controller
      ↓
Control Output
      ↓
gazebo_force_node
      ↓
Gazebo AUV
      ↓
Gazebo Position
      ↓
force_depth_feedback_node
      ↓
Current Depth
      ↓
PID Controller
```

这样，Gazebo 不再只是一个“动画播放器”。

AUV 的运动状态由 Gazebo 更新，然后再反馈给控制器。

这也是普通版和受力版最主要的区别。

---

# 12. 为什么重新拆分 PID 节点

普通版中的：

```text
auv_pid_node.cpp
```

本身已经包含：

```text
PID
+
AUV 一维模型
```

也就是说，它会自己计算 AUV 下一时刻的位置。

但是进入 Gazebo 受力版之后，如果继续让它更新位置，会出现两个“被控对象”：

```text
C++ 一维模型
```

和

```text
Gazebo 模型
```

同时计算运动。

这会导致逻辑混乱。

因此受力版中增加：

```text
pid_controller_only_node.cpp
```

这个节点只保留：

```text
读取当前深度
      ↓
计算误差
      ↓
PID
      ↓
发布控制输出
```

不再负责更新 AUV 位置。

这一步让我逐渐认识到：

> **控制器和被控对象应该是两个不同的概念。**

普通版由于学习阶段的需要，把它们写在了同一个节点里。

而受力版则开始真正把两部分拆开。

---

# 13. 项目目录

当前项目主要结构如下：

```text
pid_ros_gazebo_auvcontrol/
│
├── src/
│   ├── auv_description/
│   ├── auv_gazebo/
│   └── auv_pid_ros/
│
├── experiment_figures/
│
├── auv_pid_result.csv
├── auv_pid_10m.csv
├── auv_pid_15m.csv
├── auv_pid_20m.csv
│
├── auv_pid_force.csv
├── auv_pid_force_depth.png
├── auv_pid_force_error.png
├── auv_pid_force_output.png
│
├── plot.py
├── plot_auv_pid_force.py
│
├── .gitignore
└── README.md
```

ROS2 编译后，本地还会生成：

```text
build/
install/
log/
```

这些属于编译生成文件，因此没有上传到 GitHub。

---

# 14. 各文件作用说明

## 14.1 `src/`

ROS2 工作空间最主要的源码目录。

真正需要长期维护的 ROS2 Package 都放在这里。

---

# 15. `src/auv_description/`

负责 AUV 模型描述。

其中主要包含 AUV 的：

```text
几何形状
尺寸
质量
惯性
collision
visual
link
```

等模型信息。

---

## `simple_auv.urdf.xacro`

项目中使用的简化 AUV 模型。

目前模型主要服务于控制算法和 Gazebo 仿真验证，并没有追求非常复杂的几何外形。

我在这个阶段更关注：

```text
模型能否正确加载
坐标系是否正确
质量与惯性是否合理
能否稳定参与控制
```

而不是模型外观是否足够逼真。

这是因为我的重点仍然是学习控制与 ROS2，而不是进行完整的水动力建模。

---

## `CMakeLists.txt`

负责：

```text
auv_description
```

这个 ROS2 Package 的编译和安装配置。

例如将：

```text
urdf/
meshes/
```

等文件安装到 ROS2 可以找到的位置。

---

## `package.xml`

描述：

```text
包名称
版本
依赖关系
构建工具
运行依赖
```

等 ROS2 Package 信息。

---

# 16. `src/auv_gazebo/`

这个包主要负责 Gazebo 环境与启动。

---

## `worlds/empty_underwater.world`

Gazebo 仿真世界文件。

主要负责定义：

```text
仿真世界
重力设置
仿真时间步长
环境
水面视觉效果等
```

当前项目主要研究垂向控制，因此 world 仍然是一个较为简化的水下环境。

它并不是完整的 CFD 或高精度海洋环境。

---

## `launch/simple_auv.launch.py`

用于：

```text
启动 Gazebo
加载 world
读取 AUV 模型
生成 simple_auv
```

它解决的问题是：

> 如何通过一条 ROS2 launch 命令，把 Gazebo 和 AUV 模型一起启动。

---

## `launch/gazebo_visual_depth_control.launch.py`

普通 Gazebo 展示版的启动文件。

主要负责将：

```text
Gazebo
AUV 模型
PID 节点
位姿同步节点
```

组合起来。

普通版运行逻辑为：

```text
PID
↓
一维模型
↓
深度
↓
Gazebo 位姿同步
```

---

## `launch/gazebo_force_depth_control.launch.py`

Gazebo 受力版主要启动文件。

主要负责启动：

```text
Gazebo
simple_auv
pid_controller_only_node
gazebo_force_node
force_depth_feedback_node
```

从而组成完整闭环：

```text
Gazebo Depth
      ↓
PID
      ↓
Force
      ↓
Gazebo
      ↓
Gazebo Depth
```

这个 launch 文件也是目前受力版本最核心的入口之一。

---

## `CMakeLists.txt`

用于安装：

```text
launch/
worlds/
```

等 Gazebo 相关资源。

---

## `package.xml`

记录 `auv_gazebo` 所依赖的 ROS2 和 Gazebo Package。

---

# 17. `src/auv_pid_ros/`

这个包是项目中主要的控制程序部分。

---

## `src/auv_pid_node.cpp`

最初的核心程序。

它承担了：

```text
PID 控制
AUV 一维模型
测量噪声
低通滤波
积分限幅
输出限幅
输出变化率限制
CSV 保存
ROS2 Topic 发布
```

等功能。

从软件工程角度来看，这个节点现在承担的功能比较多。

但对于项目早期阶段，它帮助我快速搭建了完整闭环，并确认 PID 算法能够正常工作。

因此我没有在项目一开始就过度拆分程序，而是：

```text
先实现
→ 先验证
→ 再拆分
```

---

## `src/pid_controller_only_node.cpp`

Gazebo 受力版使用的 PID 控制节点。

与：

```text
auv_pid_node.cpp
```

最大的区别是：

```text
它不再包含 AUV 一维运动模型。
```

它只负责：

```text
读取目标深度
读取当前 Gazebo 深度
计算误差
执行 PID
输出控制量
```

这样控制器和 Gazebo 中的 AUV 被控对象就真正分离开来。

---

## `src/force_depth_feedback_node.cpp`

Gazebo 受力版的深度反馈节点。

主要负责：

```text
读取 Gazebo 中 AUV 的位置
      ↓
获得 z 坐标
      ↓
depth = -z
      ↓
发布 AUV 当前深度
```

这个节点相当于受力闭环中的“传感器”。

没有它，PID 就无法知道 Gazebo 中的 AUV 实际运动到了哪里。

---

## `src/gazebo_force_node.cpp`

Gazebo 受力版中的执行节点。

主要功能：

```text
订阅 PID 控制输出
      ↓
进行控制量与力的转换
      ↓
形成 z 方向作用力
      ↓
通过 Gazebo 接口施加到 AUV
```

基本思想为：

```text
force_z = -force_scale × control_output
```

其中负号来自：

```text
AUV 深度向下为正
Gazebo z 向上为正
```

而：

```text
force_scale
```

负责控制 PID 输出转换成实际 Gazebo 外力后的大小。

在受力版调试过程中，`force_scale` 是一个非常重要的参数。

如果太大：

```text
AUV 运动过快
出现严重振荡
甚至直接飞走
```

如果太小：

```text
AUV 很难到达目标深度
响应非常缓慢
```

---

## `CMakeLists.txt`

负责：

```text
编译各个 C++ 节点
注册 executable
链接 ROS2 / Gazebo 依赖
install 可执行程序
```

当新增：

```text
pid_controller_only_node
gazebo_force_node
force_depth_feedback_node
```

时，都需要在这里增加对应的编译配置。

---

## `package.xml`

定义该 ROS2 包所使用的依赖。

随着 Gazebo 受力版的加入，这个文件也需要补充相应的 ROS2 / Gazebo 依赖。

---

# 18. 数据与绘图文件

## `auv_pid_result.csv`

普通 PID 仿真的基础实验数据。

用于记录运行过程中的：

```text
时间
深度
误差
PID 输出
以及相关状态量
```

具体字段以当前代码实际写入内容为准。

---

## `auv_pid_10m.csv`

普通 Gazebo 版本：

```text
目标深度 10 m
```

的实验结果。

---

## `auv_pid_15m.csv`

普通 Gazebo 版本：

```text
目标深度 15 m
```

的实验结果。

---

## `auv_pid_20m.csv`

普通 Gazebo 版本：

```text
目标深度 20 m
```

的实验结果。

保留多组目标深度实验的目的，是验证控制器不是只能够针对单一目标值工作。

---

## `auv_pid_force.csv`

Gazebo 受力版本实验数据。

与普通版最大的区别是：

> 数据中的 AUV 运动响应来自 Gazebo 参与运动计算后的闭环结果。

因此这个文件是当前受力版本比较重要的实验结果。

---

## `plot.py`

用于处理普通 PID / 普通 Gazebo 版本的 CSV 数据，并绘制实验曲线。

通过 Python 绘图比只观察终端输出更加直观，可以清楚观察：

```text
深度是否收敛
是否存在超调
误差如何变化
控制量是否振荡
```

---

## `plot_auv_pid_force.py`

专门用于 Gazebo 受力版本的数据处理和绘图。

读取：

```text
auv_pid_force.csv
```

并生成主要受力闭环实验图。

---




# 19. `experiment_figures/`

用于集中保存普通版的实验图片。

主要目的是避免大量实验图片全部堆放在项目根目录，同时方便 GitHub README 后续直接引用实验结果。

---


# 21. `README.md`

也就是当前文件。

主要用于记录：

```text
项目目的
项目结构
各文件用途
控制思路
运行方式
实验结果
开发过程
问题与思考
后续计划
```

我希望 README 不只是“如何运行程序”的说明，也能够记录整个项目从简单版本逐渐演化到受力版本的思考过程。

---

# 22. 普通版与受力版的区别

这是目前项目中我认为最重要的一部分。

## 普通版

```text
Target Depth
      ↓
PID Controller
      ↓
1D AUV Model
      ↓
Current Depth
      ↓
Gazebo Pose Sync
      ↓
Gazebo Visualization
```

这里：

```text
C++ 一维模型 = 被控对象
Gazebo = 可视化工具
```

---

## 受力版

```text
               ┌───────────────────────────┐
               │                           │
               ↓                           │
        Current Depth                      │
               ↓                           │
       PID Controller                      │
               ↓                           │
        Control Output                     │
               ↓                           │
       gazebo_force_node                   │
               ↓                           │
          Vertical Force                   │
               ↓                           │
          Gazebo AUV                       │
               ↓                           │
force_depth_feedback_node                  │
               │                           │
               └───────────────────────────┘
```

这里：

```text
PID Controller = 控制器

gazebo_force_node = 执行接口

Gazebo AUV = 被控对象

force_depth_feedback_node = 状态反馈
```

相比普通版，这样的结构让我对：

```text
Controller
Plant
Actuator
Feedback
```

之间的关系有了更加直观的理解。

---

# 23. 为什么同时保留两个版本

完成受力版之后，我并没有删除普通版。

因为它们代表了两个不同阶段。

普通版的优点：

```text
简单
稳定
容易理解
容易调试
适合观察 PID 本身
```

受力版的优点：

```text
模块更加清晰
Gazebo 真正进入闭环
更接近机器人仿真的实际结构
有利于继续扩展
```

因此我希望保留整个演进过程，而不仅仅留下最终版本。

---

# 24. 编译

进入工作空间：

```bash
cd ~/pid_ros_gazebo_auvcontrol
```

重新编译：

```bash
colcon build --symlink-install
```

加载环境：

```bash
source install/setup.bash
```

每次重新打开终端后，都需要重新：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

# 25. 运行 Gazebo

首先可以运行基础 Gazebo AUV 环境：

```bash
ros2 launch auv_gazebo simple_auv.launch.py
```

用于确认：

```text
Gazebo 能正常启动
AUV 能正常生成
world 能正常加载
```

---

# 26. 运行普通版

普通版的逻辑为：

```text
PID
+
一维 AUV 模型
+
Gazebo 位姿同步
```

启动后，可以通过 ROS2 Topic 查看：

```bash
ros2 topic list
```

以及：

```bash
ros2 topic echo /auv/output
```

```bash
ros2 topic echo /auv/error
```

观察控制器运行情况。

---

# 27. 运行 Gazebo 受力版

受力版本使用：

```bash
ros2 launch auv_gazebo gazebo_force_depth_control.launch.py
```

基本闭环为：

```text
Gazebo 深度
→ PID
→ 控制输出
→ Gazebo 外力
→ Gazebo 运动
→ Gazebo 深度
```

受力版本运行时，需要注意不要同时启动会直接修改 Gazebo 位姿的节点。

否则：

```text
一个节点施加外力
+
另一个节点直接修改位置
```

会造成两个运动逻辑互相冲突。

---

# 28. 实验结果

## 普通版

目前保存了多组不同目标深度实验：

```text
10 m
15 m
20 m
```

通过这些实验主要观察：

```text
不同目标值下是否都能够完成下潜
误差是否收敛
控制输出是否合理
是否出现明显持续振荡
```

---

## Gazebo 受力版

受力版实验主要保存：

```text
Depth Response
Error
Control Output
```

对应：

```text
auv_pid_force_depth.png
auv_pid_force_error.png
auv_pid_force_output.png
```

这些结果说明目前已经建立起：

```text
Gazebo 状态反馈
+
PID 控制
+
Gazebo 外力
```

组成的基本闭环。

---

# 29. 项目开发过程中遇到的问题

这个项目的完成过程并不是一次写完的，中间遇到了不少问题。

例如：

```text
Gazebo 坐标方向和深度方向相反
```

```text
外力太小时模型几乎不运动
```

```text
外力太大时模型运动过快
```

```text
加入阻尼后模型一度几乎不动
```

```text
PID 参数在简单一维模型中可以工作，
但接入 Gazebo 受力模型后需要重新调整
```

```text
展示版和受力版不能同时使用两套位置更新逻辑
```

这些问题让我逐渐意识到：

> 仿真能运行，并不代表控制系统的结构就是正确的。

很多问题并不是简单修改一个 PID 参数就可以解决，而是需要先检查：

```text
数据从哪里来
控制量去了哪里
谁在更新状态
坐标方向是否一致
控制器和被控对象有没有重复
```

---

# 30. 我在这个项目中的思考过程

回头看整个项目，我的思路发生了比较明显的变化。

最开始我的关注点是：

```text
代码能不能跑
PID 能不能收敛
```

随后变成：

```text
怎么把它放进 ROS2
```

再后来是：

```text
怎么让 Gazebo 里的 AUV 跟着它运动
```

但是做到普通 Gazebo 版之后，我开始意识到：

```text
Gazebo 只是跟着我的计算结果移动，
它本身并没有成为控制系统中的被控对象。
```

于是才继续做了受力版本。

到了受力版之后，我又发现：

```text
原来写在一个节点中的 PID 和 AUV 模型需要拆开。
```

最终逐渐形成：

```text
传感器反馈
→ 控制器
→ 执行机构
→ 被控对象
→ 传感器反馈
```

这样的思考方式。

对于有经验的机器人开发者来说，这可能只是非常基础的结构。

但对我而言，从“写一个 PID 程序”到真正理解为什么要把这些模块拆开，是这个项目带给我最重要的收获之一。

---

# 31. 心得体会

这个项目是我学习 ROS2、Gazebo 和机器人控制过程中完成的一个基础项目。

目前无论从控制理论、软件工程还是机器人仿真的角度来看，它都还有很多不成熟的地方。

我目前仍然处于学习阶段，因此这个项目并不是为了证明自己已经掌握了完整的 AUV 控制技术，而更像是对这一阶段学习过程的一次记录。

刚开始的时候，我更多是在关注：

```text
程序有没有报错
曲线能不能收敛
Gazebo 里的模型能不能动
```

随着项目逐渐推进，我开始慢慢意识到，仅仅“运行成功”并不是最终目的。

例如普通版虽然能够很好地展示 AUV 的下潜过程，但进一步分析之后会发现，Gazebo 其实并没有参与真实的状态更新。

正是这个问题促使我继续尝试受力版本。

受力版的实现过程也让我意识到，一个实际控制系统远比最初的一维 PID 程序复杂。

除了控制算法之外，还需要考虑：

```text
模型
坐标系
反馈
执行机构
控制周期
输出限制
系统阻尼
通信接口
软件结构
```

等很多问题。

现在回头看项目中的代码，仍然能够看到不少初学阶段留下来的痕迹。

例如部分变量名称还不够规范，一些功能最开始集中写在一个节点中，模型也进行了较大的简化。

但我并不打算刻意隐藏这些过程。

因为对我来说，这个仓库除了保存最终结果之外，也记录了自己从：

```text
一个简单的 PID 程序
```

逐渐走向：

```text
ROS2 节点化
Gazebo 可视化
Gazebo 状态反馈
Gazebo 受力闭环
```

的过程。

我知道，目前实现的仍然只是非常基础的一维定深控制，与真正的 AUV 控制系统还有很大的距离。

还有许多知识需要继续学习，例如：

```text
更完整的 AUV 动力学
水动力系数
浮力和重力
六自由度运动
姿态控制
轨迹跟踪
MPC
LQR
机械臂控制
传感器模型
更加规范的 ROS2 软件架构
真实机器人控制
```

因此，我更愿意把这个项目看作一个起点，而不是终点。

以后随着学习的深入，我希望能够不断回来看这个仓库，把现在比较简单的实现逐渐替换成更加规范、更加真实的模型和控制方法。

我也希望未来能够从：

```text
“让仿真运行起来”
```

逐渐走向：

```text
“理解为什么这样设计”
```

再进一步走向：

```text
“能够独立设计一个可靠的机器人控制系统”
```

目前还有很多不懂的地方，也还有很长的路需要学习。

但能够把最初的一个 PID 小程序一步一步做到现在的 ROS2 + Gazebo 受力闭环，对我来说已经是一个很有意义的学习过程。

希望以后再次回到这个仓库时，可以看到自己在控制理论、C++、ROS2、Gazebo 和机器人系统方面比现在走得更远。


---

# 34. 最后

这是一个学习中的项目。

代码并不完美，模型也并不复杂，其中很多实现还有进一步优化和规范的空间。

如果未来继续更新这个仓库，我希望始终保留一个原则：

> **先理解，再实现；先完成最小闭环，再逐步增加复杂度。**

感谢阅读。
