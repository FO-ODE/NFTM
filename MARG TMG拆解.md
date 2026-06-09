# MARG TMG拆解

状态: 进行中

## A. Contact-aware State Estimation

### 公式 (8)：状态 State

$$
\mathbf{x} \in SO(3) \times \R^{27}, \ \dim(\mathbf{x}) = 30
$$

$$
\mathbf{x} =
\left[
\mathbf{R}_{wb} \quad
\mathbf{p}_{wb} \quad
\mathbf{v}_{wb} \quad
\mathbf{b}_{a} \quad
\mathbf{b}_{\omega} \quad
\mathbf{p}_{f_1} \cdots \mathbf{p}_{f_4} \quad
\mathbf{g}
\right]
\tag{8}
$$

| 状态量 | 含义 | 所属空间 | 自由度 |
| --- | --- | --- | --- |
| $\mathbf{R}_{wb}$ | body frame 到 world frame 的旋转 | $SO(3)$ | 3 |
| $\mathbf{p}_{wb}, \mathbf{v}_{wb}$ | body 在 world 中的位置和速度 | $\mathbb{R}^3$ | 3 |
| $\mathbf{b}_a, \mathbf{b}_{\omega}$ | 加速度计和陀螺仪 bias | $\mathbb{R}^3$ | 3 |
| $\mathbf{p}_{f_i}$ | 第 $i$ 个接触足端点在 world 中的位置 | $\mathbb{R}^3$ | 3 |
| $g$ | 重力向量 | $\mathbb{R}^3$ | 3 |

相比普通 LiDAR-inertial odometry，额外把四个足端接触点位置  $\mathbf{p}_{f_i}$  加入状态，用于约束机身位置漂移，特别是高度漂移

---

### 公式 (9)：前向传播

$$
\bar{\mathbf{x}}_{t+1}
=
\bar{\mathbf{x}}_{t}
\boxplus
\left(
\mathbf{\Phi}(\bar{\mathbf{x}}_{t}, \mathbf{u}_{t}, \mathbf{0}) \Delta t
\right)
\tag{9}
$$

ESKF 的 propagation step, 上一时刻传播状态 $\bar{\mathbf{x}}_t$  通过系统动力学函数 $\mathbf{\Phi}$ 前向积分到 $\bar{\mathbf{x}}_{t+1}$

---

### 公式 (10)：前向传播函数 (State Transition Model)

$$
\Phi(\bar{\mathbf{x}}_{t}, \mathbf{u}_{t}, \mathbf{n}_{t})
=
\begin{bmatrix}
\boldsymbol{\omega}_{m_t} - \mathbf{b}_{\omega_t} - \mathbf{n}_{\omega_t} \\
\mathbf{v}_{wb_t} \\
\mathbf{R}_{wb_t}(\mathbf{a}_{m_t} - \mathbf{b}_{a_t} - \mathbf{n}_{a_t}) + \mathbf{g}_{t} \\
\mathbf{n}_{b_{a_t}} \\
\mathbf{n}_{b_{\omega_t}} \\
\mathbf{n}_{p_{f_i,t}} \\
\mathbf{0}_{3 \times 1}
\end{bmatrix}
\tag{10}
$$

这个函数定义了每个状态分量如何随时间变化：

第一行是姿态变化，由陀螺仪测量 $\boldsymbol{\omega}_{m_t}$ 减去 gyro bias 和噪声得到

第二行是位置变化率，即速度 $\mathbf{v}_{wb_t}$

第三行是速度变化率，由 IMU 加速度经旋转变换到 world frame 后再加上重力 $\mathbf{g}_t$

第四、五行是 IMU bias 的随机游走噪声

第六行是足端接触点位置噪声, 论文中特别说明，在 swing phase，由于足端没有稳定接触地面，$\mathbf{p}_{f_i}$ 的不确定性会变大，因此可以给它较大的过程噪声

最后一行表示重力向量在传播中保持不变

---

### 公式 (11)：误差状态线性化

$$
\tilde{\mathbf{x}}
=
\mathbf{x}
\boxminus
\bar{\mathbf{x}}
\approx
\mathbf{F}_{\tilde{\mathbf{x}}}\tilde{\mathbf{x}}
+
\mathbf{F}_{\mathbf{n}}\mathbf{n}
\tag{11}
$$

这是误差状态的线性化表达
真实状态 $\mathbf{x}$ 和传播状态  $\bar{\mathbf{x}}$ 之间的差定义为误差状态 $\tilde{\mathbf{x}}$
由于系统是非线性的，ESKF 需要在当前传播状态附近线性化

$\mathbf{F}_{\tilde{\mathbf{x}}}$ 是对误差状态的 Jacobian

$\mathbf{F}_{\mathbf{n}}$ 是对噪声的 Jacobian

---

## LiDAR Measurement

### 公式 (12)：LiDAR 点到平面测量残差 $\mathbf{h}_j(\mathbf{x}_t,{}^L\mathbf{p}_j + {}^L\mathbf{n}_j)$

$$
0
=
\mathbf{u}_{j}^{T}
\left(
{}^{G}\mathbf{T}_{I}
{}^{I}\mathbf{T}_{L}
\left(
{}^{L}\mathbf{p}_{j}
+
{}^{L}\mathbf{n}_{j}
\right)
-
{}^{G}\mathbf{q}_{j}
\right)
\tag{12}
$$

$\mathbf{u}_j$ 是对应平面的单位法向量

$^{I}\mathbf{T}_{L} = ({}^{I}\mathbf{R}_{L}, {}^{I}\mathbf{t}_{L})$ 是 LiDAR 到 IMU 的外参

${}^{G}\mathbf{T}_{I}$ 是 IMU 到 world 的变换， 假设 IMU frame 和 body frame 重合，因此其旋转部分等于 $\mathbf{R}_{wb}$

---

### 公式 (13)：LiDAR 测量线性化模型

公式 (12) 在线性化点 $\bar{\mathbf{x}}_t$ 附近的一阶近似：

$$
0
\simeq
\mathbf{h}_{j}(\bar{\mathbf{x}}_{t}, \mathbf{0})
+
\mathbf{H}_{j}\tilde{\mathbf{x}}_{t}
+
\mathbf{n}_{j}
\tag{13}
$$

它把非线性的 LiDAR 点到平面残差写成：

当前传播状态下的残差 $\mathbf{h}_{j}(\bar{\mathbf{x}}_{t}, \mathbf{0})$（线性化时，设LiDAR测量噪声为0）

加上 Jacobian 与误差状态的乘积 $\mathbf{H}_{j}\tilde{\mathbf{x}}_{t}$

再加测量噪声 $\mathbf{n}_{j} \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_j)$

---

### 公式 (14)：LiDAR 残差 Jacobian

$$
\mathbf{H}_{j}
=
\left.
\frac{
\partial \mathbf{h}_{j}(\bar{\mathbf{x}}_{t} \boxplus \tilde{\mathbf{x}}_{t}, \mathbf{0})
}{
\partial \tilde{\mathbf{x}}_{t}
}
\right|_{\tilde{\mathbf{x}}_{t}=\mathbf{0}}
= 0 
\\
=
\mathbf{u}_{j}^{T}
\left[
-\bar{\mathbf{R}}_{wb}
\left(
{}^{I}\mathbf{R}_{L}{}^{L}\mathbf{p}_{j}
+
{}^{I}\mathbf{t}_{L}
\right)^{\wedge}
\quad
\mathbf{I}_{3 \times 3}
\quad
\mathbf{0}_{3 \times 21}
\right]
\tag{14}
$$

这是 LiDAR 点到平面残差对误差状态的 Jacobian, 核心项是：

$$
-\bar{\mathbf{R}}_{wb}
\left(
{}^{I}\mathbf{R}_{L}{}^{L}\mathbf{p}_{j}
+
{}^{I}\mathbf{t}_{L}
\right)^{\wedge}
$$

它对应残差对姿态误差的偏导；$\mathbf{I}_{3\times3}$  对应残差对位置误差的偏导

$\mathbf{0}_{3\times21}$ 表示该 LiDAR 残差不直接约束速度、IMU bias、足端点、重力等后续状态分量

---

## Kinematics Measurement

### 公式 (15)：足端相对位置测量

$$
\mathbf{p}_{f_i^{rel}}
=
\mathbf{R}_{wb}^{T}
\cdot
(\mathbf{p}_{wb} - \mathbf{p}_{f_i})
\tag{15}
$$

足端接触点相对于 body frame 的位置。world 中 body 位置是 $\mathbf{p}_{wb}$，足端接触点位置是 $\mathbf{p}_{f_i}$，两者相减后再乘 $\mathbf{R}_{wb}^{T}$，就把这个向量转到 body frame 下

这个量可以由机器人运动学和关节编码器得到

---

### 公式 (16)：接触足端速度残差

$$
\mathbf{h}_{cv}
=
\mathbf{v}_{wb}
+
\mathbf{R}_{wb}
\cdot
\left(
\mathbf{v}_{f_i^{rel}}
+
(\boldsymbol{\omega}_{m} - \mathbf{b}_{\omega})
\times
\mathbf{p}_{f_i^{rel}}
\right)
\tag{16}
$$

contact velocity residual, 假设足端没有相对地面滑动，那么接触足端在 world frame 中的速度应该接近 0, 公式中：

$\mathbf{v}_{wb}$ 是 body 的 world 速度

$\mathbf{v}_{f_i^{rel}}$ 是由运动学得到的足端相对 body 的速度

$(\boldsymbol{\omega}_{m} - \mathbf{b}_{\omega}) \times \mathbf{p}_{f_i^{rel}}$ 是 body 旋转引起的足端速度项

如果机器人脚确实稳定踩在地面上，则这个残差应接近 0

如果漂移或滑动，残差会变大

---

### 公式 (17)：接触足端位置残差

$$
\mathbf{h}_{cp}
=
\mathbf{p}_{f_i^{rel}}
-
\mathbf{R}_{wb}^{T}
\cdot
(\mathbf{p}_{wb} - \mathbf{p}_{f_i})
\tag{17}
$$

解释：这是 contact position residual。它比较两种方式得到的足端相对位置：

$\mathbf{p}_{f_i^{rel}}$  由运动学测量得到

$\mathbf{R}_{wb}^{T}
\cdot
(\mathbf{p}_{wb} - \mathbf{p}_{f_i})$ 由状态估计中的 body 位置和足端 world 位置反推得到的

两者差值用于约束 body pose 和 foot contact point state

---

### 公式 (18)：接触足端速度残差 Jacobian

$$
\mathbf{H}_{cv}
=
\left.
\frac{
\partial \mathbf{h}_{cv}(\bar{\mathbf{x}}_{t} \boxplus \tilde{\mathbf{x}}_{t}, \mathbf{0})
}{
\partial \tilde{\mathbf{x}}_{t}
}
\right|_{\tilde{\mathbf{x}}_{t}=\mathbf{0}}
=
\\
\left[
-\bar{\mathbf{R}}_{wb}
\cdot
\left(
\mathbf{v}_{f_i^{rel}}
+
(\boldsymbol{\omega}_{m} - \bar{\mathbf{b}}_{\omega})
\wedge
\mathbf{p}_{f_i^{rel}}
\right)^{\wedge}
\;
\mathbf{0}_{3 \times 3}
\;
\mathbf{I}_{3 \times 3}
\;
\mathbf{0}_{3 \times 3}
\;
\bar{\mathbf{R}}_{wb}
\cdot
\mathbf{p}_{f_i^{rel}}^{\wedge}
\;
\mathbf{0}_{3 \times 12}
\right]
\tag{18}
$$

$\mathbf{H}_{cv}$  是 $\mathbf{h}_{cv}$ 对误差状态的 Jacobian

它把足端 “接触时速度应为零” 的约束线性化，用于 Kalman update

关键作用是：当足端被判定为接触时，足端运动学提供了 body 速度和姿态的约束，从而抑制状态估计漂移

---

### 公式 (19)：接触足端位置残差 Jacobian

$$
\mathbf{H}_{cp}
=
\left.
\frac{
\partial \mathbf{h}_{cp}(\bar{\mathbf{x}}_{t} \boxplus \tilde{\mathbf{x}}_{t}, \mathbf{0})
}{
\partial \tilde{\mathbf{x}}_{t}
}
\right|_{\tilde{\mathbf{x}}_{t}=\mathbf{0}}
\\
=
\left[
\bar{\mathbf{R}}_{wb}
\cdot
(\mathbf{p}_{f_i^{rel}})^{\wedge}
\quad
-\mathbf{I}_{3 \times 3}
\quad
\mathbf{0}_{3 \times 9}
\quad
\mathbf{I}_{3 \times 12}
\right]
\tag{19}
$$

这是 $\mathbf{h}_{cp}$ 对误差状态的 Jacobian

它主要约束 body 位置、姿态和足端接触点位置

直观理解：如果脚踩在地面上，足端在 world frame 中应该相对稳定，那么 body 的位置和姿态不能随便漂

---

## State Update

### 公式 (20)：MAP 优化目标

$$
\operatorname{minimize}_{\tilde{\mathbf{x}}_{t}}
\left(
\left\|
\mathbf{x}_{t}
\boxminus
\bar{\mathbf{x}}_{t}
\right\|_{\bar{\mathbf{P}}_{t}^{-1}}^{2}
+
\sum_{j=1}^{m}
\left\|
\mathbf{h}_{j}
+
\mathbf{H}_{j}\tilde{\mathbf{x}}_{t}
\right\|_{\mathbf{R}_{j}^{-1}}^{2}
\right.
\\
\left.
+
\left\|
\mathbf{h}_{cv}(\bar{\mathbf{x}}_{t}, \mathbf{u}_{t}, \mathbf{0}, \mathbf{0})
+
\mathbf{H}_{cv}\tilde{\mathbf{x}}_{t}
\right\|_{\boldsymbol{\Sigma}_{cv}^{-1}}^{2}
+
\left\|
\mathbf{h}_{cp}(\bar{\mathbf{x}}_{t}, \mathbf{u}_{t}, \mathbf{0})
+
\mathbf{H}_{cp}\tilde{\mathbf{x}}_{t}
\right\|_{\boldsymbol{\Sigma}_{cp}^{-1}}^{2}
\right)
\tag{20}
$$

这是状态更新阶段的 MAP estimation

它把多个误差项合在一个加权最小二乘问题中：

第一项是 prior，即当前状态不能偏离传播状态太多

第二项是所有 LiDAR 点到平面的残差

第三项是接触足端速度残差

第四项是接触足端位置残差

权重分别由协方差或噪声矩阵决定，例如 $\bar{\mathbf{P}}_{t}^{-1},\quad
\mathbf{R}_{j}^{-1},\quad
\boldsymbol{\Sigma}_{cv}^{-1},\quad
\boldsymbol{\Sigma}_{cp}^{-1}$

这些项的作用是把 LiDAR 几何约束和 leg kinematics contact constraint 一起用于状态修正

---

### 公式 (21)：相邻帧状态更新

$$
\mathring{\mathbf{x}}_{t}
=
\mathbf{\Phi}(\bar{\mathbf{x}}_{t}, \mathbf{u}_{t}, \mathbf{0})\Delta t
\boxplus
\mathbf{K}_{t}
\left[
\mathbf{h}_{1}
\cdots
\mathbf{h}_{m}
\quad
\mathbf{h}_{cv}
\quad
\mathbf{h}_{cp}
\right]^{T}
\tag{21}
$$

解释：通过 Kalman gain $\mathbf{K}_t$，把 LiDAR 残差、足端速度残差、足端位置残差一起用于更新状态，得到相邻帧间的局部状态估计 $\mathring{\mathbf{x}}_t$ 

这个局部状态之后用于 ego-centric elevation map 的更新，而不是强依赖一个长期全局一致的 pose

---

# B. Ego-centric Elevation Mapping

## Local Map Sliding

### 公式 (22)：全局索引更新

$$
\mathbf{g}_{i}
=
\mathring{\mathbf{R}}_{wb}
\mathbf{N}(\mathbf{g}_{i})
+
\mathring{\mathbf{p}}_{wb}
\oslash
\mathbf{r}
\tag{22}
$$

local map sliding 中 grid global index 的更新

论文中维护一个局部 elevation map，尺寸为：

$$
\mathbf{L} \in \mathbb{R}^{3}
$$

分辨率为 $\mathbf{r}$

并离散成 $N$  个 cell。每个 cell 有 global index：

$$
\mathbf{g}_{i}
=
(g_{i_x}, g_{i_y}, g_{i_z})
$$

公式中：

$$
\mathring{\mathbf{R}}_{wb}, \mathring{\mathbf{p}}_{wb}
$$

是由前面 state estimation 得到的相邻帧增量旋转和平移

$\oslash$ 是 element-wise division

它的作用是根据机器人局部运动更新地图网格索引，而不是每次重建整张地图

---

### 公式 (23)：局部索引归一化

$$
\mathbf{l}_{i}
=
\operatorname{normalize}(\mathbf{g}_{i}, \mathbf{L})
\tag{23}
$$

global index 更新后，需要映射回局部地图范围内的 local index：

$$
\mathbf{l}_{i}
=
(l_{i_x}, l_{i_y}, l_{i_z})
$$

这个 normalize 操作用于保证 cell 仍然落在 local elevation map 的尺寸 $\mathbf{L}$ 内。也就是通过 hash/local sliding 实现零拷贝或低开销的局部地图滚动

---

## Local Map Updating

### 公式 (24)：log-odds occupancy update

$$
\mathbf{C}_{pro|t}
=
\mathbf{C}_{pro|t-1}
+
n_{hit}
\log
\left(
\frac{
p_{hit}
}{
1 - p_{hit}
}
\right)
+
n_{miss}
\log
\left(
\frac{
p_{miss}
}{
1 - p_{miss}
}
\right)
\tag{24}
$$

这是 occupancy grid 的 log-odds 更新。论文先用 SOR 过滤点云噪声，然后维护 cached frame $\mathbf{C}$，通过 ray casting 记录每个 grid cell 的占据概率

$n_{hit}$ 表示该 cell 被 LiDAR ray 命中的次数

$n_{miss}$ 表示 ray 穿过但没有命中的次数

$p_{hit}, p_{miss}$ 是对应的占据概率参数

如果一个 cell 经常被 hit，它的 occupancy probability 会升高

如果经常被 miss，它的 occupancy probability 会降低

---

### 公式 (25)：occupancy probability 上下界截断

$$
\mathbf{C}_{pro|t}
=
\max
\left(
\min(\mathbf{C}_{pro|t}, T_{high}),
T_{low}
\right)
\tag{25}
$$

解释：这是对 occupancy probability 的 bounded update。通过上下阈值：

$$
T_{low}, \quad T_{high}
$$

限制 $\mathbf{C}_{pro|t}$ 的范围，避免概率无限累积。它的目的有两个：

第一，防止静态障碍被过度确信，导致动态变化无法及时反映

第二，增强对噪声、动态物体和 LiDAR beam divergence 的鲁棒性

---

## Priori Ray Interpolation 部分

这一小节在原文中没有给出编号公式，但有几个关键符号定义：

$\hat{\mathbf{h}}_{t}$ 表示最终提取出的 relative terrain map

$\hat{\mathbf{h}}(x_i, y_i)$ 表示 occupancy grid 在水平平面某一列中的最高 occupied voxel height

$\hat{\mathbf{h}}(x_f, y_f)$ 表示 forward traversal 中从距离 $d_{far}$  的最远 occupied cell 得到的 elevation value

$\hat{\mathbf{h}}(x_n, y_n)$ 表示 reverse traversal 中从距离 $d_{near}$ 的最近 occupied cell 得到的 elevation value

这一步的作用是补全 missing data 区域：正向遍历和反向遍历分别从远端、近端 occupied cell 给空列赋高程值，从而保持 terrain continuity，同时保留 gap、edge 这类关键结构

最后 $\hat{\mathbf{h}}_t$ 被送入 elevation net，生成 elevation feature $\mathbf{e}_{t}^{h}$

用于策略网络