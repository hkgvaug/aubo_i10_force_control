# Contact-Rich Robotic Manipulation in Construction (Diffusion Policy + L1 Adaptive Control)

本项目基于普林斯顿大学学术论文 **《Contact-Rich Robotic Manipulation in Construction via Zero-Shot Learning: A Diffusion Policy-Guided Adaptive Control Framework》** 进行完整工程实现与仿真/实机落地开发。

专门针对建筑装配与智能建造领域中的**接触密集型机器人操作（Contact-Rich Robotic Manipulation）**任务（如木结构榫卯装配 Lap Joint / Mortise-and-Tenon、管件对接 Pipe Assembly 等高精度、大接触阻力、强卡阻风险的操作场景），构建了结合**多模态扩散策略（Diffusion Policy）**与**L1 启发式自适应力控（L1-Inspired Adaptive Control）**的双层控制架构。

---

## 目录
- [1. 项目背景与解决的核心问题](#1-项目背景与解决的核心问题)
- [2. 核心架构与设计思想](#2-核心架构与设计思想)
- [3. 模块结构与代码组织](#3-模块结构与代码组织)
- [4. 环境配置与依赖](#4-环境配置与依赖)
- [5. 快速上手指南](#5-快速上手指南)
  - [5.1 运行单元测试验证自适应控制器](#51-运行单元测试验证自适应控制器)
  - [5.2 采集装配仿真示范数据](#52-采集装配仿真示范数据)
  - [5.3 训练 1D CNN 扩散策略](#53-训练-1d-cnn-扩散策略)
  - [5.4 运行仿真对比基准与统计学评估](#54-运行仿真对比基准与统计学评估)
  - [5.5 闭环推理与实机流式伺服接口](#55-闭环推理与实机流式伺服接口)
- [6. 核心算法原理简析](#6-核心算法原理简析)
- [7. 实验基准与统计学评估方法](#7-实验基准与统计学评估方法)

---

## 1. 项目背景与解决的核心问题

在木结构建筑、预制构件装配及管道连接等建造施工场景中，传统工业机器人依赖离线轨迹规划或纯位置控制，面临以下关键挑战：
1. **接触卡滞与力过载（Contact Jamming & Force Exceedance）**：配合公差往往在毫米甚至亚毫米级，微小的定位或角度偏差会导致接触力激增，损坏工件或触发机械臂急停。
2. **多模态规划与死锁脱困（Multimodal Planning）**：传统强化学习或模仿学习策略容易陷入局部接触死锁，缺乏鲁棒的多模态探索与恢复动作。
3. **高控制延迟（Control Latency）**：传统 ROS/MoveIt 轨迹缓冲机制存在 600ms~1.2s 的通信规划滞后，无法应对瞬态接触反力。

**本项目的解决方案：**
- **上层（宏观规划）**：采用 **Diffusion Policy（扩散策略）**，利用 DDPM 条件去噪生成平滑且兼具多模态的多步动作轨迹序列；
- **底层（微观力控）**：引入 **L1 启发式自适应控制器**，以 10 Hz / 100 Hz 循环根据实时 6 维末端力/力矩信号动态估计接触扰动，主动施加阻尼衰减与反力自适应修正，彻底解决卡死和力过载问题；
- **底层硬件通信**：针对遨博（AUBO）等工业协作臂设计流式底层伺服插值接口，实现 $\le 20\text{ms}$ 低延迟闭环控制。

---

## 2. 核心架构与设计思想

系统采用**双层分层闭环架构**：

```
                +------------------------------------------------+
                |        9D 观测缓存 [Pos(3D) + Wrench(6D)]       |
                |               (观测时域 To = 2)                 |
                +-----------------------+------------------------+
                                        |
                                        v
                +------------------------------------------------+
                |     1D CNN Diffusion Policy (高层动作生成)      |
                |   - 条件向量: 18D (2 x 9D)                      |
                |   - 预测时域: Tp = 16 步动作轨迹                 |
                |   - 执行步长: Ta = 3 步输出                     |
                +-----------------------+------------------------+
                                        | u_pol (策略原始期望增量)
                                        v
+------------------+    +------------------------------------------------+
|  末端 6D 力传感器  | -> |    L1-Inspired Adaptive Controller (底层力控)  |
| (6-DoF F/T Sensor)|   |   - 状态预测器与误差估计 (State Predictor)      |
+------------------+    |   - 双曲正切力门控激活 (tanh Force Gating)      |
                        |   - 接触阻尼比例缩放 factor beta               |
                        |   - 一阶低通滤波与安全盒投影 (Box Clipping)    |
                        +-----------------------+------------------------+
                                                | u_cmd (自适应补偿后动作)
                                                v
                        +------------------------------------------------+
                        |   执行执行器 (PyBullet Sim / AUBO Real Robot)   |
                        |   - 10 Hz 决策 -> 100 Hz 样条插值驱动机械臂    |
                        +------------------------------------------------+
```

---

## 3. 模块结构与代码组织

```text
contact_rich_control/
├── __init__.py                     # 模块导出定义
├── l1_adaptive_controller.py       # L1 启发式自适应控制器（实现论文公式 Eqs. 3-14）
├── diffusion_policy.py             # 1D CNN 结构条件扩散策略网络（DDPM 去噪模型）
├── contact_assembly_closed_loop.py # 端到端闭环推理流水线（整合观测、策略、L1 与环境 step）
├── aubo_stream_servo_interface.py  # 遨博机械臂底层低延迟流式伺服驱动接口
├── pybullet_mortise_data_collector.py # PyBullet 仿真环境榫卯装配专家演示采集器
├── dataset_generator.py            # 轨迹数据生成与 HDF5 导出工具
├── train_diffusion_policy.py       # 扩散策略网络训练脚本（滑动窗口数据加载与优化）
├── test_l1_adaptive.py             # L1 自适应控制器的单元测试与动力学验证
├── eval_sim_benchmarks.py          # PyBullet 仿真环境基准评测对比（Baseline vs. Ours）
├── statistical_evaluator.py        # 学术级统计检验工具（Kruskal-Wallis、Mann-Whitney U、CLES）
├── checkpoints/                    # 预训练模型权重存放目录
│   └── diffusion_policy.pt         # 训练完成的扩散策略模型权重
└── data/                           # 示范与评测数据集存放目录
    ├── sample_dataset.h5           # 示例示范数据
    └── timber_mortise_5demos.h5    # 榫卯装配 5 组示范数据
```

---

## 4. 环境配置与依赖

核心依赖环境包括 Python 3.8+、PyTorch、PyBullet 及科学计算库：

```bash
pip install torch numpy scipy h5py pybullet
```

若需连接实际机械臂硬件，需配置对应的 C/C++ SDK（如 `libauborobotcontrol.so`）或机械臂通信驱动。

---

## 5. 快速上手指南

### 5.1 运行单元测试验证自适应控制器
验证自由空间运动透传、接触阻力阻尼响应、扰动估计与安全投影截断：
```bash
python test_l1_adaptive.py
```

### 5.2 采集装配仿真示范数据
在 PyBullet 榫卯仿真环境中，注入低频带限随机扰动并以 100 Hz 模拟物理碰撞，以 10 Hz 降采样导出 HDF5 数据集：
```bash
python pybullet_mortise_data_collector.py
```
生成的数据集将自动保存在 `data/` 目录下。

### 5.3 训练 1D CNN 扩散策略
从 HDF5 数据集中提取观测时域 $T_o=2$ 与预测时域 $T_p=16$ 的滑动窗口样本，训练条件扩散策略网络：
```bash
python train_diffusion_policy.py
```
训练的模型权重会自动输出到 `checkpoints/diffusion_policy.pt`。

### 5.4 运行仿真对比基准与统计学评估
执行 100+ 轮蒙特卡洛随机装配评测，对比纯扩散策略基线（Baseline）与结合自适应控制的新方法（Ours）：
```bash
python eval_sim_benchmarks.py
```
评测完成后，将自动输出峰值接触力统计（中位数、均值、IQR）、成功率、Mann-Whitney U 检验显著性 $p$ 值以及通用语言效应量（CLES）。

### 5.5 闭环推理与实机流式伺服接口
- **仿真/实机闭环调用**：参考 `contact_assembly_closed_loop.py` 中的 `ContactAssemblyPipeline`，通过传入策略模型与控制器配置即可执行 `run_rollout()`。
- **实机硬件驱动**：参考 `aubo_stream_servo_interface.py`，支持将 10 Hz 笛卡尔增量以 100 Hz 微插值并通过 CAN/底层 SDK 直接发送给协作机械臂伺服环。

---

## 6. 核心算法原理简析

### 6.1 扩散策略（Diffusion Policy）
- **观测向量**：包含末端笛卡尔位姿 $\mathbf{p} \in \mathbb{R}^3$ 与六维力矩 $\mathbf{F} \in \mathbb{R}^6$，合并为 9D 特征。取历史 $T_o=2$ 步展平为 18D 条件向量。
- **动作表征**：未来笛卡尔位移增量 $\mathbf{u} = [\Delta x, \Delta y, \Delta z]^T$。
- **去噪网络**：采用 1D 卷积残差块（Conv1D + Mish + FiLM 条件特征调制），在推理时支持多步反向 DDPM 采样（或 10 步加速采样）。

### 6.2 L1 启发式自适应控制律
底层控制器在每个时间步按以下步骤更新：
1. **状态预测器（State Predictor）**：
   $$\hat{\mathbf{p}}_{k+1} = \hat{\mathbf{p}}_k + a_{pred} (\hat{\mathbf{p}}_k - \mathbf{p}_k) \Delta t + \mathbf{u}^{cmd}_k \Delta t + \hat{\boldsymbol{\sigma}}_k \Delta t$$
2. **预测误差与扰动自适应（Adaptation Law with Force Gating）**：
   $$\mathbf{e}_k = \hat{\mathbf{p}}_k - \mathbf{p}_k$$
   $$\alpha_k = \frac{1}{2} \left[ 1 + \tanh\left( (\|\mathbf{F}_k\| - F_0) k_d \right) \right]$$
   $$\hat{\boldsymbol{\sigma}}_{k+1} = \lambda \hat{\boldsymbol{\sigma}}_k - \alpha_k \Gamma \mathbf{e}_k$$
3. **一阶低通滤波与力阻尼缩放**：
   $$\beta_k = \frac{1}{1 + k_d \|\mathbf{F}_k\|}$$
   $$\mathbf{u}^{cmd}_k = \text{Proj}_{\Delta_{max}} \left( \beta_k \mathbf{u}^{pol}_k - \hat{\boldsymbol{\sigma}}^{filt}_k \right)$$
   - 在自由空间中（$\|\mathbf{F}\| \approx 0$），$\alpha \approx 0, \beta \approx 1$，完全跟随扩散策略期望轨迹；
   - 在剧烈接触时（$\|\mathbf{F}\| > F_0$），$\beta < 1$ 迅速衰减速度防冲撞，自适应律补偿接触法向扰动，实现柔顺入榫。

---

## 7. 实验基准与统计学评估方法

项目集成了专为机器人顶会（如 ICRA / IROS / RSS）设计的统计评估套件（`statistical_evaluator.py`）：
- **Kruskal-Wallis 检验**：跨组检验接触力分布差异；
- **单侧 Mann-Whitney U 检验**：严格验证自适应方法是否显著降低接触峰值力（$p < 0.05$ 并结合 Bonferroni 校正）；
- **效应量量化**：
  - **CLES（Common Language Effect Size）**：衡量随机抽取一次装配时，自适应方法接触力小于基线策略的统计概率；
  - **Cohen's d**：衡量标准化均值差异大小。

---

## 8. 致谢与参考

- 理论架构参考文献：
  *Ibrahimov et al., "Contact-Rich Robotic Manipulation in Construction via Zero-Shot Learning: A Diffusion Policy-Guided Adaptive Control Framework", Princeton University.*
- 扩散策略参考：
  *Cheng Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion", RSS 2023.*
