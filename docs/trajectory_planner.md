# 轨迹规划模块说明

本文档梳理 `legged_interface` 中脚端轨迹规划（`SwingTrajectoryPlanner`）的输入、生成流程、数学形式以及在 MPC/WBC 中的使用方式，便于调参和二次开发。

## 1. 模块位置与依赖
- 主要源码：`legged_interface/src/foot_planner/SwingTrajectoryPlanner.cpp`、头文件 `include/legged_interface/foot_planner/SwingTrajectoryPlanner.h`。
- 运行入口：`LeggedInterface::setupReferenceManager()` 创建 `SwingTrajectoryPlanner` 并交给 `SwitchedModelReferenceManager`。
- 配置来源：`task.info` 的 `swing_trajectory_config` 段（抬脚高度、时间缩放、偏置等）、`reference.info`（默认关节、COM 高度）以及 `gait.info`（接触模式）。

## 2. 输入数据
| 输入 | 来源 | 说明 |
| --- | --- | --- |
| `ModeSchedule` | gait scheduler | 定义每条腿在各时间段的接触模式和事件时间。 |
| `TargetTrajectories` | OCS2 MPC 参考 | 提供机身位置/姿态/速度等“期望”量，用于预测落脚点。 |
| `current_feet_position` | 状态估计 | 当前真实的脚端位置。 |
| `body_vel_world` | 状态估计 | 世界系下机身线速度/角速度。 |
| `body_vel_cmd` | 用户指令 | 期望机身线速度/角速度，影响落脚点偏置。 |
| `feet_bias_*` | 配置 | 线脚建模的两个虚拟接触点偏置（前端用 `feet_bias_x1`，后端用 `feet_bias_x2`，左右腿配合 `±feet_bias_y` 展开，`feet_bias_z` 为名义高度）。 |

## 3. 轨迹生成流程
1. **接触序列展开**：`extractContactFlags()` 将 `ModeSchedule` 转成每条腿的接触布尔序列；`updateFootSchedule()` 找到每段摆动/支撑的起止事件索引。
2. **支撑期更新落脚点**：线脚每条腿包含“前/后”两个虚拟接触点，分别带有自己的偏置（`bias = [feet_bias_x1, ±feet_bias_y, feet_bias_z]` 或 `bias = [feet_bias_x2, ±feet_bias_y, feet_bias_z]`）。若当前时刻 `initTime` 尚未到达轨迹段的落地时间，调用 `calNextFootPos()` 计算两个点各自的下一支撑位 `next_stance_position`：

   $$
   \begin{aligned}
   p_{\text{next}} &= p_{\text{body}}(t_0) + p_{\text{shoulder}} + p_{\text{symmetry}} + p_{\text{centrifugal}}, \\
   p_{\text{shoulder}} &= (t_f - t_0)\,(0.5\,v_{\text{body}} + 0.5\,v_{\text{cmd}}) + R(\boldsymbol{\theta}_{\text{mid}})\,\text{bias}, \\
   p_{\text{symmetry}} &= (t_{\text{mid}} - t_f)\,v_{\text{body}} + k\left(v_{\text{body}} - v_{\text{cmd}}\right),\quad k = 0.03, \\
   p_{\text{centrifugal}} &= 0.5\,\sqrt{\tfrac{z_{\text{body}}}{9.81}}\;\left(v_{\text{body}} \times \omega_{\text{cmd}}\right).
   \end{aligned}
   $$

   其中 bias 由 `feet_bias_x1/x2`, `feet_bias_y`, `feet_bias_z` 组合，$R(\cdot)$ 为机身姿态旋转矩阵，$t_{\text{mid}}$ 为下一支撑区间中点时间。
3. **生成样条**：
   - **摆动段**调用 `genSwingTrajs()`：为 X/Y/Z 分别构建三/四节点的 cubic spline，并缓存到 `feetX/Y/ZTrajs_`。
   - **支撑段**生成常值样条（位置保持 `next_stance_position`）。
4. **缓存给下游**：轨迹和起止时间写入 `BufferedValue`，供 MPC/WBC/可视化异步读取。

## 4. XYZ 轨迹公式
### 4.1 X/Y 水平轨迹
使用 3 个节点的三次 Hermite 样条，保证起落速度为 0，中点速度与位移成比例：
```
Node0 = { t_s, start_xy, 0 }
Node1 = { t_m, (1-ℓ1) start_xy + ℓ1 stop_xy, k1 (stop_xy - start_xy)/(t_f - t_s) }
Node2 = { t_f, stop_xy, 0 }
```
- 常量：`a1 = 0.417 → t_m = (1-a1) t_s + a1 t_f`，`ℓ1 = 0.650`，`k1 = 1.770`。
- 轨迹呈平滑 S 形，加速度由样条自动保证连续，对长摆动会产生更大水平速度。

### 4.2 Z 竖直轨迹
1. 抬脚峰值：`max_z = max(start_z, stop_z) + scaling * swingHeight`，其中 `scaling = min(1, (t_f - t_s)/swingTimeScale)`，确保短时间摆动自动降低摆高。
2. 采用 4 节点样条：
```
Node0 = { t_s, start_z, 0 }
Node1 = { t_s + a1 Δt, ℓ1 max_z, k1 ℓ1 (max_z - start_z)/(a1 Δt) }
Node2 = { t_s + a2 Δt, ℓ2 max_z + (1-ℓ2) stop_z, k2 ℓ2 (stop_z - max_z)/((1-a2) Δt) }
Node3 = { t_f, stop_z, k3 ℓ2 (stop_z - max_z)/((1-a2) Δt) }
```
- `Δt = t_f - t_s`，常量 `a1=0.251, ℓ1=0.749, k1=1.338, a2=0.630, ℓ2=0.570, k2=1.633, k3=0`。
- Node1/2 控制上抛与下摆的斜率，`k3=0` 让落地速度近似 0，可与 `touchDownVelocity` 协同避免硬碰撞。

### 4.3 轨迹查询
- `getX/Y/ZpositionConstraint(leg, t)` 和 `getX/Y/ZvelocityConstraint` 根据时间在 `feetTrajsEvents_` 中查段索引，再调用样条 `position()`、`velocity()`。
- `threadSaftyGetPosVel()` 一次性输出多个采样点的 `(pos, vel)`，供可视化与 MPC 约束使用。

## 5. 参数对轨迹的影响
| 参数 | 作用 |
| --- | --- |
| `swingHeight` | 抬脚峰值，配合 `swingTimeScale` 对短摆动做比例缩放。 |
| `swingTimeScale` | 小于该时间的摆动会线性缩短摆高，防止“时间短却抬太高”。 |
| `feet_bias_x1/x2/y/z` | 不同腿的名义落脚位置偏置，决定 `stop_pos` 的基本形状。 |
| `next_position_z` | 落脚高度（支撑段 Z 值）。 |
| `liftOffVelocity`/`touchDownVelocity` | 可用于调整样条导数，减少抬脚或落脚冲击（当前实现通过常数 `k*` 体现，如需细调可替换常数）。 |
| `body_vel_cmd` | 期望机身速度，影响 `p_shoulder`、`p_symmetry`，等价于“命令的行走方向/速度”。 |

## 6. 下游模块如何使用
- **MPC**：`SwitchedModelReferenceManager` 将脚端位置/速度参考喂给 `XYReferenceConstraintCppAd`、`ZeroVelocityConstraintCppAd`、`NormalVelocityConstraintCppAd` 等，使最优控制计算时遵循预期摆动轨迹。
- **WBC**：`WbcBase::formulateSwingLegTask()` 读取 `posDesired`、`velDesired`，以 PD 形式生成摆动腿末端加速度任务，从而在实物控制中追踪样条。
- **状态估计/可视化**：`threadSaftyGetStartStopTime()` 提供每条腿的起落时刻，用于提前检测触地或绘制轨迹。

## 7. 调参建议
1. 先在 `task.info` 调整 `swingHeight`、`swingTimeScale`、`feet_bias_*`，确保规划落脚点合理再进入 MPC/WBC。
2. 若脚端落地偏前后，可改 `feet_bias_x1/x2`；偏左右则调 `feet_bias_y`。
3. 若短时间步态仍抬脚过高，可适当减小 `swingTimeScale`（缩短到 0.1~0.12）。
4. 需要更“对称”或“偏后”轨迹，可修改 `genSwingTrajs()` 中的常量 `a1, ℓ1, k1, z_*`，但需保持上升/下降段斜率连续。

## 8. 参考
- `ocs2_legged_robot` 官方示例中的 `SwingTrajectoryPlanner`（同源）
- 论文 *Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control*（`calNextFootPos` 注释引用）
