# pyro_wl_power_ctrl — 轮腿底盘功率控制模块

## 1. 功能概述

本模块对双轮腿机器人的两个轮电机进行**逐电机独立功率预测与限制**，核心设计目标：

1. **平衡力矩保护**：维持机体俯仰平衡与腿长控制的力矩（`tau_balance`）永远不被限制，防止机器人摔倒。
2. **运动力矩限制**：速度前进与偏航转向合并为 `tau_motion`，超功率时按比例缩减。
3. **逐电机独立参数**：两个电机分别配置 k1/k2/k3，适配电机个体差异。
4. **能量缓冲环**（可选）：通过 PD 控制动态调整功率上限，充分利用超级电容缓冲能量。

---

## 2. 功率模型

对电机 $i$，预测功率为：

$$
P_i = \tau_i \cdot \omega_i + k_1^{(i)} |\omega_i| + k_2^{(i)} \tau_i^2 + k_3^{(i)}
$$

| 项 | 含义 |
|---|---|
| $\tau_i \cdot \omega_i$ | 机械输出功率（含反电动势） |
| $k_1^{(i)} \|\omega_i\|$ | 速度相关损耗：铁损、摩擦（W·s/rad） |
| $k_2^{(i)} \tau_i^2$ | 铜损（电流²×电阻，W/N·m²） |
| $k_3^{(i)}$ | 静态损耗（驱动板、控制器等，W） |

> **参数标定**：在不同 (τ, ω) 工作点下实测功率，用最小二乘回归拟合 k1/k2/k3。

---

## 3. 力矩分解

```
tau_total_i  =  tau_motion_i  +  tau_balance_i
```

| 分量 | 含义 | 是否限制 |
|---|---|---|
| `tau_balance` | Upitch（俯仰 LQR）+ Uleg（腿长 LQR） | **永不限制** |
| `tau_motion`  | Uspeed（速度 PID）± Uyaw（偏航 PID） | 超功率时缩减 |

---

## 4. 限制算法

### 4.1 总流程

```
每控制周期（1 ms）：

① update_energy_loop(P_referee, E_feedback, E_target)
      → 根据电容能量状态动态调整 _configured_max_power

② update(cmd[2], tau_out[2])
      → Step 1：逐电机预测总指令功率 P_cmd_i
      → Step 2：sum(P_cmd_i) ≤ P_max → 直接输出，结束
      → Step 3：按比例分配预算  P_alloc_i = P_max * P_cmd_i / sum(P_cmd_j)
      → Step 4：逐电机求解最大允许力矩（二次方程），叠加 tau_balance
```

### 4.2 能量环（PD 控制器）

$$
P_{\max} = P_{\text{referee}} - \left( K_p \cdot e + K_d \cdot \Delta e \right)
$$

$$
e = E_{\text{target}} - E_{\text{feedback}}
$$

- 当电容能量低于目标时，$e > 0$，输出 $P_{\max}$ 下调，减少放电。
- 当电容能量高于目标时，$e < 0$，输出 $P_{\max}$ 上调（上限：$P_{\text{referee}} + \text{cap\_max\_bonus}$），加速消耗冗余能量。
- 最终结果被钳制到 $[\text{min\_max\_power},\ P_{\text{referee}} + \text{cap\_max\_bonus}]$。

### 4.3 逐电机二次方程求解

为电机 $i$ 分配了预算 $P_{\text{alloc},i}$ 后，求解满足功率预算的最大总力矩 $x$：

$$
k_2^{(i)} x^2 + \omega_i x + \left( k_1^{(i)} |\omega_i| + k_3^{(i)} - P_{\text{alloc},i} \right) = 0
$$

设 $A = k_2^{(i)}$，$B = \omega_i$，$C = k_1^{(i)}|\omega_i| + k_3^{(i)} - P_{\text{alloc},i}$，$\Delta = B^2 - 4AC$。

**根选取规则：**

| 情形 | 处理 |
|---|---|
| $k_2 \approx 0$（线性退化） | $x = -C / \omega$，符号不匹配则取 0 |
| $\Delta < 0$（无实根） | 取抛物线顶点 $-B/(2A)$（功率最低点） |
| $\Delta \approx 0$（重根） | 单根，符号不匹配则取 0 |
| 两根，符号各异 | 取与指令同号的根 |
| 两根，符号相同 | 取绝对值较大的根（限制量最小） |
| 两根均异号 | 取顶点 $-B/(2A)$ |

### 4.4 平衡保护

若 $P_{\text{alloc},i} \le P(\tau_{\text{balance},i},\ \omega_i)$，说明仅平衡力矩就已耗尽预算，直接令 `tau_out[i] = tau_balance[i]`（运动力矩归零）。

---

## 5. API 说明

### `wl_power_ctrl_cfg_t`（配置结构体）

| 字段 | 类型 | 说明 |
|---|---|---|
| `k1[2]` | float | 速度损耗系数，右=0，左=1 |
| `k2[2]` | float | 铜损系数 |
| `k3[2]` | float | 静态损耗 |
| `energy_kp` | float | 能量环比例增益 |
| `energy_kd` | float | 能量环微分增益 |
| `min_max_power` | float | 功率上限下界（安全底线，W） |
| `cap_max_bonus` | float | 电容可额外提供的功率（W） |

### `wl_wheel_cmd_t`（每轮指令）

| 字段 | 说明 |
|---|---|
| `tau_motion`  | 速度+偏航合并力矩，可被限制 |
| `tau_balance` | 平衡力矩，永不限制 |
| `omega`       | 轮转速（rad/s），右轮正号，左轮取反 |

### 公有方法

| 方法 | 说明 |
|---|---|
| `init(cfg)` | 初始化，必须首先调用 |
| `update_energy_loop(P_ref, E_fb, E_tgt)` | 更新功率上限（每周期调用） |
| `set_max_power(P)` | 直接设置功率上限（绕过能量环） |
| `update(cmd, tau_out)` | 核心限制步骤，输出受限力矩 |
| `predict_power(i, tau, omega)` | 预测单电机功率（可用于监控/调试） |
| `get_cmd_power()` | 获取本周期总指令功率 |
| `get_restricted_power()` | 获取限制后总实际功率 |
| `get_configured_max_power()` | 获取当前生效功率上限 |

---

## 6. 使用示例

以下示例展示在底盘 1 ms 控制任务中的典型集成方式。

### 6.1 初始化

```cpp
#include "pyro_wl_power_ctrl.h"

static pyro::wl_power_ctrl_t s_power_ctrl;

void chassis_init()
{
    pyro::wl_power_ctrl_cfg_t cfg;

    // 右轮电机参数（实测标定）
    cfg.k1[0] = 0.10f;   // W·s/rad
    cfg.k1[1] = 0.11f;   // 左轮略有差异

    cfg.k2[0] = 0.012f;  // W/(N·m)²
    cfg.k2[1] = 0.013f;

    cfg.k3[0] = 2.5f;    // W（静态）
    cfg.k3[1] = 2.5f;

    cfg.energy_kp    = 1.5f;
    cfg.energy_kd    = 0.1f;
    cfg.min_max_power = 5.0f;
    cfg.cap_max_bonus = 20.0f;  // 超级电容最多额外提供 20 W

    CHECK_PYRO_RET(s_power_ctrl.init(&cfg));
}
```

### 6.2 控制周期（1 ms 任务）

```cpp
// 索引约定
constexpr uint8_t R = 0;
constexpr uint8_t L = 1;

void chassis_control_task()
{
    // ① 获取 LQR 输出（来自 wl_chassis_t）
    float T_w[2];  // 右轮总力矩，左轮总力矩
    float tau_balance[2];

    // tau_balance = 俯仰/腿长 LQR 项之和（需调用者自行计算）
    // 示例：只含 pitch 稳定项
    tau_balance[R] = lqr_gain_r[4] * (beta_ref - beta)
                   + lqr_gain_r[5] * (0.0f - d_beta);
    tau_balance[L] = lqr_gain_l[4] * (beta_ref - beta)
                   + lqr_gain_l[5] * (0.0f - d_beta);

    // ② 填写功率控制器输入
    pyro::wl_wheel_cmd_t cmd[2];

    cmd[R].tau_balance = tau_balance[R];
    cmd[R].tau_motion  = T_w[R] - tau_balance[R];   // 剩余 = 速度 + 偏航分量
    cmd[R].omega       = +wheel_drv[R]->get_current_rotate();

    cmd[L].tau_balance = tau_balance[L];
    cmd[L].tau_motion  = T_w[L] - tau_balance[L];
    cmd[L].omega       = -wheel_drv[L]->get_current_rotate();  // 左轮取反

    // ③ 更新能量环（使用裁判系统功率与电容能量）
    s_power_ctrl.update_energy_loop(referee_power_limit,
                                    cap_energy_feedback,
                                    cap_energy_target);

    // ④ 功率限制
    float tau_out[2];
    s_power_ctrl.update(cmd, tau_out);

    // ⑤ 发送到电机（注意底盘侧左轮符号约定）
    wheel_drv[R]->send_torque(fp32_constrain(tau_out[R], -20.0f, 20.0f));
    wheel_drv[L]->send_torque(fp32_constrain(tau_out[L], -20.0f, 20.0f));

    // ⑥ 调试输出
    float P_cmd = s_power_ctrl.get_cmd_power();
    float P_out = s_power_ctrl.get_restricted_power();

    // 预测单电机功率（用于监控）
    float P_R = s_power_ctrl.predict_power(R, tau_out[R], cmd[R].omega);
    float P_L = s_power_ctrl.predict_power(L, tau_out[L], cmd[L].omega);
}
```

### 6.3 绕过能量环（调试 / 离线测试）

```cpp
// 直接指定功率上限，不使用电容能量反馈
s_power_ctrl.set_max_power(80.0f);
s_power_ctrl.update(cmd, tau_out);
```

---

## 7. 参数标定建议

1. 固定机器人（防止行走），在不同转速 $\omega$ 下给定恒定力矩 $\tau$，记录实际功率。
2. 用最小二乘法对数据点拟合：

$$
P = \tau \omega + k_1 |\omega| + k_2 \tau^2 + k_3
$$

3. 左右电机分别标定，填入 `k1[0]/k1[1]` 等。
4. `k3` 可在电机零速、零力矩时直接测量驱动板待机功率。

---

## 8. 注意事项

- `omega` 符号约定：**右轮正号**（`+get_current_rotate()`），**左轮负号**（`-get_current_rotate()`），与 `tau_motion` 方向保持一致。
- `tau_balance` 不经过任何功率限制，若实际 k1/k2/k3 标定不准，仅 `tau_motion` 的限制精度受影响，不影响平衡安全。
- `min_max_power` 建议设为静态损耗之和（两电机 k3 之和）以上，避免功率预算低于待机损耗导致输出为零。
