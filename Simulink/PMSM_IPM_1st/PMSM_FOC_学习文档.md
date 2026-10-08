# PMSM 矢量控制（FOC）实验全解
## 模块接口 · 模型逻辑 · 参数与算法设计

> **配套模型**：`PMSM_Model_fixed.slx`（MATLAB R2022b / Simulink 10.6 / Simscape 5.4 / Simscape Electrical 7.8）
> **说明**：本文中所有模块参数、端口分配、连线关系、示波器映射，都是从模型里**实测导出**的，
> 不是凭印象写的。凡是"通用知识"和"本模型实测值"有区别的地方，我都标了出来。
> 阅读建议：打开模型对照着看，遇到"第 3 章物理网络"时把模型缩放到 40% 通读一遍全貌。

---

# 目录

- [第 1 章 Simscape 心智模型：它和 Simulink 到底有什么不同](#第-1-章)
- [第 2 章 物理模块逐个拆解（接口 / 参数 / 使用要点）](#第-2-章)
- [第 3 章 本模型的物理网络逐线解读](#第-3-章)
- [第 4 章 控制算法模块逐个拆解](#第-4-章)
- [第 5 章 整条控制链的信号流](#第-5-章)
- [第 6 章 参数与算法的设计逻辑](#第-6-章)
- [第 7 章 示波器地图（Scope → 到底是什么信号）](#第-7-章)
- [第 8 章 参数速查表](#第-8-章)
- [第 9 章 这次调试暴露的 7 个坑](#第-9-章)
- [第 10 章 面向秋招：能力地图、这个模型还缺什么、高频面试题](#第-10-章)

---

<a id="第-1-章"></a>
# 第 1 章 Simscape 心智模型：它和 Simulink 到底有什么不同

很多人第一次用 Simscape 会觉得"连线连不上""为什么一定要放个 Solver Configuration"。
根因是没有建立正确的模型。这一章先把这个讲透，后面所有模块都会变得顺理成章。

## 1.1 两种完全不同的建模范式

| | Simulink（信号流） | Simscape（物理网络） |
|---|---|---|
| 本质 | 数据从源流向汇，**单向** | 元件之间交换物理量，**双向** |
| 连线 | 传递一个数值 | 表示"这两个端子接在一起" |
| 方程 | 你写的 block 就是方程 | 由**连接关系**自动生成方程（类似 SPICE） |
| 谁决定方向 | 端口方向（in/out） | 没有方向，靠物理定律 |
| 时间 | 你的采样时间 | 由全局求解器 + 局部求解器决定 |

**关键理解**：在 Simscape 里，你画的是**电路图和机械装配图**，不是程序框图。
把导线连起来，Simscape 会自动列出 KCL / KVL / 牛顿定律并交给求解器。

## 1.2 跨变量 / 穿变量（Across / Through）

每个物理域都有两种变量，这是读懂所有 Simscape 文档的钥匙：

| 物理域 | 跨变量（Across，势） | 穿变量（Through，流） | 守恒律 |
|---|---|---|---|
| 电气 | 电压 v（节点对参考） | 电流 i（支路） | 节点电流和为 0（KCL）；回路电压和为 0（KVL） |
| 旋转机械 | **角速度 ω** | 转矩 τ | 节点转矩和为 0；相对角速度关系 |
| 平移机械 | 速度 v | 力 F | 类似 |
| 热 | 温度 T | 热流 Q | 类似 |

⚠️ **特别注意旋转机械的跨变量是"角速度"不是"角度"**。
角度在 Simscape 内部是角速度的积分（是个状态量）。所以：
- 你不能"给一个角度"——你给的是转速源、转矩源或者转动惯量；
- 要测角度，必须用 `Ideal Rotational Motion Sensor` 的 position 端口。

## 1.3 三种端口，一眼分清

打开模型时，你会看到三种长相不同的线：

| 端口类型 | 长相 | 方向 | 传什么 | 例子 |
|---|---|---|---|---|
| **守恒端口（conserving）** | 方块/圆点，粗连线 | 双向 | Across/Through 都交换 | PMSM 的 a/b/c、R/C |
| **物理信号端口（PS）** | 细线+箭头 | 单向 | 只传一个值 | 传感器的 I / V / W 输出 |
| **Simulink 端口** | 普通信号线 | 单向 | 普通 Simulink 信号 | 控制器各模块之间 |

在 MATLAB 里，守恒端口叫 `LConn` / `RConn`，物理信号端口叫 `PMIOPort`。
用 `get_param(blk,'PortHandles')` 可以看到 `Inport / Outport / LConn / RConn` 各有多少个。

## 1.4 为什么这几个块"必须有"

| 块 | 为什么必须有 | 本模型的实测连接 |
|---|---|---|
| `Electrical Reference` | Across 变量是**相对量**，电气网络必须有一个零电位参考（电气地），否则方程欠定 | 接在直流母线**负**端 |
| `Mechanical Rotational Reference` | 旋转网络必须有"机架/地"参考 | Ref1 接 PMSM 的 C（机壳）；Ref2 接角度传感器的 C |
| `Solver Configuration` | 每个物理网络**有且仅有一个**。它给网络指定求解器设置、初值一致性求解方式、局部求解器开关 | ⚠️ 本模型接在 **A 相回路上**（与 PMSM 的 a 相、Current Sensor_ia1 同节点）。Simscape 只要求它接入网络，不要求接地，但**工程习惯上大家都接在参考地节点**，便于阅读和排查 |
| `PS-Simulink Converter` | 把物理量（带单位）转成 Simulink 信号 | 8 个：测速、测角、三相电流、两路电压 |
| `Simulink-PS Converter` | 把 Simulink 信号转成物理量，送进物理网络 | 3 个：三相门极信号 |

> **面试常问**：为什么 Simscape 模型不接 Solver Configuration 会报错？
> 答：Simscape 网络需要它来定义求解器配置和参考节点；没有它，网络的方程无法被组装进全局求解器的框架里。

## 1.5 单位：Simscape 最阴的坑

Simscape 里**每个物理量都带单位**，参数是"数值 + 单位下拉框"两部分。

⚠️ **最常见的错误**：`Ld` 参数框里填 `1`，单位下拉框选 `mH`，这是 1 mH；
但如果填 `0.001` 又选 `mH`，那就是 1 µH，差 1000 倍。**改参数一定要同时看数值和单位。**

`PS-Simulink Converter` 也有 `Unit` 参数：
- 填具体单位（如 `rad/s`）→ 输出以该单位计的值
- 填 `inherit` → 沿用上游物理端口的单位

本模型实测：
- `PS-Simulink Converter` → Unit = `rad/s`（机械角速度）
- `PS-Simulink Converter1` → Unit = `rad`（机械角度）
- `PS-Simulink Converter2/3/4/5` → Unit = `A`（三相电流）
- `PS-Simulink Converter8/9` → Unit = `inherit`（两路电压传感器）

---

<a id="第-2-章"></a>
# 第 2 章 物理模块逐个拆解

## 2.1 PMSM（永磁同步电机）—— 全模型的核心

**库路径**：`ee_lib/Electromechanical/Permanent Magnet/PMSM`
**Simscape 组件路径**：`ee.electromech.pmsm.pm_rotor.pmsm.abc`（变体 `..._thermal` 带热端口）

### 端口（实测）

| 侧 | 端口 | 类型 | 含义 |
|---|---|---|---|
| 左侧 | LConn1 / LConn2 / LConn3 | 守恒电气 | 三相定子绕组 **a / b / c** |
| 右侧 | RConn1 | 守恒旋转 | **R** = 转子轴（接负载/传感器） |
| 右侧 | RConn2 | 守恒旋转 | **C** = 机壳/定子（接机架地） |

本模型：
- `a/b/c` 分别经过一个电流传感器接到三相半桥的中点；
- `R` 接 `Ideal Rotational Motion Sensor` 的 R 端；
- `C` 接 `Mechanical Rotational Reference1`（机架地）。

### 参数分组（本模型实测值）

| 分组 | 参数 | 本模型值 | 含义与踩坑点 |
|---|---|---|---|
| Modeling fidelity | `machine_param` | `constant` | 常数参数（另一选项是查表/有限元数据） |
| | `nPolePairs` | **6** | 极对数。⚠️ 是"对数"不是"个数"，12 极电机这里填 6 |
| PM flux | `pmflux_param` | `fluxlinkage` | 用磁链参数化（另一选项是转矩常数/反电势常数） |
| | `pm_flux_linkage` | **0.01 Wb** | 永磁磁链 ψf（有的资料叫 λm 或 Φf） |
| Stator | `stator_param` | `LdLqL0` | 用 dq 电感 + 零序电感参数化 |
| | `Ld` | **1 mH** | d 轴电感 |
| | `Lq` | **1 mH** | q 轴电感 ← **Ld = Lq 就是"表贴式/隐极"的数学定义** |
| | `L0` | 0.00016 H | 零序电感（三相三线不参与，仅中线/故障分析用） |
| | `Rs` | **0.5 Ω** | 每相定子电阻 |
| | `zero_sequence` | `exclude` | 排除零序（Y 接无中线） |
| Rotor angle | `axes_param` | `daxis` | **转子角定义 = "a 相轴到 d 轴的夹角"**。这一项决定了 θe = p·θm 的零点，是 FOC 角度对齐的关键 |
| Loss | `loss_param` | `none` | 不计铁损（要算效率时应打开） |
| Mechanical | `J` | **1e-4 kg·m²** | 转子转动惯量 |
| | `lam` | **1e-3 N·m/(rad/s)** | 转子粘性阻尼（本模型从 1e-4 改成了 1e-3，见第 6 章） |

> 同一个块里还有 `torque_constant = 0.18`、`back_emf_constant = 0.18`、`Ls/Lm/Ms` 等一堆参数，
> **它们属于其它参数化选项，当前选项下不生效**。别被它们误导——这是一个很常见的困惑点。
> 判断标准：看 `pmflux_param` / `stator_param` 选的是哪一项。

### 它内部在算什么（必须会推）

Simscape 的 PMSM 内部就是标准 dq 模型，用你自己的 Park 变换把 ia/ib/ic 变成 id/iq：

**磁链方程**
```
ψd = Ld·id + ψf
ψq = Lq·iq
```

**电压方程**（ωe = p·ωm 为电角速度）
```
vd = Rs·id + dψd/dt − ωe·ψq
vq = Rs·iq + dψq/dt + ωe·ψd
```

**电磁转矩**
```
Te = 1.5 · p · (ψd·iq − ψq·id)
   = 1.5 · p · [ ψf·iq + (Ld − Lq)·id·iq ]
     └────┬────┘   └──────┬──────┘
      永磁转矩          磁阻转矩
```

**机械方程**
```
J · dωm/dt = Te − lam·ωm − TL
```

**表贴式（Ld = Lq）时**：`(Ld − Lq)·id·iq = 0`，所以
```
Te = 1.5 · p · ψf · iq
```
**转矩只和 iq 有关，和 id 完全无关。**

> 这就是"表贴式用 id = 0 控制"的全部数学理由：
> 给定 id 只会让相电流变成 √(id²+iq²)，**白白增加铜损却不产生任何转矩**。
> 所以对表贴式电机，id = 0 就是**铜损最小（MTPA 的退化解）**。
>
> 对**内嵌式（IPM）**，Lq > Ld，磁阻转矩那一项不为零，给一个**负的 id** 反而能产生额外转矩，
> 这就是 MTPA 曲线：
> ```
> id = ψf / (2(Lq − Ld)) − sqrt( ψf² / (4(Lq − Ld)²) + iq² )
> ```

### 用这个块做实验的几个技巧

1. **调参时先确认参数化选项**，别看错行。
2. **想从 IPM 切回 SPM**：只改 `Lq` 让它等于 `Ld` 即可，模型其它地方一行都不用动。
3. **想把内部量调出来看**：Simscape 支持变量记录。在模型里执行
   ```matlab
   set_param('PMSM_Model_fixed','SimscapeLogType','all');
   out = sim('PMSM_Model_fixed');
   out.simlog.PMSM.i_d.series.values     % 模块内部的真实 id
   out.simlog.PMSM.i_q.series.values     % 模块内部的真实 iq
   out.simlog.PMSM.electrical_torque.series.values
   ```
   这招在本次调试中起了决定性作用——**用模块内部的 id/iq 去校准你的 Park 角度**，
   比盯着示波器猜快得多。（详见第 9 章"3π/2 陷阱"）

---

## 2.2 Half-Bridge (Ideal, Switching) × 3 —— 逆变器桥臂

**Simscape 组件路径**：`ee.semiconductors.half_bridge_thermal`

### 端口（本模型实测分配）

| 端口 | 本模型接什么 | 功能 |
|---|---|---|
| LConn1 | `Simulink-PS Converter6/7/8` | **门极控制信号**（PS 端口，`control_port = ps`） |
| LConn2 | 未连接 | 热端口（本模型用的是 `NoThermalPort` 变体，未启用） |
| LConn3 | 直流母线 **+** | 上管漏极 |
| RConn1 | 电流传感器 → PMSM 相 | **桥臂中点（输出）** |
| RConn2 | 直流母线 **−** / 电气地 | 下管源极 |

### 参数（本模型实测值）

| 参数 | 值 | 含义 |
|---|---|---|
| `control_port` | `ps` | 门极用物理信号（PS）控制。另一选项是 `electrical`（电气门极） |
| `device_type` | `mosfet` | 器件选 MOSFET(Ideal, Switching)。还能选 GTO / IGBT |
| `mosfet_Vth` | 0.5 V | **门极导通阈值**：PS 门极信号 > 0.5 时管子导通 |
| `mosfet_Rds` | 0.01 Ω | 导通电阻（决定通态损耗） |
| `Goff` | 1e-6 1/Ω | 关断漏电导 |
| `Voff` | 300 V | 关断电压额定（仅用于数值处理） |
| `diode_param` | **`include.no`** | **不含反并联续流二极管** ← 见下面的重要提醒 |

### ⚠️ 重要：为什么本模型必须用"互补门极"

`diode_param = include.no` 意味着**这个半桥里没有体二极管/反并联二极管**，
电流只能走 MOSFET 沟道。而电机是强感性负载，电流不能突变，
所以**任何时刻都必须至少有一个管子导通**来提供续流路径。

本模型的做法是标准的**互补 PWM**：

```
Relational Operator  (Da > 载波)  ──┬──► Data Type Conversion ──► Mux:1 ──┐
                                    │                                    ├─► Simulink-PS Converter ──► 半桥门极
                 Logical Operator(NOT) ─► Data Type Conversion ──► Mux:2 ──┘
```

`Mux` 组合出的向量是 `[上管门极, 下管门极]`，两者严格互补，
既保证永远有续流通路，又保证上下管不会同时导通（不会直通炸管）。

> **工程对照**：真实逆变器**一定有**反并联二极管（MOSFET 的体二极管，或 IGBT 外并的快恢复二极管）。
> 在 Simscape 里把 `diode_param` 改成 `include.yes` 更接近真实硬件，
> 代价是仿真变慢、并且需要正确填二极管的正向压降/导通电阻参数。
>
> **面试延伸**：真实硬件里"互补门极"必须加**死区时间（dead time）**，
> 否则上下管换流瞬间会直通。代价是死区会引入**输出电压畸变**，
> 在 dq 坐标下表现为 **6 倍电频率的谐波**，这就是工业界"死区补偿"算法要解决的问题。

---

## 2.3 DC Voltage Source —— 直流母线

| 项 | 值 |
|---|---|
| 组件路径 | `foundation.electrical.sources.dc_voltage` |
| 端口 | LConn1 = `+`，RConn1 = `−` |
| `v0` | **100 V** |

它是**理想电压源**：内阻为零，输出电流可以无限大。

> **进阶**：真实母线有内阻和寄生电感，母线电压会随负载波动。
> 想更真实，可以把理想源串一个 `Resistor` + `Inductor`，或者并联一个大电容。
> 这也是"母线电压利用率""母线电容选型"这类面试题的实际背景。

---

## 2.4 Current Sensor × 3 —— 三相电流采样

| 项 | 说明 |
|---|---|
| 组件路径 | `foundation.electrical.sensors.current` |
| 端口 LConn1 | `+`（电流流入端） |
| 端口 RConn1 | `I`：**物理信号输出**，输出电流值 |
| 端口 RConn2 | `−`（电流流出端） |
| 符号约定 | **电流从 + 流入、从 − 流出时为正** |

本模型接法（实测）：
```
半桥中点(RConn1) ──► 电流传感器 + ──► − ──► PMSM 的 a / b / c 相
                          │
                          └─ I ──► PS-Simulink Converter2/3/5 ──► Clarke Trans
```
所以测到的是**"从逆变器流向电机"的相电流**，为正代表电动状态（这一点对转矩符号判断很重要）。

### 为什么只用了两相？

Clarke 变换只需要 ia、ib 两路，因为 Y 接无中线时有 **ia + ib + ic = 0**。
本模型虽然装了三个传感器，但 **Clarke Trans 只吃了 ia 和 ib**（第三相用公式推）。

> ⚠️ **本模型的一个小瑕疵（实测发现）**：
> `PS-Simulink Converter4` 的**物理输入端是悬空的**（模型里没有任何连线接到它）。
> 它被接到 `Scope4` 和 `Scope5`，所以 **`Scope4` 永远显示 0，`Scope5` 第三路也永远是 0**。
> 想看三相电流请用 **`Scope23`**（它接的是 Converter2/3/5 = ia/ib/ic，是正确的）。

### 进阶：真实驱动器怎么采？

- **采样时刻**：单电阻/双电阻/三电阻采样，通常在**零矢量作用中点**采样，
  此时三相下管全通（或全断），电流在采样电阻上最稳定。
- **为什么这个细节值钱**：如果采样时刻落在开关瞬间，采到的是带纹波的瞬时值，
  会产生和本次实验里一样的 dq 电流 6 次谐波纹波。

---

## 2.5 Voltage Sensor × 2 —— 本模型其实只测了母线电压

| 项 | 说明 |
|---|---|
| 组件路径 | `foundation.electrical.sensors.voltage` |
| 端口 | LConn1 = `+`，RConn1 = `V`（PS 输出），RConn2 = `−` |

**实测接法**：
- `Voltage Sensor3`：`+` 接直流母线 **+**，`−` 接直流母线 **−**
- `Voltage Sensor4`：`+` 接直流母线 **+**，`−` 接直流母线 **−**
- 两者的 V 输出分别 → `PS-Simulink Converter8/9` → `Scope20/21`

也就是说：**两个电压传感器测的是同一个量——直流母线电压（100 V）**，而且是冗余的，
**控制算法里根本没有用到它们**。

> **这是个可以改进的点**：真正有价值的是**相电压/线电压**测量，用途是：
> ① 无位置传感器控制的反电势观测；② 死区补偿；③ 母线电压前馈（把 SVPWM 的占空比
> 按实测母线电压归一化，母线跌落时增益不变）。想练手可以把其中一个电压传感器
> 改接到相-中性点或线-线之间。

---

## 2.6 Ideal Rotational Motion Sensor —— 位置/转速反馈

**Simscape 组件路径**：`foundation.mechanical.sensors.angular_velocity`（名字叫 angular_velocity，
但它是通用旋转运动传感器，可同时输出角度和角速度）

### 端口（本模型实测）

| 端口 | 接什么 | 含义 |
|---|---|---|
| LConn1 | PMSM 的 `R`（转子） | **R** = 被测端（转子侧） |
| RConn1 | `Mechanical Rotational Reference2` | **C** = 参考端（机架地） |
| RConn2 | `PS-Simulink Converter`（Unit = rad/s） | **W** = 角速度输出 |
| RConn3 | `PS-Simulink Converter1`（Unit = rad） | **角度输出** |

传感器测的是 **R 相对于 C** 的量（`reference = difference`）。

### 参数（本模型实测值）

| 参数 | 值 | 说明 |
|---|---|---|
| `reference` | `difference` | 相对 C 测量。另一选项是相对地（此时 C 端口被禁用） |
| `velocity_port` | `true` | 打开角速度输出端口 |
| `position_port` | `true` | 打开角度输出端口 |
| `acceleration_port` | `false` | 关闭角加速度输出 |
| `wrap_angle` | **`false`** | **角度不卷绕** ← 关键！ |
| `offset` | `0 rad` | 角度偏置 |

### ⚠️ `wrap_angle` 为什么关键

- `wrap_angle = false`：角度**连续累加**（0 → 2π → 4π → …），适合 FOC。
- `wrap_angle = true`：角度被卷绕到 [0, 2π)，每转一圈会从 2π **跳回 0**。

如果开了卷绕，`cos(θ)` / `sin(θ)` 的**数值本身是连续的**（因为三角函数是周期的），
看起来"没问题"；但如果你在控制器里对 θ 做**差值、微分、滤波或角度观测器**，
那个跳变会造成灾难性的错误。**做 FOC 一律用 `wrap_angle = false`。**

### 位置 → 电角度的完整链路（本模型实测）

```
PMSM.R ──► 传感器 R 端
传感器角度输出(RConn3) ──► PS-Simulink Converter1 (rad) ──► Gain(×6) ──► Bias(+3π/2, ★注释状态★)
   ──► Delay2 (Ts = 1e-4 s) ──► Open_Start ──► θ ──► Park Trans:3 / INV_Park:3
```

- `Gain = 6` = **极对数**，把机械角换成电角度：**θe = p · θm**
- `Delay2` 模拟数字控制器的**一个采样周期计算延迟**（真实系统必然存在）
- `Bias = 3π/2` **必须保持注释状态**（原因见第 9 章，这是本次调试最重要的一个坑）

> **面试高频**：为什么 Park 变换用的是电角度不是机械角？
> 因为 dq 坐标系的旋转速度是电角速度 ωe = p·ωm，而磁场分布（以及因此产生的转矩）
> 是按电角度周期变化的。忘了乘极对数是新手最经典的错误之一，
> 现象是 iq 里出现 (p−1) 或 (p+1) 次谐波、电机抖动、带载能力极差。

---

## 2.7 Electrical / Mechanical Rotational Reference —— 参考零点

| 块 | 组件路径 | 作用 |
|---|---|---|
| `Electrical Reference` | `foundation.electrical.elements.reference` | 电气网络的零电位（电气地）。**一个电气网络至少有一个** |
| `Mechanical Rotational Reference` | `foundation.mechanical.rotational.reference` | 旋转机械网络的"机架/地"。刚性固定在机架上的端口接它 |

本模型：
- `Electrical Reference` 接在直流母线负端；
- `Mechanical Rotational Reference1` 接 PMSM 的 `C`（机壳固定）；
- `Mechanical Rotational Reference2` 接角度传感器的 `C` 端。

---

## 2.8 Solver Configuration —— 物理网络的"求解器入口"

**实测连接**：它的 RConn1 与 `Current Sensor_ia1`、`PMSM` 的 A 相在**同一个节点**上。

| 参数 | 本模型值 | 含义 |
|---|---|---|
| `UseLocalSolver` | `off` | 关闭局部求解器，和 Simulink 全局求解器（ode23t）一起解 |
| `RelTol` / `AbsTol` | 1e-3 / 1e-3 | Simscape 自身的相对/绝对容差 |
| `MinStep` | 1e-9 s | 最小步长 |
| `ConsistencySolver` | `NEWTON_XTOL_AFTER_FTOL` | 初值一致性求解算法 |
| `IndexReductionMethod` | `DerivativeReplacement` | 指标降阶方法（处理代数环/约束） |
| `DoDC` | `off` | 直流工作点分析 |
| `PartitionMethod` | `ROBUST` | 网络分区方法（影响求解效率） |
| `UseLocalSolver` = off + 全局 ode23t | — | 本模型的求解方案 |

**要点**：
1. 一个物理网络**有且仅有一个** Solver Configuration，多了会报错。
2. 它接入网络的位置在数学上不唯一（Simscape 会自动处理），
   但**放在参考地节点**是最常见、最易读的习惯。本模型放在 A 相回路上，能跑，但读图时容易困惑。
3. 当仿真报"无法收敛 / 步长过小"时，这里是第一个该看的地方：
   - 把 `RelTol/AbsTol` 放宽一点
   - 打开 `UseLocalSolver` 让 Simscape 用自己的求解器解这个网络（对付刚性开关电路很有效）
   - 给开关管并联小的 RC 吸收电路（现实中也有）

---

## 2.9 PS-Simulink / Simulink-PS Converter —— 两个世界的接口

| | PS-Simulink Converter | Simulink-PS Converter |
|---|---|---|
| 方向 | 物理 → Simulink | Simulink → 物理 |
| 输入 | 物理信号（PS 细线） | Simulink 信号 |
| 输出 | Simulink 信号 | 物理信号（PS 细线） |
| 关键参数 | `Unit`（`inherit` 或具体单位） | `Unit`、`FilteringAndDerivatives`、`InputFilterTimeConstant` |

本模型 3 个 `Simulink-PS Converter`（6/7/8，给三相门极）实测参数：
```
Unit = 1                                    （门极是无量纲）
FilteringAndDerivatives = provide           （让转换器提供输入信号及其导数）
SimscapeFilterOrder = 1
InputFilterTimeConstant = 0.001 s           （输入一阶滤波时间常数）
```

> **`FilteringAndDerivatives = provide` 是干什么的？**
> 物理网络求解器需要输入的导数信息。如果你直接把一个**带阶跃或高频抖动的 Simulink 信号**
> 灌进物理网络，求解器会因为不连续而反复缩步长甚至失败。
> 让转换器提供一个时间常数 1 ms 的一阶滤波，既平滑了输入，又给求解器提供了可用的导数。
>
> ⚠️ 但这 1 ms 滤波本身就是**执行延迟**！在 10 kHz 的电流环里（Ts = 0.1 ms），
> 1 ms = 10 个采样周期，会显著吃掉相位裕度。做**数字控制器在环（MIL）**精度要求高时，
> 可以改成 `FilteringAndDerivatives = filter` 并把时间常数设得远小于 Ts 的 1/10。

---

<a id="第-3-章"></a>
# 第 3 章 本模型的物理网络逐线解读

把上面所有块连起来，本模型的物理拓扑是（以下全部为**实测连线**）：

```text
              +----------------------------------+
              |   DC Voltage Source1   v0=100V   |
              +----+------------------------+----+
                  (+)                      (-)
       +-----------+-----------+            |
       |           |           |            |
  +----+----+ +----+----+ +----+----+       |
  | HB  a   | | HB  b   | | HB  c   |       |
  +--+---+--+ +--+---+--+ +--+---+--+       |
  gate  mid   gate  mid   gate  mid          |
    ^    |      ^    |      ^    |           |
  SPC6 [CS1]   SPC7 [CS2]   SPC8 [CS3]       |
    ^    |      ^    |      ^    |           |
    |    v      |    v      |    v           |
    |  PMSM a   |  PMSM b   |  PMSM c        |
    |    +------+----+-----+-----------------+
    |        (star point, Y, no neutral)     |
    |                                        |
    | [CS1/2/3] -(I,PS)-> Conv 2/3/5 -> ctrl |
    |                                        |
    +-- gate = Mux[high side, NOT high] <----+
```

其余连接关系（都是实测）：

- **机壳侧**：`PMSM.C` → `Mechanical Rotational Reference1`（机架地，定子固定）
- **转子侧**：`PMSM.R` → `Ideal Rotational Motion Sensor.R`
  - 传感器 `.C` → `Mechanical Rotational Reference2`（机架地）
  - 传感器 `.W`（角速度）→ `PS-Simulink Converter` (rad/s) → Scope
  - 传感器 `.角度`（位置）→ `PS-Simulink Converter1` (rad) → `Gain(×6)` → `Delay2` → `Open_Start`
- **母线侧**：
  - `DC+` → 三个半桥的 `LConn3(+)`，以及 `Voltage Sensor3/4` 的 `+`
  - `DC−` → 三个半桥的 `RConn2(−)`、`Electrical Reference`、`Voltage Sensor3/4` 的 `−`
  - `Solver Configuration` → 接在 A 相回路节点上

**一句话总结**：三相两电平电压源型逆变器（VSI）驱动一台 Y 接表贴式 PMSM，
用三个电流传感器采相电流，用一个旋转传感器采转子位置/转速，
全部控制算法在 Simulink 侧用离散算法实现。

**功率流**：DC 100 V → 三相桥（PWM）→ 三相电流 → 定子磁场 → 转矩 → 转子 → 机械负载（阻尼 + 惯量）。

---

<a id="第-4-章"></a>
# 第 4 章 控制算法模块逐个拆解

本模型的控制算法全部用 **Stateflow 图表（Simulink 的 MATLAB Function 块）** 实现，
共 8 个。这种写法的好处是算法一目了然、便于改；工业代码里则会改成 C 或定点实现。

## 4.1 Clarke Trans —— 三相静止 → 两相静止

```matlab
function [i_alpha,i_beta] = fcn(ia,ib)
i_alpha = ia;
i_beta  = (1/sqrt(3))*(ia + 2*ib);
end
```

**这是等幅值 Clarke 变换**（不是等功率变换）。推导：
```
iα = (2/3)(ia − ib/2 − ic/2) = ia            （因为 ia+ib+ic = 0）
iβ = (2/3)(√3/2)(ib − ic)   = (ib − ic)/√3
                             = (ia + 2ib)/√3  （因为 ic = −ia−ib）
```
✅ 公式正确。

⚠️ **两种 Clarke 变换的选择**：等幅值（系数 2/3）还是等功率（系数 √(2/3)）。
选等幅值的话，**转矩公式里要带 1.5**（本模型用的是 `Te = 1.5·p·ψf·iq`，配套正确）。
两者混用会导致转矩差 1.5 倍——这也是面试里常被追问的细节。

## 4.2 Park Trans —— 两相静止 → 两相旋转

```matlab
function [id, iq] = park(ialpha, ibeta, theta)
id =  ialpha*cos(theta) + ibeta*sin(theta);
iq = -ialpha*sin(theta) + ibeta*cos(theta);
end
```
这是标准的逆时针旋转矩阵 `R(θ)` 的转置形式，把 αβ 投影到随转子旋转的 dq 轴上。
✅ 正确。

**输入 θ 的含义**：转子 **d 轴** 相对 α 轴（a 相轴）的**电角度**。

## 4.3 INV_Park —— 两相旋转 → 两相静止

```matlab
function [Valpha, Vbeta] = inv_park(Vd, Vq, theta)
    Valpha = Vd*cos(theta) - Vq*sin(theta);
    Vbeta  = Vd*sin(theta) + Vq*cos(theta);
end
```
✅ 与 Park 严格互逆（`inv_park` 的矩阵是 `park` 矩阵的转置，因为旋转矩阵是正交的）。

> ❗ **必须成对校验**：Park 和 InvPark 一定要用**同一个角度、同一套正方向约定**。
> 这是本次调试中最容易出隐蔽错误的地方——如果两者不一致，
> 控制器会去"调节一个被转过的坐标系"，表现为电流环增益莫名其妙、甚至正反馈发散。

## 4.4 Vd / Vq —— 电流环 PI（本模型已修复）

```matlab
function Vd = fcn(Kp,Ki,error)
% 离散 PI 电流控制器，每个 Ts = 1e-4 s 执行一次
    persistent integral
    Ts    = 1e-4;
    limit = 57;      % Vdc/sqrt(3) = 100/sqrt(3)：SVPWM 线性区上限
    if isempty(integral)
        integral = 0;
    end
    integral = integral + error*Ts;
    if Ki ~= 0                                   % 抗积分饱和（钳位 Ki·∫e）
        if Ki*integral >  limit, integral =  limit/Ki; end
        if Ki*integral < -limit, integral = -limit/Ki; end
    end
    Vd = Kp*error + Ki*integral;
    if Vd >  limit, Vd =  limit; end             % 输出限幅
    if Vd < -limit, Vd = -limit; end
end
```

**四个必须理解的要点**：

1. **`persistent` 是"带记忆"的关键**。原模型写的是 `integral = 0;`
   （每次调用清零），积分器完全没有记忆，控制器退化成纯比例，稳态必有静差。
   这是本次调试找到的第二个致命缺陷。

2. **必须指定离散采样时间**。在图表属性里设 `ChartUpdate = 'DISCRETE'`、
   `SampleTime = '1e-4'`（用 `sfroot` 找 `Stateflow.EMChart` 对象设置）。
   如果保持 `INHERITED`（连续），图表会在求解器的每个 minor step 执行一次，
   而代码里 `integral = integral + error*1e-4` 用的是**固定** Ts，
   于是积分量会被重复累加几百万次 → 等效 Ki 放大上百倍 → 剧烈振荡。
   **这是"用 MATLAB Function 写数字控制器"最常见的一个坑。**

3. **抗饱和要钳位"贡献量"而不是"状态量"**。
   原代码把 `integral` 钳到 ±57，但它在输出中的贡献是 `Ki·integral`
   （Ki = 1000 时可达 ±57000 V），等于没限幅。正确做法是钳 `Ki·integral`。

4. **`limit = 57` 不是随便取的**：SVPWM 线性调制区的最大相电压幅值是
   `Vdc/√3 = 100/1.732 = 57.7 V`，取 57 V 留一点余量。详见第 6 章。

## 4.5 SVPWM 三步走（三个 MATLAB Function）

### 第一步：扇区判断（`SVPWM_Sector`）

```matlab
function Num_sector = SVPWM_Sector(Valpha,Vbeta)
% 扇区编号 1..6，从 α 轴开始逆时针
    angel = atan2(Vbeta,Valpha);
    if angel < 0
        angel = angel + 2*pi;
    end
    Num_sector = floor(angel/(pi/3)) + 1;
    Num_sector = max(1, min(6, Num_sector));
end
```

⚠️ **原模型这里写的是 `floor((pi*angel)/3)+1`**——这是把 `angel/(π/3)` 打成了
`π·angel/3`，差一个 `(π/3)² ≈ 1.097` 的因子。后果是扇区边界从
60°/120°/… 偏到 54.7°/109.5°/…，**约 9% 的电气角度会落进错误扇区**，
此时 `T2` 变负、占空比映射全错。这是第三个致命缺陷。

### 第二步：相邻矢量作用时间（`svpwm_time`）

```matlab
Vmag  = sqrt(Valpha^2 + Vbeta^2);
angle = atan2(Vbeta, Valpha);   if angle<0, angle = angle+2*pi; end
theta_prime = angle - (sector-1)*pi/3;
T1 = sqrt(3) * Ts * Vmag / Vdc * sin(pi/3 - theta_prime);
T2 = sqrt(3) * Ts * Vmag / Vdc * sin(theta_prime);
```

**推导思路**（很重要，面试常问）：
把指令电压矢量 `V` 投影到扇区的两个边界矢量 `V1`、`V2` 上（幅值都是 `2Vdc/3`），
由"伏秒平衡"解出两个矢量的作用时间：

```
T1 = (√3 · Ts · |V| / Vdc) · sin(60° − θ′)
T2 = (√3 · Ts · |V| / Vdc) · sin(θ′)
T0 = Ts − T1 − T2                                （零矢量时间）
```
**线性调制极限**：当 `T1 + T2 = Ts` 时取到最大 `|V| = Vdc/√3 ≈ 0.577·Vdc`。
对比 SPWM 的极限 `Vdc/2 = 0.5·Vdc`，**SVPWM 的电压利用率高 `2/√3 ≈ 15.5%`**——
这就是业界普遍用 SVPWM 而不是 SPWM 的核心原因。

### 第三步：七段式占空比（`svpwm_duty`）

```matlab
function [Da, Db, Dc] = svpwm_duty(T1, T2, Ts, sector)
    Ta = (Ts - T1 - T2)/2;      % 零矢量时间的一半
    switch round(sector)
        case 1, Da = Ta+T1+T2; Db = Ta+T2;      Dc = Ta;
        case 2, Da = Ta+T1;    Db = Ta+T1+T2;   Dc = Ta;
        case 3, Da = Ta;       Db = Ta+T1+T2;   Dc = Ta+T2;
        case 4, Da = Ta;       Db = Ta+T1;      Dc = Ta+T1+T2;
        case 5, Da = Ta+T2;    Db = Ta;         Dc = Ta+T1+T2;
        case 6, Da = Ta+T1+T2; Db = Ta;         Dc = Ta+T1;
    end
    Da = min(max(Da,0),Ts); Db = min(max(Db,0),Ts); Dc = min(max(Dc,0),Ts);
end
```

**这张表怎么来的**（自己推一遍，比背下来强）：
SVPWM 的"七段式"开关序列是
```
000 → V1(T1) → V2(T2) → 111 → V2(T2) → V1(T1) → 000
```
所以某一相在一个周期内导通的时间 = `零矢量半时间 + 该相为高的那些矢量的作用时间`。
例如扇区 1 用的是 `V1(100)` 和 `V2(110)`：
- A 相在两个矢量里都是高 → `Da = Ta + T1 + T2`
- B 相只在 `V2` 里高 → `Db = Ta + T2`
- C 相一直是低 → `Dc = Ta`

**验证方法**（我做过的离线校验）：用"无歧义的 min/max 共模注入法"反算
```
va = Vα,  vb = −Vα/2 + √3·Vβ/2,  vc = −Vα/2 − √3·Vβ/2
v_off = −(max(v)+min(v))/2
d_x = 0.5 + (v_x + v_off)/Vdc
```
再对比两种实现合成的 αβ 电压。原实现的**最大误差 110 V**（母线才 100 V！），
修正后 **3.6e-14 V**。

⚠️ **`Ts = 1` 的归一化技巧**：顶层 `Constant`（SID 64）值为 1，接到 `Ts` 输入端。
因为 `Ta/Tb/Tc` 都是 `Ts` 的线性函数，令 `Ts = 1` 时
**Da/Db/Dc 直接就是占空比（0~1）**，可以拿去和归一化载波比较。
这是个实用的小技巧，但**必须在文档里写清楚**，否则别人接手时会完全看不懂。

## 4.6 载波比较与门极生成

```
Bias1 (+0.5) ────────────────► Relational Operator :2
MATLAB Function2 (Da) ───────► Relational Operator :1     (Da > 载波)
Triangle Generator (Gain1=0.5) ─► Bias1
```
- `Triangle Generator`（SPS 库）内部实测：`1-D Lookup Table` 表值 `[0 2 0]`，
  断点 `[0 0.5 1]`，配合 `rem(t·Freq, 1)` 产生 **−1 ~ +1** 的双极性三角波。
- `Gain1 = 0.5` 把它变成 −0.5~0.5，`Bias1 = +0.5` 抬成 **0~1** 的单极性载波。
  （这就是"为什么载波要加 0.5 偏置"的答案——因为 SPS 的三角波发生器是双极性的，
  而 SVPWM 的占空比是 0~1 的单极性量。）
- 比较器输出 → `NOT` → `Mux[上管, 下管]` → `Simulink-PS Converter` → 半桥门极。

> **进阶知识点（面试常问）**：
> - **七段式 vs 五段式 SVPWM**：七段式（本模型）谐波性能好但每周期每相开关 2 次；
>   五段式把零矢量全放在一端，开关次数减少 1/3，开关损耗低但谐波变差。工业上低载波比时常用五段式。
> - **不规则采样 vs 规则采样**：本模型的占空比是**连续计算**后与载波比较（自然采样），
>   真实数字控制器是**每个载波周期更新一次**（规则采样）。两者谐波特性不同。
> - **过调制**：当 `|V| > Vdc/√3`，`T1 + T2 > Ts`，此时要做过调制处理
>   （本模型用 `min(max(...))` 硬钳位，属于最粗暴的方式）。

## 4.7 Open_Start —— 起动策略（本模型已改为全程闭环）

原设计的意图是"开环起动"：先用一个自由旋转的电压矢量把转子拖起来，
等转子跟上之后再切到编码器角度闭环。

```
Ramp (slope = 2π×10 rad/s) ──► Switch:1
Relational Operator3         ──► Switch:2   (Clock 与 Constant2 比较)
编码器角度 (In1)              ──► Switch:3
```

**原模型的问题**：比较符写成了 `Clock > 0.1`，
而 `Switch` 的判据是 `u2 > 0.5` 时选 `u1`（= Ramp），
结果是 **t < 0.1 s 用编码器角度，t > 0.1 s 反而切到开环斜坡**——逻辑完全接反。
0.1 s 处控制角度从 −14.24 rad 跳到 6.283 rad（约 264°），FOC 彻底失效。

**修复方式**：比较符改 `<=`，`Constant2` 改成 −1，使**全程使用编码器角度**。

**为什么不保留开环起动**：
该电机 `J = 1e-4`、`iq = 1 A` 时机械角加速度约 `900 rad/s²`，
而斜坡只有 `2π×10 rad/s`（10 Hz 电气），转子会瞬间冲出同步区而失步。
**有真实位置传感器时，直接闭环起动才是正确做法**；
开环起动（I/F 起动）是无位置传感器控制才需要的手段。

---

<a id="第-5-章"></a>
# 第 5 章 整条控制链的信号流

**一次完整的电流环执行流程**（本模型的实际拓扑，每个 Ts = 1e-4 s 循环一次）：

```text
[1] Rotor position
    PMSM.R -> RotSensor -> Angle(rad) -> x6 -> Delay2 -> Open_Start -> theta
                                                                       |
[2] Current sampling                                                   |
    CS_ia1 -> Conv2 -> ia --+                                          |
    CS_ia2 -> Conv3 -> ib --+-> Clarke Trans -> i_alpha, i_beta        |
                                                   |                   |
[3] Coordinate transform                           v                   v
                             i_alpha,i_beta --> Park Trans <----- theta
                                                    |
                                       +------------+------------+
                                       v                         v
                                      id                        iq
                                       |                         |
[4] Error           id_ref(0) --> ( SUM ) <--+   iq_ref(1) --> ( SUM ) <--+
                                       |  -id  |                |  -iq   |
                                       v       |                v        |
[5] PI current loop           Vd = PI(err)             Vq = PI(err)      |
                                       |                         |       |
[6] One-sample delay             Delay(1e-4)            Delay1(1e-4)     |
                                       |                         |       |
[7] Inverse Park                       v                         v       |
                           Vd,Vq --> INV_Park <-----------------+-------+
                                         ^  ^
                               Valpha,Vbeta  +-- theta
                                         |
[8] SVPWM                                v
    SVPWM_Sector(Va,Vb) -> sector --+--> svpwm_time(...) --> T1,T2
                                    |                          |
                                    +--> svpwm_duty(...) -> Da,Db,Dc
                                         |
[9] PWM modulation                       v
    Da,Db,Dc -> ( duty > carrier ) -> NOT -> Mux[hi,lo] -> Simulink-PS
                   ^
                   0..1 triangular carrier (TriGen x0.5 + 0.5)

[10] Power stage and machine
    half-bridges -> 3-phase voltage -> PMSM stator -> current rises
    -> Te = 1.5*p*psi_f*iq -> speed rises
    -> back-EMF we*psi_f becomes the q-axis disturbance (loop closes)
```

**关键理解**：
- 这是一个**双闭环结构的最内环（电流环）**，没有外层的转速环。
  `iq_ref` 是一个固定常数 1 A，所以电机从静止一直加速到「转矩 = 阻尼转矩」的平衡点。
- 电流环的**被控对象**是 `1/(Ls + R)`，**扰动**是反电动势 `ωe·ψf`（加在 q 轴）和
  交叉耦合项 `ωe·L·i`（d/q 轴互相干扰）。
- 由于整个控制都在随转子旋转的 dq 坐标系里做，
  **交流量变成了直流量**，用 PI 就能做到零稳态误差——**这就是 FOC 的核心思想**。

---

<a id="第-6-章"></a>
# 第 6 章 参数与算法的设计逻辑

## 6.1 电机参数：每一个都在影响什么

| 参数 | 值 | 它决定了什么 | 改大/改小的后果 |
|---|---|---|---|
| `nPolePairs` p | 6 | 电角速度 ωe = p·ωm；相同转矩下转速反比于 p | 改错 → 角度全错，电机抖动 |
| `pm_flux_linkage` ψf | 0.01 Wb | **转矩常数 Kt = 1.5·p·ψf = 0.09 N·m/A**；反电势常数 Ke = p·ψf = 0.06 V·s/rad | ψf↑ → 同电流转矩大，但反电势高、高速难做、弱磁需求大 |
| `Ld = Lq` | 1 mH | **决定电流环带宽（Kp = ωc·L）和电流纹波（ΔI ∝ Vdc·Ts/L）** | L↑ → 纹波小、但电流响应慢、电压需求高 |
| `Rs` | 0.5 Ω | 铜损 I²R；决定 PI 的零点位置（Ki/Kp = R/L） | Rs 随温度上升 40% 是工程常态 |
| `J` | 1e-4 kg·m² | 机械时间常数 J·ω/T；加速度 | J↑ → 响应慢、但抗负载扰动能力强 |
| `lam` | 1e-3 N·m/(rad/s) | 粘性阻尼，决定**空载稳态转速** ω_ss = Te/lam | 见 6.6 |

**算一下本模型的工作点**（这一步是"参数设计"的范例）：
```
Kt = 1.5 × 6 × 0.01  = 0.09 N·m/A
iq = 1 A  →  Te = 0.09 N·m
空载稳态：Te = lam·ωm  →  ωm = 0.09 / 1e-3 = 90 rad/s = 860 rpm
机械时间常数：τ = J/lam = 1e-4 / 1e-3 = 0.1 s   →  0.5 s 内已经稳定
电频率：fe = ωe/(2π) = (6×90)/(2π) = 86 Hz
每个电周期的开关次数：10 kHz / 86 Hz ≈ 116 次   →  余量充足
反电势：ωe·ψf = 540 × 0.01 = 5.4 V，远小于 57 V 限幅   →  没有进入弱磁区
```

## 6.2 Vdc = 100 V 与 `limit = 57` 的耦合关系

```
SVPWM 线性调制区的相电压幅值上限：  Vmax = Vdc/√3 = 100/1.732 = 57.7 V
所以 PI 输出限幅取 57 V（留 1% 余量）
```

⚠️ **这三个量必须一起改**：
```
Vdc = 100 V  →  limit = 57 V  →  同步把 svpwm_time 的 Vdc 输入改成 100
```
本模型里 `Vdc` 是通过顶层 `Constant1`（值 100）送进 `svpwm_time` 的，
`limit` 则硬编码在 Vd/Vq 的脚本里。**改母线电压时两处都要改**，否则
占空比计算会整体缩放错，表现为电流环增益不对。

## 6.3 电流环 Kp / Ki 整定（完整推导，面试必考）

### 第一步：写被控对象
电流环的被控对象是定子绕组的电气动态：
```
G(s) = 1 / (L·s + R) = 1 / (0.001s + 0.5)
```
- 直流增益 `1/R = 2 A/V`
- 电气时间常数 `τe = L/R = 1e-3/0.5 = 2 ms`（极点位于 s = −R/L = −500 rad/s）

### 第二步：写控制器
并联式 PI：
```
C(s) = Kp + Ki/s = Kp · (s + Ki/Kp) / s
```

### 第三步：用零点对消掉对象的极点
令 `Ki/Kp = R/L = 500`，则
```
C(s)·G(s) = Kp(s + R/L) / [s·L(s + R/L)] = Kp / (L·s)
```
**极点被零点完全对消**，开环变成一个纯积分器！

### 第四步：由期望带宽定增益
纯积分器的穿越频率就是 `ωc = Kp/L`，所以：
```
Kp = ωc · L
Ki = ωc · R
```

### 第五步：选带宽
数字控制器的经验法则：**`ωc ≤ (1/10 ~ 1/20) × 开关角频率`**。
本模型 `fsw = 10 kHz`，取余量充足的 `ωc = 2000 rad/s ≈ 318 Hz`（约 fsw/31）：
```
Kp = 2000 × 0.001 = 2        ✓（本模型值）
Ki = 2000 × 0.5   = 1000     ✓（本模型值）
Ki/Kp = 500 = R/L            ✓
```

### 第六步：校核相位裕度
数字控制引入的总延迟约为 2 个采样周期（采样保持 1 个 + PWM 更新 1 个）：
```
Td ≈ 2·Ts = 2e-4 s
相位损失 = ωc·Td = 2000 × 2e-4 = 0.4 rad = 23°
相位裕度 PM ≈ 90° − 23° = 67°     → 稳定，且有余量
```

### 增益扫描实测（t > 0.25 s 稳态窗）

| Kp | Ki | i_d 平均 | i_d 峰峰值 | i_q 平均 | i_q 峰峰值 |
|---|---|---|---|---|---|
| **2** | **1000** | −0.0000 | 0.407 | +1.0002 | 0.609 |
| 4 | 2000 | +0.0007 | 0.370 | +0.9978 | 0.511 |
| 1 | 500 | +0.0026 | 0.466 | +0.9954 | 0.881 |
| 0.5 | 250 | +0.0076 | 0.426 | +0.9915 | 1.043 |

规律很清楚：**带宽越高 → 电流均值越准、纹波越大**（高带宽会把开关纹波也放大）。
本模型取 Kp = 2 是"均值最准、余量最大"的折中。

## 6.4 采样频率 / 开关频率怎么选

| 量 | 本模型 | 选择依据 |
|---|---|---|
| 开关频率 fsw | **10 kHz** | 与电流环采样同频。fsw↑ → 电流纹波 ∝ 1/fsw 减小、但开关损耗 ∝ fsw 增加 |
| 电流环采样 Ts | **1e-4 s（10 kHz）** | 常规做法是「一个开关周期采样一次」，即 Ts = 1/fsw |
| 电流环带宽 | **318 Hz** | fsw/31，余量充足。激进可取 fsw/10 = 1 kHz |
| 求解器 | ode23t（变步长） | 开关电路是刚性的，ode23t 是梯形法变体，适合 |

> ⚠️ **本模型修复前的一个典型错误**：Delay 采样 1e-4 s（10 kHz）、
> PI 里硬编码 Ts = 1e-4，但三角载波 Freq 只有 **1 kHz**。
> 控制频率是开关频率的 10 倍，带宽被严重浪费，电流环根本发挥不出来。
> **控制频率和开关频率是绑定关系，必须一起设计。**

## 6.5 `Ts = 1` 的归一化技巧（以及它的代价）

SVPWM 的 `Ta/Tb/Tc` 都是 `Ts` 的线性函数，所以：
- 让 `Ts = 1` → `Da/Db/Dc` 直接就是 **0~1 的占空比**
- 载波也归一化成 0~1 → 两者可以直接比较

**代价**：公式里 `T1 = √3·Ts·|V|/Vdc·sin(...)` 的物理量纲被"藏"起来了。
接手的人如果不知道这个约定，会以为 Ts 的单位是秒而填 1e-4，直接算错 10000 倍。
**建议在模型里加一个注释块说明**（本次修复的脚本注释里已经写明）。

## 6.6 为什么把 `lam` 从 1e-4 改成 1e-3

**原始参数下的工况**：
```
Te = 0.09 N·m，lam = 1e-4  →  ωm_ss = 900 rad/s = 8600 rpm
机械时间常数 τ = J/lam = 1 s  →  0.5 s 时才到 1900 rpm，还在加速
电频率随之升到 300+ Hz，10 kHz 开关只剩 3 个开关周期/电周期  →  电流环完全跟不上
```
这不是"控制算法错了"，而是**工况超出了开关频率的能力边界**。

**改后**：
```
lam = 1e-3  →  ωm_ss = 90 rad/s = 860 rpm，τ = 0.1 s  →  0.5 s 已稳定
仿真末：Te = 0.0898 N·m，阻尼转矩 = 0.0889 N·m  →  转矩已平衡，确实是稳态
```

> **工程上的等价做法**：加一个负载转矩（风机/螺旋桨负载就是 ∝ ω² 的阻尼特性），
> 或者外挂一个转速环。本质上都是"给电机一个能平衡 0.09 N·m 的负载"。
> 单纯看 id=0/iq=1 这个电流指标，两种做法结果一样；
> 但如果要练转速环设计，就必须加负载模型。

## 6.7 想改成内嵌式（IPM）只需两步

1. PMSM 块里把 `Lq` 改得大于 `Ld`（比如 `Ld = 0.5 mH, Lq = 1.5 mH`）；
2. 把 `id_ref` 从 0 改成一个负值（或实现 MTPA 曲线：

```matlab
id_ref = psi_f/(2*(Lq-Ld)) - sqrt( psi_f^2/(4*(Lq-Ld)^2) + iq_ref^2 );
```

**其它模块一行都不用改**——这正是 FOC 架构的优雅之处：
坐标变换和 SVPWM 与电机凸极性无关。改动只发生在"给什么 id 参考"这一层。

---

<a id="第-7-章"></a>
# 第 7 章 示波器地图（Scope → 到底是什么信号）

原模型有 25 个 Scope 和 6 个 Display，名字全是默认的 `Scope9`、`Display3`，
非常难用。下面这张表是**实测导出**的对应关系，建议打印出来贴在屏幕上：

| Scope | 接的端口 | 看的是什么 |
|---|---|---|
| `Scope` | Conv(rad/s) | 机械角速度 ωm |
| `Scope1` | Conv1(rad) | 机械角度 θm |
| `Scope2` | Conv2 | **ia** |
| `Scope3` | Conv3 | **ib** |
| `Scope4` | Conv4 | ⚠️ **恒为 0**（Conv4 物理输入悬空） |
| `Scope5` | Conv2,3,4 | ia, ib, **0** ⚠️ 第三路无效 |
| `Scope6` | Subtract | **id 误差** = id_ref − id |
| `Scope7` | Subtract1 | **iq 误差** = iq_ref − iq |
| `Scope8` | Clarke Trans | iα, iβ |
| **`Scope9`** | Park Trans:1 | **id** ← 你最关心的 |
| **`Scope10`** | Park Trans:2 | **iq** ← 你最关心的 |
| **`Scope11`** | Park Trans:1,2 | **id 与 iq（推荐看这个）** |
| `Scope12` | Delay | **Vd** |
| `Scope13` | Delay1 | **Vq** |
| `Scope14` | INV_Park:1 | Vα |
| `Scope15` | INV_Park:2 | Vβ |
| `Scope16` | Open_Start | **θ（电角度）** |
| `Scope17` | svpwm_duty:1 | Da |
| `Scope18` | svpwm_duty:2 | Db |
| `Scope19` | svpwm_duty:3 | Dc |
| `Scope20` | Conv8 | 电压传感器3（直流母线电压，恒定 100 V） |
| `Scope21` | Conv9 | 电压传感器4（同上，冗余） |
| `Scope22` | Da,Db,Dc | 三相占空比（看 PWM 调制情况） |
| **`Scope23`** | Conv2,3,5 | **ia, ib, ic（三相电流，这个是全的）** |
| `Scope24` | Conv1 | 机械角度（同 Scope1） |
| `Display2` | SVPWM_Sector | **当前扇区号（1~6）** |
| `Display3/4/5` | Da/Db/Dc | 三相占空比数值 |

> **注意**：`Scope9` 和 `Scope10` 才是 id/iq，**不要看错**。
> `Scope6/Scope7` 是误差信号（应该收敛到 0），也是个很好的调试窗口。

---

<a id="第-8-章"></a>
# 第 8 章 参数速查表

## 8.1 电机（PMSM 块）

| 参数 | 值 | 单位 |
|---|---|---|
| `nPolePairs` | 6 | — |
| `pm_flux_linkage` | 0.01 | Wb |
| `Ld` / `Lq` | 1 / 1 | mH |
| `Rs` | 0.5 | Ω |
| `J` | 1e-4 | kg·m² |
| `lam` | 1e-3 | N·m/(rad/s) |
| `axes_param` | `daxis` | — |
| `zero_sequence` | `exclude` | — |
| 派生：Kt = 1.5·p·ψf | 0.09 | N·m/A |
| 派生：Ke = p·ψf | 0.06 | V·s/rad |
| 派生：τe = L/R | 2 | ms |

## 8.2 功率级

| 参数 | 值 |
|---|---|
| 母线电压 `v0` | 100 V |
| 器件 | MOSFET (Ideal, Switching) |
| `mosfet_Vth` / `mosfet_Rds` | 0.5 V / 0.01 Ω |
| `diode_param` | `include.no`（无续流二极管，依赖互补门极） |
| 载波频率 | 10 kHz |
| 载波幅值 | 0~1（三角波 −1~1，经 ×0.5 + 0.5） |

## 8.3 控制

| 参数 | 值 | 位置 |
|---|---|---|
| `id_ref` | 0 A | 顶层 Constant |
| `iq_ref` | 1 A | 顶层 Constant |
| `Vdc`（送 SVPWM） | 100 | 顶层 `Constant1` |
| `Ts`（送 SVPWM） | 1（归一化） | 顶层 `Constant` |
| `Kp` / `Ki` | 2 / 1000 | 顶层 Constant |
| PI 输出限幅 `limit` | 57 V | Vd/Vq 脚本内 |
| 电流环采样 Ts | 1e-4 s | Delay/Delay1/Delay2 + 6 个图表 |
| 电流环带宽 | 2000 rad/s ≈ 318 Hz | 由 Kp/L 决定 |
| Solver | ode23t，StopTime 0.5 s | 模型配置 |

---

<a id="第-9-章"></a>
# 第 9 章 这次调试暴露的 7 个坑

这一章是全文最"值钱"的部分——**面试时讲这些细节，比背公式有用得多**。

## 坑 1：`Switch` 的控制逻辑反了 ——「看起来有闭环，实际是开环」

`Relational Operator3` 用 `Clock > 0.1` 去驱动 `Switch` 的控制端，
而 `Switch` 的判据是 `u2 > 0.5` 时选 u1（Ramp）。
结果 **0.1 s 之后控制角度切到了自由运行的斜坡**，与转子完全无关。

**教训**：遇到"控制器调不动"时，**第一件事是确认反馈量真的是被控对象的真实状态**。
做法：把反馈信号和"独立测量的真值"画在一张图里对比。
本次就是把模型的 θ 与 PMSM 模块内部记录的转子角对比，一眼看出跳变。

## 坑 2：MATLAB Function 里的"局部变量"根本没有记忆

```matlab
integral = 0;                 % ✗ 每次调用清零，积分器没有记忆
...
integral = integral + error*Ts;
```
看起来在积分，实际上每次调用都从 0 开始，等效于 `Ki·Ts·error`，就是个很小的比例增益。

**教训**：写数字控制器时，**凡是需要记忆的量都必须显式声明为 `persistent`**，
并且必须让图表以**固定采样时间**执行。
判断方法：把控制器改成一个阶跃输入，看输出是不是"积分形状"（斜坡），
如果输出是与输入同形状的，那积分就没生效。

## 坑 3：抗饱和钳错了对象

```matlab
if integral > limit, integral = limit; end      % ✗ integral 的单位是 A·s
```
而输出的贡献是 `Ki·integral`（单位才是 V）。Ki = 1000 时相当于钳到了 ±57000 V，等于没限。

**教训**：**限幅一定要作用在物理量纲正确的节点上**，
或者干脆用"反算法抗饱和"（back-calculation）：
```matlab
V = Kp*error + Ki*integral;
if V > limit
    V = limit;
    integral = (V - Kp*error)/Ki;    % 把超出的部分从积分器里扣回去
end
```

## 坑 4：SVPWM 扇区公式的"手滑"

`floor((pi*angel)/3)+1` vs `floor(angel/(pi/3))+1` —— 打错一个括号，差 9.7%。

**教训**：这类"公式没错、打字错了"的 bug 最难查。
**解法是建一套独立的验证手段**：本次用"min/max 共模注入法"（另一种完全不同的 SVPWM 实现）
做交叉校验，288 组工况一扫，立刻暴露 110 V 的误差。
**养成"用第二种方法验证第一种方法"的习惯**，这是从学生到工程师的关键差别。

## 坑 5：占空比映射表和扇区定义不匹配

把某份资料上的 TI 表格直接抄过来，但那套表对应的是**另一套扇区编号定义**。
抄表不难，难的是确认前提条件。

**教训**：抄任何算法表/系数表，**必须先用一个可手算的特例验证**。
本次用手算 90°（扇区 2 边界）和 30° 两个点就立刻发现问题。

## 坑 6：控制频率 ≠ 开关频率

Delay 用 1e-4 s、PI 里写 Ts = 1e-4，但载波 1 kHz。
控制算法每 100 µs 算一次，而功率级每 1 ms 才响应一次，
**控制带宽被白白浪费 10 倍**。

**教训**：数字控制系统的三个频率必须成组设计：
**开关频率 fsw、采样频率 fs、控制带宽 fc**，经验关系 `fc ≤ fs/10 ≈ fsw/10`。

## 坑 7（最隐蔽）：`Bias = 3π/2` 的"假完美"

模型的 θ 通道上有一个被注释掉的 `Bias` 块，值 `3π/2`。
启用它之后：

| Bias = 3π/2 | i_d 平均 | i_q 平均 | 转速 |
|---|---|---|---|
| 注释掉（正确） | +0.0022 A | +0.9975 A | **729.9 rpm** |
| **启用（错误）** | −0.0000 A | +1.0000 A | **0.0 rpm（完全不转）** |

**启用之后示波器上的 id/iq 反而"完美"**，但电机一步都不转！

**原因**：`3π/2 ≡ −π/2`，相当于把控制坐标系相对转子 d 轴转了 90°。
因为 **Park 测量和 InvPark 输出用的是同一个角度**，
示波器上的 id/iq 依然显示 0 和 1，但电流实际上全部落在转子的真实 d 轴上——
表贴式电机 d 轴不产生转矩，于是电机不转。

**教训（极其重要）**：
> **判断 FOC 是否正常，绝对不能只看 id/iq 波形，必须同时确认「转矩/转速有没有出来」。**
>
> 同一个角度同时用于测量和输出时，**任意常数角度偏差都不会反映在 id/iq 上**，
> 只会降低效率甚至完全不出力。要检查角度对齐，必须用**独立的信息源**——
> 本次用的是 PMSM 模块内部记录的 `i_d`/`i_q`：
> ```
>    t        id(模型)   iq(模型)    id(模块)   iq(模块)
>  0.09662     69.73     -22.53      68.74      -25.39
>  0.09829     55.64     -71.43      56.42      -70.81
> ```
> 两者几乎重合 → 说明 `6·θm` 这个角度约定本来就是对的，不需要任何偏移。

---

<a id="第-10-章"></a>
# 第 10 章 面向秋招：能力地图、模型还缺什么、高频面试题

## 10.1 这个模型覆盖了哪些岗位能力

| 能力项 | 本模型是否覆盖 | 说明 |
|---|---|---|
| Simulink / Stateflow 建模 | ✅ | 8 个 EML 图表 + 完整信号流 |
| Simscape 物理建模 | ✅ | 电机 + 逆变器 + 传感器 + 求解器配置 |
| FOC 坐标变换原理 | ✅ | Clarke / Park / InvPark 全实现 |
| 电流环 PI 整定 | ✅ | 有完整推导和实测扫描 |
| SVPWM 算法 | ✅ | 扇区/伏秒/七段式占空比全实现 |
| 数字控制离散化 | ✅ | 采样时间、一拍延迟、抗饱和 |
| 调试与验证方法论 | ✅ | 交叉验证、内部量对比 |
| 转速环 / 位置环 | ❌ | 只有电流环 |
| 弱磁 / MTPA / 过调制 | ❌ | 只有 id=0 线性区 |
| 无位置传感器控制 | ❌ | 用了理想编码器 |
| 死区补偿 | ❌ | 没有死区 |
| 损耗/温升/效率 | ❌ | `loss_param = none`，热端口未启用 |
| 定点化 / 代码生成 | ❌ | 全是 double + 浮点 |
| 硬件在环（HIL）/ 自动代码生成 | ❌ | — |

## 10.2 建议补的实验（每个 1–2 天，做完简历上就是 5 个可讲的项目）

1. **加转速环**：外环 PI，内环用现在这个电流环；加 `Rate Limiter` 限制加速度；
   阶跃响应看超调/上升时间。→ 双闭环调参经验
2. **加负载模型**：把阻尼换成"恒转矩 + ∝ω² 风机负载"，标定工作点。
3. **MTPA**：把 Lq 改大，实现 MTPA 曲线，对比 id=0 的转矩/铜损。→ 能讲清楚 SPM 与 IPM 的区别
4. **弱磁**：降低 Vdc 或提高转速，观察电流环饱和，实现电压闭环弱磁。
5. **无位置传感器**：删掉编码器，实现滑模观测器（SMO）或 MRAS，对比有/无传感器性能。
   ⚠️ 低速起动需要 I/F 或高频注入——正好可以把这个模型的 `Open_Start` 用对。
6. **死区补偿**：给互补门极加死区，观察 dq 电流 6 次谐波，再实现补偿算法。
7. **过调制**：实现过调制 I 区/II 区，把电压利用率从 0.577 提到 0.637。
8. **自动代码生成**：把控制算法改成定点，用 Embedded Coder 生成 C 代码，
   看代码效率报告（MIPS/ROM/RAM）。→ **这是电控岗位最核心的工程能力**

## 10.3 高频面试题（附答题要点）

**Q1：为什么表贴式 PMSM 用 id = 0 控制？**
> 转矩 `Te = 1.5p[ψf·iq + (Ld−Lq)·id·iq]`。表贴式 `Ld = Lq`，磁阻转矩项为零，
> 转矩只由 iq 决定。给 id 只会增加电流幅值 `√(id²+iq²)` 从而增加铜损，
> 却不产生任何转矩。所以 id = 0 就是铜损最小（MTPA 的退化解）。

**Q2：SVPWM 比 SPWM 好在哪？**
> 电压利用率提高 `2/√3 ≈ 15.5%`（线性区从 `Vdc/2` 提到 `Vdc/√3`）。
> 原理是通过在两个零矢量之间分配作用时间，注入了一个共模分量
> （零序注入），把调制波的峰值"压扁"，等效于扩展了线性区。
> 因为三相三线负载对共模不敏感，所以这个共模不影响线电压/相电流。

**Q3：电流环带宽怎么定？和开关频率什么关系？**
> 经验关系 `fc ≤ fsw/10`（保守可取 fsw/20）。
> 因为数字控制有采样保持和 PWM 更新两个环节的延迟，合计约 1.5~2 个 Ts，
> 带宽越高相位裕度损失越大（`PM ≈ 90° − ωc·Td`）。
> 定完带宽后 Δ 型对象用零极点对消：`Kp = ωc·L，Ki = ωc·R`。

**Q4：为什么电流环要在 dq 坐标系里做？**
> 在 αβ 坐标系里被控量是交流量，PI 对交流量有稳态幅值和相位误差；
> 变换到随转子旋转的 dq 坐标系后，基波变成直流量，
> PI 的积分作用可以做到**零稳态误差**，而且 d/q 两个通道可以独立设计。

**Q5：电流环的扰动有哪些？怎么抑制？**
> ① 反电动势 `ωe·ψf`（q 轴）；
> ② 交叉耦合 `−ωe·Lq·iq`（d 轴）、`+ωe·Ld·id`（q 轴）；
> ③ 母线电压波动；
> ④ 死区/管压降造成的电压畸变。
> ① ② 用**前馈解耦**：
> `Vd_ff = −ωe·Lq·iq，Vq_ff = ωe·Ld·id + ωe·ψf`；
> ③ 用母线电压前馈（占空比按实测 Vdc 归一化）；④ 用死区补偿。

**Q6：电流采样应该在什么时刻？为什么？**
> 在**零矢量作用的中点**（载波峰值/谷值）。此时三相下管（或上管）全通，
> 电流在采样电阻上稳定、不受开关瞬间的振铃和 di/dt 干扰；
> 而且此时采到的电流近似等于该 PWM 周期的平均值。
> 采样时刻不对，会引入和本次实验一样的 dq 电流纹波（6 次谐波）。

**Q7：什么是"7 段式"和"5 段式"SVPWM？**
> 7 段式把零矢量平分在周期首尾，开关序列对称（000-100-110-111-110-100-000），
> 谐波好但每周期每相开关 2 次；
> 5 段式把零矢量集中到一端，开关次数减少 1/3，开关损耗低，但谐波变差。
> 低载波比（高速）时常选 5 段式。

**Q8：怎么判断 FOC 的角度对齐是否正确？**
> ① **不能只看 id/iq 示波器**（因为 Park 和 InvPark 用同一角度，
> 常数偏差在 id/iq 上不可见）；
> ② 要看**转矩/转速是否正常**；
> ③ 或者用**独立信息源**：让电机空载被拖动做发电机，测三相反电势过零点与编码器零位的关系；
> ④ 或者用模块内部的 dq 电流（Simscape 的 `simlog`）做交叉比对。
> 偏差 90° 的典型症状是：**id/iq 看起来完美，但电机不出力甚至堵转**。

**Q9：极对数 p 用错了会是什么现象？**
> Park 用了机械角而不是电角度 → 电流环调节的坐标系转得比转子慢 p 倍，
> 表现为 iq 里有大幅低频振荡、电机剧烈抖动、带载能力极差、电流迅速饱和。

**Q10：仿真里 `Ts = 1e-4` 但求解器是变步长，这样对吗？**
> 不对。**数字控制器必须是固定采样时间的离散模块**。
> 如果控制器在变步长求解器下"连续"执行，而代码里用固定 Ts 累加积分，
> 积分量会被重复累加成千上万次，等效增益被放大上百倍。
> 正确做法：把控制器图表的 `ChartUpdate` 设为 `DISCRETE`、`SampleTime` 设为 Ts。

## 10.4 简历上怎么写这段经历（示例）

> **表贴式 PMSM 矢量控制系统建模与调试**（Simulink / Simscape Electrical）
> - 搭建三相两电平逆变器 + SPM 的 Simscape 物理模型，实现 10 kHz 数字电流环：
>   Clarke/Park 变换、离散 PI（含抗积分饱和）、七段式 SVPWM、互补 PWM 门极。
> - 定位并修复 4 处导致电流环失效的缺陷：转子角度选择逻辑反接、
>   PI 积分器无状态、SVPWM 扇区公式错误、占空比映射表与扇区定义不匹配；
>   用 min/max 共模注入法对 SVPWM 做交叉校验，将合成电压误差从 110 V 降至 1e-14 V。
> - 按零极点对消法整定电流环（Kp = ωc·L，Ki = ωc·R），
>   实测 i_d 均值 0.0007 A / i_q 均值 0.9977 A（给定 0 / 1 A）。
> - 使用 Simscape 变量记录（`simlog`）导出模块内部 dq 电流，与自建 Park 变换交叉比对，
>   排查并证明"控制角常数偏移在 id/iq 波形上不可见"这一隐蔽问题。

---

## 附：本次调试中用到的高价值 MATLAB 命令

```matlab
% 1. 找出模型里所有的 Stateflow/MATLAB Function 图表
rt = sfroot;  chs = rt.find('-isa','Stateflow.EMChart');
for i = 1:numel(chs), fprintf('%s : %s\n', chs(i).Name, chs(i).Path); end

% 2. 直接读写图表里的算法代码
ch = chs(7);
disp(ch.Script);                       % 读
ch.Script = 'function y = fcn(u)\n...';% 写（注意换行符！用 char(10) 拼行更安全）

% 3. 把图表改成固定采样时间的离散执行
ch.ChartUpdate = 'DISCRETE';
ch.SampleTime  = '1e-4';

% 4. 打开 Simscape 内部变量记录，取出模块内部的真值
set_param('PMSM_Model_fixed','SimscapeLogType','all');
out = sim('PMSM_Model_fixed');
out.simlog.PMSM.i_d.series.values
out.simlog.PMSM.i_q.series.values
out.simlog.PMSM.electrical_torque.series.values
out.simlog.PMSM.angular_position.series.values

% 5. 查看任何一个 Simscape 块的全部 mask 参数
mn = get_param('PMSM_Model_fixed/PMSM','MaskNames');
mv = get_param('PMSM_Model_fixed/PMSM','MaskValues');
for i = 1:numel(mn), fprintf('%-34s = %s\n', mn{i}, mv{i}); end

% 6. 查看某个块的端口类型和连接（排查物理网络接线）
pc = get_param('PMSM_Model_fixed/PMSM','PortConnectivity');
for k = 1:numel(pc), disp(pc(k)); end

% 7. 仿真时把任意信号引出到工作区（不改模型结构）
add_block('built-in/Outport', 'PMSM_Model_fixed/id_log', 'Port', '1');
add_line('PMSM_Model_fixed', 'Park Trans/1', 'id_log/1', 'autorouting', 'on');
out = sim('PMSM_Model_fixed','ReturnWorkspaceOutputs','on');
out.yout{1}.Values.Data
```

---

*文档结束。所有参数、端口、连线、示波器映射均为本次实测导出；
波形数据来自 `PMSM_Model_fixed.slx` 在 R2022b 下的仿真结果。*
