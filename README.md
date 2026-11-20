# STM32F103C8T6 四轴飞控

一块 **STM32F103C8T6** 上的裸机四轴飞行控制固件：没有 RTOS，主循环跑一次 2ms 节拍切出 6 档任务，三组串级 PID 把遥控杆量算成四路 PWM 占空比。

配套的**单文件可视化文档**（11 张图，离线可看）：[`飞控代码可视化文档.html`](./飞控代码可视化文档.html)

> 手机上看：下载上面那个文件用浏览器打开，双指放大或横屏更清楚。

---

## 这个仓库是什么

- **固件**：`Flight_Control/`，CMake + STM32CubeMX 工程，可直接编译烧录。
- **可视化文档**：`飞控代码可视化文档.html`，把固件拆成 7 节 11 张图（模块关系、6 档节拍时序、生命周期、阅读路线、数据结构、函数调用链、分层架构）。每张图里点节点能看到 `文件:行号` 来源。
- **设计说明**：`四轴飞控设计笔记.xmind`，记录**为什么这么写**——CubeMX 的 11 步配置（含不能用硬件 I2C1 的 ST 勘误）、构建模式取舍、每个模块的设计意图。
- **规划中的内容**：遥控器固件、原理图、PCB、学习笔记（见文末「路线图」）。

代码规模：应用部分 87 个 `.c/.h`，10,606 行；另有 ST 官方 HAL/CMSIS 77 个源文件原样引入。

资源占用（Debug 构建实测）：**FLASH 59,636 B / 64 KB（91.0%）**，**RAM 4,128 B / 20 KB（20.2%）**。Flash 余量只剩约 5.9 KB，加功能前先算一下空间。

---

## 三层材料怎么配合

这个仓库刻意把信息分成三层，各回答一个问题：

| 层 | 材料 | 回答的问题 |
|---|---|---|
| **为什么这么设计** | `四轴飞控设计笔记.xmind` | 为什么用 DWT 不用 SysTick？为什么不用硬件 I2C1？64KB 装不下时改什么？ |
| **模块怎么连** | [`飞控代码可视化文档.html`](./飞控代码可视化文档.html) | 谁给谁送数据、6 档节拍里谁先谁后、状态怎么迁移、一层怎么拼到另一层 |
| **具体怎么算** | 源码里的逐行注释 | 每个函数的边界条件、状态迁移条件、公式与阈值从哪来 |

建议的顺序：**先看 xmind 知道“为什么” → 再看页面知道“谁跟谁连” → 最后读代码看“一行一行怎么算”**。

> `四轴飞控设计笔记.xmind` 需要 XMind 打开（2020 及以后版本）。文件里另存了一份 `content.xml` 兼容占位，用第三方工具打开只会看到一张警告图，这是 XMind 的正常行为，不是文件损坏。

---

## 目录结构

```
STM32F103C8T6_Quadcopter/
├── 飞控代码可视化文档.html   可视化文档：怎么连（7 张图，自包含 12 MB）
├── 四轴飞控设计笔记.xmind        设计说明：为什么这么写（需 XMind 打开）
└── Flight_Control/             固件工程
    ├── Core/                   CubeMX 生成：main.c、时钟、各外设 Init、中断（23 个文件）
    ├── Driver/                 自写外设驱动：MPU6050、SPL06、NRF24L01、MOTOR、BATTERY（12）
    ├── HALPort/                端口适配：PWM/SPI/I2C_SOFT/UART1/ADC/IWDG/DWT/LED/EEPROM（20）
    ├── Middleware/             算法与业务：APP、SCHEDULER、CONTROL、PID、IMU、FLOW、
    │                           REMOTE、ANO_DT、ANO_PARAM、LED_CTRL、FILTER…（32）
    ├── Drivers/                ST 官方 HAL + CMSIS，不要改
    ├── cmake/                  工具链文件与 CubeMX 生成的构建脚本
    ├── CMakeLists.txt / CMakePresets.json
    ├── Flight_Control.ioc      CubeMX 工程文件
    └── STM32F103XX_FLASH.ld    链接脚本
```

> 注意目录名：`Driver`（单数，自己写的驱动）、`HALPort`（端口适配）是作者的命名；`Drivers`（复数）才是 ST 官方包。

---

## 硬件与外设

| 功能 | 总线 / 引脚 | 关键参数 |
|---|---|---|
| 主控与时钟 | HSE 8 MHz，PLL ×9 | SYSCLK 72 MHz；APB1 36 MHz、APB2 72 MHz；ADC 由 PCLK2/6 供钟 |
| MPU6050 | 软件 I2C，PB6=SCL、PB7=SDA | ±2000 °/s、±4 g，DLPF 42 Hz，采样 333 Hz；上电先等静止（≤500 ms 超时）再采 256 组，只校 az 与陀螺零偏，不要求水平放置 |
| SPL06 气压计 | 同一条软件 I2C | 气压 / 温度；标定系数开机加载 |
| NRF24L01 | SPI2 + CE=PA11、CSN=PB12 | 频道 66、2 Mbps、地址 `AABBCC0001`、32 字节动态载荷 |
| PMW3901 光流 | USART1，单字节中断 | 19200 8N1，11 字节帧，`qual ≥ 25` 才算有效 |
| 调试打印 | USART2 | 115200；`printf` 已开 `-u _printf_float` 支持 `%f` |
| 四路电机 | TIM2_CH1/2=PA0/PA1，TIM3_CH1/2=PB4/PB5（部分重映射） | PSC=3、ARR=999 → 18 kHz（推算），占空比 0~1000 |
| 电池采样 | ADC1_IN8=PB0 + DMA 循环 | 12 bit，分压 1/2，采样 55.5 周期 |
| 独立看门狗 | IWDG | PSC=64、Reload=625（LSI 40 kHz 时约 1 s，推算） |
| 参数存储 | 内部 Flash 末两页 Page62/63 | 26 个 `uint16` 虚拟地址（0~23 PID、24/25 配平），双页轮换磨损均衡 |

“推算”= 由代码常量算出来的量，代码里没有直接写这个数字。

---

## 软件分层

```
main.c / app.c / scheduler.c        应用与调度
        ↓
CONTROL · PID · IMU · FLOW · REMOTE 中间件（纯算法，不含 HAL 头文件）
ANO_DT · ANO_PARAM · LED_CTRL
        ↓
MPU6050 · NRF24L01 · SPL06 · MOTOR 设备驱动
+ PWM_PORT · SPI_PORT · I2C_SOFT    端口适配
UART1_PORT · ADC_PORT · IWDG_PORT
        ↓
ST HAL + CMSIS → TIMx->CCR / GPIOx->BSRR
```

中间件层不认识任何寄存器：换 MCU 只改最下面两层。协议层与硬件解耦靠注册钩子，例如 `ANO_DT_Set_Tx_Hook(NRF24L01_Tx_Packet)`。

---

## 调度：一个 2ms 节拍切 6 档

`Scheduler_Run()` 在主循环里被反复调用，用 DWT 微秒计数判断是否满 2 ms；每满一拍，6 个计数器一起 +1，各自到阈值就执行任务并清零。

| 档位 | 阈值（2ms 计数） | 任务 | 做什么 |
|---|---|---|---|
| 2 ms | 1 | `Task_2ms` | 喂狗 → 读 MPU6050 → 姿态串级 PID → 混控 → 写 CCR |
| 4 ms | 2 | `Task_4ms` | NRF 收包 + 功能字分发 + 模式状态机 + 遥测 6 拍轮转 |
| 6 ms | 3 | `Task_6ms` | 四元数 + Mahony 姿态解算（全固件唯一写 `s_angle` 的地方） |
| 10 ms | 5 | `Task_10ms` | 遥控解析、气压、光流、定点、高度、缓降、0x02 帧回发 |
| 50 ms | 25 | `Task_50ms` | 电池低压迟滞判定、校准命令、LED 状态机 |
| 1000 ms | 500 | `Task_1000ms` | 遥控帧数锁存、光流在线统计、`printf` 汇总（行尾 `fw=` 版本号） |

节拍采用“跳过丢拍”策略：主循环卡顿 10 ms 只补执行 1 拍，不堆积任务。

---

## 控制环

- **3 组串级 PID，共 12 个环**：姿态（角度→角速度，6 个）、高度（位置→速度，2 个）、光流定点（位置→速度，4 个）。
- **`ki` 全为 0**：纯 P / PD，没有积分抗静差；缓降另有一个私有环（`kp=1.0 / ki=0.25 / kd=0`），不在参数表里。
- **状态机命名**：每个状态机都有自己的枚举，定义在各自的 `.c` 文件里，共同的骨架是 `IDLE → 中间态 → RUN → EXIT`：

| 状态机 | 文件 | 状态序列 |
|---|---|---|
| 姿态 | `control_attitude.c` | `ATT_IDLE` → `ATT_RESET`（复位 8 个 PID，本拍不输出）→ `ATT_RUN` → `ATT_EXIT` |
| 混控 | `control_mixer.c` | `MIX_IDLE` → `MIX_ARMED`（等油门 > 1100 或起飞/抛飞）→ `MIX_RUN` → `MIX_EXIT` |
| 定高 | `control_height.c` | `HGT_IDLE` → `HGT_READY`（记录油门保持值）→ `HGT_RUN` → `HGT_EXIT` |
| 缓降 | `control_descent.c` | `DESC_IDLE` → `DESC_ACTIVE` → `DESC_DONE` |
| 解锁 | `remote.c` | `RMT_IDLE` → `RMT_WAIT_HIGH` → `RMT_WAIT_LOW` → `RMT_ARM` → `RMT_RUN` → `RMT_EXIT` |

- **混控矩阵**（X 型四轴，对角同转向）：

```
m1 = base + rate_x - rate_y + rate_z
m2 = base + rate_x + rate_y - rate_z
m3 = base - rate_x + rate_y + rate_z
m4 = base - rate_x - rate_y - rate_z
```

- **几个关键阈值**：`ARM_THR=1100`（进入控制的油门门限）、`THR_CUT=1020`（低于此值四路强制 0）、`BASE_LIMIT=700`（基准上限，留 300 给姿态修正）、`MOTOR_LIMIT=1000`（单路上限 = 一个 PWM 周期）。
- **解锁三步**：油门 < 1150 → > 1800 保持 6 拍 → 再 < 1150；右下角手势（油门 < 1200 且 yaw > 1800）任意时刻立即上锁。
- **高度源与定点窗口**：光流高度在 50~4000 mm 且 `Flow.ok` 时优先；激光失效时先保持最后有效值 1.5 s，再以 50 cm/s 平滑切到气压高度；光流定点只在 `control_high ∈ (20, 400) cm` 且摇杆在 ±150 死区内接管，出界即退回摇杆直通。

**灯语**（`LED_Ctrl_Update`，20 Hz，自上而下优先级递减）：

| 状态 | 灯效 |
|---|---|
| 初始化失败 | 4 灯 5 Hz 方波（死循环内） |
| 校准中 | 4 灯常亮 |
| 失控锁定 | 急闪 300 ms 周期 |
| 上电准备中 | 4 灯流水 600 ms 一轮（上电 3 s 内且未解锁） |
| 链路丢失 | 慢闪 2 s 周期 |
| 电池低 | 双闪 800 ms |
| 锁定待机 | 快闪 500 ms |
| 解锁·无头 | 全亮 |
| 解锁·有头 | B+H 亮；A 亮 = 光流定点接管，A 灭 = 摇杆直通 |

上电时四元数直接用首帧加速度初始化到当前倾角（tilt init），不再从 0° 慢慢爬；校准完成后的 3 秒流水只是“准备中”提示，之后没连上遥控就转慢闪、连上了就转锁定快闪。若去掉 tilt init，Mahony 的时间常数约 1.25 s，斜面开机要等 6~8 秒才收敛。

**配平**（`control_trim.c`）：未解锁时把油门、横滚、俯仰三个摇杆同时拉到最低（偏航不参与）、保持 2 秒进入测量——静止 2 秒 → 提示原地转 180° → 再静止 2 秒，两次读数取平均。地面倾斜在两次读数里符号相反、平均后抵消，剩下的就是机体固定偏置（芯片零偏 + 安装倾斜）；结果存 EEPROM，之后所有期望角都减去它。因为偏置属于机体，**换场地不用重测**；要重测再触发一次即可。

---

## 编译

需要：CMake ≥ 3.22、Ninja、`arm-none-eabi-gcc`（工具链文件 `cmake/gcc-arm-none-eabi.cmake`，`-mcpu=cortex-m3`）。

```bash
cd Flight_Control
cmake --preset Debug
cmake --build --preset Debug
```

上电（或复位）会先打印一行版本横幅：

```
=== STM32F103C8T6 Quadcopter  FW v0.11.1 ===
```

版本号在 `Flight_Control/Middleware/APP/Inc/fw_version.h` 里手动维护，改功能/修 bug 时递增；1 秒调试行的行尾也有 `fw=`。

产物是 `build/Debug/Flight_Control.elf`（以及 `.map`）。工程没有内置 objcopy 步骤，需要 `.bin` 自己转：

```bash
arm-none-eabi-objcopy -O binary build/Debug/Flight_Control.elf build/Debug/Flight_Control.bin
```

用 ST-Link / DAP-Link 下载 elf 或 bin 均可。编译配置里已经用 `-Os -g3` 压体积并保留调试信息（C8T6 只有 64 KB Flash）。

> `build/` 已在 `.gitignore` 里，不会进仓库。

---

## 怎么读这份代码

0. **先看 `四轴飞控设计笔记.xmind`** —— 每个模块“为什么这么写”都在那里，比从代码倒推快得多。
1. `Core/Src/main.c` — 时钟怎么配、外设 Init 顺序、主循环只有一句 `Scheduler_Run()`。
2. `Middleware/APP/Src/app.c` — 初始化链的依赖顺序都在这（先 I2C 才能 Init MPU6050，先 NRF 才能注册发送钩子）。
3. `Middleware/SCHEDULER/Src/scheduler.c` — 6 档任务怎么分派。
4. `Middleware/CONTROL/Inc/control.h` — **先读文件头注释**：五个状态机的命名约定、全部阈值宏、每个函数的调用节拍与 `dt`。
5. `Middleware/CONTROL/Src/control_attitude.c` → `control_mixer.c` — 串级怎么串、四路怎么算。
6. `Driver/MOTOR/Src/motor.c` → `HALPort/PWM_PORT/Src/pwm_port.c` → `Core/Src/tim.c` — 最短的一条“到底”链：`duty → 通道映射 → __HAL_TIM_SET_COMPARE → TIM2->CCR1`。

更细的路线（13 步）和逐图讲解都在[可视化文档](./飞控代码可视化文档.html)里。

### 想自己从零搭一遍？

上面是**读**的顺序（跟着调用关系走）。**搭**的顺序正好相反：从 DWT / LED 这种最底层开始，一次只加一条链路，每加一层都有一个看得见的验收现象。

`Flight_Control/CMakeLists.txt` 里 `target_sources` 的分组注释就是按这个顺序排的（12 个阶段，顺序请勿重排）；页面「④ 阅读路线」末尾也有一张同样的表，带每个阶段的验收标准。

---

## 已知的坑

- `Task_10ms` 里 6 个调用的 `dt` 分两种标度：`Control_Height_Get` / `Control_Descent_Update` / `Control_Height_Calc` 用真实 10ms（`TASK_10MS_DT`），另外三个 `Pixel_Flow_Fix` / `Control_Position_Update` / `Control_Height_Update` 用 `TASK_10MS_DT_CTRL = 0.006f`（控制环标度）。这几个环的微分项按该标度整定，改成 0.01 会让它们整体变成 60%。宏与理由写在 `scheduler.c` 顶部。
- `Control_Init()` 只写增益和业务标志，**不清 PID 内部状态**（`integ / prevError / out`）；要在解锁那一拍靠 `PID_Reset` 统一清零。
- `IMU_Get_Angle` 的 `dt` 必须与 6 ms 真实周期一致，否则积分增益和收敛速度都会偏。
- 光流 `loc_x` 来自 `out_y_i`、`loc_y` 来自 `out_x_i`（模块安装方向决定），不要“顺手改正”。
- 光流 `cpi` 随激光高度（50~4000 mm）换算，保证不同高度下 `loc` 都是真实厘米；激光无效时退回 50 cm 标定值。高度先做低通再算，避免增益抖动。
- 姿态的“水平”参考由首帧加速度直接初始化、之后由 Mahony 用重力持续修正，开机只要求静止、不要求放平；加计 `ax/ay` 的固定残余（芯片零偏 + 安装倾斜）由遥控器微调补偿。
- **低空地面效应**：离地一个桨径（5 寸约 12.7 cm）以内，推力变大、气压高度被桨流污染、光流位移失真；20 cm 以下没有光流定点，只剩姿态保持，0.5° 的固定偏置就会变成漂移。起降时快速通过低空段、别在 20 cm 以下悬停/定点，气压计加海绵隔离，桨先做平衡。

---

## 可视化文档是怎么来的

- 每张图都是一个独立的 Archify HTML，用 `data:text/html;base64,…` 内嵌进页面（首屏那张不用 `loading="lazy"`，其余都懒加载），所以整页**不发任何网络请求**，断网也能看。
- 页面底部有一行 **构建标记**（时间 + `v3-scroll-fix` 之类的版本名）。**看不到这一行就是你打开的是旧文件或浏览器缓存**，按 `Ctrl+F5`（手机上拉刷新）强制刷新。
- 图里 `meta.repository.revision` 指向源码基线提交 `3d0522b`；页面文件本身在更晚的提交里，因此**页面所在 commit 与图内 revision 不同**是正常的。
- 图里标「源码确证」的是直接读到的代码，标「推算」的是由代码常量算出的量（例如 PWM 频率、看门狗超时）。

---

## 路线图

- [x] 飞控固件（本仓库）
- [x] 单文件可视化文档
- [ ] 遥控器（发射端）固件
- [ ] 原理图
- [ ] PCB
- [ ] 相关学习笔记 / 调参记录

新内容会作为与 `Flight_Control/` 同级的新目录加进来。

---

## 免责声明

这是一个学习 / 实验项目，没有做任何适航或安全认证。试飞有风险，请自负其责：

- **调试和烧录时先拆桨**，只靠 USB 供电验证逻辑。
- 首次试飞在开阔无人的场地，先用小油门确认姿态方向正确。
- 电池低压保护、失联保护都只是软件兜底，不能替代飞手判断。

## 许可证

本项目使用 **MIT 许可证**，全文见 [LICENSE](./LICENSE)。

用大白话说清楚它意味着什么：

- **你可以随便用**：复制、修改、拿去商用、改成闭源再发布，都不用问作者；
- **只有一个要求**：保留版权声明和许可证原文，也就是别把作者名字删掉；
- **作者不担责任**：飞坏了、炸机了、烧了板子，都由使用的人自己负责。

第三方代码不在这份许可证范围内：`Flight_Control/Drivers/` 下的 ST 官方 HAL 库与 CMSIS 有各自的 `LICENSE.txt`，以它们为准。
