# STM32F103C8T6 四轴飞控

一块 **STM32F103C8T6** 上的裸机四轴飞行控制固件：没有 RTOS，主循环跑一次 2ms 节拍切出 6 档任务，三组串级 PID 把遥控杆量算成四路 PWM 占空比。

配套的**单文件可视化文档**（11 张图，离线可看）：[`flight-control-index.html`](./flight-control-index.html)

> 手机上看：下载上面那个文件用浏览器打开，双指放大或横屏更清楚。

---

## 这个仓库是什么

- **固件**：`Flight_Control/`，CMake + STM32CubeMX 工程，可直接编译烧录。
- **可视化文档**：`flight-control-index.html`，把固件拆成 7 节 11 张图（模块关系、6 档节拍时序、生命周期、阅读路线、数据结构、函数调用链、分层架构）。每张图里点节点能看到 `文件:行号` 来源。
- **规划中的内容**：遥控器固件、原理图、PCB、学习笔记（见文末「路线图」）。

代码规模：应用部分 87 个 `.c/.h`，10,606 行；另有 ST 官方 HAL/CMSIS 77 个源文件原样引入。

---

## 目录结构

```
STM32F103C8T6_Quadcopter/
├── flight-control-index.html   可视化文档（自包含，12 MB）
├── Flight_Control.xmind        思维导图
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
| MPU6050 | 软件 I2C，PB6=SCL、PB7=SDA | ±2000 °/s、±4 g，DLPF 42 Hz，采样 333 Hz；上电 256 组零偏标定 |
| SPL06 气压计 | 同一条软件 I2C | 气压 / 温度；标定系数开机加载 |
| NRF24L01 | SPI2 + CE=PA11、CSN=PB12 | 频道 66、2 Mbps、地址 `AABBCC0001`、32 字节动态载荷 |
| PMW3901 光流 | USART1，单字节中断 | 19200 8N1，11 字节帧，`qual ≥ 25` 才算有效 |
| 调试打印 | USART2 | 115200；`printf` 已开 `-u _printf_float` 支持 `%f` |
| 四路电机 | TIM2_CH1/2=PA0/PA1，TIM3_CH1/2=PB4/PB5（部分重映射） | PSC=3、ARR=999 → 18 kHz（推算），占空比 0~1000 |
| 电池采样 | ADC1_IN8=PB0 + DMA 循环 | 12 bit，分压 1/2，采样 55.5 周期 |
| 独立看门狗 | IWDG | PSC=64、Reload=625（LSI 40 kHz 时约 1 s，推算） |
| 参数存储 | 内部 Flash 末两页 Page62/63 | 24 个 `uint16` 虚拟地址，双页轮换磨损均衡 |

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
| 1000 ms | 500 | `Task_1000ms` | 遥控帧数锁存、光流在线统计、`printf` 汇总 |

节拍采用“跳过丢拍”策略：主循环卡顿 10 ms 只补执行 1 拍，不堆积任务。

---

## 控制环

- **3 组串级 PID，共 12 个环**：姿态（角度→角速度，6 个）、高度（位置→速度，2 个）、光流定点（位置→速度，4 个）。
- **`ki` 全为 0**：纯 P / PD，没有积分抗静差；缓降另有一个私有环（`kp=1.0 / ki=0.25 / kd=0`），不在参数表里。
- **状态机命名**：每个状态机都有自己的枚举，定义在各自的 `.c` 文件里，不再共用一套 1/2/11/31/255 的编号。共同的骨架是 `IDLE → 中间态 → RUN → EXIT`：

| 状态机 | 文件 | 状态序列 |
|---|---|---|
| 姿态 | `control_attitude.c` | `ATT_IDLE` → `ATT_RESET`（复位 8 个 PID，本拍不输出）→ `ATT_RUN` → `ATT_EXIT` |
| 混控 | `control_mixer.c` | `MIX_IDLE` → `MIX_ARMED`（等油门 > 1100 或起飞/抛飞）→ `MIX_RUN` → `MIX_EXIT` |
| 定高 | `control_height.c` | `HGT_IDLE` → `HGT_READY`（记录油门保持值）→ `HGT_RUN` → `HGT_EXIT` |
| 缓降 | `control_descent.c` | `DESC_IDLE` → `DESC_ACTIVE` → `DESC_DONE` |
| 解锁 | `remote.c` | `RMT_IDLE` → `RMT_WAIT_HIGH` → `RMT_WAIT_LOW` → `RMT_ARM` → `RMT_RUN` → `RMT_EXIT` |

  这套命名只是改了标识符，**逻辑与数值行为完全没变**。

- **混控矩阵**（X 型四轴，对角同转向）：

```
m1 = base + rate_x - rate_y + rate_z
m2 = base + rate_x + rate_y - rate_z
m3 = base - rate_x + rate_y + rate_z
m4 = base - rate_x - rate_y - rate_z
```

- **几个关键阈值**：`ARM_THR=1100`（进入控制的油门门限）、`THR_CUT=1020`（低于此值四路强制 0）、`BASE_LIMIT=700`（基准上限，留 300 给姿态修正）、`MOTOR_LIMIT=1000`（单路上限 = 一个 PWM 周期）。
- **解锁三步**：油门 < 1150 → > 1800 保持 6 拍 → 再 < 1150；右下角手势（油门 < 1200 且 yaw > 1800）任意时刻立即上锁。

---

## 编译

需要：CMake ≥ 3.22、Ninja、`arm-none-eabi-gcc`（工具链文件 `cmake/gcc-arm-none-eabi.cmake`，`-mcpu=cortex-m3`）。

```bash
cd Flight_Control
cmake --preset Debug
cmake --build --preset Debug
```

产物是 `build/Debug/Flight_Control.elf`（以及 `.map`）。工程没有内置 objcopy 步骤，需要 `.bin` 自己转：

```bash
arm-none-eabi-objcopy -O binary build/Debug/Flight_Control.elf build/Debug/Flight_Control.bin
```

用 ST-Link / DAP-Link 下载 elf 或 bin 均可。编译配置里已经用 `-Os -g3` 压体积并保留调试信息（C8T6 只有 64 KB Flash）。

> `build/` 已在 `.gitignore` 里，不会进仓库。

---

## 怎么读这份代码

1. `Core/Src/main.c` — 时钟怎么配、外设 Init 顺序、主循环只有一句 `Scheduler_Run()`。
2. `Middleware/APP/Src/app.c` — 初始化链的依赖顺序都在这（先 I2C 才能 Init MPU6050，先 NRF 才能注册发送钩子）。
3. `Middleware/SCHEDULER/Src/scheduler.c` — 6 档任务怎么分派。
4. `Middleware/CONTROL/Inc/control.h` — **先读文件头注释**：状态编号、全部阈值宏、每个函数的调用节拍与 `dt`。
5. `Middleware/CONTROL/Src/control_attitude.c` → `control_mixer.c` — 串级怎么串、四路怎么算。
6. `Driver/MOTOR/Src/motor.c` → `HALPort/PWM_PORT/Src/pwm_port.c` → `Core/Src/tim.c` — 最短的一条“到底”链：`duty → 通道映射 → __HAL_TIM_SET_COMPARE → TIM2->CCR1`。

更细的路线（13 步）和逐图讲解都在[可视化文档](./flight-control-index.html)里。

---

## 已知的坑

- `Task_10ms` 里各个调用的 `dt` 并不统一：`Pixel_Flow_Fix` / `Control_Position_Update` / `Control_Height_Update` 传 `0.006f`，`Control_Height_Get` / `Control_Descent_Update` 传 `0.01f`，`Control_Height_Calc` 传的是 `10`（毫秒）。这是代码原样。
- `Control_Init()` 只写增益和业务标志，**不清 PID 内部状态**（`integ / prevError / out`）；要在解锁那一拍靠 `PID_Reset` 统一清零。
- `IMU_Get_Angle` 的 `dt` 必须与 6 ms 真实周期一致，否则积分增益和收敛速度都会偏。
- 光流 `loc_x` 来自 `out_y_i`、`loc_y` 来自 `out_x_i`（模块安装方向决定），不要“顺手改正”。

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
