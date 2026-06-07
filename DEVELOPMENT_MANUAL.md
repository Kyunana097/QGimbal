# QGimbal 二维云台下位机开发手册

> STM32F407IGH6 | FreeRTOS | BMI088 IMU | QD4310 无刷电机 | CAN + UART 通信
>
> 最后更新: 2026-06-06

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件架构](#2-硬件架构)
3. [软件架构](#3-软件架构)
4. [FreeRTOS 任务](#4-freertos-任务)
5. [数据流详解](#5-数据流详解)
6. [IMU 子系统](#6-imu-子系统)
7. [电机控制子系统](#7-电机控制子系统)
8. [PID 控制库](#8-pid-控制库)
9. [串口通信协议](#9-串口通信协议)
10. [CAN 通信协议](#10-can-通信协议)
11. [启动流程 (时序图)](#11-启动流程)
12. [调试指南](#12-调试指南)
13. [关键参数速查](#13-关键参数速查)
14. [文件索引](#14-文件索引)

---

## 1. 系统概述

本程序实现了一个**二维云台稳定器**，使用大疆 C 板 (STM32F407IGH6) 作为主控芯片。

**核心功能:**
- 读取 IMU (BMI088) 数据，通过 Mahony 互补滤波计算云台欧拉角
- 双 PID 闭环控制 Yaw/Pitch 两个无刷电机实现姿态稳定
- 通过串口 (USART6) 接收上位机指令、回传遥测数据
- 通过 CAN 总线 (CAN1) 与 QD4310 电机驱动器通信
- PWM 控制激光笔

**控制频率:** IMU 采样和 PID 控制均为 ~1kHz

---

## 2. 硬件架构

```
┌─────────────────────────────────────────────────────────┐
│                    STM32F407IGH6                        │
│                                                         │
│  SPI1 ────────→ BMI088 (陀螺仪 + 加速度计)               │
│                 PA4=CS_ACCEL  PB0=CS_GYRO              │
│                 PC4=INT_ACCEL  PC5=INT_GYRO             │
│                                                         │
│  CAN1 ────────→ QD4310 电机驱动器 ×2                     │
│                 PD0=CAN1_RX  PD1=CAN1_TX               │
│                 YawMotor  ID=0x00 (CAN ID 0x400)        │
│                 PitchMotor ID=0x01 (CAN ID 0x401)       │
│                                                         │
│  USART6 ──────→ 上位机 (香橙派 / PC)                     │
│                 PG9=RX  PG14=TX  @115200 8N1            │
│                                                         │
│  TIM1_CH1 ────→ 激光笔 PWM (PE9, 1kHz)                  │
│                                                         │
│  HSI 16MHz → PLL→ 168MHz 系统时钟                       │
└─────────────────────────────────────────────────────────┘
```

### GPIO 引脚分配

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA4 | CS1_ACCEL | BMI088 加速度计片选 |
| PA7 | SPI1_MOSI | SPI 数据输出 |
| PB0 | CS1_GYRO | BMI088 陀螺仪片选 |
| PB3 | SPI1_SCK | SPI 时钟 |
| PB4 | SPI1_MISO | SPI 数据输入 |
| PC4 | INT1_ACCEL | 加速度计数据就绪中断 |
| PC5 | INT1_GYRO | 陀螺仪数据就绪中断 |
| PD0 | CAN1_RX | CAN 总线接收 |
| PD1 | CAN1_TX | CAN 总线发送 |
| PE9 | TIM1_CH1 | 激光笔 PWM 输出 |
| PG9 | USART6_RX | 串口接收 |
| PG14 | USART6_TX | 串口发送 |

### CAN 总线参数

| 参数 | 值 |
|------|-----|
| 波特率 | 1 Mbps |
| Prescaler | 3 |
| BS1 | 7 TQ |
| BS2 | 6 TQ |
| SJW | 2 TQ |
| 自动重传 | ENABLE |

---

## 3. 软件架构

```
QGimbal/
├── Core/                      ← STM32CubeMX 自动生成
│   ├── Inc/
│   │   ├── main.h             ← GPIO 宏定义, TIM1 声明
│   │   ├── can.h              ← CAN1 句柄
│   │   ├── usart.h            ← USART6 句柄
│   │   ├── FreeRTOSConfig.h   ← FreeRTOS 配置
│   │   └── stm32f4xx_hal_conf.h ← HAL 模块开关
│   └── Src/
│       ├── main.c             ← 入口, 时钟配置, TIM1 初始化
│       ├── freertos.c         ← 任务创建 (MX_FREERTOS_Init)
│       ├── can.c              ← CAN1 初始化
│       ├── usart.c            ← USART6 初始化
│       ├── stm32f4xx_it.c     ← 中断向量 (DMA2_Stream0 外移)
│       └── stm32f4xx_hal_msp.c ← 外设 MSP 初始化 (含 TIM1)
│
├── App/                       ← 应用层任务
│   ├── task_public.h          ← 任务函数声明 (extern "C")
│   ├── GimbalTask.cpp          ← 云台控制任务 + CAN 回调 ★
│   ├── TransmitTask.cpp        ← 串口收发任务 ★
│   └── DebugTask.cpp           ← 空闲调试任务
│
└── UserLib/                   ← 用户库
    ├── Gimbal/
    │   └── Gimbal.h            ← Gimbal 类 (稳定控制核心) ★
    ├── QD4310/
    │   ├── QD4310.h            ← 电机驱动类 (CAN 通信)
    │   └── QD4310.cpp          ← 电机驱动实现
    ├── PID/
    │   └── PID.h               ← PID 控制库
    └── BMI088/                 ← IMU 子系统
        ├── IMU_Task.cpp        ← IMU 任务 + DMA IRQ ★
        ├── IMU_Task.h          ← IMU 常量
        ├── MahonyAHRS.h        ← Mahony 姿态解算
        ├── bsp_spi.h/c         ← SPI DMA 驱动
        └── BMI088Driver/       ← BMI088 底层驱动
```

带 ★ 的文件是**核心业务文件**，理解它们就理解了整个系统。

---

## 4. FreeRTOS 任务

共有 **5 个任务**，在 `freertos.c:MX_FREERTOS_Init()` 中创建：

| 任务 | 文件 | 优先级 | 栈大小 | 职责 |
|------|------|--------|--------|------|
| **InsTask** | `IMU_Task.cpp` | `osPriorityRealtime` (最高) | 512B | IMU 读取 + 姿态解算, 通知 GimbalTask |
| **GimbalTask** | `GimbalTask.cpp` | `osPriorityNormal` | 1KB | PID 控制 + CAN 收发, 通知 TransmitTask |
| **TransmitTask** | `TransmitTask.cpp` | `osPriorityNormal` | 1KB | 组装遥测帧 → UART DMA 发送 |
| **ReceiveTask** | `TransmitTask.cpp` | `osPriorityNormal` | 1KB | UART 接收 → 命令分发 |
| **DebugTask** | `DebugTask.cpp` | `osPriorityNormal` | 512B | 空循环 (调试占位) |

### 任务间通信 (通知机制)

```
InsTask ──(xTaskNotifyGive)──→ GimbalTask
  "IMU 数据就绪, 请计算 PID"

GimbalTask ──(xTaskNotifyGive)──→ TransmitTask
  "PID 计算完成, 请发送遥测帧"

UART 空闲中断 ──(xQueueSendToBack)──→ ReceiveTask
  "收到 12 字节数据包, 请处理"
```

---

## 5. 数据流详解

### 5.1 主数据流 (IMU → PID → 电机)

```
┌──────────┐    BMI088 数据就绪中断 (PC4/PC5)
│ 硬件层    │
│ BMI088   │──→ GPIO EXTI 中断
│ 传感器    │    HAL_GPIO_EXTI_Callback()
└──────────┘    设置 gyro_update_flag / accel_update_flag
                     ↓
                imu_cmd_spi_dma()    ← 启动 SPI DMA 传输
                     ↓
                DMA 传输完成中断
                DMA2_Stream0_IRQHandler()  ← 在 IMU_Task.cpp 中自定义
                     ↓
                vTaskNotifyGiveFromISR(InsHandle)  ← 通知 IMU 任务
                     ↓
┌──────────┐
│ InsTask  │    BMI088_gyro_read_over()  ← 解析陀螺数据
│ (1kHz)   │    BMI088_accel_read_over() ← 解析加计数据
│          │    减去 gyro_bias (上电校准的零偏)
│          │    MahonyAHRS.update()      ← 姿态融合
│          │    get_angle()              ← 四元数→欧拉角
│          │    xTaskNotifyGive(GimbalTaskHandle)
└──────────┘
                     ↓
┌──────────┐
│ Gimbal   │    Ctrl_ISR(INS_angle[0], INS_angle[1])
│ Task     │       │
│ (1kHz)   │       ├─ 读取 YawMotor.angle / PitchMotor.angle (CAN 反馈)
│          │       ├─ PID_yaw_imu.calc(yaw_imu_angle)   → setCurrent()
│          │       ├─ PID_pitch_imu.calc(pitch_imu_angle) + gravity_comp*cos
│          │       │                                        → setCurrent()
│          │       └─ xTaskNotifyGive(TransmitTaskHandle)
└──────────┘
                     ↓
┌──────────┐    QD4310::setCurrent()
│ CAN 总线  │    SendCommand(CURRENT, value)
│ (1Mbps)  │    HAL_CAN_AddTxMessage(&hcan1, ...)
└──────────┘
                     ↓
┌──────────┐
│ QD4310   │    电机驱动器内部电流闭环 → 电机绕组 → 云台运动
│ 电机驱动  │
└──────────┘
```

### 5.2 串口接收流 (上位机 → STM32)

```
上位机发送 12 字节 ReceivePackage
         ↓
USART6 DMA 接收 → 空闲中断 (12 字节收完)
         ↓
HAL_UARTEx_RxEventCallback(huart6, 12)
    │
    ├─ Size == 12 → xQueueSendToBackFromISR()  入队
    └─ Size != 12 → 丢弃
         ↓
StartReceiveTask (解除阻塞)
    xQueueReceive() → ReceivePackage
    │
    ├─ 校验和验证 (前 11 字节和 == check_sum)
    ├─ laser_enabled  → TIM1_CH1 PWM 占空比
    ├─ enabled        → gimbal.enable() / disable()
    ├─ stability_enabled → gimbal.enable_stability() / disable_stability()
    └─ yaw_speed / pitch_speed → gimbal.Ctrl()
```

### 5.3 串口发送流 (STM32 → 上位机)

```
GimbalTask 每次 Ctrl_ISR 完成后
    xTaskNotifyGive(TransmitTaskHandle)
         ↓
StartTransmitTask (解除阻塞)
    填充 TransmitPackage:
      imu_angles[3]       ← INS_angle
      yaw/pitch_imu_angle  ← gimbal 成员
      yaw/pitch_motor_angle ← YawMotor/PitchMotor.angle
      laser_enabled        ← TIM1 比较值
      enabled / stability  ← gimbal 状态
      计算 checksum
    HAL_UART_Transmit_DMA(&huart6, ...)  ← DMA 发送 32 字节
```

### 5.4 CAN 反馈流 (电机 → STM32)

```
QD4310 电机主动上报 (CAN ID 0x500+Y)
         ↓
HAL_CAN_RxFifo0MsgPendingCallback()
    HAL_CAN_GetRxMessage()
    │
    ├─ StdId == 0x500 → YawMotor.update(rx_data)
    └─ StdId == 0x501 → PitchMotor.update(rx_data)
         ↓
    QD4310::update():
      enabled = rx[0] & 0x01
      current = rx[2:4] × 10A / INT16_MAX
      speed   = rx[4:6] × 1000rpm / INT16_MAX
      angle   = rx[6:8] × 2π / UINT16_MAX
```

---

## 6. IMU 子系统

**文件:** `UserLib/BMI088/IMU_Task.cpp`, `MahonyAHRS.h`

### 6.1 传感器

BMI088: 6 轴 IMU (3 轴陀螺仪 + 3 轴加速度计)

| 参数 | 值 |
|------|-----|
| 陀螺仪量程 | ±2000 dps |
| 加速度计量程 | ±3g |
| 陀螺灵敏度 | 0.001065 rad/s/LSB |
| 加速度灵敏度 | 0.000897 m/s²/LSB |

### 6.2 数据读取方式

SPI DMA 模式，由数据就绪引脚 (INT1) 外部中断触发:

1. 加速度计 INT1 → `HAL_GPIO_EXTI_Callback(INT1_ACCEL_Pin)` → 启动 SPI DMA
2. 陀螺仪 INT1 → `HAL_GPIO_EXTI_Callback(INT1_GYRO_Pin)` → 启动 SPI DMA
3. DMA 完成中断 → `DMA2_Stream0_IRQHandler()` → 通知 InsTask

### 6.3 姿态解算

使用 **Mahony 互补滤波** (MahonyAHRS.h):

$$\text{四元数 } \mathbf{q} = [q_0, q_1, q_2, q_3]$$

- sampleFreq = 1000 Hz
- twoKp = 1.0 (比例增益)
- twoKi = 0.0 (积分增益 — **未启用**, 因无磁力计)

$$\text{Mahony更新: } \mathbf{q}_{t+1} = \mathbf{q}_t + \dot{\mathbf{q}} \cdot \Delta t$$

加速度计仅修正 Pitch 和 Roll (重力矢量), Yaw 角无绝对参考。

四元数 → 欧拉角:

$$\text{yaw} = \arctan2(2(q_0q_3+q_1q_2), 2(q_0^2+q_1^2)-1)$$

$$\text{pitch} = \arcsin(-2(q_1q_3-q_0q_2))$$

$$\text{roll} = \arctan2(2(q_0q_1+q_2q_3), 2(q_0^2+q_3^2)-1)$$

### 6.4 陀螺零偏校准 (上电自动)

**位置:** `IMU_Task.cpp:80-96`

BMI088 初始化完成后、SPI DMA 启动前, 采集 500 个静止陀螺样本, 计算三轴均值作为零偏:

```cpp
for (int i = 0; i < 500; i++) {
    BMI088_read(gyro, accel, &temp);  // 阻塞读取
    gyro_bias[0] += gyro[0];
    gyro_bias[1] += gyro[1];
    gyro_bias[2] += gyro[2];
    osDelay(1);
}
gyro_bias[0/1/2] /= 500;
```

**要求:** 校准期间(约 0.5 秒)云台必须保持**完全静止**, 否则校准不准会导致 Yaw 漂移。

主循环中每次读取陀螺数据后减去零偏:

```cpp
bmi088_real_data.gyro[0] -= gyro_bias[0];
bmi088_real_data.gyro[1] -= gyro_bias[1];
bmi088_real_data.gyro[2] -= gyro_bias[2];
```

---

## 7. 电机控制子系统

**文件:** `UserLib/Gimbal/Gimbal.h`, `UserLib/QD4310/`

### 7.1 Gimbal 类

核心类, 持有两个电机引用 + 两个 PID 控制器。

**公有方法:**

| 方法 | 说明 |
|------|------|
| `enable()` | 使能两个电机 |
| `disable()` | 电机输出 0A 电流后禁用 |
| `enable_stability(return_to_center=false)` | 启动增稳, true 则 pitch 回零点 |
| `disable_stability()` | 关闭增稳 (电机以速度模式运行) |
| `Ctrl(yaw_rpm, pitch_rpm)` | 设置目标速度 (rpm, ±50) |
| `Ctrl_ISR(yaw_imu, pitch_imu)` | **核心控制函数**, 1kHz 调用 |

**成员变量:**

| 变量 | 说明 |
|------|------|
| `yaw_imu_angle` / `pitch_imu_angle` | 当前 IMU 角度 (rad) |
| `PID_yaw_imu` / `PID_pitch_imu` | 位置式 PID 控制器 |
| `pitch_max = 0.5 rad` | pitch 轴最大仰角 (±28.6°) |
| `gravity_comp = 1.0 A` | pitch 轴重力前馈补偿系数 |

### 7.2 Ctrl_ISR 控制逻辑

```
if (!enabled) return;

if (!stability_enabled):
    Yaw:   setSpeed(target_yaw_speed)      ← 速度模式
    Pitch: setSpeed(target_pitch_speed)     ← 速度模式 (有限位保护)

if (stability_enabled):
    Yaw:   更新 PID 目标 (+ Ctrl 输入)
           PID_yaw_imu.calc(yaw_imu_angle)
           setCurrent(PID输出)             ← 电流模式

    Pitch: 更新 PID 目标 (+ Ctrl 输入)
           限位检查 (pitch_max = 0.5 rad)
           PID_pitch_imu.calc(pitch_imu_angle)
           + gravity_comp × cos(pitch_imu_angle)  ← 重力前馈
           setCurrent(总输出)              ← 电流模式
```

**稳定模式 vs 非稳定模式:**

| 模式 | 控制方式 | 使用场景 |
|------|---------|---------|
| 稳定模式 ON | **电流控制** (setCurrent) | PID 位置闭环, 姿态稳定 |
| 稳定模式 OFF | **速度控制** (setSpeed) | 上位机手动操控, 限位活动 |

### 7.3 QD4310 电机驱动器

**CAN 命令:**

| 命令 | 代码 | 数据范围 | 说明 |
|------|------|---------|------|
| ENABLE | 0x01 | - | 使能电机 |
| DISABLE | 0x02 | - | 禁用电机 |
| CURRENT | 0x03 | ±10A | 电流 (扭矩) 控制 |
| SPEED | 0x04 | ±1000 rpm | 速度控制 |
| ANGLE | 0x05 | [0, 2π] rad | 绝对角度控制 |
| LOW_SPEED | 0x06 | ±1000 rpm | 低速模式 |
| STEP_ANGLE | 0x07 | ±2π rad | 相对角度步进 |

**CAN 地址:**
- YawMotor: `0x400 + 0x00 = 0x400`
- PitchMotor: `0x400 + 0x01 = 0x401`

**CAN 反馈 (电机上报):**
- YawMotor 反馈: `CAN ID 0x500`, 8 字节
- PitchMotor 反馈: `CAN ID 0x501`, 8 字节

**反馈数据格式 (QD4310::update):**

| 偏移 | 类型 | 含义 | 换算 |
|------|------|------|------|
| 0 | uint8 | 状态 (bit0=enabled) | - |
| 2 | int16 | 电流 | ×10/INT16_MAX → A |
| 4 | int16 | 转速 | ×1000/INT16_MAX → rpm |
| 6 | uint16 | 绝对角度 | ×2π/UINT16_MAX → rad |

### 7.4 电机物理参数 (Pitch, CAN ID=001)

| 参数 | 值 |
|------|-----|
| 极对数 | 14 |
| KV 值 | 33 rpm/V |
| 额定电压 | 24V |
| 相电感 | 4.74 mH |
| 相电阻 | 10.90 Ω |
| 扭矩常数 | 0.27 Nm/A |
| 最大电流 | 1.65 A |

---

## 8. PID 控制库

**文件:** `UserLib/PID/PID.h`

### 8.1 类型

| 类型 | 公式 | 用途 |
|------|------|------|
| `position_type` | $u = K_p e(t) + K_i \sum e(t) + K_d [e(t)-e(t-1)]$ | 云台位置稳定 |
| `delta_type` | $\Delta u = K_p[e(t)-e(t-1)] + K_i e(t) + K_d[e(t)-2e(t-1)+e(t-2)]$ | 增量式 |

### 8.2 Yaw 轴 PID 参数

| 参数 | 值 | 范围 |
|------|-----|------|
| kp | 5.0 | - |
| ki | 0.1 | - |
| kd | 110.0 | - |
| 积分限幅 | ±1.0 | - |
| 输出限幅 | ±1.8 A | - |

### 8.3 Pitch 轴 PID 参数

| 参数 | 值 | 范围 |
|------|-----|------|
| kp | 4.6 | - |
| ki | 0.17 | - |
| kd | 30.0 | - |
| 积分限幅 | ±1.0 | - |
| 输出限幅 | ±1.8 A | - |
| 重力前馈 | 1.0 A | cos(pitch_angle) 调制 |

### 8.4 PID 公式 (位置式)

```
error = target - input
sum_error += error                         (积分累加)
sum_error = clamp(sum_error, ±1.0)         (积分限幅)
output = kp × error + ki × sum_error + kd × (error - last_error)
output = clamp(output, ±1.8)               (输出限幅)
```

**注意:** `enable_stability()` 会调用 `set_sum_error(0)` 重置积分, 防止上次运行时残留的积分导致电流跳变。

---

## 9. 串口通信协议

**硬件:** USART6, PG9(RX) / PG14(TX), 115200 baud, 8N1

### 9.1 上位机 → STM32 (ReceivePackage, 12 字节)

| 偏移 | 类型 | 字段 | 说明 |
|------|------|------|------|
| 0 | float32 LE | yaw_speed | 偏航速度 (rpm, ±50) |
| 4 | float32 LE | pitch_speed | 俯仰速度 (rpm, ±50) |
| 8 | uint8 | laser_enabled | 0=关, 1=开, 其他=不变 |
| 9 | uint8 | enabled | 0=禁用, 1=使能, 其他=不变 |
| 10 | uint8 | stability_enabled | 0=关增稳, 1=开增稳, 其他=不变 |
| 11 | uint8 | check_sum | 前 11 字节累加和取低 8 位 |

**Python 打包示例:**
```python
import struct
data = struct.pack('<ffBBB', yaw_speed, pitch_speed, laser, enabled, stability)
checksum = sum(data) & 0xFF
packet = data + bytes([checksum])
```

**注意:** 必须一次性发送完整 12 字节, 中间不能有间隔, 否则 DMA 空闲中断会提前触发。

### 9.2 STM32 → 上位机 (TransmitPackage, 32 字节)

| 偏移 | 类型 | 字段 | 说明 |
|------|------|------|------|
| 0 | float32 LE ×3 | imu_angles | yaw, pitch, roll (rad) |
| 12 | float32 LE | yaw_imu_angle | 目标 yaw 角 (rad) |
| 16 | float32 LE | pitch_imu_angle | 目标 pitch 角 (rad) |
| 20 | float32 LE | yaw_motor_angle | yaw 电机角度 (rad) |
| 24 | float32 LE | pitch_motor_angle | pitch 电机角度 (rad) |
| 28 | uint8 | laser_enabled | 激光状态 |
| 29 | uint8 | enabled | 使能状态 |
| 30 | uint8 | stability_enabled | 增稳状态 |
| 31 | uint8 | check_sum | 前 31 字节累加和取低 8 位 |

**发送频率:** ~1kHz (跟随 GimbalTask 通知)

---

## 10. CAN 通信协议

### 10.1 STM32 → 电机 (命令)

| CAN ID | 目标 | 数据长度 | 格式 |
|--------|------|---------|------|
| 0x400 | YawMotor | 3 字节 | [cmd:1B][value:int16LE] |
| 0x401 | PitchMotor | 3 字节 | [cmd:1B][value:int16LE] |

### 10.2 电机 → STM32 (反馈)

| CAN ID | 来源 | 数据长度 | 格式 |
|--------|------|---------|------|
| 0x500 | YawMotor | 8 字节 | [status][reserved][current][speed][angle] |
| 0x501 | PitchMotor | 8 字节 | 同上 |

### 10.3 CAN 过滤器配置

```cpp
FilterBank = 0
FilterMode = IDMASK (标识符掩码模式)
FilterScale = 32BIT
FilterIdLow/High = 0x0000 0000
FilterMaskIdLow/High = 0x0000 0000  ← 接收所有 CAN 消息
FilterFIFO = FIFO0
SlaveStartFilterBank = 14
```

**实际生效:** 通过软件过滤, 只处理 `StdId == 0x500` 和 `0x501`。

---

## 11. 启动流程

```
上电
 │
 ├─ main()
 │   ├─ HAL_Init()
 │   ├─ SystemClock_Config()       168MHz
 │   ├─ MX_GPIO_Init()
 │   ├─ MX_DMA_Init()
 │   ├─ MX_SPI1_Init()
 │   ├─ MX_CAN1_Init()
 │   ├─ MX_USART6_UART_Init()
 │   ├─ MX_TIM1_Init()            激光 PWM
 │   ├─ osKernelInitialize()
 │   └─ osKernelStart()           启动 FreeRTOS 调度器
 │
 ├─ InsTask (最高优先级, 立即执行)
 │   ├─ osDelay(7 ticks)          等待稳定
 │   ├─ BMI088_init()             初始化传感器
 │   ├─ 陀螺零偏校准 (500 样本, ~0.5s)  ← 此时必须静止
 │   ├─ MahonyAHRS 初始化
 │   ├─ SPI1_DMA_init()
 │   └─ 主循环:
 │       ├─ 等待 DMA 完成通知
 │       ├─ 解析陀螺/加计数据
 │       ├─ 减去 gyro_bias
 │       ├─ MahonyAHRS.update()
 │       ├─ get_angle() → INS_angle[3]
 │       └─ 通知 GimbalTask
 │
 ├─ GimbalTask
 │   ├─ CAN_InterfaceInit()       设置 CAN 过滤器, 启动 CAN
 │   ├─ YawMotor.enable()         使能 yaw
 │   ├─ PitchMotor.enable()       使能 pitch
 │   ├─ osDelay(2000)             等待 IMU 初始化完成
 │   ├─ TIM1_CH1 PWM 开启 (激光)
 │   ├─ gimbal.enable()
 │   ├─ osDelay(50)
 │   ├─ gimbal.enable_stability(true)  ← pitch 回零点, 增稳启动
 │   └─ 主循环:
 │       ├─ 等待 InsTask 通知
 │       ├─ gimbal.Ctrl_ISR(INS_angle[0], INS_angle[1])
 │       └─ 通知 TransmitTask
 │
 ├─ TransmitTask (阻塞态, 等通知)
 │   收到通知 → 组包 → DMA 发送 32 字节遥测 → 再次阻塞
 │
 └─ ReceiveTask (阻塞态, 等队列数据)
     收到数据 → 校验 → 分发命令 → 再次阻塞
```

**关键时序:**
- t=0: 上电
- t≈10ms: InsTask 开始陀螺校准 (0.5s)
- t≈510ms: IMU 开始输出姿态角
- t≈2000ms: GimbalTask 结束等待, 开启增稳
- t≈2000ms+: 云台正常工作

---

## 12. 调试指南

### 12.1 关键断点

| 文件:行 | 用途 |
|--------|------|
| `GimbalTask.cpp:67` | Ctrl_ISR 调用 (每秒~1000次) |
| `GimbalTask.cpp:89-101` | CAN 反馈回调 |
| `TransmitTask.cpp:47` | 遥测帧发送后 |
| `TransmitTask.cpp:72` | 收到上位机数据包 |
| `TransmitTask.cpp:97` | UART 空闲中断回调 |
| `IMU_Task.cpp:109` | Mahony 更新 |

### 12.2 关键监视变量

| 变量 | 含义 |
|------|------|
| `INS_angle[0/1/2]` | IMU 欧拉角 (rad) |
| `YawMotor.angle` | yaw 电机编码器角度 (rad) |
| `PitchMotor.angle` | pitch 电机编码器角度 (rad) |
| `YawMotor.current` | yaw 电机反馈电流 (A) |
| `PitchMotor.current` | pitch 电机反馈电流 (A) |
| `gimbal.enabled` | 是否使能 |
| `gimbal.stability_enabled` | 是否增稳 |
| `receive_package` | 上位机发来的最新指令 (在 StartReceiveTask 中) |
| `gyro_bias[2]` | Z 轴陀螺零偏校准值 (rad/s) |

### 12.3 串口测试 (PC 直连)

```bash
# 用 USB-UART 连接 STM32 的 PG14(TX) 和 PG9(RX)
# 运行 Python 测试脚本
python3 tools/gimbal_test.py --port COM3 monitor

# 发送使能命令
python3 tools/gimbal_test.py --port COM3 enable
```

波特率: 115200, 8N1

### 12.4 常见问题

| 症状 | 可能原因 | 排查方法 |
|------|---------|---------|
| 电机不转 | stability_enabled 未设? | 检查 `gimbal.stability_enabled` |
| 电机发烫 | 持续大电流 | 监视 `PitchMotor.current`, 检查重力前馈 |
| Yaw 不停旋转 | 陀螺校准不准 | 检查 `gyro_bias[2]`, 确保上电静止 |
| 串口无响应 | 校验失败 / 包长不对 | 在 RxEventCallback 打断点, 看 Size 值 |
| 上电抖动 | 驱动器纯 P 控制振荡 | 已修复 (去掉 setAngle) |
| 无法回零 | enable_stability() 未传 true | 检查调用参数 |

---

## 13. 关键参数速查

### 控制参数

| 参数 | 位置 | 值 |
|------|------|-----|
| 控制周期 (Ts) | GimbalTask.cpp:34 | 0.001s (1kHz) |
| pitch_max | Gimbal.h:131 | 0.5 rad (~28.6°) |
| gravity_comp | Gimbal.h:132 | 1.0 A |
| yaw PID (kp/ki/kd) | GimbalTask.cpp:24-26 | 5.0 / 0.1 / 110.0 |
| pitch PID (kp/ki/kd) | GimbalTask.cpp:30-32 | 4.6 / 0.17 / 30.0 |
| PID 输出限幅 | GimbalTask.cpp:25/31 | ±1.8 A |
| PID 积分限幅 | GimbalTask.cpp:26/32 | ±1.0 |

### 通信参数

| 参数 | 位置 | 值 |
|------|------|-----|
| UART 波特率 | usart.c:44 | 115200 |
| CAN 波特率 | can.c:41-45 | 1 Mbps |
| 遥测包大小 | TransmitTask.cpp:14-24 | 32 字节 |
| 指令包大小 | TransmitTask.cpp:26-33 | 12 字节 |

### 任务参数

| 任务 | 优先级 | 栈大小 |
|------|--------|--------|
| InsTask | osPriorityRealtime | 512B |
| GimbalTask | osPriorityNormal | 1KB |
| TransmitTask | osPriorityNormal | 1KB |
| ReceiveTask | osPriorityNormal | 1KB |

---

## 14. 文件索引

| 文件 | 行数 | 主要内容 |
|------|------|---------|
| `Core/Src/main.c` | 227 | main(), 时钟配置, TIM1 PWM 初始化 |
| `Core/Src/freertos.c` | 175 | 5 个 FreeRTOS 任务创建 |
| `Core/Src/can.c` | 126 | CAN1 初始化 (1Mbps) |
| `Core/Src/usart.c` | 163 | USART6 初始化 (115200) |
| `Core/Inc/main.h` | 85 | 外设句柄声明, GPIO 宏 |
| `App/GimbalTask.cpp` | 102 | GimbalTask 任务, CAN 回调, Gimbal 初始化 |
| `App/TransmitTask.cpp` | 109 | 串口收发任务, 协议结构体 |
| `UserLib/Gimbal/Gimbal.h` | 148 | Gimbal 类 (稳定控制核心) |
| `UserLib/QD4310/QD4310.cpp` | 52 | 电机 CAN 命令实现 |
| `UserLib/QD4310/QD4310.h` | 64 | 电机驱动类定义 |
| `UserLib/PID/PID.h` | 116 | PID 控制库 |
| `UserLib/BMI088/IMU_Task.cpp` | 221 | IMU 任务, 陀螺校准, DMA IRQ |
| `UserLib/BMI088/MahonyAHRS.h` | 196 | Mahony 互补滤波 |
