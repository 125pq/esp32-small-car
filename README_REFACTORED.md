# ESP32 麦轮小车项目技术文档（Refactored）

本文档基于当前 main 目录实际代码整理，聚焦工程结构、运行逻辑和参数语义。

更新时间：2026-03-19

## 1. 当前代码快照

### 1.1 真实目录（无 lib 子目录）

```text
main/
├── main.ino
├── Config.h
├── Motor.h/.cpp
├── MecanumControl.h/.cpp
├── WebControl.h/.cpp
├── LineTracker.h/.cpp
├── LineFollower.h/.cpp
├── Ultrasonic.h/.cpp
├── ObstacleAvoidance.h/.cpp
└── Display.h/.cpp
```

### 1.2 功能总览

- Web 手动控制：全向平移 + 旋转 + 速度滑条
- 自动巡线：模式映射 + 右转路口动作 + 遇障后后程流程
- 自动避障：前进-后退-随机转向状态机
- 运动执行：麦轮逆运动学 + 速度斜率限制

## 2. 系统启动与主循环

### 2.1 setup() 时序

1. 初始化串口与 I2C
2. 初始化 OLED（失败则停机）
3. 初始化超声波、巡线、电机
4. 执行电机测试 motor.test()
5. MPU6050 初始化与零偏校准
6. 初始化 MecanumControl
7. 连接 WiFi，启动 WebServer
8. 绑定巡线/避障对象到 WebControl
9. LED 闪烁提示初始化完成

### 2.2 loop() 时序

1. webControl.handleClient()
2. mpu6050.update()
3. lineFollower.update()
4. obstacleAvoidance.update()
5. mecanumControl.update()
6. delay(1)

说明：两个自动模块都通过 mecanumControl.setTargetVelocity() 写目标速度，后执行的模块会覆盖前者目标值。

## 3. 模块职责与依赖

### 3.1 Config

文件：main/Config.h

集中定义：

- 引脚映射
- WiFi 与端口
- 麦轮几何参数
- 速度限制与平滑参数
- 巡线和后程策略参数

### 3.2 Motor

文件：main/Motor.h, main/Motor.cpp

职责：4 路电机 PWM 与方向控制。

注意：

- 电机 2 和电机 4 使用反向逻辑
- 电机 4 单独走特殊方向分支

### 3.3 MecanumControl

文件：main/MecanumControl.h, main/MecanumControl.cpp

职责：将目标速度 (vx, vy, omega) 转换为四轮角速度并输出 PWM。

当前实现是开环控制，不做 PID 闭环角度修正（angleOutput 固定为 0）。

逆运动学：

$$
\begin{aligned}
w_1 &= \frac{V_x - V_y - (L_x+L_y)\omega}{R} \\
w_2 &= \frac{V_x + V_y + (L_x+L_y)\omega}{R} \\
w_3 &= \frac{V_x + V_y - (L_x+L_y)\omega}{R} \\
w_4 &= \frac{V_x - V_y + (L_x+L_y)\omega}{R}
\end{aligned}
$$

输出前会做两级限制：

- 目标到执行速度的斜率限幅（slew rate）
- 轮速与 PWM 限幅

### 3.4 LineTracker

文件：main/LineTracker.h, main/LineTracker.cpp

职责：读取 4 路巡线传感器并编码为 4 bit 状态。

位组合逻辑：state = x2 | (x1 << 1) | (x3 << 2) | (x4 << 3)

### 3.5 LineFollower

文件：main/LineFollower.h, main/LineFollower.cpp

职责：巡线主状态机，包含：

- 基础 pattern 映射控制
- 右转路口触发与动作序列
- 巡线中超声检测
- 遇障后后程流程（Retreat -> ReverseTrack -> GarageMove）

关键点：

- 传感器语义为 0=在线，1=离线
- 支持 Web 实时调参（LFP）与恢复默认（LFR）
- 右转逻辑为“预延时直行 + 固定时长旋转 + 回线确认”

### 3.6 ObstacleAvoidance

文件：main/ObstacleAvoidance.h, main/ObstacleAvoidance.cpp

职责：独立自动避障状态机：

- FORWARD：前进巡航
- BACKWARD：遇障后退
- TURN_LEFT / TURN_RIGHT：随机转向

注意：该模块阈值与时长常量在 ObstacleAvoidance.cpp 内定义：

- OBSTACLE_DISTANCE = 5.0 cm
- BACKWARD_DURATION = 500 ms
- TURN_DURATION = 400 ms

### 3.7 WebControl

文件：main/WebControl.h, main/WebControl.cpp

职责：WiFi 建链、HTTP 路由、控制页面、指令分发、巡线参数下发。

### 3.8 Ultrasonic / Display

文件：main/Ultrasonic.h/.cpp, main/Display.h/.cpp

- Ultrasonic：pulseIn 20ms 超时测距
- Display：OLED 显示封装（含 showMecanumInfo 调试页）

## 4. 引脚映射

来自 main/Config.h。

| 类别 | 宏 | 默认值 |
|---|---|---|
| 巡线 | IO_X1/IO_X2/IO_X3/IO_X4 | 35/34/39/36 |
| 电机 PWM | IO_M1PWM/2PWM/3PWM/4PWM | 32/18/33/19 |
| 电机方向 | IO_M1IN1/2, IO_M2IN1/2, IO_M3IN1/2, IO_M4IN1/2 | 14/13, 17/5, 26/27, 16/4 |
| 超声波 | IO_TRIG/IO_ECHO | 23/25 |
| OLED | OLED_ADDRESS | 0x3C |
| 指示灯 | LED_PIN | 2 |

## 5. 关键配置参数

### 5.1 基础运动与平滑

| 参数 | 默认值 | 说明 |
|---|---:|---|
| MAX_LINEAR_SPEED | 0.4 | 最大线速度 m/s |
| MAX_ROTATION_SPEED | 0.8 | 最大角速度 rad/s |
| DEAD_ZONE | 0.01 | 输入死区 |
| COMMAND_LINEAR_SLEW_RATE | 1.2 | 线速度变化率限制 |
| COMMAND_OMEGA_SLEW_RATE | 4.0 | 角速度变化率限制 |

### 5.2 巡线默认参数（编译期）

| 参数 | 默认值 |
|---|---:|
| LF_PATTERN_SLIGHT_TURN_RATIO | 0.45 |
| LF_PATTERN_MEDIUM_TURN_RATIO | 0.75 |
| LF_PATTERN_LARGE_TURN_RATIO | 0.90 |
| LF_PATTERN_SLIGHT_SPEED_RATIO | 0.92 |
| LF_PATTERN_MEDIUM_SPEED_RATIO | 0.80 |
| LF_PATTERN_LARGE_SPEED_RATIO | 0.65 |
| LF_START_BOOST_MS | 4000 |
| LF_START_BOOST_RATIO | 1.35 |
| LF_RIGHT_TURN_OMEGA_RATIO | 6.0 |
| LF_RIGHT_TURN_90_MS | 1000 |
| LF_RIGHT_TURN_PRE_DELAY_MS | 350 |
| LF_OBSTACLE_DISTANCE_CM | 8.0 |
| LF_OBSTACLE_RETREAT_MS | 1000 |
| LF_OBSTACLE_RETREAT_MAX_MS | 3000 |

## 6. Web API 规格

### 6.1 路由

| 路径 | 方法 | 作用 |
|---|---|---|
| / | GET | 返回控制页面 |
| /control | GET | 执行控制命令 |
| /lineParams | GET | 返回巡线参数 JSON |

### 6.2 control 命令

| cmd | 参数 | 作用 |
|---|---|---|
| F/B/L/R | - | 前后/左右平移 |
| FL/FR/BL/BR | - | 斜向移动 |
| CL/CR | - | 原地左旋/右旋 |
| S | - | 停车并停止 LF/OA |
| SP | val=0..100 | 设置速度比例 |
| LF | val=1/0 | 启停巡线 |
| OA | val=1/0 | 启停避障 |
| LFP | key + val | 修改巡线调参项 |
| LFR | - | 巡线参数恢复默认 |

### 6.3 LFP 可调键位

| key | 范围 | 默认值 | 说明 |
|---|---|---:|---|
| pst | 0.10..2.00 | 0.45 | 轻微偏移转向比例 |
| pmt | 0.20..2.50 | 0.75 | 中等偏移转向比例 |
| plt | 0.30..5.00 | 0.90 | 大偏移转向比例 |
| pss | 0.30..1.00 | 0.92 | 轻微偏移速度比例 |
| pms | 0.30..1.00 | 0.80 | 中等偏移速度比例 |
| pls | 0.20..1.00 | 0.65 | 大偏移速度比例 |
| sbd | 0..5000 | 4000 | 启动加速时长 ms |
| sbr | 1.00..2.00 | 1.35 | 启动加速倍率 |
| rpd | 0..1200 | 350 | 右转前直行补偿 ms |
| orm | 100..3000 | 1000 | 后程最短后退 ms |
| orx | 300..6000 | 3000 | 后程最长后退 ms |
| prb | 0.10..1.50 | 0.62 | 后程后退速度比例 |
| prl | 0.05..1.20 | 0.86 | 后程左移速度比例 |
| prv | 0.10..1.50 | 0.82 | 倒车循迹后退比例 |
| prg | 0.05..1.50 | 0.42 | 倒车循迹横向增益 |
| prm | 0.05..1.00 | 0.26 | 倒车循迹横向限幅 |
| psb | 0..2000 | 220 | 阶段切换平滑时长 ms |
| pgm | 300..8000 | 1350 | 入库动作时长 ms |
| pgx | 0.10..1.20 | 0.67 | 入库前向速度比例 |
| pgy | 0.10..1.20 | 0.85 | 入库右移速度比例 |

## 7. 与旧文档的关键差异

- 旧文档中的 lib/ 路径不再适用，实际模块都在 main/
- 当前运动控制为开环麦轮模型，不是 PID 闭环控制
- 增加了巡线后程流程与 Web 端参数热调能力
- Web 页面支持 /lineParams 参数回读和 LFR 恢复默认

## 8. 最小上手步骤

1. 在 main/Config.h 修改 WIFI_SSID 和 WIFI_PASSWORD
2. 按 Config.h 完成接线
3. Arduino IDE 选择 ESP32 板并安装依赖库
4. 烧录 main/main.ino
5. 上电后等待 OLED 显示 IP，用浏览器访问

## 9. 调试建议

- 首次启动 MPU 校准时保持小车静止
- 电机方向异常优先检查 Motor.cpp 的反向逻辑和接线
- 巡线不稳定时，先降低 SP，再调 pst/pmt/plt 与 pss/pms/pls
- 旋转偏慢优先调 WEB_ROTATE_BOOST

## 10. 许可证

见仓库根目录 LICENSE。
