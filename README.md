# ESP32 麦轮小车（Web 控制 + 巡线 + 避障）

基于 ESP32 的四轮麦轮小车控制项目，支持：

- 网页遥控（前后左右、斜移、原地旋转、速度调节）
- 自动巡线（含右转路口处理与后程入库流程）
- 自动避障（超声波检测 + 后退 + 随机转向）
- OLED 状态显示（WiFi/IP/调试信息）

当前代码已模块化，核心都在 main 目录下。

## 目录结构

```text
esp32-small-car/
├── main/
│   ├── main.ino
│   ├── Config.h
│   ├── Motor.h/.cpp
│   ├── MecanumControl.h/.cpp
│   ├── WebControl.h/.cpp
│   ├── LineTracker.h/.cpp
│   ├── LineFollower.h/.cpp
│   ├── Ultrasonic.h/.cpp
│   ├── ObstacleAvoidance.h/.cpp
│   └── Display.h/.cpp
├── README.md
└── README_REFACTORED.md
```

## 硬件与依赖

### 硬件

- ESP32 开发板
- L298N 电机驱动 + 4 个电机（麦轮底盘）
- HC-SR04 超声波
- 4 路巡线传感器
- 0.96" I2C OLED（SSD1306，地址 0x3C）
- MPU6050

### Arduino 依赖库

- Adafruit GFX Library
- Adafruit SSD1306
- MPU6050_tockn

说明：WiFi.h、WebServer.h、Wire.h 为 ESP32/Arduino 环境自带库。

## 快速开始

### 1. 配置 WiFi

编辑 main/Config.h：

```cpp
#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"
```

### 2. 连接引脚

关键引脚（详见 main/Config.h）：

- 巡线：IO_X1=35, IO_X2=34, IO_X3=39, IO_X4=36
- 超声波：TRIG=23, ECHO=25
- 电机 PWM：32, 18, 33, 19
- 电机方向：14/13, 17/5, 26/27, 16/4
- LED：2

### 3. 编译与烧录

1. 使用 Arduino IDE 打开 main/main.ino
2. 选择开发板（ESP32 Dev Module 或兼容板）
3. 安装上面的依赖库
4. 编译并上传

### 4. 上电流程

启动时程序会依次执行：

1. OLED 初始化
2. 超声波/巡线/电机初始化
3. 电机自检（motor.test）
4. MPU6050 校准（小车需静止）
5. WiFi 连接并启动 Web 服务
6. OLED 显示 IP

## Web 控制说明

浏览器访问 OLED 显示的 IP 地址（端口默认 80）。

### 页面功能

- 方向：前进、后退、左移、右移、停止
- 斜移：左前、右前、左后、右后
- 旋转：左旋、右旋
- 模式：巡线启动/停止、避障启动/停止
- 调参：巡线参数滑块实时下发

### 核心接口

- GET /
	- 返回控制页面
- GET /control?cmd=...&val=...
	- 下发动作、模式、速度、巡线参数
- GET /lineParams
	- 返回巡线参数 JSON

常用 cmd：

- F/B/L/R/FL/FR/BL/BR/CL/CR/S
- SP（速度 0-100）
- LF（巡线开关，val=1/0）
- OA（避障开关，val=1/0）
- LFP（巡线参数设置，带 key+val）
- LFR（巡线参数恢复默认）

## 模式关系与优先级

- 通过网页按钮启动时，巡线与避障会互斥（启动 A 会先停 B）。
- 紧急停止 S 会同时停止巡线与避障。
- loop 中调用顺序为：
	- webControl.handleClient
	- mpu6050.update
	- lineFollower.update
	- obstacleAvoidance.update
	- mecanumControl.update
- 如果异常同时开启两个自动模式，后执行的 obstacleAvoidance 目标速度会覆盖前者。

## 关键配置说明

主要参数都在 main/Config.h：

- 速度上限：MAX_LINEAR_SPEED, MAX_ROTATION_SPEED
- 指令平滑：COMMAND_LINEAR_SLEW_RATE, COMMAND_OMEGA_SLEW_RATE
- 巡线参数：LF_PATTERN_*、LF_RIGHT_TURN_*、LF_OBSTACLE_*、LF_POST_*
- 手动旋转补偿：WEB_ROTATE_BOOST

补充：ObstacleAvoidance.cpp 内还有独立阈值常量（如 OBSTACLE_DISTANCE=5.0cm）。

## 常见问题

### WiFi 连不上

- 检查 SSID/密码
- 确保热点是 2.4GHz
- 观察串口输出与 OLED 提示

### 巡线偏移或抖动

- 先调速度（SP）
- 再调基础巡线参数（pst/pmt/plt/pss/pms/pls）
- 必要时调整 LF_TURN_LEFT_GAIN / LF_TURN_RIGHT_GAIN

### 旋转感觉偏慢

- 调整 WEB_ROTATE_BOOST（main/Config.h）

## 许可证

见 LICENSE。
