# ESP32 麦轮小车项目 - 重构版

## 项目简介

本项目是一个基于 ESP32 的四轮麦轮小车控制系统，支持 Web 网页控制、麦轮运动学、PID 控制等功能。代码已重构为模块化架构，便于阅读和维护。

## 项目结构

```
esp32-small-car/
├── main/
│   └── main.ino              # 主程序文件（简化后）
├── lib/
│   ├── Config/
│   │   └── Config.h          # 全局配置文件（引脚定义、参数配置）
│   ├── Motor/
│   │   ├── Motor.h           # 电机控制头文件
│   │   └── Motor.cpp         # 电机控制实现
│   ├── Ultrasonic/
│   │   ├── Ultrasonic.h      # 超声波传感器头文件
│   │   └── Ultrasonic.cpp    # 超声波传感器实现
│   ├── LineTracker/
│   │   ├── LineTracker.h     # 巡线传感器头文件
│   │   └── LineTracker.cpp   # 巡线传感器实现
│   ├── Display/
│   │   ├── Display.h         # OLED显示头文件
│   │   └── Display.cpp       # OLED显示实现
│   ├── MecanumControl/
│   │   ├── MecanumControl.h  # 麦轮运动控制头文件
│   │   └── MecanumControl.cpp# 麦轮运动控制实现
│   └── WebControl/
│       ├── WebControl.h      # Web服务器控制头文件
│       └── WebControl.cpp    # Web服务器控制实现
└── README_REFACTORED.md      # 本说明文件
```

## 模块说明

### 1. Config（配置模块）
- **文件**: `lib/Config/Config.h`
- **功能**: 集中管理所有引脚定义和全局参数
- **内容**:
  - 电机引脚（PWM和方向控制）
  - 传感器引脚（超声波、巡线）
  - WiFi配置
  - 麦轮参数（轮径、轮距）
  - PID参数
  - 滤波器参数

### 2. Motor（电机控制模块）
- **文件**: `lib/Motor/Motor.h`, `lib/Motor/Motor.cpp`
- **功能**: 控制四个电机的方向和速度
- **主要方法**:
  - `init()`: 初始化电机引脚
  - `setSpeed(s1, s2, s3, s4)`: 设置四个电机速度（-255 ~ 255）
  - `stop()`: 停止所有电机
  - `test()`: 电机测试（依次运行各电机）

### 3. Ultrasonic（超声波传感器模块）
- **文件**: `lib/Ultrasonic/Ultrasonic.h`, `lib/Ultrasonic/Ultrasonic.cpp`
- **功能**: 超声波测距
- **主要方法**:
  - `init()`: 初始化超声波引脚
  - `getDistance()`: 获取距离（厘米）
  - `getRawPulse()`: 获取原始脉冲时间

### 4. LineTracker（巡线传感器模块）
- **文件**: `lib/LineTracker/LineTracker.h`, `lib/LineTracker/LineTracker.cpp`
- **功能**: 四路巡线传感器状态读取
- **主要方法**:
  - `init()`: 初始化巡线传感器引脚
  - `getState()`: 获取四个传感器状态（4位二进制）
  - `getSensor(n)`: 获取单个传感器状态

### 5. Display（OLED显示模块）
- **文件**: `lib/Display/Display.h`, `lib/Display/Display.cpp`
- **功能**: OLED显示屏控制
- **主要方法**:
  - `init()`: 初始化OLED
  - `show(text1, text2, text3)`: 显示1-3行文本
  - `showMecanumInfo(...)`: 显示麦轮小车实时信息
  - `clear()`: 清空显示

### 6. MecanumControl（麦轮运动控制模块）
- **文件**: `lib/MecanumControl/MecanumControl.h`, `lib/MecanumControl/MecanumControl.cpp`
- **功能**: 麦轮运动学计算和PID控制
- **主要方法**:
  - `init()`: 初始化PID控制器
  - `setTargetVelocity(vx, vy, omega)`: 设置目标速度
  - `update()`: 执行PID控制（在loop中调用）
  - `getTargetVelocity(...)`: 获取当前目标速度
  - `getAngleOutput()`: 获取角度PID输出

### 7. WebControl（Web服务器控制模块）
- **文件**: `lib/WebControl/WebControl.h`, `lib/WebControl/WebControl.cpp`
- **功能**: WiFi连接和Web服务器
- **主要方法**:
  - `init()`: 初始化WiFi和Web服务器
  - `handleClient()`: 处理客户端请求（在loop中调用）
  - `isConnected()`: 检查WiFi连接状态
  - `getIPAddress()`: 获取IP地址

## 使用方法

### 1. 配置WiFi
在 `lib/Config/Config.h` 中修改：
```cpp
#define WIFI_SSID "your_wifi_name"
#define WIFI_PASSWORD "your_password"
```

### 2. 编译上传
使用 Arduino IDE 或 PlatformIO：
```bash
# PlatformIO
pio run -t upload
```

### 3. Web控制
上传成功后，OLED会显示IP地址，在浏览器中访问该地址即可控制小车。

## 主要特性

1. **模块化设计**: 每个功能独立封装，易于维护和扩展
2. **清晰的代码结构**: 类似 STM32 项目的组织方式
3. **完整的注释**: 中英文注释，便于理解
4. **集中配置**: 所有参数在 Config.h 中统一管理
5. **Web控制界面**: 支持方向控制、斜向移动、原地旋转、速度调节

## 依赖库

需要在 Arduino IDE 或 PlatformIO 中安装以下库：
- Adafruit SSD1306
- Adafruit GFX Library
- MPU6050_tockn
- PID_v1

## 硬件连接

详细的引脚连接请参考 `lib/Config/Config.h` 中的定义。

## 注意事项

1. 首次运行会进行MPU6050校准，需要保持小车静止
2. 电机测试会依次运行各电机，确保电机接线正确
3. WiFi连接失败会显示在OLED上
4. 建议在测试时降低速度参数

## 与原版对比

**原版（main.ino 731行）**:
- 所有代码在一个文件中
- 难以定位和修改特定功能
- 参数分散在各处

**重构版（main.ino 约100行）**:
- 功能分模块，每个模块独立文件
- 便于阅读和维护
- 参数集中管理
- 类似其他两个项目的架构

## 后续扩展

添加新功能时，只需：
1. 在 `lib/` 下创建新模块文件夹
2. 编写对应的 `.h` 和 `.cpp` 文件
3. 在 `main.ino` 中包含并使用

例如添加蓝牙控制：
```cpp
// lib/BluetoothControl/BluetoothControl.h
// lib/BluetoothControl/BluetoothControl.cpp
// 在 main.ino 中 #include "BluetoothControl.h"
```

## 作者

原作者：喵屋 (2025.9.18)  
重构：2025.12.4

## 许可证

请参考项目根目录的 LICENSE 文件
