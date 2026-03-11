/**
 * @file Config.h
 * @brief ESP32小车全局配置文件
 * @author 喵屋
 * @date 2025.12.4
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/**
 * @brief 巡线传感器引脚定义
 */
#define IO_X1 35
#define IO_X2 34
#define IO_X3 39
#define IO_X4 36

/**
 * @brief 电机接口定义
 * 使用L298N电机驱动模块，每个电机需要PWM信号和两个方向控制引脚
 */
#define IO_M1PWM 32  // 电机1 PWM速度控制
#define IO_M2PWM 18  // 电机2 PWM速度控制
#define IO_M3PWM 33  // 电机3 PWM速度控制
#define IO_M4PWM 19  // 电机4 PWM速度控制
#define IO_M1IN1 14  // 电机1 方向控制1
#define IO_M1IN2 13  // 电机1 方向控制2
#define IO_M2IN1 17  // 电机2 方向控制1
#define IO_M2IN2 5   // 电机2 方向控制2
#define IO_M3IN1 26  // 电机3 方向控制1
#define IO_M3IN2 27  // 电机3 方向控制2
#define IO_M4IN1 16  // 电机4 方向控制1
#define IO_M4IN2 4   // 电机4 方向控制2

/**
 * @brief 超声波接口定义
 */
#define IO_TRIG 23   // 超声波触发引脚
#define IO_ECHO 25   // 超声波回波引脚

/**
 * @brief OLED显示定义
 */
#define SCREEN_WIDTH 128   // OLED宽度
#define SCREEN_HEIGHT 64   // OLED高度
#define OLED_ADDRESS 0x3C  // OLED I2C地址

/**
 * @brief WiFi配置
 */
#define WIFI_SSID "park"           // WiFi名称
#define WIFI_PASSWORD "86534633"   // WiFi密码
#define UDP_PORT 3000              // UDP监听端口
#define WEB_PORT 80                // Web服务器端口

/**
 * @brief 麦轮小车参数
 */
#define WHEEL_RADIUS 0.03  // 轮子半径 (米)
#define LX 0.085           // 前后轮距的一半 (米)
#define LY 0.08            // 左右轮距的一半 (米)

/**
 * @brief PID控制参数
 */
#define ANGLE_KP 2.0       // 角度PID比例系数
#define ANGLE_KI 0.01      // 角度PID积分系数
#define ANGLE_KD 0.0       // 角度PID微分系数

#define SPEED_KP 0.5       // 速度PID比例系数
#define SPEED_KI 0.01      // 速度PID积分系数
#define SPEED_KD 0.05      // 速度PID微分系数

#define DEAD_ZONE 0.01      // 死区阈值

/**
 * @brief 速度限制参数
 */
#define MAX_LINEAR_SPEED 0.15   // 最大线速度 (m/s)
#define MAX_ROTATION_SPEED 1.0 // 最大旋转速度 (rad/s)

/**
 * @brief 滤波器参数
 */
#define ANGLE_FILTER 0.8   // 角度滤波器系数 (0-1, 越高越平滑)
#define GYRO_FILTER 0.9    // 陀螺仪滤波器系数

/**
 * @brief 其他配置
 */
#define LED_PIN 2          // 板载LED引脚

#endif // CONFIG_H
