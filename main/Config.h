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
 * @brief 运动控制参数
 */
#define DEAD_ZONE 0.01              // 死区阈值

// 指令变化率限制（用于抑制电机抖动）
#define COMMAND_LINEAR_SLEW_RATE 1.2f   // 线速度最大变化率 (m/s^2)
#define COMMAND_OMEGA_SLEW_RATE 4.0f    // 角速度最大变化率 (rad/s^2)

// 基础循迹参数
#define LF_PATTERN_SLIGHT_TURN_RATIO 0.45f   // 轻微偏移转向比例
#define LF_PATTERN_MEDIUM_TURN_RATIO 0.75f   // 中等偏移转向比例
#define LF_PATTERN_LARGE_TURN_RATIO 3.00f    // 大偏移转向比例
#define LF_PATTERN_SLIGHT_SPEED_RATIO 0.92f  // 轻微偏移前进速度比例
#define LF_PATTERN_MEDIUM_SPEED_RATIO 0.80f  // 中等偏移前进速度比例
#define LF_PATTERN_LARGE_SPEED_RATIO 0.65f   // 大偏移前进速度比例
#define LF_RIGHT_TURN_OMEGA_RATIO 6.0f      // 右转路口旋转速度比例
#define LF_RIGHT_TURN_90_MS 850UL            // 右转90度动作时长 (ms)
#define LF_TURN_LEFT_GAIN 1.45f              // 巡线左转补偿增益（用于左转偏慢场景）
#define LF_TURN_RIGHT_GAIN 1.15f             // 巡线右转补偿增益
#define LF_OBSTACLE_DISTANCE_CM 8.0f     // 巡线时触发避障的距离阈值 (cm)
#define LF_OBSTACLE_RETREAT_MS 450UL     // 遇障后左后退持续时间 (ms)
#define LF_OBSTACLE_MEASURE_INTERVAL_MS 80UL // 巡线时超声测距周期 (ms)
#define LF_DEBUG_PATTERN 1                // 临时调试：打印巡线pattern与命中分支（0-关闭，1-开启）
#define LF_DEBUG_PRINT_INTERVAL_MS 120UL  // 调试打印最小间隔 (ms)

// Web手动旋转补偿参数
#define WEB_ROTATE_BOOST 2.60f            // 旋转总增益，解决旋转明显慢于直行
#define WEB_ROTATE_LEFT_GAIN 1.15f        // 左旋补偿增益
#define WEB_ROTATE_RIGHT_GAIN 1.00f       // 右旋补偿增益

/**
 * @brief 速度限制参数
 */
#define MAX_LINEAR_SPEED 0.4   // 最大线速度 (m/s)
#define MAX_ROTATION_SPEED 0.8 // 最大旋转速度 (rad/s)

/**
 * @brief 滤波器参数
 */
#define ANGLE_FILTER 0.5   // 角度滤波器系数 (0-1, 越高越平滑)
#define GYRO_FILTER 0.9    // 陀螺仪滤波器系数

/**
 * @brief 其他配置
 */
#define LED_PIN 2          // 板载LED引脚

#endif // CONFIG_H
