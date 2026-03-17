/**
 * @file Ultrasonic.h
 * @brief 超声波测距模块头文件
 * @author 黄竞亿
 * @date 2025.12.4
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include "Config.h"

/**
 * @class Ultrasonic
 * @brief 超声波测距传感器类
 */
class Ultrasonic {
public:
    /**
     * @brief 构造函数
     */
    Ultrasonic();

    /**
     * @brief 初始化超声波传感器引脚
     */
    void init();

    /**
     * @brief 超声波测距
     * @return 返回距离值（单位：厘米）
     */
    float getDistance();

    /**
     * @brief 获取原始脉冲时间
     * @return 返回脉冲持续时间（单位：微秒）
     */
    long getRawPulse();

private:
    int trigPin;
    int echoPin;
};

#endif // ULTRASONIC_H
