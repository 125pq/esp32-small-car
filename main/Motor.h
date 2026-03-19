/**
 * @file Motor.h
 * @brief 电机控制模块头文件
 * @author 黄竞亿
 * @date 2025.12.4
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Config.h"

/**
 * @class Motor
 * @brief 四轮电机控制类
 */
class Motor {
public:
    /**
     * @brief 构造函数
     */
    Motor();

    /**
     * @brief 初始化电机引脚
     */
    void init();

    /**
     * @brief 设置四个电机的方向和速度
     * @param speed1 电机1速度 (-255 to 255)
     * @param speed2 电机2速度 (-255 to 255)
     * @param speed3 电机3速度 (-255 to 255)
     * @param speed4 电机4速度 (-255 to 255)
     */
    void setSpeed(int speed1, int speed2, int speed3, int speed4);

    /**
     * @brief 停止所有电机
     */
    void stop();

    /**
     * @brief 测试电机（依次运行每个电机）
     */
    void test();

private:
    /**
     * @brief 设置单个电机速度
     * @param motorNum 电机编号 (1-4)
     * @param speed 速度 (-255 to 255)
     */
    void setMotorSpeed(int motorNum, int speed);
};

#endif // MOTOR_H
