/**
 * @file MecanumControl.h
 * @brief 麦轮运动控制模块头文件
 * @author 125pq
 * @date 2025.12.4
 */

#ifndef MECANUMCONTROL_H
#define MECANUMCONTROL_H

#include <Arduino.h>
#include <MPU6050_tockn.h>
#include "Config.h"
#include "Motor.h"

/**
 * @class MecanumControl
 * @brief 麦轮小车运动控制类（包含运动学模型）
 */
class MecanumControl {
public:
    /**
     * @brief 构造函数
     * @param motor 电机控制对象引用
     * @param mpu MPU6050对象引用
     */
    MecanumControl(Motor& motor, MPU6050& mpu);

    /**
     * @brief 初始化控制器
     */
    void init();

    /**
     * @brief 设置目标速度
     * @param vx X轴速度 (米/秒)
     * @param vy Y轴速度 (米/秒)
     * @param omega 旋转速度 (弧度/秒)
     */
    void setTargetVelocity(float vx, float vy, float omega);

    /**
     * @brief 执行运动控制更新（在loop中调用）
     */
    void update();

    /**
     * @brief 获取目标速度
     */
    void getTargetVelocity(float& vx, float& vy, float& omega);

    /**
     * @brief 获取角度输出（当前固定为0）
     */
    float getAngleOutput();

private:
    Motor& motor;
    MPU6050& mpu6050;

    // 目标运动状态
    float targetVx;
    float targetVy;
    float targetOmega;

    // 平滑后的实际执行速度
    float currentVx;
    float currentVy;
    float currentOmega;
    unsigned long lastControlUpdate;

    // 保留输出接口兼容显示模块
    float angleOutput;

    /**
     * @brief 斜率限制函数
     */
    float applySlewLimit(float current, float target, float maxDelta);

    /**
     * @brief 麦轮运动学模型
     * @param Vx X轴速度
     * @param Vy Y轴速度
     * @param omega 旋转速度
     * @param w1 电机1角速度 (输出)
     * @param w2 电机2角速度 (输出)
     * @param w3 电机3角速度 (输出)
     * @param w4 电机4角速度 (输出)
     */
    void mecanumKinematics(float Vx, float Vy, float omega, 
                          float& w1, float& w2, float& w3, float& w4);
};

#endif // MECANUMCONTROL_H
