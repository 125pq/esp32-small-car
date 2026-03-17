/**
 * @file MecanumControl.cpp
 * @brief 麦轮运动控制模块实现文件
 * @author 喵屋
 * @date 2025.12.4
 */

#include "MecanumControl.h"

MecanumControl::MecanumControl(Motor& motor, MPU6050& mpu)
    : motor(motor), mpu6050(mpu),
      targetVx(0), targetVy(0), targetOmega(0),
      currentVx(0), currentVy(0), currentOmega(0), lastControlUpdate(0),
      angleOutput(0) {}

void MecanumControl::init() {
    lastControlUpdate = millis();
    Serial.println("Mecanum Controller Initialized");
}

void MecanumControl::setTargetVelocity(float vx, float vy, float omega) {
    // 应用死区处理
    targetVx = (fabs(vx) < DEAD_ZONE) ? 0 : vx;
    targetVy = (fabs(vy) < DEAD_ZONE) ? 0 : vy;
    targetOmega = (fabs(omega) < DEAD_ZONE) ? 0 : omega;

    // 纯开环控制：不使用角度修正输出
    angleOutput = 0;
}

void MecanumControl::update() {
    unsigned long now = millis();
    float dt = (now - lastControlUpdate) / 1000.0f;
    if (lastControlUpdate == 0 || dt <= 0.0f || dt > 0.2f) {
        dt = 0.02f;
    }
    lastControlUpdate = now;

    float linearDelta = COMMAND_LINEAR_SLEW_RATE * dt;
    float omegaDelta = COMMAND_OMEGA_SLEW_RATE * dt;

    currentVx = applySlewLimit(currentVx, targetVx, linearDelta);
    currentVy = applySlewLimit(currentVy, targetVy, linearDelta);
    currentOmega = applySlewLimit(currentOmega, targetOmega, omegaDelta);

    // 如果目标和当前都接近零，直接停机
    if (fabs(targetVx) < DEAD_ZONE && fabs(targetVy) < DEAD_ZONE && fabs(targetOmega) < DEAD_ZONE &&
        fabs(currentVx) < DEAD_ZONE && fabs(currentVy) < DEAD_ZONE && fabs(currentOmega) < DEAD_ZONE) {
        currentVx = 0.0f;
        currentVy = 0.0f;
        currentOmega = 0.0f;
        motor.stop();
        angleOutput = 0;
        return;
    }
    
    // 纯开环速度控制
    float adjustedVx = currentVx;
    float adjustedVy = currentVy;
    float adjustedOmega = currentOmega;
    angleOutput = 0;
    
    // 计算麦轮运动学
    float w1, w2, w3, w4;
    mecanumKinematics(adjustedVx, adjustedVy, adjustedOmega, w1, w2, w3, w4);
    
    // 限制电机速度在合理范围内
    w1 = constrain(w1, -30.0, 30.0);
    w2 = constrain(w2, -30.0, 30.0);
    w3 = constrain(w3, -30.0, 30.0);
    w4 = constrain(w4, -30.0, 30.0);
    
    // 将理论速度转换为PWM值
    int pwm1 = constrain(w1 * 20, -255, 255);
    int pwm2 = constrain(w2 * 20, -255, 255);
    int pwm3 = constrain(w3 * 20, -255, 255);
    int pwm4 = constrain(w4 * 20, -255, 255);
    
    // 设置电机速度
    motor.setSpeed(pwm1, pwm2, pwm3, pwm4);
}

float MecanumControl::applySlewLimit(float current, float target, float maxDelta) {
    float delta = target - current;
    if (delta > maxDelta) {
        return current + maxDelta;
    }
    if (delta < -maxDelta) {
        return current - maxDelta;
    }
    return target;
}

void MecanumControl::mecanumKinematics(float Vx, float Vy, float omega, 
                                       float& w1, float& w2, float& w3, float& w4) {
    // 麦轮运动学方程
    w1 = (Vx - Vy - (LX + LY) * omega) / WHEEL_RADIUS;
    w2 = (Vx + Vy + (LX + LY) * omega) / WHEEL_RADIUS;
    w3 = (Vx + Vy - (LX + LY) * omega) / WHEEL_RADIUS;
    w4 = (Vx - Vy + (LX + LY) * omega) / WHEEL_RADIUS;
}

void MecanumControl::getTargetVelocity(float& vx, float& vy, float& omega) {
    vx = targetVx;
    vy = targetVy;
    omega = targetOmega;
}

float MecanumControl::getAngleOutput() {
    return angleOutput;
}
