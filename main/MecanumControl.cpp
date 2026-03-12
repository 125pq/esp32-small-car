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
      angleSetpoint(0), angleInput(0), angleOutput(0),
      speedSetpoint(0), speedInput(0), speedOutput(0),
      filteredAngleZ(0), filteredGyroZ(0),
      filteredAccX(0), filteredAccY(0) {
    
    // 创建PID控制器
    anglePID = new PID(&angleInput, &angleOutput, &angleSetpoint, 
                      ANGLE_KP, ANGLE_KI, ANGLE_KD, DIRECT);
    speedPID = new PID(&speedInput, &speedOutput, &speedSetpoint, 
                      SPEED_KP, SPEED_KI, SPEED_KD, DIRECT);
}

void MecanumControl::init() {
    // 初始化角度PID控制器
    anglePID->SetMode(AUTOMATIC);
    anglePID->SetOutputLimits(-100, 100);
    anglePID->SetSampleTime(20);
    
    // 初始化速度PID控制器
    speedPID->SetMode(AUTOMATIC);
    speedPID->SetOutputLimits(-255, 255);
    speedPID->SetSampleTime(20);
    
    Serial.println("PID Controllers Initialized");
}

void MecanumControl::setTargetVelocity(float vx, float vy, float omega) {
    // 记录之前是否有速度（用于判断是否是从静止启动）
    bool wasMoving = (speedSetpoint > DEAD_ZONE);

    // 应用死区处理
    targetVx = (fabs(vx) < DEAD_ZONE) ? 0 : vx;
    targetVy = (fabs(vy) < DEAD_ZONE) ? 0 : vy;
    targetOmega = (fabs(omega) < DEAD_ZONE) ? 0 : omega;
    
    // 设置速度PID的目标值
    speedSetpoint = sqrt(targetVx * targetVx + targetVy * targetVy);
    
    // 逻辑修正：防止在持续按键移动时不断重置目标角度
    if (speedSetpoint > DEAD_ZONE) {
        mpu6050.update();
        // 只有当：从静止开始移动 且 目标旋转为0（直行模式）时，才锁定当前朝向为目标
        // 如果已经在移动中，保持原有的 angleSetpoint 不变，以便 PID 能将车纠正回原始路线
        if (!wasMoving && fabs(targetOmega) < DEAD_ZONE) {
            angleSetpoint = mpu6050.getAngleZ();
        }
    }
}

void MecanumControl::update() {
    // 如果所有目标速度都接近零，则停止电机
    if (fabs(targetVx) < DEAD_ZONE && 
        fabs(targetVy) < DEAD_ZONE && 
        fabs(targetOmega) < DEAD_ZONE) {
        motor.stop();
        return;
    }
    
    // 更新MPU6050数据
    mpu6050.update();
    
    // 应用低通滤波器到MPU6050数据
    filteredAngleZ = ANGLE_FILTER * filteredAngleZ + 
                     (1 - ANGLE_FILTER) * mpu6050.getAngleZ();
    filteredGyroZ = GYRO_FILTER * filteredGyroZ + 
                    (1 - GYRO_FILTER) * mpu6050.getGyroZ();
    
    // 角度PID控制（保持或转向到目标角度）
    angleInput = filteredAngleZ;
    anglePID->Compute();
    
    // 应用PID修正到目标运动状态
    float adjustedVx = targetVx;
    float adjustedVy = targetVy;
    float adjustedOmega = targetOmega;
    
    // CRITICAL FIX: 将 PID 输出应用到旋转速度上（修复PID无效的问题）
    // 只有在设定为直行(Omega=0)且有线速度时，才启用PID辅助纠偏
    if (fabs(targetOmega) < DEAD_ZONE && speedSetpoint > DEAD_ZONE) {
        // 缩放 PID 输出以适配角速度 (rad/s)。
        // 假设 PID 输出范围(-100~100)，0.02~0.05 是比较合理的缩放系数。
        // 可配合网页端的 P 参数调节灵敏度 (建议 KP: 1.0 ~ 5.0)
        adjustedOmega += angleOutput * 0.05; 
    }
    
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

void MecanumControl::setAnglePID(double kp, double ki, double kd) {
    if (anglePID) anglePID->SetTunings(kp, ki, kd);
    Serial.printf("Angle PID set to: P=%.2f, I=%.2f, D=%.2f\n", kp, ki, kd);
}

void MecanumControl::setSpeedPID(double kp, double ki, double kd) {
    if (speedPID) speedPID->SetTunings(kp, ki, kd);
    Serial.printf("Speed PID set to: P=%.2f, I=%.2f, D=%.2f\n", kp, ki, kd);
}
