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
    // 应用死区处理
    targetVx = (fabs(vx) < DEAD_ZONE) ? 0 : vx;
    targetVy = (fabs(vy) < DEAD_ZONE) ? 0 : vy;
    targetOmega = (fabs(omega) < DEAD_ZONE) ? 0 : omega;
    
    // 设置速度PID的目标值
    speedSetpoint = sqrt(targetVx * targetVx + targetVy * targetVy);
    
    // 只有当目标速度非零时才更新角度设定值
    if (speedSetpoint > DEAD_ZONE) {
        mpu6050.update();
        angleSetpoint = mpu6050.getAngleZ();
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

    // 航向锁定逻辑：
    // 如果用户没有要求旋转 (targetOmega == 0)，则启用PID来保持角度
    if (fabs(targetOmega) < DEAD_ZONE) {
        // 计算PID输出
        angleInput = filteredAngleZ;
        anglePID->Compute();
        
        // 叠加PID输出到旋转速度上，用于修正偏差
        // 注意方向：如果偏移为正（向左偏），需要向右转（负Omega）修正
        // PID库通常计算 Output = Kp * error + ...
        // 如果 Setpoint = 0, Input = 10 (偏左10度), Error = -10
        // Output 会是负数，正好对应向右转，符合预期
        // 但需要注意 PID 模式是 DIRECT 还是 REVERSE
        adjustedOmega += angleOutput / 100.0; // 将PID输出缩放到弧度/秒 (假设PID输出范围是-100~100)
    } else {
        // 如果用户正在手动旋转，则更新设定值为当前角度，以便停止旋转后锁定新方向
        angleSetpoint = filteredAngleZ;
        // 重置PID积分项，防止积累
        // 此PID库没有直接reset积分的函数，可以通过重新初始化或设置为MANUAL再切回AUTOMATIC实现，
        // 或者简单地在此处不调用Compute，让积分项保持（但可能不理想）
        // 简单做法：跟随
        angleInput = filteredAngleZ;
    }
    
    // 计算麦轮运动学
    // 修正：运动学方程中 omega 单位通常是 rad/s。
    // 如果 adjustedOmega 过大，可能导致电机饱和。
    float w1_rad, w2_rad, w3_rad, w4_rad;
    mecanumKinematics(adjustedVx, adjustedVy, adjustedOmega, w1_rad, w2_rad, w3_rad, w4_rad);
    
    // 限制电机速度在合理范围内
    // 假设最大物理转速约为 20 rad/s (待定，根据电机参数调整)
    float max_w = 20.0; 
    w1_rad = constrain(w1_rad, -max_w, max_w);
    w2_rad = constrain(w2_rad, -max_w, max_w);
    w3_rad = constrain(w3_rad, -max_w, max_w);
    w4_rad = constrain(w4_rad, -max_w, max_w);
    
    // 将理论速度转换为PWM值 (0-255)
    // 假设 20 rad/s 对应 PWM 255
    // 简单映射：PWM = w * (255 / max_w)
    float rad_to_pwm = 255.0 / max_w;
    
    int pwm1 = (int)(w1_rad * rad_to_pwm);
    int pwm2 = (int)(w2_rad * rad_to_pwm);
    int pwm3 = (int)(w3_rad * rad_to_pwm);
    int pwm4 = (int)(w4_rad * rad_to_pwm);
    
    // 加上死区处理，如果PWM太小电机不动
    // (在Motor类里处理可能更好，但这里可以先做简单处理)

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
    return (float)angleOutput;
}

double MecanumControl::getAngleInput() {
    return angleInput;
}

double MecanumControl::getAngleSetpoint() {
    return angleSetpoint;
}

void MecanumControl::setAnglePIDTunings(double Kp, double Ki, double Kd) {
    anglePID->SetTunings(Kp, Ki, Kd);
}

void MecanumControl::getAnglePIDTunings(double& Kp, double& Ki, double& Kd) {
    Kp = anglePID->GetKp();
    Ki = anglePID->GetKi();
    Kd = anglePID->GetKd();
}
