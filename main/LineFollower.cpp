/**
 * @file LineFollower.cpp
 * @brief 寻线控制模块实现文件
 * @author Copilot
 * @date 2026.3.2
 */

#include "LineFollower.h"
#include "Config.h"

// 假设传感器布局从左到右为 S1, S2, S3, S4
// S1: 35 (Bit 1), S2: 34 (Bit 0), S3: 39 (Bit 2), S4: 36 (Bit 3)
// 状态位: 
// Bit 0: S2 (中左)
// Bit 1: S1 (最左)
// Bit 2: S3 (中右)
// Bit 3: S4 (最右)

LineFollower::LineFollower(LineTracker& tracker, MecanumControl& control) 
    : lineTracker(tracker), mecanumControl(control), running(false) {
    baseSpeed = 0.5 * MAX_LINEAR_SPEED;   // 基础前进速度
    turnSpeed = 1.0;                      // NEW: 降低P系数，约为原来的40%
    lastLineTime = 0;
    isLost = false;
}

void LineFollower::start() {
    running = true;
    lastLineTime = millis();
    isLost = false;
    Serial.println("Line Follower Started");
}

void LineFollower::stop() {
    running = false;
    mecanumControl.setTargetVelocity(0, 0, 0);
    Serial.println("Line Follower Stopped");
}

void LineFollower::setSpeed(float speed) {
    baseSpeed = speed * MAX_LINEAR_SPEED;
    // turnSpeed 稍微随速度增加，但整体比例降低
    // 假设 speed 是 0.5 (中速), MAX_LINEAR 0.4. baseSpeed = 0.2.
    // turnSpeed = 0.8 + 0.4 = 1.2 rad/s. 对于 error=3 (最大偏离)，omega=3.6 rad/s (太快了!)
    // error 范围约 -3 到 3. 
    // 期望最大修正角速度约 1.0 - 1.5 rad/s
    turnSpeed = (0.3 + (speed * 0.5)); // 修改系数
}

bool LineFollower::isRunning() {
    return running;
}

void LineFollower::update() {
    if (!running) return;

    uint8_t state = lineTracker.getState();
    // state bits: S4 S3 S1 S2 (3 2 1 0)

    bool s1 = (state >> 1) & 1; // S1 左外
    bool s2 = (state >> 0) & 1; // S2 左内
    bool s3 = (state >> 2) & 1; // S3 右内
    bool s4 = (state >> 3) & 1; // S4 右外
    
    float error = 0;
    int activeCount = 0;
    
    // 计算加权偏差 (S1:-3, S2:-1, S3:1, S4:3)
    // 负值表示线在左侧（车偏右），正值表示线在右侧（车偏左）
    if (s1) { error -= 3.0; activeCount++; }
    if (s2) { error -= 1.0; activeCount++; }
    if (s3) { error += 1.0; activeCount++; }
    if (s4) { error += 3.0; activeCount++; }
    
    if (activeCount > 0) {
        error /= activeCount; // 归一化误差，范围约 -3 到 +3
        lastLineTime = millis();
    } else {
        // 未检测到线
        if (millis() - lastLineTime < 150) {
            // 短暂丢线（可能是虚线或传感器间隙），假设误差为0保持直行以平稳过渡
            error = 0;
        } else {
            // 丢失过久，停止
            mecanumControl.setTargetVelocity(0, 0, 0);
            return;
        }
    }
    
    // 如果是十字路口或大面积黑块（3个以上传感器触发）
    if (activeCount >= 3) {
        error = 0; // 视为直行
    }

    // P控制计算旋转量
    // Error 范围约 -3.0 ~ +3.0
    // Error < 0 (左侧触发) -> 需要左转 (Omega > 0)
    // Error > 0 (右侧触发) -> 需要右转 (Omega < 0)
    float omega = -error * turnSpeed; 
    
    // 限制最大旋转速度，防止飞车
    omega = constrain(omega, -MAX_ROTATION_SPEED * 1.5, MAX_ROTATION_SPEED * 1.5);

    // 动态速度调整：当急转弯（误差大）时降低线速度
    float currentVx = baseSpeed;
    if (abs(error) > 1.5) {
        currentVx *= 0.4; // 遇到急弯大幅减速 (40%速度)
    } else if (abs(error) > 0.8) { // 稍微偏离也减速
        currentVx *= 0.7;
    }

    // 将计算出的线速度和角速度应用到麦轮控制器
    // 麦轮运动学模型会自动将其转换为左右轮的速度差
    mecanumControl.setTargetVelocity(currentVx, 0, omega);
}
