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
    turnSpeed = 0.5 * MAX_ROTATION_SPEED;   // 转向速度
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
    turnSpeed = speed * MAX_ROTATION_SPEED; 
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
    
    // 如果所有传感器都在线上 (十字路口)
    if (s1 && s2 && s3 && s4) {
        // 十字路口直行
         mecanumControl.setTargetVelocity(baseSpeed, 0, 0);
         lastLineTime = millis();
         return;
    }

    // 如果所有传感器都不在线上 (可能是虚线空隙，或者偏离)
    if (!s1 && !s2 && !s3 && !s4) {
        // 如果之前是在线的，现在丢失了，可能是虚线
        // 简单处理：保持上一次的运动状态一段短时间，或者减速直行
        // 这里选择：如果丢失时间短，继续直行；丢失时间长，停止
        if (millis() - lastLineTime < 500) { // 500ms 虚线容忍度
            // 继续之前的方向或者直行
            // 这里简单直行
             mecanumControl.setTargetVelocity(baseSpeed, 0, 0);
        } else {
            // 真的丢线了，停止或原地旋转寻找
             mecanumControl.setTargetVelocity(0, 0, 0);
        }
        return;
    }

    lastLineTime = millis(); // 只要检测到线就更新时间

    // 正常巡线逻辑
    // 理想情况：S2 和 S3 在线上 (0101 -> 5)
    if (s2 && s3) {
        // 直行
        mecanumControl.setTargetVelocity(baseSpeed, 0, 0);
    }
    // 左偏 (S2或S1在线，S3S4不在线) -> 需要右转纠正 ??? 不对
    // 如果车身偏右，左边的传感器会碰到线。 如果车身偏左，右边的传感器会碰到线。
    // 假设黑线在中间。
    // 正常：     [ ] [X] [X] [ ]  (S2, S3在线)
    // 车身偏右： [X] [X] [ ] [ ]  (S1, S2在线) -> 需要左转? 
    //           或者只有 S1 在线 -> 需要大幅左转?
    //           Wait, if sensor layout is Left to Right: S1 S2 S3 S4
    //           Line is static. Car moves right. Line appears to move left relative to car.
    //           So S1 detects line. So we need to turn LEFT to bring line back to center.
    
    else if (s1 && s2) { // 稍微偏右 -> 左转
        mecanumControl.setTargetVelocity(baseSpeed, 0, turnSpeed * 0.5); 
    }
    else if (s1) { // 严重偏右 -> 大幅左转
        mecanumControl.setTargetVelocity(baseSpeed * 0.5, 0, turnSpeed);
    }
    else if (s3 && s4) { // 稍微偏左 -> 右转
        mecanumControl.setTargetVelocity(baseSpeed, 0, -turnSpeed * 0.5);
    }
    else if (s4) { // 严重偏左 -> 大幅右转
        mecanumControl.setTargetVelocity(baseSpeed * 0.5, 0, -turnSpeed);
    }
    else if (s2) { // 只有S2在线，稍微偏右
         mecanumControl.setTargetVelocity(baseSpeed, 0, turnSpeed * 0.3);
    }
    else if (s3) { // 只有S3在线，稍微偏左
         mecanumControl.setTargetVelocity(baseSpeed, 0, -turnSpeed * 0.3);
    }
    else {
        // 其他情况 (例如 S1和S4在线? 可能是路口或者是错误信号)
        mecanumControl.setTargetVelocity(baseSpeed * 0.5, 0, 0);
    }
}
