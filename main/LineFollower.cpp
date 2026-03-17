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
    turnSpeed = 0.5 * MAX_ROTATION_SPEED; // 转向速度上限
    filteredError = 0.0f;
    prevError = 0.0f;
    lastOmegaCmd = 0.0f;
    lastControlTime = 0;
    lastLineTime = 0;
    isLost = false;
}

void LineFollower::start() {
    running = true;
    lastLineTime = millis();
    lastControlTime = lastLineTime;
    isLost = false;
    filteredError = 0.0f;
    prevError = 0.0f;
    lastOmegaCmd = 0.0f;
    Serial.println("Line Follower Started");
}

void LineFollower::stop() {
    running = false;
    filteredError = 0.0f;
    prevError = 0.0f;
    lastOmegaCmd = 0.0f;
    mecanumControl.setTargetVelocity(0, 0, 0);
    Serial.println("Line Follower Stopped");
}

void LineFollower::setSpeed(float speed) {
    float normalizedSpeed = constrain(speed, 0.05f, 1.0f);
    baseSpeed = normalizedSpeed * MAX_LINEAR_SPEED;
    turnSpeed = constrain(normalizedSpeed * MAX_ROTATION_SPEED,
                          MAX_ROTATION_SPEED * 0.2f,
                          MAX_ROTATION_SPEED);
}

bool LineFollower::isRunning() {
    return running;
}

void LineFollower::update() {
    if (!running) return;

    unsigned long now = millis();
    float dt = (now - lastControlTime) / 1000.0f;
    if (lastControlTime == 0 || dt <= 0.0f || dt > 0.2f) {
        dt = 0.02f;
    }
    lastControlTime = now;

    uint8_t state = lineTracker.getState();
    // state bits: S4 S3 S1 S2 (3 2 1 0)

    bool s1 = (state >> 1) & 1; // S1 左外
    bool s2 = (state >> 0) & 1; // S2 左内
    bool s3 = (state >> 2) & 1; // S3 右内
    bool s4 = (state >> 3) & 1; // S4 右外

    int activeCount = (s1 ? 1 : 0) + (s2 ? 1 : 0) + (s3 ? 1 : 0) + (s4 ? 1 : 0);

    // 十字路口等全黑区域，优先直行穿过
    if (activeCount == 4) {
        lastLineTime = now;
        isLost = false;
        filteredError *= 0.5f;
        prevError = filteredError;
        lastOmegaCmd = 0.0f;
        mecanumControl.setTargetVelocity(baseSpeed, 0, 0);
        return;
    }

    if (activeCount > 0) {
        // 连续误差：左负右正
        float errorSum = 0.0f;
        if (s1) errorSum -= 3.0f;
        if (s2) errorSum -= 1.0f;
        if (s3) errorSum += 1.0f;
        if (s4) errorSum += 3.0f;

        float rawError = errorSum / activeCount;
        filteredError = LF_ERROR_FILTER_ALPHA * filteredError +
                        (1.0f - LF_ERROR_FILTER_ALPHA) * rawError;

        float dError = (filteredError - prevError) / dt;
        dError = constrain(dError, -8.0f, 8.0f);
        prevError = filteredError;

        float omegaRaw = -(filteredError * turnSpeed * 0.85f + dError * LF_DAMPING_GAIN);
        float omegaCmd = LF_OMEGA_SMOOTH_ALPHA * lastOmegaCmd +
                         (1.0f - LF_OMEGA_SMOOTH_ALPHA) * omegaRaw;
        omegaCmd = constrain(omegaCmd, -turnSpeed, turnSpeed);

        float errorNorm = constrain(fabs(filteredError) / 3.0f, 0.0f, 1.0f);
        float speedScale = 1.0f - (errorNorm * LF_SPEED_REDUCTION_GAIN);
        speedScale = max(speedScale, LF_MIN_FORWARD_RATIO);
        float vxCmd = baseSpeed * speedScale;

        lastLineTime = now;
        isLost = false;
        lastOmegaCmd = omegaCmd;
        mecanumControl.setTargetVelocity(vxCmd, 0, omegaCmd);
        return;
    }

    // 丢线处理：短时保持，随后温和搜索，最后停止
    isLost = true;
    unsigned long lostDuration = now - lastLineTime;
    if (lostDuration < LF_LOST_HOLD_MS) {
        lastOmegaCmd *= LF_LOST_OMEGA_DECAY;
        mecanumControl.setTargetVelocity(baseSpeed * 0.75f, 0, lastOmegaCmd);
        return;
    }

    if (lostDuration < LF_LOST_TIMEOUT_MS) {
        float searchDirection = (lastOmegaCmd >= 0.0f) ? 1.0f : -1.0f;
        if (fabs(lastOmegaCmd) < DEAD_ZONE) {
            searchDirection = (filteredError <= 0.0f) ? 1.0f : -1.0f;
        }
        float searchOmega = searchDirection * turnSpeed * LF_SEARCH_OMEGA_RATIO;
        float searchVx = baseSpeed * LF_SEARCH_SPEED_RATIO;
        lastOmegaCmd = searchOmega;
        mecanumControl.setTargetVelocity(searchVx, 0, searchOmega);
        return;
    }

    lastOmegaCmd = 0.0f;
    mecanumControl.setTargetVelocity(0, 0, 0);
}
