/**
 * @file LineFollower.cpp
 * @brief 寻线控制模块实现文件
 * @author 黄竞亿
 * @date 2026.3.2
 */

#include "LineFollower.h"
#include "Config.h"

// 传感器逻辑顺序从左到右为 S1, S2, S3, S4。
// 由于当前安装/接线，左侧前两路在位解析中需要互换：
// S1 使用 Bit 0，S2 使用 Bit 1，S3 使用 Bit 2，S4 使用 Bit 3。
// 状态位: 
// Bit 0: S1 (最左)
// Bit 1: S2 (中左)
// Bit 2: S3 (中右)
// Bit 3: S4 (最右)

LineFollower::LineFollower(LineTracker& tracker, MecanumControl& control, Ultrasonic* ultrasonicSensor, MPU6050* imuSensor) 
    : lineTracker(tracker), mecanumControl(control), ultrasonic(ultrasonicSensor), imu(imuSensor), running(false) {
    baseSpeed = 0.5 * MAX_LINEAR_SPEED;   // 基础前进速度
    turnSpeed = 0.7 * MAX_ROTATION_SPEED; // 转向速度上限（提高默认转向力度）
    resetTuningToDefault();
    obstacleRetreating = false;
    obstacleRetreatStartTime = 0;
    lastObstacleMeasureTime = 0;
    lastObstacleDistance = 400.0f;
    rightTurning = false;
    rightTurnArmed = true;
    rightTurnStartTime = 0;
    rightTurnStartYawDeg = 0.0f;
    rightTurnTargetDeg = 90.0f;
    rightTurnStopDeg = 3.0f;
    rightTurnStopGyroDegPerSec = 8.0f;
    rightTurnTimeoutMs = 1400UL;
    rightTurnStableCount = 0;
}

void LineFollower::start() {
    running = true;
    obstacleRetreating = false;
    obstacleRetreatStartTime = 0;
    lastObstacleMeasureTime = 0;
    lastObstacleDistance = 400.0f;
    rightTurning = false;
    rightTurnArmed = true;
    rightTurnStartTime = 0;
    rightTurnStartYawDeg = 0.0f;
    rightTurnStableCount = 0;
    Serial.println("Line Follower Started");
}

void LineFollower::stop() {
    running = false;
    obstacleRetreating = false;
    obstacleRetreatStartTime = 0;
    rightTurning = false;
    rightTurnArmed = true;
    rightTurnStartTime = 0;
    rightTurnStartYawDeg = 0.0f;
    rightTurnStableCount = 0;
    mecanumControl.setTargetVelocity(0, 0, 0);
    Serial.println("Line Follower Stopped");
}

void LineFollower::setUltrasonic(Ultrasonic* ultrasonicSensor) {
    ultrasonic = ultrasonicSensor;
    lastObstacleMeasureTime = 0;
    lastObstacleDistance = 400.0f;
}

void LineFollower::setImu(MPU6050* imuSensor) {
    imu = imuSensor;
}

void LineFollower::setSpeed(float speed) {
    float normalizedSpeed = constrain(speed, 0.0f, 1.0f);
    if (normalizedSpeed <= DEAD_ZONE) {
        baseSpeed = 0.0f;
        turnSpeed = 0.0f;
        return;
    }

    baseSpeed = normalizedSpeed * MAX_LINEAR_SPEED;
    turnSpeed = constrain(normalizedSpeed * MAX_ROTATION_SPEED * 1.35f,
                          MAX_ROTATION_SPEED * 0.20f,
                          MAX_ROTATION_SPEED);
}

bool LineFollower::setTuning(const String& key, float value) {
    if (key == "pst") {
        patternSlightTurnRatio = constrain(value, 0.10f, 2.00f);
        return true;
    }
    if (key == "pmt") {
        patternMediumTurnRatio = constrain(value, 0.20f, 2.50f);
        return true;
    }
    if (key == "plt") {
        patternLargeTurnRatio = constrain(value, 0.30f, 5.00f);
        return true;
    }
    if (key == "pss") {
        patternSlightSpeedRatio = constrain(value, 0.30f, 1.00f);
        return true;
    }
    if (key == "pms") {
        patternMediumSpeedRatio = constrain(value, 0.30f, 1.00f);
        return true;
    }
    if (key == "pls") {
        patternLargeSpeedRatio = constrain(value, 0.20f, 1.00f);
        return true;
    }
    if (key == "rto") {
        rightTurnOmegaRatio = constrain(value, 0.30f, 1.20f);
        return true;
    }
    if (key == "rtm") {
        rightTurn90Ms = (unsigned long)constrain(value, 200.0f, 2500.0f);
        return true;
    }
    if (key == "odc") {
        obstacleDistanceCm = constrain(value, 3.0f, 50.0f);
        return true;
    }
    if (key == "orm") {
        obstacleRetreatMs = (unsigned long)constrain(value, 100.0f, 3000.0f);
        return true;
    }
    if (key == "omi") {
        obstacleMeasureIntervalMs = (unsigned long)constrain(value, 20.0f, 1000.0f);
        return true;
    }

    return false;
}

void LineFollower::resetTuningToDefault() {
    patternSlightTurnRatio = LF_PATTERN_SLIGHT_TURN_RATIO;
    patternMediumTurnRatio = LF_PATTERN_MEDIUM_TURN_RATIO;
    patternLargeTurnRatio = LF_PATTERN_LARGE_TURN_RATIO;
    patternSlightSpeedRatio = LF_PATTERN_SLIGHT_SPEED_RATIO;
    patternMediumSpeedRatio = LF_PATTERN_MEDIUM_SPEED_RATIO;
    patternLargeSpeedRatio = LF_PATTERN_LARGE_SPEED_RATIO;
    rightTurnOmegaRatio = LF_RIGHT_TURN_OMEGA_RATIO;
    rightTurn90Ms = LF_RIGHT_TURN_90_MS;
    obstacleDistanceCm = LF_OBSTACLE_DISTANCE_CM;
    obstacleRetreatMs = LF_OBSTACLE_RETREAT_MS;
    obstacleMeasureIntervalMs = LF_OBSTACLE_MEASURE_INTERVAL_MS;
}

String LineFollower::getTuningJson() const {
    String json = "{";
    json += "\"pst\":" + String(patternSlightTurnRatio, 3) + ",";
    json += "\"pmt\":" + String(patternMediumTurnRatio, 3) + ",";
    json += "\"plt\":" + String(patternLargeTurnRatio, 3) + ",";
    json += "\"pss\":" + String(patternSlightSpeedRatio, 3) + ",";
    json += "\"pms\":" + String(patternMediumSpeedRatio, 3) + ",";
    json += "\"pls\":" + String(patternLargeSpeedRatio, 3) + ",";
    json += "\"rto\":" + String(rightTurnOmegaRatio, 3) + ",";
    json += "\"rtm\":" + String(rightTurn90Ms) + ",";
    json += "\"odc\":" + String(obstacleDistanceCm, 2) + ",";
    json += "\"orm\":" + String(obstacleRetreatMs) + ",";
    json += "\"omi\":" + String(obstacleMeasureIntervalMs);
    json += "}";
    return json;
}

bool LineFollower::isRunning() {
    return running;
}

bool LineFollower::isObstacleTooClose(unsigned long now) {
    if (ultrasonic == nullptr) {
        return false;
    }

    if (lastObstacleMeasureTime == 0 ||
        now - lastObstacleMeasureTime >= obstacleMeasureIntervalMs) {
        float distance = ultrasonic->getDistance();
        if (distance > 0.0f) {
            lastObstacleDistance = distance;
        } else {
            // 超声超时或错误时，按远距离处理，避免误触发。
            lastObstacleDistance = 400.0f;
        }
        lastObstacleMeasureTime = now;
    }

    return lastObstacleDistance > 0.0f && lastObstacleDistance < obstacleDistanceCm;
}

float LineFollower::wrapDeg180(float deg) {
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

void LineFollower::startRightTurnByImu(unsigned long now) {
    rightTurning = true;
    rightTurnArmed = false;
    rightTurnStartTime = now;
    rightTurnStableCount = 0;

    if (imu != nullptr) {
        imu->update();
        rightTurnStartYawDeg = imu->getAngleZ();
    } else {
        rightTurnStartYawDeg = 0.0f;
    }
}

bool LineFollower::updateRightTurnByImu(unsigned long now) {
    if (imu == nullptr) {
        if (now - rightTurnStartTime < rightTurn90Ms) {
            float turnOmega = -turnSpeed * rightTurnOmegaRatio;
            turnOmega = constrain(turnOmega, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
            mecanumControl.setTargetVelocity(0, 0, turnOmega);
            return false;
        }
        return true;
    }

    imu->update();
    float yawDeg = imu->getAngleZ();
    float gyroZDegPerSec = imu->getGyroZ();

    float turnedDeg = fabs(wrapDeg180(yawDeg - rightTurnStartYawDeg));
    float remainDeg = rightTurnTargetDeg - turnedDeg;

    bool angleReady = remainDeg <= rightTurnStopDeg;
    if (angleReady) {
        // 已接近目标角，先停转等待角速度衰减，避免过冲持续扩大。
        mecanumControl.setTargetVelocity(0, 0, 0);
    } else {
        float omegaScale = constrain(remainDeg / rightTurnTargetDeg, 0.30f, 1.00f);
        float turnOmega = -turnSpeed * rightTurnOmegaRatio * omegaScale;
        turnOmega = constrain(turnOmega, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
        mecanumControl.setTargetVelocity(0, 0, turnOmega);
    }

    bool gyroReady = fabs(gyroZDegPerSec) <= rightTurnStopGyroDegPerSec;

    if (angleReady && gyroReady) {
        rightTurnStableCount++;
    } else {
        rightTurnStableCount = 0;
    }

    if (rightTurnStableCount >= 3) {
        return true;
    }

    if (now - rightTurnStartTime >= rightTurnTimeoutMs) {
        return true;
    }

    return false;
}

void LineFollower::update() {
    if (!running) return;

    unsigned long now = millis();

    // 巡线模式内置简易避障：遇障先左后退，再恢复巡线。
    if (obstacleRetreating) {
        if (now - obstacleRetreatStartTime < obstacleRetreatMs) {
            mecanumControl.setTargetVelocity(-baseSpeed, baseSpeed * 2, 0);
            
            return;
        }
        obstacleRetreating = false;
        lastObstacleDistance = 400.0f;
        lastObstacleMeasureTime = 0;
    }

    if (isObstacleTooClose(now)) {
        obstacleRetreating = true;
        obstacleRetreatStartTime = now;
        mecanumControl.setTargetVelocity(-baseSpeed, baseSpeed * 2, 0);
        return;
    }

    if (rightTurning) {
        if (!updateRightTurnByImu(now)) {
            return;
        }
        rightTurning = false;
    }

    uint8_t state = lineTracker.getState();
    // state bits(raw): bit3 bit2 bit1 bit0

    bool s1 = (state >> 0) & 1; // S1 左外（与 S2 互换后解析）
    bool s2 = (state >> 1) & 1; // S2 左内（与 S1 互换后解析）
    bool s3 = (state >> 2) & 1; // S3 右内
    bool s4 = (state >> 3) & 1; // S4 右外

    // 当前巡线传感器语义：0 表示在线，1 表示离线。
    bool l1 = !s1;
    bool l2 = !s2;
    bool l3 = !s3;
    bool l4 = !s4;

    // pattern顺序按传感器物理顺序：S1 S2 S3 S4（左->右）
    uint8_t pattern = (s1 ? 0b1000 : 0) |
                      (s2 ? 0b0100 : 0) |
                      (s3 ? 0b0010 : 0) |
                      (s4 ? 0b0001 : 0);

#if defined(LF_DEBUG_PATTERN) && LF_DEBUG_PATTERN
    static uint8_t lastPattern = 0xFF;
    static uint8_t lastDecision = 0xFF;
    static unsigned long lastDebugPrintTime = 0;

    auto logPatternDecision = [&](uint8_t decision, const char* label) {
        bool changed = (pattern != lastPattern) || (decision != lastDecision);
        if (!changed && now - lastDebugPrintTime < LF_DEBUG_PRINT_INTERVAL_MS) {
            return;
        }

        Serial.print("[LF] p=");
        for (int bit = 3; bit >= 0; --bit) {
            Serial.print((pattern >> bit) & 0x01);
        }
        Serial.print(" d=");
        Serial.println(label);

        lastPattern = pattern;
        lastDecision = decision;
        lastDebugPrintTime = now;
    };
#endif

    if (pattern != 0b1000) {
        rightTurnArmed = true;
    }

    if (pattern == 0b1000 && rightTurnArmed) {
        startRightTurnByImu(now);
#if defined(LF_DEBUG_PATTERN) && LF_DEBUG_PATTERN
        logPatternDecision(1, "RIGHT_TURN_TRIGGER");
#endif
        if (rightTurning) {
            updateRightTurnByImu(now);
        }
        return;
    }

    float mappedOmega = 0.0f;
    float mappedSpeedRatio = 1.0f;
    bool mappedState = true;
    uint8_t mappedDecision = 0;
    const char* mappedLabel = "UNKNOWN";

    switch (pattern) {
        case 0b1001: // 在线正中
        case 0b0000: // 十字路口，直行通过
            mappedDecision = 2;
            mappedLabel = "STRAIGHT";
            mappedOmega = 0.0f;
            mappedSpeedRatio = 1.0f;
            break;
        case 0b1111: // 丢线，保持最后状态继续前进
            mappedDecision = 3;
            mappedLabel = "NO_LINE_KEEP";
            mappedOmega = 0.0f;
            mappedSpeedRatio = 1.0f;
            break;
        case 0b1011: // 轻微偏左
            mappedDecision = 4;
            mappedLabel = "SLIGHT_LEFT";
            mappedOmega = turnSpeed * patternSlightTurnRatio;
            mappedSpeedRatio = patternSlightSpeedRatio;
            break;
        case 0b1101: // 轻微偏右
            mappedDecision = 5;
            mappedLabel = "SLIGHT_RIGHT";
            mappedOmega = -turnSpeed * patternSlightTurnRatio;
            mappedSpeedRatio = patternSlightSpeedRatio;
            break;
        case 0b0011: // 中等偏左
            mappedDecision = 6;
            mappedLabel = "MEDIUM_LEFT";
            mappedOmega = turnSpeed * patternMediumTurnRatio;
            mappedSpeedRatio = patternMediumSpeedRatio;
            break;
        case 0b1100: // 中等偏右
            mappedDecision = 7;
            mappedLabel = "MEDIUM_RIGHT";
            mappedOmega = -turnSpeed * patternMediumTurnRatio;
            mappedSpeedRatio = patternMediumSpeedRatio;
            break;
        case 0b0111: // 大幅偏左
            mappedDecision = 8;
            mappedLabel = "LARGE_LEFT";
            mappedOmega = turnSpeed * patternLargeTurnRatio;
            mappedSpeedRatio = patternLargeSpeedRatio;
            break;
        case 0b1110: // 大幅偏右
            mappedDecision = 9;
            mappedLabel = "LARGE_RIGHT";
            mappedOmega = -turnSpeed * patternLargeTurnRatio;
            mappedSpeedRatio = patternLargeSpeedRatio;
            break;
        default:
            mappedState = false;
            break;
    }

    if (mappedState) {
        if (mappedOmega > 0.0f) {
            mappedOmega *= LF_TURN_LEFT_GAIN;
        } else if (mappedOmega < 0.0f) {
            mappedOmega *= LF_TURN_RIGHT_GAIN;
        }
        float vxCmd = baseSpeed * mappedSpeedRatio;
#if defined(LF_DEBUG_PATTERN) && LF_DEBUG_PATTERN
        logPatternDecision(mappedDecision, mappedLabel);
#endif
        mecanumControl.setTargetVelocity(vxCmd, 0, mappedOmega);
        return;
    }

    int activeCount = (l1 ? 1 : 0) + (l2 ? 1 : 0) + (l3 ? 1 : 0) + (l4 ? 1 : 0);
    if (activeCount > 0) {
        // 基础兜底：未显式映射状态时按重心误差直接修正
        float errorSum = 0.0f;
        if (l1) errorSum -= 3.0f;
        if (l2) errorSum -= 1.0f;
        if (l3) errorSum += 1.0f;
        if (l4) errorSum += 3.0f;

        float rawError = errorSum / activeCount;
        float omegaCmd = -rawError * (turnSpeed / 3.0f);
        if (omegaCmd > 0.0f) {
            omegaCmd *= LF_TURN_LEFT_GAIN;
        } else if (omegaCmd < 0.0f) {
            omegaCmd *= LF_TURN_RIGHT_GAIN;
        }
        omegaCmd = constrain(omegaCmd, -turnSpeed, turnSpeed);
        float vxCmd = baseSpeed * patternMediumSpeedRatio;
#if defined(LF_DEBUG_PATTERN) && LF_DEBUG_PATTERN
        logPatternDecision(10, "FALLBACK");
#endif
        mecanumControl.setTargetVelocity(vxCmd, 0, omegaCmd);
        return;
    }

    // 无线可循时直接停车，不做温和搜索
#if defined(LF_DEBUG_PATTERN) && LF_DEBUG_PATTERN
    logPatternDecision(11, "NO_LINE_STOP");
#endif
    mecanumControl.setTargetVelocity(0, 0, 0);
}
