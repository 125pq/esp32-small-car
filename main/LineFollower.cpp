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

LineFollower::LineFollower(LineTracker& tracker, MecanumControl& control, Ultrasonic* ultrasonicSensor) 
    : lineTracker(tracker), mecanumControl(control), ultrasonic(ultrasonicSensor), running(false) {
    baseSpeed = 0.5 * MAX_LINEAR_SPEED;   // 基础前进速度
    turnSpeed = 0.7 * MAX_ROTATION_SPEED; // 转向速度上限（提高默认转向力度）
    resetTuningToDefault();
    filteredError = 0.0f;
    prevError = 0.0f;
    lastOmegaCmd = 0.0f;
    lastControlTime = 0;
    lastLineTime = 0;
    isLost = false;
    obstacleRetreating = false;
    obstacleRetreatStartTime = 0;
    lastObstacleMeasureTime = 0;
    lastObstacleDistance = 400.0f;
    rightTurning = false;
    rightTurnArmed = true;
    rightTurnStartTime = 0;
}

void LineFollower::start() {
    running = true;
    lastLineTime = millis();
    lastControlTime = lastLineTime;
    isLost = false;
    filteredError = 0.0f;
    prevError = 0.0f;
    lastOmegaCmd = 0.0f;
    obstacleRetreating = false;
    obstacleRetreatStartTime = 0;
    lastObstacleMeasureTime = 0;
    lastObstacleDistance = 400.0f;
    rightTurning = false;
    rightTurnArmed = true;
    rightTurnStartTime = 0;
    Serial.println("Line Follower Started");
}

void LineFollower::stop() {
    running = false;
    filteredError = 0.0f;
    prevError = 0.0f;
    lastOmegaCmd = 0.0f;
    obstacleRetreating = false;
    obstacleRetreatStartTime = 0;
    rightTurning = false;
    rightTurnArmed = true;
    rightTurnStartTime = 0;
    mecanumControl.setTargetVelocity(0, 0, 0);
    Serial.println("Line Follower Stopped");
}

void LineFollower::setUltrasonic(Ultrasonic* ultrasonicSensor) {
    ultrasonic = ultrasonicSensor;
    lastObstacleMeasureTime = 0;
    lastObstacleDistance = 400.0f;
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
    if (key == "efa") {
        errorFilterAlpha = constrain(value, 0.30f, 0.98f);
        return true;
    }
    if (key == "dg") {
        dampingGain = constrain(value, 0.0f, 0.20f);
        return true;
    }
    if (key == "osa") {
        omegaSmoothAlpha = constrain(value, 0.30f, 0.98f);
        return true;
    }
    if (key == "srg") {
        speedReductionGain = constrain(value, 0.0f, 0.95f);
        return true;
    }
    if (key == "mfr") {
        minForwardRatio = constrain(value, 0.10f, 0.95f);
        return true;
    }
    if (key == "lod") {
        lostOmegaDecay = constrain(value, 0.50f, 0.99f);
        return true;
    }
    if (key == "sor") {
        searchOmegaRatio = constrain(value, 0.20f, 1.20f);
        return true;
    }
    if (key == "ssr") {
        searchSpeedRatio = constrain(value, 0.10f, 0.90f);
        return true;
    }
    if (key == "tg") {
        turnGain = constrain(value, 0.20f, 1.50f);
        return true;
    }
    if (key == "dec") {
        dErrorClamp = constrain(value, 1.0f, 30.0f);
        return true;
    }
    if (key == "lhm") {
        lostHoldMs = (unsigned long)constrain(value, 50.0f, 2000.0f);
        if (lostTimeoutMs <= lostHoldMs + 50UL) {
            lostTimeoutMs = lostHoldMs + 50UL;
        }
        return true;
    }
    if (key == "ltm") {
        unsigned long minTimeout = lostHoldMs + 50UL;
        lostTimeoutMs = (unsigned long)constrain(value, (float)minTimeout, 6000.0f);
        return true;
    }

    return false;
}

void LineFollower::resetTuningToDefault() {
    errorFilterAlpha = LF_ERROR_FILTER_ALPHA;
    dampingGain = LF_DAMPING_GAIN;
    omegaSmoothAlpha = LF_OMEGA_SMOOTH_ALPHA;
    speedReductionGain = LF_SPEED_REDUCTION_GAIN;
    minForwardRatio = LF_MIN_FORWARD_RATIO;
    lostOmegaDecay = LF_LOST_OMEGA_DECAY;
    searchOmegaRatio = LF_SEARCH_OMEGA_RATIO;
    searchSpeedRatio = LF_SEARCH_SPEED_RATIO;
    turnGain = 1.20f;
    dErrorClamp = 12.0f;
    lostHoldMs = LF_LOST_HOLD_MS;
    lostTimeoutMs = LF_LOST_TIMEOUT_MS;
}

String LineFollower::getTuningJson() const {
    String json = "{";
    json += "\"efa\":" + String(errorFilterAlpha, 3) + ",";
    json += "\"dg\":" + String(dampingGain, 3) + ",";
    json += "\"osa\":" + String(omegaSmoothAlpha, 3) + ",";
    json += "\"srg\":" + String(speedReductionGain, 3) + ",";
    json += "\"mfr\":" + String(minForwardRatio, 3) + ",";
    json += "\"lod\":" + String(lostOmegaDecay, 3) + ",";
    json += "\"sor\":" + String(searchOmegaRatio, 3) + ",";
    json += "\"ssr\":" + String(searchSpeedRatio, 3) + ",";
    json += "\"tg\":" + String(turnGain, 3) + ",";
    json += "\"dec\":" + String(dErrorClamp, 3) + ",";
    json += "\"lhm\":" + String(lostHoldMs) + ",";
    json += "\"ltm\":" + String(lostTimeoutMs);
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
        now - lastObstacleMeasureTime >= LF_OBSTACLE_MEASURE_INTERVAL_MS) {
        float distance = ultrasonic->getDistance();
        if (distance > 0.0f) {
            lastObstacleDistance = distance;
        } else {
            // 超声超时或错误时，按远距离处理，避免误触发。
            lastObstacleDistance = 400.0f;
        }
        lastObstacleMeasureTime = now;
    }

    return lastObstacleDistance > 0.0f && lastObstacleDistance < LF_OBSTACLE_DISTANCE_CM;
}

void LineFollower::update() {
    if (!running) return;

    unsigned long now = millis();

    // 巡线模式内置简易避障：遇障先左后退，再恢复巡线。
    if (obstacleRetreating) {
        if (now - obstacleRetreatStartTime < LF_OBSTACLE_RETREAT_MS) {
            mecanumControl.setTargetVelocity(-baseSpeed, baseSpeed, 0);
            return;
        }
        obstacleRetreating = false;
        lastObstacleDistance = 400.0f;
        lastObstacleMeasureTime = 0;
        filteredError = 0.0f;
        prevError = 0.0f;
        lastOmegaCmd = 0.0f;
        lastLineTime = now;
        isLost = false;
    }

    if (isObstacleTooClose(now)) {
        obstacleRetreating = true;
        obstacleRetreatStartTime = now;
        mecanumControl.setTargetVelocity(-baseSpeed, baseSpeed, 0);
        return;
    }

    if (rightTurning) {
        if (now - rightTurnStartTime < LF_RIGHT_TURN_90_MS) {
            float turnOmega = -turnSpeed * LF_RIGHT_TURN_OMEGA_RATIO;
            turnOmega = constrain(turnOmega, -turnSpeed, turnSpeed);
            lastOmegaCmd = turnOmega;
            mecanumControl.setTargetVelocity(0, 0, turnOmega);
            return;
        }
        rightTurning = false;
        filteredError = 0.0f;
        prevError = 0.0f;
        lastOmegaCmd = 0.0f;
        lastLineTime = now;
        isLost = false;
    }

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
    // pattern顺序按传感器物理顺序：S1 S2 S3 S4（左->右）
    uint8_t pattern = (s1 ? 0b1000 : 0) |
                      (s2 ? 0b0100 : 0) |
                      (s3 ? 0b0010 : 0) |
                      (s4 ? 0b0001 : 0);

    if (pattern != 0b0111) {
        rightTurnArmed = true;
    }

    if (pattern == 0b0111 && rightTurnArmed) {
        rightTurning = true;
        rightTurnArmed = false;
        rightTurnStartTime = now;
        lastLineTime = now;
        isLost = false;
        filteredError = 0.0f;
        prevError = 0.0f;
        float turnOmega = -turnSpeed * LF_RIGHT_TURN_OMEGA_RATIO;
        turnOmega = constrain(turnOmega, -turnSpeed, turnSpeed);
        lastOmegaCmd = turnOmega;
        mecanumControl.setTargetVelocity(0, 0, turnOmega);
        return;
    }

    if (activeCount > 0) {
        float mappedOmega = 0.0f;
        float mappedSpeedRatio = 1.0f;
        bool mappedState = true;

        switch (pattern) {
            case 0b0110: // 在线正中
            case 0b1111: // 十字路口，直行通过
                mappedOmega = 0.0f;
                mappedSpeedRatio = 1.0f;
                break;
            case 0b0100: // 轻微偏左
                mappedOmega = turnSpeed * LF_PATTERN_SLIGHT_TURN_RATIO;
                mappedSpeedRatio = LF_PATTERN_SLIGHT_SPEED_RATIO;
                break;
            case 0b0010: // 轻微偏右
                mappedOmega = -turnSpeed * LF_PATTERN_SLIGHT_TURN_RATIO;
                mappedSpeedRatio = LF_PATTERN_SLIGHT_SPEED_RATIO;
                break;
            case 0b1100: // 中等偏左
                mappedOmega = turnSpeed * LF_PATTERN_MEDIUM_TURN_RATIO;
                mappedSpeedRatio = LF_PATTERN_MEDIUM_SPEED_RATIO;
                break;
            case 0b0011: // 中等偏右
                mappedOmega = -turnSpeed * LF_PATTERN_MEDIUM_TURN_RATIO;
                mappedSpeedRatio = LF_PATTERN_MEDIUM_SPEED_RATIO;
                break;
            case 0b1000: // 大幅偏左
                mappedOmega = turnSpeed * LF_PATTERN_LARGE_TURN_RATIO;
                mappedSpeedRatio = LF_PATTERN_LARGE_SPEED_RATIO;
                break;
            case 0b0001: // 大幅偏右
                mappedOmega = -turnSpeed * LF_PATTERN_LARGE_TURN_RATIO;
                mappedSpeedRatio = LF_PATTERN_LARGE_SPEED_RATIO;
                break;
            default:
                mappedState = false;
                break;
        }

        if (mappedState) {
            float omegaCmd = omegaSmoothAlpha * lastOmegaCmd +
                             (1.0f - omegaSmoothAlpha) * mappedOmega;
            omegaCmd = constrain(omegaCmd, -turnSpeed, turnSpeed);

            float vxCmd = baseSpeed * mappedSpeedRatio;
            lastLineTime = now;
            isLost = false;
            filteredError = 0.0f;
            prevError = 0.0f;
            lastOmegaCmd = omegaCmd;
            mecanumControl.setTargetVelocity(vxCmd, 0, omegaCmd);
            return;
        }

        // 连续误差：左负右正
        float errorSum = 0.0f;
        if (s1) errorSum -= 3.0f;
        if (s2) errorSum -= 1.0f;
        if (s3) errorSum += 1.0f;
        if (s4) errorSum += 3.0f;

        float rawError = errorSum / activeCount;
        filteredError = errorFilterAlpha * filteredError +
                (1.0f - errorFilterAlpha) * rawError;

        float dError = (filteredError - prevError) / dt;
        dError = constrain(dError, -dErrorClamp, dErrorClamp);
        prevError = filteredError;

        float omegaRaw = -(filteredError * turnSpeed * turnGain + dError * dampingGain);
        float omegaCmd = omegaSmoothAlpha * lastOmegaCmd +
                 (1.0f - omegaSmoothAlpha) * omegaRaw;
        omegaCmd = constrain(omegaCmd, -turnSpeed, turnSpeed);

        float errorNorm = constrain(fabs(filteredError) / 3.0f, 0.0f, 1.0f);
        float speedScale = 1.0f - (errorNorm * speedReductionGain);
        speedScale = max(speedScale, minForwardRatio);
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
    if (lostDuration < lostHoldMs) {
        lastOmegaCmd *= lostOmegaDecay;
        mecanumControl.setTargetVelocity(baseSpeed * 0.75f, 0, lastOmegaCmd);
        return;
    }

    if (lostDuration < lostTimeoutMs) {
        float searchDirection = (lastOmegaCmd >= 0.0f) ? 1.0f : -1.0f;
        if (fabs(lastOmegaCmd) < DEAD_ZONE) {
            searchDirection = (filteredError <= 0.0f) ? 1.0f : -1.0f;
        }
        float searchOmega = searchDirection * turnSpeed * searchOmegaRatio;
        float searchVx = baseSpeed * searchSpeedRatio;
        lastOmegaCmd = searchOmega;
        mecanumControl.setTargetVelocity(searchVx, 0, searchOmega);
        return;
    }

    lastOmegaCmd = 0.0f;
    mecanumControl.setTargetVelocity(0, 0, 0);
}
