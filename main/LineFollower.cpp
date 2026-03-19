/**
 * @file LineFollower.cpp
 * @brief 寻线控制模块实现文件
 * @author 125pq
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
    lineStartTime = 0;
    lastObstacleMeasureTime = 0;
    lastObstacleDistance = 400.0f;
    rightTurning = false;
    rightTurnArmed = true;
    rightTurnStartTime = 0;
    rightTurnStartYawDeg = 0.0f;
    rightTurnTargetDeg = 80.0f;// 右转目标角度（相对于起始方向）
    rightTurnStopDeg = 3.0f;
    rightTurnStopGyroDegPerSec = 8.0f;
    rightTurnTimeoutMs = 2300UL;
    rightTurnStableCount = 0;
    rightTurnTriggerConfirmFrames = LF_RIGHT_TURN_TRIGGER_CONFIRM_FRAMES;
    rightTurnTriggerCount = 0;
    rightTurnCrossSuppressUntil = 0;
    rightTurnDelayStartTime = 0;
    rightTurnReacquiring = false;
    rightTurnReacquireStartTime = 0;
    rightTurnReacquireTimeoutMs = LF_RIGHT_TURN_REACQUIRE_TIMEOUT_MS;
    rightTurnReacquireOmegaRatio = LF_RIGHT_TURN_REACQUIRE_OMEGA_RATIO;
    rightTurnReacquireVxRatio = LF_RIGHT_TURN_REACQUIRE_VX_RATIO;
    rightTurnReacquireLineConfirmFrames = LF_RIGHT_TURN_REACQUIRE_LINE_CONFIRM_FRAMES;
    rightTurnReacquireLineConfirmCount = 0;

    postObstacleMode = false;
    postObstacleStage = PostObstacleStage::Idle;
    postStageStartTime = 0;
    postLockYawDeg = 0.0f;
    postLockYawKp = LF_POST_LOCK_YAW_KP;
    postLockYawMaxOmega = MAX_ROTATION_SPEED * LF_POST_LOCK_YAW_MAX_OMEGA_RATIO;
    postRetreatMaxMs = LF_OBSTACLE_RETREAT_MAX_MS;
    postRetreatBackVxRatio = LF_POST_RETREAT_BACK_VX_RATIO;
    postRetreatLeftVyRatio = LF_POST_RETREAT_LEFT_VY_RATIO;
    postReverseVxRatio = LF_POST_REVERSE_VX_RATIO;
    postReverseVyGain = LF_POST_REVERSE_VY_GAIN;
    postReverseVyMaxRatio = LF_POST_REVERSE_VY_MAX_RATIO;
    postStageBlendMs = LF_POST_STAGE_BLEND_MS;
    postGarageMoveMs = LF_POST_GARAGE_MOVE_MS;
    postGarageVxRatio = LF_POST_GARAGE_VX_RATIO;
    postGarageVyRatio = LF_POST_GARAGE_VY_RATIO;
    postFinishConfirmFrames = LF_POST_FINISH_LINE_CONFIRM_FRAMES;
    postFinishConfirmCount = 0;
}

void LineFollower::start() {
    running = true;
    lineStartTime = millis();
    lastObstacleMeasureTime = 0;
    lastObstacleDistance = 400.0f;
    rightTurning = false;
    rightTurnArmed = true;
    rightTurnStartTime = 0;
    rightTurnStartYawDeg = 0.0f;
    rightTurnStableCount = 0;
    rightTurnTriggerCount = 0;
    rightTurnCrossSuppressUntil = 0;
    rightTurnDelayStartTime = 0;
    rightTurnReacquiring = false;
    rightTurnReacquireStartTime = 0;
    rightTurnReacquireLineConfirmCount = 0;

    postObstacleMode = false;
    postObstacleStage = PostObstacleStage::Idle;
    postStageStartTime = 0;
    postLockYawDeg = 0.0f;
    postFinishConfirmCount = 0;
    Serial.println("Line Follower Started");
}

void LineFollower::stop() {
    running = false;
    rightTurning = false;
    rightTurnArmed = true;
    rightTurnStartTime = 0;
    rightTurnStartYawDeg = 0.0f;
    rightTurnStableCount = 0;
    rightTurnTriggerCount = 0;
    rightTurnCrossSuppressUntil = 0;
    rightTurnDelayStartTime = 0;
    rightTurnReacquiring = false;
    rightTurnReacquireStartTime = 0;
    rightTurnReacquireLineConfirmCount = 0;

    postObstacleMode = false;
    postObstacleStage = PostObstacleStage::Idle;
    postStageStartTime = 0;
    postFinishConfirmCount = 0;
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
    if (key == "sbd") {
        startBoostMs = (unsigned long)constrain(value, 0.0f, 5000.0f);
        return true;
    }
    if (key == "sbr") {
        startBoostRatio = constrain(value, 1.00f, 2.00f);
        return true;
    }
    if (key == "rpd") {
        rightTurnPreDelayMs = (unsigned long)constrain(value, 0.0f, 1200.0f);
        return true;
    }
    if (key == "orm") {
        obstacleRetreatMs = (unsigned long)constrain(value, 100.0f, 3000.0f);
        if (obstacleRetreatMs > postRetreatMaxMs) {
            postRetreatMaxMs = obstacleRetreatMs;
        }
        return true;
    }
    if (key == "orx") {
        postRetreatMaxMs = (unsigned long)constrain(value, 300.0f, 6000.0f);
        if (postRetreatMaxMs < obstacleRetreatMs) {
            postRetreatMaxMs = obstacleRetreatMs;
        }
        return true;
    }
    if (key == "prb") {
        postRetreatBackVxRatio = constrain(value, 0.10f, 1.50f);
        return true;
    }
    if (key == "prl") {
        postRetreatLeftVyRatio = constrain(value, 0.05f, 1.20f);
        return true;
    }
    if (key == "prv") {
        postReverseVxRatio = constrain(value, 0.10f, 1.50f);
        return true;
    }
    if (key == "prg") {
        postReverseVyGain = constrain(value, 0.05f, 1.50f);
        return true;
    }
    if (key == "prm") {
        postReverseVyMaxRatio = constrain(value, 0.05f, 1.00f);
        return true;
    }
    if (key == "psb") {
        postStageBlendMs = (unsigned long)constrain(value, 0.0f, 2000.0f);
        return true;
    }
    if (key == "pgm") {
        postGarageMoveMs = (unsigned long)constrain(value, 300.0f, 8000.0f);
        return true;
    }
    if (key == "pgx") {
        postGarageVxRatio = constrain(value, 0.10f, 1.20f);
        return true;
    }
    if (key == "pgy") {
        postGarageVyRatio = constrain(value, 0.10f, 1.20f);
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
    startBoostMs = LF_START_BOOST_MS;
    startBoostRatio = LF_START_BOOST_RATIO;
    rightTurnOmegaRatio = LF_RIGHT_TURN_OMEGA_RATIO;
    rightTurn90Ms = LF_RIGHT_TURN_90_MS;
    rightTurnPreDelayMs = LF_RIGHT_TURN_PRE_DELAY_MS;
    obstacleDistanceCm = LF_OBSTACLE_DISTANCE_CM;
    obstacleRetreatMs = LF_OBSTACLE_RETREAT_MS;
    obstacleMeasureIntervalMs = LF_OBSTACLE_MEASURE_INTERVAL_MS;

    postRetreatMaxMs = LF_OBSTACLE_RETREAT_MAX_MS;
    if (postRetreatMaxMs < obstacleRetreatMs) {
        postRetreatMaxMs = obstacleRetreatMs;
    }
    postRetreatBackVxRatio = LF_POST_RETREAT_BACK_VX_RATIO;
    postRetreatLeftVyRatio = LF_POST_RETREAT_LEFT_VY_RATIO;
    postReverseVxRatio = LF_POST_REVERSE_VX_RATIO;
    postReverseVyGain = LF_POST_REVERSE_VY_GAIN;
    postReverseVyMaxRatio = LF_POST_REVERSE_VY_MAX_RATIO;
    postStageBlendMs = LF_POST_STAGE_BLEND_MS;
    postGarageMoveMs = LF_POST_GARAGE_MOVE_MS;
    postGarageVxRatio = LF_POST_GARAGE_VX_RATIO;
    postGarageVyRatio = LF_POST_GARAGE_VY_RATIO;
}

String LineFollower::getTuningJson() const {
    String json = "{";
    json += "\"pst\":" + String(patternSlightTurnRatio, 3) + ",";
    json += "\"pmt\":" + String(patternMediumTurnRatio, 3) + ",";
    json += "\"plt\":" + String(patternLargeTurnRatio, 3) + ",";
    json += "\"pss\":" + String(patternSlightSpeedRatio, 3) + ",";
    json += "\"pms\":" + String(patternMediumSpeedRatio, 3) + ",";
    json += "\"pls\":" + String(patternLargeSpeedRatio, 3) + ",";
    json += "\"sbd\":" + String(startBoostMs) + ",";
    json += "\"sbr\":" + String(startBoostRatio, 3) + ",";
    json += "\"rpd\":" + String(rightTurnPreDelayMs) + ",";
    json += "\"orm\":" + String(obstacleRetreatMs) + ",";
    json += "\"orx\":" + String(postRetreatMaxMs) + ",";
    json += "\"prb\":" + String(postRetreatBackVxRatio, 3) + ",";
    json += "\"prl\":" + String(postRetreatLeftVyRatio, 3) + ",";
    json += "\"prv\":" + String(postReverseVxRatio, 3) + ",";
    json += "\"prg\":" + String(postReverseVyGain, 3) + ",";
    json += "\"prm\":" + String(postReverseVyMaxRatio, 3) + ",";
    json += "\"psb\":" + String(postStageBlendMs) + ",";
    json += "\"pgm\":" + String(postGarageMoveMs) + ",";
    json += "\"pgx\":" + String(postGarageVxRatio, 3) + ",";
    json += "\"pgy\":" + String(postGarageVyRatio, 3);
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
    rightTurnTriggerCount = 0;
    rightTurnCrossSuppressUntil = 0;
    rightTurnDelayStartTime = 0;
    rightTurnReacquiring = false;
    rightTurnReacquireStartTime = 0;
    rightTurnReacquireLineConfirmCount = 0;

    if (imu != nullptr) {
        imu->update();
        rightTurnStartYawDeg = imu->getAngleZ();
    } else {
        rightTurnStartYawDeg = 0.0f;
    }
}

bool LineFollower::updateRightTurnByImu(unsigned long now) {
    if (rightTurnReacquiring) {
        uint8_t state = lineTracker.getState();
        bool s2 = (state >> 1) & 1;
        bool s3 = (state >> 2) & 1;
        bool centerLineFound = (!s2) || (!s3);

        if (centerLineFound) {
            if (rightTurnReacquireLineConfirmCount < rightTurnReacquireLineConfirmFrames) {
                rightTurnReacquireLineConfirmCount++;
            }
        } else {
            rightTurnReacquireLineConfirmCount = 0;
        }

        if (rightTurnReacquireLineConfirmCount >= rightTurnReacquireLineConfirmFrames) {
            rightTurnReacquiring = false;
            rightTurnReacquireLineConfirmCount = 0;
            return true;
        }

        if (now - rightTurnReacquireStartTime >= rightTurnReacquireTimeoutMs) {
            // 超时后继续回线搜索，不再直接判定右转完成。
            rightTurnReacquireStartTime = now;
            rightTurnReacquireLineConfirmCount = 0;
        }

        float omegaCmd = -turnSpeed * rightTurnReacquireOmegaRatio;
        omegaCmd = constrain(omegaCmd, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
        float vxCmd = baseSpeed * rightTurnReacquireVxRatio;
        mecanumControl.setTargetVelocity(vxCmd, 0, omegaCmd);
        return false;
    }

    // 去掉角度完成判定：先按固定时长执行右转，再进入回线阶段。
    if (now - rightTurnStartTime < rightTurn90Ms) {
        float turnOmega = -turnSpeed * rightTurnOmegaRatio;
        turnOmega = constrain(turnOmega, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
        mecanumControl.setTargetVelocity(0, 0, turnOmega);
        return false;
    }

    rightTurnReacquiring = true;
    rightTurnReacquireStartTime = now;
    rightTurnReacquireLineConfirmCount = 0;
    return false;
}

void LineFollower::update() {
    if (!running) return;

    unsigned long now = millis();
    float effectiveBaseSpeed = baseSpeed;
    if (startBoostMs > 0UL && now - lineStartTime < startBoostMs) {
        effectiveBaseSpeed = constrain(baseSpeed * startBoostRatio, 0.0f, MAX_LINEAR_SPEED);
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

    auto computePostLockOmega = [&]() -> float {
        if (imu == nullptr) {
            return 0.0f;
        }

        imu->update();
        float yawError = wrapDeg180(postLockYawDeg - imu->getAngleZ());
        if (fabs(yawError) < 1.0f) {
            return 0.0f;
        }

        float omegaCmd = yawError * postLockYawKp;
        return constrain(omegaCmd, -postLockYawMaxOmega, postLockYawMaxOmega);
    };

    auto blendByStageElapsed = [&](unsigned long stageElapsed) -> float {
        if (postStageBlendMs == 0UL || stageElapsed >= postStageBlendMs) {
            return 1.0f;
        }
        float t = (float)stageElapsed / (float)postStageBlendMs;
        return constrain(t, 0.0f, 1.0f);
    };

    if (!postObstacleMode && isObstacleTooClose(now)) {
        // 触发后程模式：记录避障前航向并切换到左后退阶段。
        postObstacleMode = true;
        postObstacleStage = PostObstacleStage::Retreat;
        postStageStartTime = now;
        postFinishConfirmCount = 0;

        rightTurning = false;
        rightTurnArmed = false;
        rightTurnDelayStartTime = 0;

        if (imu != nullptr) {
            imu->update();
            postLockYawDeg = imu->getAngleZ();
        } else {
            postLockYawDeg = 0.0f;
        }

        lastObstacleDistance = 400.0f;
        lastObstacleMeasureTime = 0;
    }

    if (postObstacleMode) {
        float lockOmegaCmd = computePostLockOmega();

        switch (postObstacleStage) {
            case PostObstacleStage::Retreat: {
                float vxCmd = -effectiveBaseSpeed * postRetreatBackVxRatio;
                float vyCmd = effectiveBaseSpeed * postRetreatLeftVyRatio;
                mecanumControl.setTargetVelocity(vxCmd, vyCmd, lockOmegaCmd);

                unsigned long stageElapsed = now - postStageStartTime;
                bool lineFound = (pattern != 0b1111);
                if ((stageElapsed >= obstacleRetreatMs && lineFound) ||
                    stageElapsed >= postRetreatMaxMs) {
                    postObstacleStage = PostObstacleStage::ReverseTrack;
                    postStageStartTime = now;
                    postFinishConfirmCount = 0;
                }
                return;
            }

            case PostObstacleStage::ReverseTrack: {
                unsigned long stageElapsed = now - postStageStartTime;

                if (pattern == 0b0000) {
                    if (postFinishConfirmCount < postFinishConfirmFrames) {
                        postFinishConfirmCount++;
                    }
                } else {
                    postFinishConfirmCount = 0;
                }

                if (postFinishConfirmCount >= postFinishConfirmFrames) {
                    postObstacleStage = PostObstacleStage::GarageMove;
                    postStageStartTime = now;
                    return;
                }

                float vxCmd = -effectiveBaseSpeed * postReverseVxRatio;
                float vyCmd = 0.0f;
                int activeCount = (l1 ? 1 : 0) + (l2 ? 1 : 0) + (l3 ? 1 : 0) + (l4 ? 1 : 0);

                if (activeCount > 0) {
                    float errorSum = 0.0f;
                    if (l1) errorSum -= 3.0f;
                    if (l2) errorSum -= 1.0f;
                    if (l3) errorSum += 1.0f;
                    if (l4) errorSum += 3.0f;

                    float rawError = errorSum / activeCount;
                    vyCmd = -rawError * (effectiveBaseSpeed * postReverseVyGain);
                    float vyLimit = effectiveBaseSpeed * postReverseVyMaxRatio;
                    vyCmd = constrain(vyCmd, -vyLimit, vyLimit);
                }

                float blend = blendByStageElapsed(stageElapsed);
                float retreatVx = -effectiveBaseSpeed * postRetreatBackVxRatio;
                float retreatVy = effectiveBaseSpeed * postRetreatLeftVyRatio;
                vxCmd = retreatVx + (vxCmd - retreatVx) * blend;
                vyCmd = retreatVy + (vyCmd - retreatVy) * blend;

                mecanumControl.setTargetVelocity(vxCmd, vyCmd, lockOmegaCmd);
                return;
            }

            case PostObstacleStage::GarageMove: {
                unsigned long stageElapsed = now - postStageStartTime;
                if (stageElapsed < postGarageMoveMs) {
                    float targetVx = effectiveBaseSpeed * postGarageVxRatio;
                    float targetVy = -effectiveBaseSpeed * postGarageVyRatio;

                    float blend = blendByStageElapsed(stageElapsed);
                    float reverseVx = -effectiveBaseSpeed * postReverseVxRatio;
                    float reverseVy = 0.0f;
                    float vxCmd = reverseVx + (targetVx - reverseVx) * blend;
                    float vyCmd = reverseVy + (targetVy - reverseVy) * blend;

                    mecanumControl.setTargetVelocity(vxCmd, vyCmd, lockOmegaCmd);
                    return;
                }

                mecanumControl.setTargetVelocity(0, 0, 0);
                running = false;
                postObstacleMode = false;
                postObstacleStage = PostObstacleStage::Idle;
                Serial.println("Line Follower Finished: Garage parked");
                return;
            }

            default:
                postObstacleMode = false;
                postObstacleStage = PostObstacleStage::Idle;
                break;
        }
    }

    if (rightTurning) {
        if (!updateRightTurnByImu(now)) {
            return;
        }
        rightTurning = false;
    }

    if (rightTurnDelayStartTime != 0) {
        if (now - rightTurnDelayStartTime < rightTurnPreDelayMs) {
            float vxCmd = effectiveBaseSpeed * LF_RIGHT_TURN_PRE_DELAY_VX_RATIO;
            mecanumControl.setTargetVelocity(vxCmd, 0, 0);
            return;
        }

        rightTurnDelayStartTime = 0;
        startRightTurnByImu(now);
        if (rightTurning) {
            updateRightTurnByImu(now);
        }
        return;
    }

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

    if (!postObstacleMode) {
        if (pattern == 0b0000) {
            // 十字路口阶段不允许立即触发右转，避免误判。
            rightTurnArmed = false;
            rightTurnTriggerCount = 0;
            rightTurnDelayStartTime = 0;
            rightTurnCrossSuppressUntil = now + LF_RIGHT_TURN_CROSS_SUPPRESS_MS;
        } else if ((l2 || l3) && now >= rightTurnCrossSuppressUntil) {
            // 回到中间传感器可见线后，重新使能右转触发。
            rightTurnArmed = true;
        }

        if (now >= rightTurnCrossSuppressUntil && pattern == 0b1000 && rightTurnArmed) {
            if (rightTurnTriggerCount < rightTurnTriggerConfirmFrames) {
                rightTurnTriggerCount++;
            }
        } else {
            rightTurnTriggerCount = 0;
        }

        if (rightTurnTriggerCount >= rightTurnTriggerConfirmFrames) {
            rightTurnTriggerCount = 0;
            rightTurnArmed = false;
            rightTurnDelayStartTime = now;
#if defined(LF_DEBUG_PATTERN) && LF_DEBUG_PATTERN
            logPatternDecision(1, "RIGHT_TURN_PENDING");
#endif
            float vxCmd = effectiveBaseSpeed * LF_RIGHT_TURN_PRE_DELAY_VX_RATIO;
            mecanumControl.setTargetVelocity(vxCmd, 0, 0);
            return;
        }
    }

    float mappedOmega = 0.0f;
    float mappedSpeedRatio = 1.0f;
    bool mappedState = true;
    uint8_t mappedDecision = 0;
    const char* mappedLabel = "UNKNOWN";

    switch (pattern) {
        case 0b1001: // 在线正中
        case 0b0000: // 十字路口，前段直行通过（后段由postObstacleMode提前拦截）
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
        float vxCmd = effectiveBaseSpeed * mappedSpeedRatio;
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
        float vxCmd = effectiveBaseSpeed * patternMediumSpeedRatio;
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
