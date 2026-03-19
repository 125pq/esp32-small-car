/**
 * @file LineFollower.h
 * @brief 寻线控制模块头文件
 * @author 黄竞亿
 * @date 2026.3.2
 */

#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include "LineTracker.h"
#include "MecanumControl.h"
#include "Ultrasonic.h"
#include <MPU6050_tockn.h>

enum class PostObstacleStage : uint8_t {
    Idle = 0,
    Retreat,
    ReverseTrack,
    GarageMove
};

class LineFollower {
public:
    /**
     * @brief 构造函数
     * @param tracker 巡线传感器引用
     * @param control 麦轮控制引用
        * @param ultrasonicSensor 超声波传感器（可选）
     */
    LineFollower(LineTracker& tracker, MecanumControl& control, Ultrasonic* ultrasonicSensor = nullptr, MPU6050* imuSensor = nullptr);

    /**
        * @brief 设置/更新超声波传感器引用
        */
    void setUltrasonic(Ultrasonic* ultrasonicSensor);

    /**
     * @brief 设置/更新IMU传感器引用
     */
    void setImu(MPU6050* imuSensor);

    /**
     * @brief 开始寻线
     */
    void start();

    /**
     * @brief 停止寻线
     */
    void stop();

    /**
     * @brief 设置巡线速度
     * @param speed 速度值 (0.0 - 1.0)
     */
    void setSpeed(float speed);

    /**
     * @brief 设置基础巡线参数
     * @param key 参数键
     * @param value 参数值
     * @return true-设置成功, false-参数不存在
     */
    bool setTuning(const String& key, float value);

    /**
     * @brief 恢复基础巡线参数默认值
     */
    void resetTuningToDefault();

    /**
     * @brief 获取基础巡线参数JSON
     */
    String getTuningJson() const;

    /**
     * @brief 更新寻线状态（在loop中调用）
     */
    void update();

    /**
     * @brief 检查是否正在运行
     */
    bool isRunning();

private:
    LineTracker& lineTracker;
    MecanumControl& mecanumControl;
    Ultrasonic* ultrasonic;
    MPU6050* imu;
    bool running;
    
    // 基础速度参数
    float baseSpeed;
    float turnSpeed;

    // 基础巡线可调参数
    float patternSlightTurnRatio;
    float patternMediumTurnRatio;
    float patternLargeTurnRatio;
    float patternSlightSpeedRatio;
    float patternMediumSpeedRatio;
    float patternLargeSpeedRatio;
    float rightTurnOmegaRatio;
    unsigned long rightTurn90Ms;
    unsigned long rightTurnPreDelayMs;
    float obstacleDistanceCm;
    unsigned long obstacleRetreatMs;
    unsigned long obstacleMeasureIntervalMs;

    // 巡线内置遇障检测缓存
    unsigned long lastObstacleMeasureTime;
    float lastObstacleDistance;

    // 右转路口动作状态
    bool rightTurning;
    bool rightTurnArmed;
    unsigned long rightTurnStartTime;
    float rightTurnStartYawDeg;
    float rightTurnTargetDeg;
    float rightTurnStopDeg;
    float rightTurnStopGyroDegPerSec;
    unsigned long rightTurnTimeoutMs;
    uint8_t rightTurnStableCount;
    uint8_t rightTurnTriggerConfirmFrames;
    uint8_t rightTurnTriggerCount;
    unsigned long rightTurnCrossSuppressUntil;
    unsigned long rightTurnDelayStartTime;
    bool rightTurnReacquiring;
    unsigned long rightTurnReacquireStartTime;
    unsigned long rightTurnReacquireTimeoutMs;
    float rightTurnReacquireOmegaRatio;
    float rightTurnReacquireVxRatio;
    uint8_t rightTurnReacquireLineConfirmFrames;
    uint8_t rightTurnReacquireLineConfirmCount;

    // 避障后的赛道后半段逻辑（锁定航向 + 纯xy）
    bool postObstacleMode;
    PostObstacleStage postObstacleStage;
    unsigned long postStageStartTime;
    float postLockYawDeg;
    float postLockYawKp;
    float postLockYawMaxOmega;
    unsigned long postRetreatMaxMs;
    float postRetreatBackVxRatio;
    float postRetreatLeftVyRatio;
    float postReverseVxRatio;
    float postReverseVyGain;
    float postReverseVyMaxRatio;
    unsigned long postGarageMoveMs;
    float postGarageVxRatio;
    float postGarageVyRatio;
    uint8_t postFinishConfirmFrames;
    uint8_t postFinishConfirmCount;

    static float wrapDeg180(float deg);
    void startRightTurnByImu(unsigned long now);
    bool updateRightTurnByImu(unsigned long now);

    bool isObstacleTooClose(unsigned long now);
};

#endif // LINEFOLLOWER_H
