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
    float obstacleDistanceCm;
    unsigned long obstacleRetreatMs;
    unsigned long obstacleMeasureIntervalMs;

    // 巡线内置遇障左后退状态
    bool obstacleRetreating;
    unsigned long obstacleRetreatStartTime;
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

    // 避障后的赛道后半段逻辑：0000 由十字直行改为截止线入库
    bool postObstacleMode;
    bool reverseFollowing;
    bool garageMoving;
    bool garageDone;
    unsigned long reverseFollowStartTime;
    unsigned long garageMoveStartTime;
    unsigned long obstacleRetreatMaxMs;
    unsigned long garageMoveMs;
    float reverseFollowSpeedRatio;
    float reverseLostlineVyRatio;
    float garageMoveVxRatio;
    float garageMoveVyRatio;
    uint8_t finishLineConfirmFrames;
    uint8_t finishLineConfirmCount;

    static float wrapDeg180(float deg);
    void startRightTurnByImu(unsigned long now);
    bool updateRightTurnByImu(unsigned long now);

    bool isObstacleTooClose(unsigned long now);
};

#endif // LINEFOLLOWER_H
