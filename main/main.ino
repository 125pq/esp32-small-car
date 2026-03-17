/**
 * @file main.ino
 * @brief ESP32 四轮麦轮小车主控程序
 * @author 喵屋
 * @date 2025.12.4
 * 
 * 功能：通过WiFi接收控制指令，控制电机运动，集成多种传感器
 * 硬件：ESP32开发板、L298N电机驱动、OLED显示屏、MPU6050陀螺仪、超声波传感器、四路巡线传感器
 */

// * 基础库
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// * 功能模块
#include "Config.h"
#include "Motor.h"
#include "Ultrasonic.h"
#include "LineTracker.h"
#include "Display.h"
#include "MecanumControl.h"
#include "WebControl.h"
#include "LineFollower.h"
#include "ObstacleAvoidance.h"

// * 全局对象
MPU6050 mpu6050(Wire);           // MPU6050陀螺仪传感器
Motor motor;                     // 电机控制器
Ultrasonic ultrasonic;           // 超声波传感器
LineTracker lineTracker;         // 巡线传感器
Display display;                 // OLED显示屏
MecanumControl mecanumControl(motor, mpu6050);  // 麦轮运动控制器
WebControl webControl(mecanumControl);          // Web服务器控制器
LineFollower lineFollower(lineTracker, mecanumControl, &ultrasonic); // 巡线控制器（内置遇障后左后退）
ObstacleAvoidance obstacleAvoidance(ultrasonic, mecanumControl); // 避障控制器

/**
 * @brief 初始化函数
 */
void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(500);

    // * 初始化LED指示灯
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // * 初始化OLED显示屏
    if (!display.init()) {
        for (;;); // 停止程序
    }

    // * 初始化各传感器和执行器
    ultrasonic.init();
    lineTracker.init();
    motor.init();

    // * 测试电机
    display.show("Motor", "Testing...");
    motor.test();

    // * 初始化MPU6050
    display.show("MPU6050", "Calibrating...");
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    // * 初始化麦轮控制器
    mecanumControl.init();

    // * 初始化WiFi和Web服务器
    display.show(WIFI_SSID, WIFI_PASSWORD, "Connecting...");
    if (webControl.init()) {
        webControl.setLineFollower(&lineFollower);
        webControl.setObstacleAvoidance(&obstacleAvoidance);
        display.show(webControl.getIPAddress());
    } else {
        display.show("WiFi Failed!");
    }

    // * LED闪烁表示初始化完成
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
    }

}

/**
 * @brief 主循环函数
 */
void loop() {
    // ! 处理Web客户端请求
    webControl.handleClient();

    // ! 寻线控制
    lineFollower.update();
    // ! 避障控制
    obstacleAvoidance.update();

    // !
    // ! 执行麦轮控制更新
    mecanumControl.update();

    // ! 更新显示信息
    mpu6050.update();
    float vx, vy, omega;
    mecanumControl.getTargetVelocity(vx, vy, omega);
    display.showMecanumInfo(
        mpu6050.getAngleZ(),
        mpu6050.getGyroZ(),
        vx, vy, omega,
        mecanumControl.getAngleOutput(),
        mpu6050.getAccX(),
        mpu6050.getAccY(),
        mpu6050.getAccZ()
    );

    delay(2);
}