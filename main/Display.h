/**
 * @file Display.h
 * @brief OLED显示模块头文件
 * @author 黄竞亿
 * @date 2025.12.4
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"

/**
 * @class Display
 * @brief OLED显示屏控制类
 */
class Display {
public:
    /**
     * @brief 构造函数
     */
    Display();

    /**
     * @brief 初始化OLED显示屏
     * @return true-成功, false-失败
     */
    bool init();

    /**
     * @brief 显示一行文本
     * @param text1 第一行文本
     */
    void show(String text1);

    /**
     * @brief 显示两行文本
     * @param text1 第一行文本
     * @param text2 第二行文本
     */
    void show(String text1, String text2);

    /**
     * @brief 显示三行文本
     * @param text1 第一行文本
     * @param text2 第二行文本
     * @param text3 第三行文本
     */
    void show(String text1, String text2, String text3);

    /**
     * @brief 显示麦轮小车信息
     * @param angleZ Z轴角度
     * @param gyroZ Z轴陀螺仪数据
     * @param vx X轴速度
     * @param vy Y轴速度
     * @param omega 旋转速度
     * @param angleOutput 角度输出（当前固定为0）
     * @param accX X轴加速度
     * @param accY Y轴加速度
     * @param accZ Z轴加速度
     */
    void showMecanumInfo(float angleZ, float gyroZ, float vx, float vy, 
                         float omega, float angleOutput, 
                         float accX, float accY, float accZ);

    /**
     * @brief 清空显示
     */
    void clear();

    /**
     * @brief 获取显示对象
     * @return Adafruit_SSD1306显示对象引用
     */
    Adafruit_SSD1306& getDisplay();

private:
    Adafruit_SSD1306 display;
    unsigned long lastUpdateTime;
};

#endif // DISPLAY_H
