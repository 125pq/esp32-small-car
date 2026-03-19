/**
 * @file Display.cpp
 * @brief OLED显示模块实现文件
 * @author 黄竞亿
 * @date 2025.12.4
 */

#include "Display.h"

Display::Display() : 
    display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1),
    lastUpdateTime(0) {
    // 构造函数
}

bool Display::init() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        return false;
    }
    display.display();  // 显示Adafruit启动画面
    return true;
}

void Display::show(String text1) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(text1);
    display.display();
}

void Display::show(String text1, String text2) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(text1);
    display.println(text2);
    display.display();
}

void Display::show(String text1, String text2, String text3) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(text1);
    display.println(text2);
    display.println(text3);
    display.display();
}

void Display::showMecanumInfo(float angleZ, float gyroZ, float vx, float vy, 
                              float omega, float angleOutput,
                              float accX, float accY, float accZ) {
    // 限制更新频率，避免闪烁
    if (millis() - lastUpdateTime < 500) {
        return;
    }
    lastUpdateTime = millis();
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    
    display.print("AngleZ: "); 
    display.print(angleZ);
    display.print(" GyroZ: "); 
    display.println(gyroZ);
    
    display.print("T-Vx: "); 
    display.print(vx, 2);
    display.print(" T-Vy: "); 
    display.println(vy, 2);
    
    display.print("T-Omega: "); 
    display.print(omega, 2);
    display.print(" A-Out: "); 
    display.println(angleOutput, 2);
    
    display.print("MPU-C: "); 
    display.print(accX, 2);
    display.print(","); 
    display.print(accY, 2);
    display.print(","); 
    display.println(accZ, 2);
    
    display.display();
}

void Display::clear() {
    display.clearDisplay();
    display.display();
}

Adafruit_SSD1306& Display::getDisplay() {
    return display;
}
