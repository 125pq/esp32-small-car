/**
 * @file SerialCommander.cpp
 * @brief 串口调试与控制模块实现文件
 * @author Copilot
 * @date 2026.3.2
 */

#include "SerialCommander.h"

SerialCommander::SerialCommander(MecanumControl& control) 
    : mecanumControl(control), reportEnable(false), lastReportTime(0) {
    inputString.reserve(50);
}

void SerialCommander::setReportEnable(bool enable) {
    reportEnable = enable;
}
/**
 * @brief 在主循环中处理串口通信
 */
void SerialCommander::update() {
    // 处理串口输入
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            processCommand(inputString);
            inputString = "";
        } else {
            inputString += inChar;
        }
    }

    // 定时上报状态
    if (reportEnable && millis() - lastReportTime > 50) { // 20Hz 上报状态
        reportStatus();
        lastReportTime = millis();
    }
}

void SerialCommander::processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.startsWith("SET_PID")) {
        // 命令格式: SET_PID <Kp> <Ki> <Kd>
        int firstSpace = cmd.indexOf(' ');
        if (firstSpace > 0) {
            String params = cmd.substring(firstSpace + 1);
            int secondSpace = params.indexOf(' ');
            if (secondSpace > 0) {
                String kpStr = params.substring(0, secondSpace);
                String rest = params.substring(secondSpace + 1);
                int thirdSpace = rest.indexOf(' ');
                
                String kiStr = (thirdSpace > 0) ? rest.substring(0, thirdSpace) : rest;
                String kdStr = (thirdSpace > 0) ? rest.substring(thirdSpace + 1) : "0";
                
                double Kp = kpStr.toDouble();
                double Ki = kiStr.toDouble();
                double Kd = kdStr.toDouble();
                
                mecanumControl.setAnglePIDTunings(Kp, Ki, Kd);
                Serial.print("PID Updated: Kp="); Serial.print(Kp);
                Serial.print(", Ki="); Serial.print(Ki, 4);
                Serial.print(", Kd="); Serial.println(Kd);
            }
        }
    } else if (cmd.startsWith("GET_PID")) {
        double Kp, Ki, Kd;
        mecanumControl.getAnglePIDTunings(Kp, Ki, Kd);
        Serial.print("Current PID: Kp="); Serial.print(Kp);
        Serial.print(", Ki="); Serial.print(Ki, 4);
        Serial.print(", Kd="); Serial.println(Kd);
    } else if (cmd.startsWith("START_REPORT")) {
        reportEnable = true;
        Serial.println("Reporting Started");
    } else if (cmd.startsWith("STOP_REPORT")) {
        reportEnable = false;
        Serial.println("Reporting Stopped");
    } else if (cmd.startsWith("HELP")) {
        Serial.println("Available Commands:");
        Serial.println("  SET_PID <Kp> <Ki> <Kd>  - Set Angle PID parameters");
        Serial.println("  GET_PID                 - Get Angle PID parameters");
        Serial.println("  START_REPORT            - Start streaming status data");
        Serial.println("  STOP_REPORT             - Stop streaming status data");
    } else {
        Serial.println("Unknown command. Type HELP for list.");
    }
}

void SerialCommander::reportStatus() {
    // 格式: 'DATA: <Input>, <Setpoint>, <Output>'
    // 这种CSV格式可以直接用于 Serial Plotter
    float angleInput = (float)mecanumControl.getAngleInput();
    float angleSetpoint = (float)mecanumControl.getAngleSetpoint();
    float angleOutput = (float)mecanumControl.getAngleOutput();
    
    Serial.print("DATA:");
    Serial.print(angleInput);
    Serial.print(",");
    Serial.print(angleSetpoint);
    Serial.print(",");
    Serial.println(angleOutput);
}
