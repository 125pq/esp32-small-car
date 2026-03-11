/**
 * @file WebControl.cpp
 * @brief Web服务器控制模块实现文件
 * @author 喵屋
 * @date 2025.12.4
 */

#include "WebControl.h"

WebControl::WebControl(MecanumControl& mecanumCtrl) 
    : mecanumControl(mecanumCtrl) {
    server = new WebServer(WEB_PORT);
}

bool WebControl::init() {
    // 设置WiFi模式
    WiFi.mode(WIFI_STA);
    
    // 连接到WiFi网络
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        Serial.print(".");
        delay(500);
        attempts++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection failed!");
        return false;
    }
    
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // 设置Web服务器路由
    server->on("/", [this]() { this->handleRoot(); });
    server->on("/control", [this]() { this->handleControl(); });
    
    // 启动Web服务器
    server->begin();
    Serial.println("Web server started");
    
    return true;
}

void WebControl::handleClient() {
    server->handleClient();
}

bool WebControl::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

String WebControl::getIPAddress() {
    return WiFi.localIP().toString();
}

void WebControl::handleRoot() {
    String html = generateHTML();
    server->send(200, "text/html", html);
}

void WebControl::handleControl() {
    String cmd = server->arg("cmd");
    String val = server->arg("val");
    
    // speed variable replaced by member variable currentSpeed
    
    if (cmd == "SP") {
        currentSpeed = val.toFloat() / 100.0;
        // Optional: Update current motion if moving?
        // For now, just store the speed.
        if (lineFollower != nullptr) {
            lineFollower->setSpeed(currentSpeed);
        }
        if (obstacleAvoidance != nullptr) {
            obstacleAvoidance->setSpeed(currentSpeed);
        }
    } else if (cmd == "F") {
        mecanumControl.setTargetVelocity(currentSpeed * MAX_LINEAR_SPEED, 0, 0);
    } else if (cmd == "B") {
        mecanumControl.setTargetVelocity(-currentSpeed * MAX_LINEAR_SPEED, 0, 0);
    } else if (cmd == "L") {
        mecanumControl.setTargetVelocity(0, currentSpeed * MAX_LINEAR_SPEED, 0);
    } else if (cmd == "R") {
        mecanumControl.setTargetVelocity(0, -currentSpeed * MAX_LINEAR_SPEED, 0);
    } else if (cmd == "FL") {
        mecanumControl.setTargetVelocity(currentSpeed * MAX_LINEAR_SPEED, currentSpeed * MAX_LINEAR_SPEED, 0);
    } else if (cmd == "FR") {
        mecanumControl.setTargetVelocity(currentSpeed * MAX_LINEAR_SPEED, -currentSpeed * MAX_LINEAR_SPEED, 0);
    } else if (cmd == "BL") {
        mecanumControl.setTargetVelocity(-currentSpeed * MAX_LINEAR_SPEED, currentSpeed * MAX_LINEAR_SPEED, 0);
    } else if (cmd == "BR") {
        mecanumControl.setTargetVelocity(-currentSpeed * MAX_LINEAR_SPEED, -currentSpeed * MAX_LINEAR_SPEED, 0);
    } else if (cmd == "CL") {
        mecanumControl.setTargetVelocity(0, 0, currentSpeed * MAX_ROTATION_SPEED);
    } else if (cmd == "CR") {
        mecanumControl.setTargetVelocity(0, 0, -currentSpeed * MAX_ROTATION_SPEED);
    } else if (cmd == "S") {
        mecanumControl.setTargetVelocity(0, 0, 0);
        if (lineFollower != nullptr) {
            lineFollower->stop();
        }
        if (obstacleAvoidance != nullptr) {
            obstacleAvoidance->stop();
        }
    } else if (cmd == "LF") {
        if (lineFollower != nullptr) {
            String val = server->arg("val");
            if (val == "1") {
                if (obstacleAvoidance != nullptr) obstacleAvoidance->stop();
                lineFollower->setSpeed(currentSpeed);
                lineFollower->start();
            } else {
                lineFollower->stop();
            }
        }
    } else if (cmd == "OA") {
        if (obstacleAvoidance != nullptr) {
            String val = server->arg("val");
            if (val == "1") {
                if (lineFollower != nullptr) lineFollower->stop();
                obstacleAvoidance->setSpeed(currentSpeed);
                obstacleAvoidance->start();
            } else {
                obstacleAvoidance->stop();
            }
        }
    }
    
    server->send(200, "text/plain", "OK");
}

String WebControl::generateHTML() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;text-align:center;}";
    html += ".control{display:inline-block;margin:10px;}";
    html += "button{width:80px;height:80px;margin:5px;font-size:16px;}";
    html += ".slider{width:300px;margin:10px;}</style></head>";
    html += "<body><h1>Mecanum Wheel Car Control</h1>";
    
    // 方向控制
    html += "<div class='control'><h2>Direction</h2>";
    html += "<button onclick=\"control('F')\">Forward</button><br>";
    html += "<button onclick=\"control('L')\">Left</button>";
    html += "<button onclick=\"control('R')\">Right</button><br>";
    html += "<button onclick=\"control('B')\">Backward</button>";
    html += "<button onclick=\"control('S')\">Stop</button></div>";
    
    // 斜向移动
    html += "<div class='control'><h2>Diagonal</h2>";
    html += "<button onclick=\"control('FL')\">Front-Left</button>";
    html += "<button onclick=\"control('FR')\">Front-Right</button><br>";
    html += "<button onclick=\"control('BL')\">Back-Left</button>";
    html += "<button onclick=\"control('BR')\">Back-Right</button></div>";
    
    // 旋转控制
    html += "<div class='control'><h2>Rotation</h2>";
    html += "<button onclick=\"control('CL')\">Rotate Left</button><br>";
    html += "<button onclick=\"control('CR')\">Rotate Right</button></div>";
    
    // 巡线控制
    html += "<div class='control'><h2>Line Follow</h2>";
    htm避障控制
    html += "<div class='control'><h2>Obstacle Avoidance</h2>";
    html += "<button onclick=\"controlVal('OA', '1')\">Start</button>";
    html += "<button onclick=\"controlVal('OA', '0')\">Stop</button></div>";

    // l += "<button onclick=\"controlVal('LF', '1')\">Start</button>";
    html += "<button onclick=\"controlVal('LF', '0')\">Stop</button></div>";

    // 速度控制
    int currentSpeedInt = (int)(currentSpeed * 100);
    html += "<div class='control'><h2>Speed</h2>";
    html += "<input type='range' id='speed' class='slider' min='0' max='100' value='" + String(currentSpeedInt) + "' onchange='updateSpeed()'>";
    html += "<p>Speed: <span id='speedValue'>" + String(currentSpeedInt) + "</span>%</p></div>";
    
    html += "<script>";
    html += "function control(cmd){fetch('/control?cmd='+cmd);}";
    html += "function controlVal(cmd, val){fetch('/control?cmd='+cmd+'&val='+val);}";
    html += "function updateSpeed(){";
    html += "var speed = document.getElementById('speed').value;";
    html += "document.getElementById('speedValue').innerHTML = speed;";
    html += "fetch('/control?cmd=SP&val='+speed);}";
    html += "</script></body></html>";
    
    return html;
}
