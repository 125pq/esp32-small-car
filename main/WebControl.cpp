/**
 * @file WebControl.cpp
 * @brief Web服务器控制模块实现文件
 * @author 黄竞亿
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
    server->on("/lineParams", [this]() { this->handleLineParams(); });
    
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

void WebControl::handleLineParams() {
    if (lineFollower == nullptr) {
        server->send(200, "application/json", "{}");
        return;
    }
    server->send(200, "application/json", lineFollower->getTuningJson());
}

void WebControl::handleControl() {
    String cmd = server->arg("cmd");
    String val = server->arg("val");
    
    // speed variable replaced by member variable currentSpeed
    
    // Debug output
    // Serial.print("CMD: "); Serial.println(cmd);
    
    if (cmd == "SP") {
        currentSpeed = constrain(val.toFloat() / 100.0f, 0.0f, 1.0f);
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
        float omegaCmd = currentSpeed * MAX_ROTATION_SPEED * WEB_ROTATE_BOOST * WEB_ROTATE_LEFT_GAIN;
        mecanumControl.setTargetVelocity(0, 0, omegaCmd);
    } else if (cmd == "CR") {
        float omegaCmd = currentSpeed * MAX_ROTATION_SPEED * WEB_ROTATE_BOOST * WEB_ROTATE_RIGHT_GAIN;
        mecanumControl.setTargetVelocity(0, 0, -omegaCmd);
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
    } else if (cmd == "LFP") {
        if (lineFollower == nullptr) {
            server->send(503, "text/plain", "Line follower unavailable");
            return;
        }
        String key = server->arg("key");
        if (key.length() == 0 || !server->hasArg("val")) {
            server->send(400, "text/plain", "Missing key or val");
            return;
        }
        float tuneValue = server->arg("val").toFloat();
        if (!lineFollower->setTuning(key, tuneValue)) {
            server->send(400, "text/plain", "Unknown tuning key");
            return;
        }
    } else if (cmd == "LFR") {
        if (lineFollower == nullptr) {
            server->send(503, "text/plain", "Line follower unavailable");
            return;
        }
        lineFollower->resetTuningToDefault();
    }
    
    server->send(200, "text/plain", "OK");
}

String WebControl::generateHTML() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;text-align:center;background:#f7f7f7;margin:0;padding:10px;}";
    html += ".control{display:inline-block;margin:10px;vertical-align:top;background:#fff;padding:10px;border-radius:8px;box-shadow:0 2px 8px rgba(0,0,0,.1);} ";
    html += "button{width:88px;height:44px;margin:4px;font-size:14px;border-radius:6px;border:1px solid #ccc;background:#fafafa;}";
    html += ".slider{width:260px;margin:6px 0;}";
    html += ".param{margin:8px 0;text-align:left;}";
    html += ".param label{display:block;font-size:13px;color:#333;}";
    html += ".smallbtn{width:auto;height:34px;padding:0 12px;}";
    html += "</style></head>";
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
    html += "<button onclick=\"controlVal('LF', '1')\">Start</button>";
    html += "<button onclick=\"controlVal('LF', '0')\">Stop</button></div>";
    

    // 避障控制
    html += "<div class='control'><h2>Obstacle Avoidance</h2>";
    html += "<button onclick=\"controlVal('OA', '1')\">Start</button>";
    html += "<button onclick=\"controlVal('OA', '0')\">Stop</button></div>";

    // 速度控制
    int currentSpeedInt = (int)(currentSpeed * 100);
    html += "<div class='control'><h2>Speed</h2>";
    html += "<input type='range' id='speed' class='slider' min='0' max='100' value='" + String(currentSpeedInt) + "' oninput='updateSpeed()' onchange='updateSpeed(true)'>";
    html += "<p>Speed: <span id='speedValue'>" + String(currentSpeedInt) + "</span>%</p></div>";

    // 基础循迹参数调节
    html += "<div class='control'><h2>Line Follow Basic Params</h2>";
    html += "<div class='param'><label>Slight Turn Ratio: <span id='v_pst'>0</span></label><input type='range' id='p_pst' class='slider' min='0.10' max='2' step='0.01' oninput=\"updateParam('pst','p_pst','v_pst')\"></div>";
    html += "<div class='param'><label>Medium Turn Ratio: <span id='v_pmt'>0</span></label><input type='range' id='p_pmt' class='slider' min='0.20' max='2.5' step='0.01' oninput=\"updateParam('pmt','p_pmt','v_pmt')\"></div>";
    html += "<div class='param'><label>Large Turn Ratio: <span id='v_plt'>0</span></label><input type='range' id='p_plt' class='slider' min='0.30' max='5' step='0.01' oninput=\"updateParam('plt','p_plt','v_plt')\"></div>";
    html += "<div class='param'><label>Slight Speed Ratio: <span id='v_pss'>0</span></label><input type='range' id='p_pss' class='slider' min='0.30' max='1.00' step='0.01' oninput=\"updateParam('pss','p_pss','v_pss')\"></div>";
    html += "<div class='param'><label>Medium Speed Ratio: <span id='v_pms'>0</span></label><input type='range' id='p_pms' class='slider' min='0.30' max='1.00' step='0.01' oninput=\"updateParam('pms','p_pms','v_pms')\"></div>";
    html += "<div class='param'><label>Large Speed Ratio: <span id='v_pls'>0</span></label><input type='range' id='p_pls' class='slider' min='0.20' max='1.00' step='0.01' oninput=\"updateParam('pls','p_pls','v_pls')\"></div>";
    html += "<div class='param'><label>Right Turn Omega Ratio: <span id='v_rto'>0</span></label><input type='range' id='p_rto' class='slider' min='0.30' max='1.20' step='0.01' oninput=\"updateParam('rto','p_rto','v_rto')\"></div>";
    html += "<div class='param'><label>Right Turn 90ms: <span id='v_rtm'>0</span></label><input type='range' id='p_rtm' class='slider' min='200' max='2500' step='10' oninput=\"updateParam('rtm','p_rtm','v_rtm')\"></div>";
    html += "<div class='param'><label>Obstacle Distance (cm): <span id='v_odc'>0</span></label><input type='range' id='p_odc' class='slider' min='3' max='50' step='0.5' oninput=\"updateParam('odc','p_odc','v_odc')\"></div>";
    html += "<div class='param'><label>Obstacle Retreat (ms): <span id='v_orm'>0</span></label><input type='range' id='p_orm' class='slider' min='100' max='3000' step='10' oninput=\"updateParam('orm','p_orm','v_orm')\"></div>";
    html += "<div class='param'><label>Obstacle Measure Interval (ms): <span id='v_omi'>0</span></label><input type='range' id='p_omi' class='slider' min='20' max='1000' step='10' oninput=\"updateParam('omi','p_omi','v_omi')\"></div>";
    html += "<button class='smallbtn' onclick='resetLineParams()'>Reset Line Params</button></div>";
    
    html += "<script>";
    html += "var lineParamTimer = {};";
    html += "var speedTimer = null;";
    html += "function control(cmd){fetch('/control?cmd='+cmd);}";
    html += "function controlVal(cmd, val){fetch('/control?cmd='+cmd+'&val='+val);}";
    html += "function sendSpeed(v){fetch('/control?cmd=SP&val='+v+'&_='+Date.now());}";
    html += "function updateSpeed(force){";
    html += "var speed = document.getElementById('speed').value;";
    html += "document.getElementById('speedValue').innerHTML = speed;";
    html += "if(force){sendSpeed(speed);return;}";
    html += "if(speedTimer){clearTimeout(speedTimer);}";
    html += "speedTimer=setTimeout(function(){sendSpeed(speed);},80);}";
    html += "function updateParam(key, sliderId, valueId){";
    html += "var v = document.getElementById(sliderId).value;";
    html += "document.getElementById(valueId).innerHTML = v;";
    html += "if(lineParamTimer[key]){clearTimeout(lineParamTimer[key]);}";
    html += "lineParamTimer[key]=setTimeout(function(){fetch('/control?cmd=LFP&key='+key+'&val='+v);},120);}";
    html += "function setLineParamValue(key, value){";
    html += "var slider=document.getElementById('p_'+key);";
    html += "var label=document.getElementById('v_'+key);";
    html += "if(slider){slider.value=value;}";
    html += "if(label){label.innerHTML=value;}}";
    html += "function loadLineParams(){";
    html += "fetch('/lineParams').then(function(resp){return resp.json();}).then(function(data){";
    html += "for(var key in data){setLineParamValue(key, data[key]);}";
    html += "}).catch(function(){});} ";
    html += "function resetLineParams(){fetch('/control?cmd=LFR').then(function(){loadLineParams();});}";
    html += "window.addEventListener('load', loadLineParams);";
    html += "</script></body></html>";
    
    return html;
}
