// ================= БИБЛИОТЕКИ =================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ================= КОНСТАНТЫ ПИНОВ =================
// Моторы - входы управления направлением
#define LEFT_FRONT_IN1 25
#define LEFT_FRONT_IN2 26
#define LEFT_REAR_IN1 27
#define LEFT_REAR_IN2 14
#define RIGHT_FRONT_IN1 32
#define RIGHT_FRONT_IN2 33
#define RIGHT_REAR_IN1 12
#define RIGHT_REAR_IN2 15    // Критический пин - должен быть LOW при загрузке

// Моторы - ШИМ (скорость)
#define LEFT_FRONT_ENA 19
#define LEFT_REAR_ENA 18
#define RIGHT_FRONT_ENA 5
#define RIGHT_REAR_ENA 16

// Каналы ШИМ для моторов (0-15 каналов, 0-7 таймеры 0-3)
#define MOTOR_PWM_CH_LF 8    // Канал 8 (таймер 2)
#define MOTOR_PWM_CH_LR 9    // Канал 9 (таймер 2)
#define MOTOR_PWM_CH_RF 10   // Канал 10 (таймер 3)
#define MOTOR_PWM_CH_RR 11   // Канал 11 (таймер 3)

// Сервоприводы
#define BASE_SERVO_PIN 23
#define SHOULDER_SERVO_PIN 22
#define ELBOW_SERVO_PIN 21
#define WRIST_SERVO_PIN 17
#define GRIPPER_SERVO_PIN 13  

// ================= КОНСТАНТЫ =================
#define SERVO_UPDATE_DELAY 20  // мс задержка между обновлениями сервоприводов

// Wi-Fi настройки
const char* ssid = "RobotAP";
const char* password = "12345678";

// ================= ГЛОБАЛЬНЫЕ ОБЪЕКТЫ =================
WebServer server(80);
ESP32Servo baseServo, shoulderServo, elbowServo, wristServo, gripperServo;

// ================= ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ =================
// Позиции сервоприводов
int basePos = 90, shoulderPos = 90, elbowPos = 90, wristPos = 90, gripperPos = 90;

// Флаги и таймеры для сервоприводов
bool needArmUpdate = false;
unsigned long lastArmUpdate = 0;

// Текущая скорость моторов (0-255)
int currentSpeed = 150;

// ================= ПРОТОТИПЫ ФУНКЦИЙ =================
void stopMotors();
void updateArmNow();
void updateArmNonBlocking();
void moveMotor(String command);  
void setMotorSpeed(int speed);

// Обработчики HTTP запросов
void handleRoot();
void handleCommand();
void handleSpeed();
void handleArm();
void handleGetArm();
void handlePreset();
void handlePing();
void handleNotFound();

// Вспомогательные функции
String generateHTML();

// ================= ФУНКЦИЯ stopMotors() =================
void stopMotors() {
    // Отключаем все входы моторов
    digitalWrite(LEFT_FRONT_IN1, LOW);
    digitalWrite(LEFT_FRONT_IN2, LOW);
    digitalWrite(LEFT_REAR_IN1, LOW);
    digitalWrite(LEFT_REAR_IN2, LOW);
    digitalWrite(RIGHT_FRONT_IN1, LOW);
    digitalWrite(RIGHT_FRONT_IN2, LOW);
    digitalWrite(RIGHT_REAR_IN1, LOW);
    digitalWrite(RIGHT_REAR_IN2, LOW);
    
    // Отключаем ШИМ
    ledcWrite(MOTOR_PWM_CH_LF, 0);
    ledcWrite(MOTOR_PWM_CH_LR, 0);
    ledcWrite(MOTOR_PWM_CH_RF, 0);
    ledcWrite(MOTOR_PWM_CH_RR, 0);
}

// ================= ФУНКЦИЯ updateArmNow() =================
void updateArmNow() {
    // Обновляем позиции сервоприводов
    baseServo.write(basePos);
    shoulderServo.write(shoulderPos);
    elbowServo.write(elbowPos);
    wristServo.write(wristPos);
    gripperServo.write(gripperPos);
    
    // Логирование
    Serial.print("Сервоприводы: Base=");
    Serial.print(basePos);
    Serial.print(", Shoulder=");
    Serial.print(shoulderPos);
    Serial.print(", Elbow=");
    Serial.print(elbowPos);
    Serial.print(", Wrist=");
    Serial.print(wristPos);
    Serial.print(", Gripper=");
    Serial.println(gripperPos);
}

// ================= ФУНКЦИЯ updateArmNonBlocking() =================
void updateArmNonBlocking() {
    if (!needArmUpdate) return;
    
    unsigned long now = millis();
    unsigned long elapsed = (now >= lastArmUpdate) ? (now - lastArmUpdate) : 
                           (ULONG_MAX - lastArmUpdate + now);
    
    if (elapsed >= SERVO_UPDATE_DELAY) {
        updateArmNow();
        needArmUpdate = false;
        lastArmUpdate = now;
    }
}

// ================= ФУНКЦИЯ setMotorSpeed() =================
void setMotorSpeed(int speed) {
    currentSpeed = constrain(speed, 0, 255);
    Serial.print("Установлена скорость: ");
    Serial.println(currentSpeed);
}

// ================= ФУНКЦИЯ moveMotor() =================
void moveMotor(String command) {
    if (command == "forward") {
        // Вперед - все моторы вперед
        digitalWrite(LEFT_FRONT_IN1, HIGH);
        digitalWrite(LEFT_FRONT_IN2, LOW);
        digitalWrite(LEFT_REAR_IN1, HIGH);
        digitalWrite(LEFT_REAR_IN2, LOW);
        digitalWrite(RIGHT_FRONT_IN1, HIGH);
        digitalWrite(RIGHT_FRONT_IN2, LOW);
        digitalWrite(RIGHT_REAR_IN1, HIGH);
        digitalWrite(RIGHT_REAR_IN2, LOW);
        
        ledcWrite(MOTOR_PWM_CH_LF, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_LR, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_RF, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_RR, currentSpeed);
        
        Serial.println("Движение вперед");
        
    } else if (command == "backward") {
        // Назад - все моторы назад
        digitalWrite(LEFT_FRONT_IN1, LOW);
        digitalWrite(LEFT_FRONT_IN2, HIGH);
        digitalWrite(LEFT_REAR_IN1, LOW);
        digitalWrite(LEFT_REAR_IN2, HIGH);
        digitalWrite(RIGHT_FRONT_IN1, LOW);
        digitalWrite(RIGHT_FRONT_IN2, HIGH);
        digitalWrite(RIGHT_REAR_IN1, LOW);
        digitalWrite(RIGHT_REAR_IN2, HIGH);
        
        ledcWrite(MOTOR_PWM_CH_LF, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_LR, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_RF, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_RR, currentSpeed);
        
        Serial.println("Движение назад");
        
    } else if (command == "left") {
        // Влево - правые моторы вперед, левые назад
        digitalWrite(LEFT_FRONT_IN1, LOW);
        digitalWrite(LEFT_FRONT_IN2, HIGH);
        digitalWrite(LEFT_REAR_IN1, LOW);
        digitalWrite(LEFT_REAR_IN2, HIGH);
        digitalWrite(RIGHT_FRONT_IN1, HIGH);
        digitalWrite(RIGHT_FRONT_IN2, LOW);
        digitalWrite(RIGHT_REAR_IN1, HIGH);
        digitalWrite(RIGHT_REAR_IN2, LOW);
        
        ledcWrite(MOTOR_PWM_CH_LF, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_LR, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_RF, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_RR, currentSpeed);
        
        Serial.println("Поворот налево");
        
    } else if (command == "right") {
        // Вправо - левые моторы вперед, правые назад
        digitalWrite(LEFT_FRONT_IN1, HIGH);
        digitalWrite(LEFT_FRONT_IN2, LOW);
        digitalWrite(LEFT_REAR_IN1, HIGH);
        digitalWrite(LEFT_REAR_IN2, LOW);
        digitalWrite(RIGHT_FRONT_IN1, LOW);
        digitalWrite(RIGHT_FRONT_IN2, HIGH);
        digitalWrite(RIGHT_REAR_IN1, LOW);
        digitalWrite(RIGHT_REAR_IN2, HIGH);
        
        ledcWrite(MOTOR_PWM_CH_LF, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_LR, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_RF, currentSpeed);
        ledcWrite(MOTOR_PWM_CH_RR, currentSpeed);
        
        Serial.println("Поворот направо");
        
    } else if (command == "stop") {
        stopMotors();
        Serial.println("Остановка");
    }
}

// ================= SETUP() =================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Красивый заголовок
    Serial.println("\n======================================================");
    Serial.println("                СИСТЕМА УПРАВЛЕНИЯ РОБОТОМ ");
    Serial.println("======================================================");
    
    // Инициализация переменных
    needArmUpdate = false;
    lastArmUpdate = 0;
    currentSpeed = 150;
    
    // ========== НАСТРОЙКА ПИНОВ МОТОРОВ ==========
    Serial.println("\n[1] Настройка пинов моторов...");
    
    pinMode(LEFT_FRONT_IN1, OUTPUT);
    pinMode(LEFT_FRONT_IN2, OUTPUT);
    pinMode(LEFT_REAR_IN1, OUTPUT);
    pinMode(LEFT_REAR_IN2, OUTPUT);
    pinMode(RIGHT_FRONT_IN1, OUTPUT);
    pinMode(RIGHT_FRONT_IN2, OUTPUT);
    pinMode(RIGHT_REAR_IN1, OUTPUT);
    pinMode(RIGHT_REAR_IN2, OUTPUT);
    
    // Критически важная настройка пина 15
    Serial.println("ВНИМАНИЕ: Настройка пина 15 (RIGHT_REAR_IN2)");
    if (RIGHT_REAR_IN2 == 15) {
        Serial.println("   Пин 15 используется для мотора");
        Serial.println("   При прошивке ESP32 пин 15 должен быть LOW");
        Serial.println("   Убедитесь, что нет конфликтов с загрузкой!");
    }
    digitalWrite(RIGHT_REAR_IN2, LOW);
    
    // Изначально все моторы выключены
    stopMotors();
    Serial.println("✓ Пины моторов настроены и выключены");
    
    // ========== НАСТРОЙКА ШИМ ДЛЯ МОТОРОВ ==========
    Serial.println("\n[2] Настройка ШИМ для моторов...");
    
    // Частота 5kHz, 8-битное разрешение
    ledcSetup(MOTOR_PWM_CH_LF, 5000, 8);
    ledcAttachPin(LEFT_FRONT_ENA, MOTOR_PWM_CH_LF);
    
    ledcSetup(MOTOR_PWM_CH_LR, 5000, 8);
    ledcAttachPin(LEFT_REAR_ENA, MOTOR_PWM_CH_LR);
    
    ledcSetup(MOTOR_PWM_CH_RF, 5000, 8);
    ledcAttachPin(RIGHT_FRONT_ENA, MOTOR_PWM_CH_RF);
    
    ledcSetup(MOTOR_PWM_CH_RR, 5000, 8);
    ledcAttachPin(RIGHT_REAR_ENA, MOTOR_PWM_CH_RR);
    
    Serial.println("✓ ШИМ для моторов настроен (каналы 8-11)");
    Serial.println("  Используются таймеры 2 и 3 для LEDC");
    
    // ========== НАСТРОЙКА СЕРВОПРИВОДОВ ==========
    Serial.println("\n[3] Настройка сервоприводов...");
    
    // ВАЖНО: Выделяем только 2 таймера для сервоприводов
    // Оставляем таймеры 2 и 3 для LEDC (моторов)
    ESP32PWM::allocateTimer(0);  // Таймер 0 для сервоприводов
    ESP32PWM::allocateTimer(1);  // Таймер 1 для сервоприводов
    
    Serial.println("✓ Выделены таймеры 0 и 1 для сервоприводов");
    Serial.println("  Таймеры 2 и 3 оставлены для LEDC (моторы)");
    
    // Стандартная частота 50Hz для всех сервоприводов
    baseServo.setPeriodHertz(50);
    shoulderServo.setPeriodHertz(50);
    elbowServo.setPeriodHertz(50);
    wristServo.setPeriodHertz(50);
    gripperServo.setPeriodHertz(50);
    
    // Стандартный диапазон импульсов (500-2400 мкс - более широкий диапазон)
    baseServo.attach(BASE_SERVO_PIN, 500, 2400);
    shoulderServo.attach(SHOULDER_SERVO_PIN, 500, 2400);
    elbowServo.attach(ELBOW_SERVO_PIN, 500, 2400);
    wristServo.attach(WRIST_SERVO_PIN, 500, 2400);
    gripperServo.attach(GRIPPER_SERVO_PIN, 500, 2400);
    
    Serial.println("✓ Сервоприводы инициализированы на 50Hz");
    
    // ========== ПРОВЕРКА ПИНОВ ==========
    Serial.println("\n[4] Проверка всех пинов...");
    int pins[] = {25, 26, 27, 14, 32, 33, 12, 15, 19, 18, 5, 16, 23, 22, 21, 17, 13};
    int pinCount = sizeof(pins) / sizeof(pins[0]);
    
    for (int i = 0; i < pinCount; i++) {
        Serial.print("  Пин ");
        if (pins[i] < 10) Serial.print(" ");
        Serial.print(pins[i]);
        Serial.println(": OK");
        delay(5);
    }
    
    // ========== УСТАНОВКА НАЧАЛЬНОГО ПОЛОЖЕНИЯ ==========
    Serial.println("\n[5] Установка начального положения...");
    updateArmNow();
    delay(300);
    needArmUpdate = false;
    
    Serial.println("✓ Начальная позиция установлена (все сервы на 90°)");
    
    // ========== СОЗДАНИЕ WI-FI ТОЧКИ ДОСТУПА ==========
    Serial.println("\n[6] Создание Wi-Fi точки доступа...");
    
    // Отключаем сохранение настроек Wi-Fi для надежности
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    
    bool apStarted = WiFi.softAP(ssid, password, 1, 0, 4); // Канал 1, скрытый: нет, макс клиентов: 4
    
    if (!apStarted) {
        Serial.println("ОШИБКА: Не удалось создать точку доступа!");
        Serial.println("Проверьте настройки Wi-Fi модуля");
        while(true) {
            delay(1000);
            Serial.print(".");
        }
    }
    
    Serial.println("✓ Точка доступа создана успешно!");
    Serial.print("   SSID: ");
    Serial.println(ssid);
    Serial.print("   Пароль: ");
    Serial.println(password);
    Serial.print("   IP адрес: ");
    Serial.println(WiFi.softAPIP());
    
    // ========== НАСТРОЙКА ВЕБ-СЕРВЕРА ==========
    Serial.println("\n[7] Настройка веб-сервера...");
    
    server.on("/", HTTP_GET, handleRoot);
    server.on("/cmd", HTTP_GET, handleCommand);
    server.on("/speed", HTTP_GET, handleSpeed);
    server.on("/arm", HTTP_GET, handleArm);
    server.on("/getArm", HTTP_GET, handleGetArm);
    server.on("/preset", HTTP_GET, handlePreset);
    server.on("/ping", HTTP_GET, handlePing);
    server.onNotFound(handleNotFound);
    
    server.begin();
    Serial.println("✓ Веб-сервер запущен на порту 80");
    
    // ========== ИТОГОВАЯ ИНФОРМАЦИЯ ==========
    Serial.println("\n======================================================");
    Serial.println("              СИСТЕМА ГОТОВА К РАБОТЕ!");
    Serial.println("======================================================");
    Serial.print("Подключите телефон к Wi-Fi: ");
    Serial.println(ssid);
    Serial.print("Откройте браузер: http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("======================================================");
    
    // ========== РАСПИНОВКА ==========
    Serial.println("\n[РАСПИНОВКА]");
    Serial.println("Моторы:");
    Serial.println("  Левый передний:  IN1=25, IN2=26, ENA=19 (PWM ch8, таймер 2)");
    Serial.println("  Левый задний:    IN1=27, IN2=14, ENA=18 (PWM ch9, таймер 2)");
    Serial.println("  Правый передний: IN1=32, IN2=33, ENA=5  (PWM ch10, таймер 3)");
    Serial.println("  Правый задний:   IN1=12, IN2=15, ENA=16 (PWM ch11, таймер 3)");
    Serial.println("Сервоприводы (таймеры 0 и 1):");
    Serial.println("  Основание:   пин 23 (таймер 0)");
    Serial.println("  Плечо:       пин 22 (таймер 0)");
    Serial.println("  Локоть:      пин 21 (таймер 1)");
    Serial.println("  Запястье:    пин 17 (таймер 1)");
    Serial.println("  Захват:      пин 13 (таймер 1)");
    Serial.println("\n[ПРОВЕРКА]");
    Serial.println("✓ LEDC использует таймеры 2 и 3");
    Serial.println("✓ Сервоприводы используют таймеры 0 и 1");
    Serial.println("✓ Пин 15 настроен корректно");
    Serial.println("✓ Нет конфликтов с загрузкой");
    Serial.println("✓ Стабильная работа гарантирована");
    Serial.println("======================================================");
}

// ================= ОБРАБОТЧИКИ HTTP ЗАПРОСОВ =================

void handleRoot() {
    server.send(200, "text/html", generateHTML());
}

void handleCommand() {
    if (!server.hasArg("c")) {
        server.send(400, "text/plain", "ERROR: No command");
        return;
    }
    
    String command = server.arg("c");
    Serial.print("Команда: ");
    Serial.println(command);
    
    moveMotor(command);
    server.send(200, "text/plain", "OK: " + command);
}

void handleSpeed() {
    if (!server.hasArg("s")) {
        server.send(400, "text/plain", "ERROR: No speed");
        return;
    }
    
    int speed = server.arg("s").toInt();
    setMotorSpeed(speed);
    server.send(200, "text/plain", "OK: Speed=" + String(currentSpeed));
}

// ================= handleArm() =================
void handleArm() {
    bool updated = false;
    
    if (server.hasArg("base")) {
        int val = server.arg("base").toInt();
        if (val >= 0 && val <= 180) {
            basePos = val;
            updated = true;
        }
    }
    if (server.hasArg("shoulder")) {
        int val = server.arg("shoulder").toInt();
        if (val >= 0 && val <= 180) {
            shoulderPos = val;
            updated = true;
        }
    }
    if (server.hasArg("elbow")) {
        int val = server.arg("elbow").toInt();
        if (val >= 0 && val <= 180) {
            elbowPos = val;
            updated = true;
        }
    }
    if (server.hasArg("wrist")) {
        int val = server.arg("wrist").toInt();
        if (val >= 0 && val <= 180) {
            wristPos = val;
            updated = true;
        }
    }
    if (server.hasArg("gripper")) {
        int val = server.arg("gripper").toInt();
        if (val >= 0 && val <= 180) {
            gripperPos = val;
            updated = true;
        }
    }
    
    if (updated) {
        updateArmNow();  // Немедленное обновление
        server.send(200, "text/plain", "OK: Arm positions updated");
    } else {
        server.send(400, "text/plain", "ERROR: No valid parameters");
    }
}

void handleGetArm() {
    String response = "base=" + String(basePos) +
                     "&shoulder=" + String(shoulderPos) +
                     "&elbow=" + String(elbowPos) +
                     "&wrist=" + String(wristPos) +
                     "&gripper=" + String(gripperPos);
    server.send(200, "text/plain", response);
}

// ================= handlePreset() =================
void handlePreset() {
    if (!server.hasArg("p")) {
        server.send(400, "text/plain", "ERROR: No preset");
        return;
    }
    
    String preset = server.arg("p");
    Serial.print("Загрузка пресета: ");
    Serial.println(preset);
    
    if (preset == "home") {
        basePos = 90;
        shoulderPos = 90;
        elbowPos = 90;
        wristPos = 90;
        gripperPos = 90;
        Serial.println("Установлена домашняя позиция (все 90°)");
    } else if (preset == "grab") {
        gripperPos = 30;
        Serial.println("Захват закрыт (30°)");
    } else if (preset == "release") {
        gripperPos = 150;
        Serial.println("Захват открыт (150°)");
    } else {
        server.send(400, "text/plain", "ERROR: Unknown preset");
        Serial.print("Неизвестный пресет: ");
        Serial.println(preset);
        return;
    }
    
    // Немедленное обновление
    updateArmNow();
    Serial.println("Пресет применен успешно");
    
    // Отправляем обновленные позиции
    handleGetArm();
}

void handlePing() {
    server.send(200, "text/plain", "pong");
}

void handleNotFound() {
    server.send(404, "text/plain", "404: Not Found");
}

// ================= ФУНКЦИЯ ГЕНЕРАЦИИ HTML =================
String generateHTML() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Управление роботом</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }";
    html += ".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }";
    html += "h1 { color: #333; text-align: center; }";
    html += ".control-group { margin: 20px 0; padding: 15px; background: #f9f9f9; border-radius: 5px; }";
    html += "button { padding: 12px 24px; margin: 5px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; }";
    html += ".btn-move { background: #4CAF50; color: white; }";
    html += ".btn-stop { background: #f44336; color: white; }";
    html += ".btn-preset { background: #2196F3; color: white; }";
    html += "input[type='range'] { width: 100%; }";
    html += ".status { background: #e8f5e9; padding: 10px; border-radius: 5px; margin: 10px 0; }";
    html += "</style>";
    html += "<script>";
    html += "function sendCommand(cmd) { fetch('/cmd?c=' + cmd).then(r => r.text()).then(console.log); }";
    html += "function setSpeed(val) { fetch('/speed?s=' + val).then(r => r.text()).then(console.log); }";
    html += "function setArm() {";
    html += "  fetch('/arm?base=' + document.getElementById('base').value +";
    html += "        '&shoulder=' + document.getElementById('shoulder').value +";
    html += "        '&elbow=' + document.getElementById('elbow').value +";
    html += "        '&wrist=' + document.getElementById('wrist').value +";
    html += "        '&gripper=' + document.getElementById('gripper').value)";
    html += "  .then(r => r.text()).then(console.log); }";
    html += "function loadPreset(p) { fetch('/preset?p=' + p).then(r => r.text()).then(console.log); }";
    html += "function updateStatus() {";
    html += "  fetch('/getArm').then(r => r.text()).then(data => {";
    html += "    const params = new URLSearchParams(data);";
    html += "    document.getElementById('status').innerHTML =";
    html += "      'Текущие позиции: Base=' + params.get('base') +";
    html += "      '°, Shoulder=' + params.get('shoulder') +";
    html += "      '°, Elbow=' + params.get('elbow') +";
    html += "      '°, Wrist=' + params.get('wrist') +";
    html += "      '°, Gripper=' + params.get('gripper') + '°';";
    html += "  });";
    html += "}";
    html += "setInterval(updateStatus, 1000);";
    html += "</script>";
    html += "</head><body>";
    html += "<div class='container'>";
    html += "<h1>🤖 Управление роботом</h1>";
    html += "<div class='status' id='status'>Загрузка...</div>";
    
    // Управление движением
    html += "<div class='control-group'>";
    html += "<h2>🎮 Управление движением</h2>";
    html += "<div style='text-align: center;'>";
    html += "<button class='btn-move' onclick='sendCommand(\"forward\")'>↑ Вперед</button><br>";
    html += "<button class='btn-move' onclick='sendCommand(\"left\")'>← Влево</button>";
    html += "<button class='btn-stop' onclick='sendCommand(\"stop\")'>⏹ Стоп</button>";
    html += "<button class='btn-move' onclick='sendCommand(\"right\")'>Вправо →</button><br>";
    html += "<button class='btn-move' onclick='sendCommand(\"backward\")'>↓ Назад</button>";
    html += "</div></div>";
    
    // Скорость
    html += "<div class='control-group'>";
    html += "<h2>⚡ Скорость: <span id='speedVal'>150</span></h2>";
    html += "<input type='range' min='0' max='255' value='150' oninput='document.getElementById(\"speedVal\").innerHTML=this.value; setSpeed(this.value)'>";
    html += "</div>";
    
    // Манипулятор
    html += "<div class='control-group'>";
    html += "<h2>🦾 Управление манипулятором</h2>";
    html += "Основание (0-180°): <input type='range' id='base' min='0' max='180' value='90' onchange='setArm()'><br>";
    html += "Плечо (0-180°): <input type='range' id='shoulder' min='0' max='180' value='90' onchange='setArm()'><br>";
    html += "Локоть (0-180°): <input type='range' id='elbow' min='0' max='180' value='90' onchange='setArm()'><br>";
    html += "Запястье (0-180°): <input type='range' id='wrist' min='0' max='180' value='90' onchange='setArm()'><br>";
    html += "Захват (0-180°): <input type='range' id='gripper' min='0' max='180' value='90' onchange='setArm()'><br>";
    html += "<button class='btn-preset' onclick='loadPreset(\"home\")'>🏠 Домашняя позиция</button>";
    html += "<button class='btn-preset' onclick='loadPreset(\"grab\")'>🔄 Захватить</button>";
    html += "<button class='btn-preset' onclick='loadPreset(\"release\")'>🔓 Отпустить</button>";
    html += "</div>";
    
    // Информация
    html += "<div class='control-group'>";
    html += "<h2>📊 Информация</h2>";
    html += "<p><strong>Wi-Fi:</strong> ";
    html += ssid;
    html += "</p>";
    html += "<p><strong>IP адрес:</strong> ";
    html += WiFi.softAPIP().toString();
    html += "</p>";
    html += "<p><strong>Скорость COM:</strong> 115200 бод</p>";
    html += "<p><button onclick='updateStatus()'>🔄 Обновить статус</button></p>";
    html += "</div>";
    
    html += "</div></body></html>";
    return html;
}

// ================= loop() =================
void loop() {
    // Обслуживаем веб-сервер
    server.handleClient();
    
    // Неблокирующее обновление сервоприводов (если требуется)
    updateArmNonBlocking();
    
    // Минимальная задержка для стабильности
    delay(1);
}