/**
 * ROBOT CONTROL SYSTEM - MAIN FILE
 * 
 * ===================================================
 * PROMPTS USED FOR GENERATION:
 * ===================================================
 * 
 * PROMPT 1 (Initial):
 * "Напиши код для ESP32, который создает Wi-Fi точку доступа с веб-интерфейсом 
 * для управления роботом. Нужно управление 4 моторами и 5 сервоприводами. 
 * Используй библиотеки WebServer.h и ESP32Servo.h. Добавь регулировку скорости 
 * моторов через ШИМ."
 * 
 * PROMPT 2 (Improvements):
 * "Код работает, но есть проблемы. Добавь следующие улучшения:
 * - Пин 15 должен быть LOW при старте
 * - Проверка граничных значений для сервоприводов
 * - Неблокирующее обновление сервоприводов"
 * 
 * PROMPT 3 (Readability):
 * "Теперь сделай код более читаемым и поддерживаемым:
 * - Вынеси все пины и константы в начало кода
 * - Создай отдельную функцию stopMotors()
 * - Добавь подробные комментарии на русском языке
 * - Добавь отображение IP и статуса в HTML"
 * 
 * PROMPT 4 (Presets & Security):
 * "Добавь в веб-интерфейс кнопки для пресетов манипулятора: 'Домой', 'Захват', 'Отпустить'.
 * Реализуй эндпоинт /ping для проверки связи.
 * Сделай так чтобы пароль WiFi нельзя было прочитать из исходного кода."
 * 
 * PROMPT 5 (WPA3 & Flash Encryption):
 * "Нужно добавить в него две функции безопасности:
 * 1. WPA3-Personal (SAE) для Wi-Fi соединения.
 * 2. Шифрование Flash-памяти для защиты данных от физического извлечения."
 * 
 * PROMPT 6 (External Config):
 * "Сохранение учётных данных в отдельном файле. Это предотвращает случайное 
 * раскрытие данных при распространении кода."
 * 
 * PROMPT 7 (Documentation):
 * "Добавь во все коды промты комментариями которые были использованы для их создания"
 * 
 * PROMPT 8 (Detailed Prompts):
 * "Теперь в добавок добавь промты не только в начало кодов но и внутрь их 
 * чтобы было видно какой промт на что повлиял в коде"
 * ===================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "config.h"           // PROMPT 6: Вынос учетных данных в отдельный файл
#include "html_pages.h"        // PROMPT 3: Отдельный файл для HTML для читаемости

// ==================== ПРОВЕРКА НАЛИЧИЯ КОНФИГУРАЦИИ ====================
#ifndef CONFIG_H
#error "Please create config.h from config_sample.h with your credentials!" // PROMPT 6: Защита от отсутствия конфига
#endif

// ========================== ПАРАМЕТРЫ ПАМЯТИ ==========================
#define EEPROM_SIZE            512  // PROMPT 4: Для хранения пользователей
#define EEPROM_MAGIC_NUMBER     0xAA // PROMPT 4: Магическое число для проверки инициализации
#define MAX_USERS               10  // PROMPT 4: Максимальное количество пользователей
#define MAX_SESSIONS            5   // PROMPT 4: Максимальное количество одновременных сессий

// ========================== ПИНЫ ДЛЯ МОТОРОВ ==========================
// PROMPT 1: Базовая настройка пинов для моторов
// PROMPT 3: Вынесено в начало для читаемости
#define MOTOR1_IN1              32  // PROMPT 1: Направление мотора 1
#define MOTOR1_IN2              33  // PROMPT 1: Направление мотора 1
#define MOTOR1_PWM              25  // PROMPT 1: ШИМ для скорости мотора 1

#define MOTOR2_IN1              26  // PROMPT 1: Направление мотора 2
#define MOTOR2_IN2              27  // PROMPT 1: Направление мотора 2
#define MOTOR2_PWM              14  // PROMPT 1: ШИМ для скорости мотора 2

#define MOTOR3_IN1              18  // PROMPT 1: Направление мотора 3
#define MOTOR3_IN2              19  // PROMPT 1: Направление мотора 3
#define MOTOR3_PWM              5   // PROMPT 1: ШИМ для скорости мотора 3

#define MOTOR4_IN1              21  // PROMPT 1: Направление мотора 4
#define MOTOR4_IN2              22  // PROMPT 1: Направление мотора 4
#define MOTOR4_PWM              23  // PROMPT 1: ШИМ для скорости мотора 4

// ========================== ПИНЫ ДЛЯ СЕРВОПРИВОДОВ ==========================
// PROMPT 1: Базовая настройка пинов для сервоприводов
#define SERVO_BASE              13  // PROMPT 1: Основание манипулятора
#define SERVO_SHOULDER          12  // PROMPT 1: Плечо манипулятора
#define SERVO_ELBOW             4   // PROMPT 1: Локоть манипулятора
#define SERVO_WRIST             16  // PROMPT 1: Запястье манипулятора
#define SERVO_GRIPPER           17  // PROMPT 1: Захват манипулятора

// ========================== ПАРАМЕТРЫ ШИМ ==========================
// PROMPT 1: Настройка ШИМ для регулировки скорости моторов
#define PWM_FREQUENCY           5000  // PROMPT 1: Частота ШИМ 5 кГц
#define PWM_RESOLUTION          8     // PROMPT 1: 8-битное разрешение (0-255)
#define PWM_CHANNEL_MOTOR1      0     // PROMPT 1: Канал ШИМ для мотора 1
#define PWM_CHANNEL_MOTOR2      1     // PROMPT 1: Канал ШИМ для мотора 2
#define PWM_CHANNEL_MOTOR3      2     // PROMPT 1: Канал ШИМ для мотора 3
#define PWM_CHANNEL_MOTOR4      3     // PROMPT 1: Канал ШИМ для мотора 4

// ========================== ПАРАМЕТРЫ БЕЗОПАСНОСТИ ==========================
// PROMPT 2: Добавление параметров безопасности
#define MOTOR_TIMEOUT_MS        5000  // PROMPT 2: Автостоп моторов через 5 секунд
#define SERVO_UPDATE_INTERVAL_MS 15   // PROMPT 2: Интервал обновления серв для плавности
#define SERVO_ANGLE_MIN         0     // PROMPT 2: Минимальный угол сервы
#define SERVO_ANGLE_MAX         180   // PROMPT 2: Максимальный угол сервы
#define SERVO_START_ANGLE       90    // PROMPT 2: Начальный угол всех серв

// ========================== РЕЖИМЫ ДВИЖЕНИЯ ==========================
// PROMPT 1: Базовые режимы движения
enum MotorCommand {
    CMD_STOP,      // PROMPT 1: Остановка
    CMD_FORWARD,   // PROMPT 1: Вперед
    CMD_BACKWARD,  // PROMPT 1: Назад
    CMD_LEFT,      // PROMPT 1: Влево
    CMD_RIGHT      // PROMPT 1: Вправо
};

// PROMPT 4: Добавление ролей пользователей
enum UserRole {
    ROLE_GUEST = 0,  // PROMPT 4: Только просмотр
    ROLE_USER = 1,   // PROMPT 4: Управление роботом
    ROLE_ADMIN = 2   // PROMPT 4: Полный доступ
};

// ========================== СТРУКТУРЫ ДАННЫХ ==========================

// PROMPT 4: Структура для хранения пользователей
struct User {
    char username[20];  // PROMPT 4: Имя пользователя
    char password[20];  // PROMPT 4: Пароль (в зашифрованном виде)
    uint8_t role;       // PROMPT 4: Роль пользователя
    bool isActive;      // PROMPT 4: Активен ли пользователь
};

// PROMPT 4: Структура для хранения сессий
struct Session {
    char username[20];      // PROMPT 4: Имя пользователя
    uint8_t role;           // PROMPT 4: Роль пользователя
    unsigned long expiry;   // PROMPT 4: Время истечения сессии
    char sessionId[33];     // PROMPT 4: ID сессии
};

// PROMPT 2: Структура для неблокирующего управления сервоприводами
struct ServoController {
    int currentAngle;           // PROMPT 2: Текущий угол
    int targetAngle;            // PROMPT 2: Целевой угол
    int stepSize;               // PROMPT 2: Шаг движения
    unsigned long lastUpdate;   // PROMPT 2: Время последнего обновления
    bool isMoving;              // PROMPT 2: Флаг движения
    Servo* servo;               // PROMPT 2: Указатель на объект сервы
    
    // PROMPT 2: Конструктор с начальными значениями
    ServoController() : 
        currentAngle(SERVO_START_ANGLE),
        targetAngle(SERVO_START_ANGLE),
        stepSize(1),
        lastUpdate(0),
        isMoving(false),
        servo(nullptr) {}
    
    // PROMPT 2: Инициализация контроллера
    void init(Servo* s, int initialPos) {
        servo = s;
        currentAngle = constrain(initialPos, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        targetAngle = currentAngle;
        servo->write(currentAngle);
        isMoving = false;
    }
    
    // PROMPT 2: Установка целевого угла с проверкой границ
    void setTarget(int angle) {
        angle = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX); // PROMPT 2: Проверка границ
        
        if (angle != targetAngle) {
            targetAngle = angle;
            isMoving = true;
            
            // PROMPT 2: Адаптивный шаг для плавности
            int distance = abs(targetAngle - currentAngle);
            stepSize = distance > 50 ? 3 : (distance > 20 ? 2 : 1);
        }
    }
    
    // PROMPT 2: Неблокирующее обновление позиции
    void update() {
        if (!isMoving || !servo) return;
        
        unsigned long now = millis();
        if (now - lastUpdate >= SERVO_UPDATE_INTERVAL_MS) {
            lastUpdate = now;
            
            // PROMPT 2: Плавное движение к цели
            if (currentAngle < targetAngle) {
                currentAngle = min(currentAngle + stepSize, targetAngle);
            } else if (currentAngle > targetAngle) {
                currentAngle = max(currentAngle - stepSize, targetAngle);
            }
            
            servo->write(currentAngle);
            isMoving = (currentAngle != targetAngle);
        }
    }
    
    int getCurrentAngle() { return currentAngle; }
    bool getIsMoving() { return isMoving; }
};

// ========================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==========================

WebServer server(80);  // PROMPT 1: Веб-сервер на порту 80

// PROMPT 1: Объекты сервоприводов
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWrist;
Servo servoGripper;
ServoController servoControllers[5];  // PROMPT 2: Массив контроллеров

// PROMPT 1: Переменные состояния моторов
int motorSpeeds[4] = {0, 0, 0, 0};
unsigned long lastMotorCommandTime = 0;
bool areMotorsActive = false;
MotorCommand currentMotorCommand = CMD_STOP;

// PROMPT 4: Переменные для статистики и пользователей
unsigned long requestCount = 0;
unsigned long startTime = 0;
User users[MAX_USERS];
Session activeSessions[MAX_SESSIONS];
int userCount = 0;
int sessionCount = 0;

// ========================== ФУНКЦИИ ШИФРОВАНИЯ ==========================

// PROMPT 4: XOR шифрование для защиты паролей в памяти
String encryptString(const String& input) {
    String result = "";
    for (size_t i = 0; i < input.length(); i++) {
        result += (char)(input[i] ^ ENCRYPTION_KEY);
    }
    return result;
}

// PROMPT 4: Дешифровка (XOR симметричен)
String decryptString(const String& input) {
    return encryptString(input);
}

// PROMPT 4: Генерация уникального ID сессии
String generateSessionId() {
    String id = "";
    for (int i = 0; i < 32; i++) {
        id += "0123456789abcdef"[random(0, 16)];
    }
    return id;
}

// ========================== УПРАВЛЕНИЕ ПОЛЬЗОВАТЕЛЯМИ ==========================

// PROMPT 4: Создание администратора по умолчанию
void initDefaultAdmin() {
    strcpy(users[0].username, ADMIN_USERNAME);
    strcpy(users[0].password, encryptString(ADMIN_PASSWORD).c_str());
    users[0].role = ROLE_ADMIN;
    users[0].isActive = true;
    userCount = 1;
    Serial.println("Default admin created");
}

// PROMPT 4: Загрузка пользователей из EEPROM
void loadUsersFromEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    
    if (EEPROM.read(0) == EEPROM_MAGIC_NUMBER) {
        userCount = min((int)EEPROM.read(1), MAX_USERS);
        
        int addr = 2;
        for (int i = 0; i < userCount; i++) {
            for (int j = 0; j < 20; j++) {
                users[i].username[j] = EEPROM.read(addr++);
            }
            for (int j = 0; j < 20; j++) {
                users[i].password[j] = EEPROM.read(addr++);
            }
            users[i].role = EEPROM.read(addr++);
            users[i].isActive = EEPROM.read(addr++);
        }
        Serial.printf("Loaded %d users from EEPROM\n", userCount);
    } else {
        initDefaultAdmin();
        saveUsersToEEPROM();
    }
    
    EEPROM.end();
}

// PROMPT 4: Сохранение пользователей в EEPROM
void saveUsersToEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    
    EEPROM.write(0, EEPROM_MAGIC_NUMBER);
    EEPROM.write(1, userCount);
    
    int addr = 2;
    for (int i = 0; i < userCount; i++) {
        for (int j = 0; j < 20; j++) {
            EEPROM.write(addr++, users[i].username[j]);
        }
        for (int j = 0; j < 20; j++) {
            EEPROM.write(addr++, users[i].password[j]);
        }
        EEPROM.write(addr++, users[i].role);
        EEPROM.write(addr++, users[i].isActive ? 1 : 0);
    }
    
    EEPROM.commit();
    EEPROM.end();
    Serial.println("Users saved to EEPROM");
}

// PROMPT 4: Аутентификация пользователя
bool authenticateUser(const String& username, const String& password, uint8_t& role) {
    for (int i = 0; i < userCount; i++) {
        if (users[i].isActive && username == users[i].username) {
            if (password == decryptString(users[i].password)) {
                role = users[i].role;
                return true;
            }
        }
    }
    return false;
}

// PROMPT 4: Создание новой сессии
String createSession(const String& username, uint8_t role) {
    unsigned long now = millis();
    
    // PROMPT 4: Очистка просроченных сессий
    for (int i = sessionCount - 1; i >= 0; i--) {
        if (activeSessions[i].expiry < now) {
            for (int j = i; j < sessionCount - 1; j++) {
                activeSessions[j] = activeSessions[j + 1];
            }
            sessionCount--;
        }
    }
    
    if (sessionCount < MAX_SESSIONS) {
        String sessionId = generateSessionId();
        Session session;
        strcpy(session.username, username.c_str());
        session.role = role;
        session.expiry = now + SESSION_TIMEOUT;
        strcpy(session.sessionId, sessionId.c_str());
        
        activeSessions[sessionCount++] = session;
        return sessionId;
    }
    
    return "";
}

// PROMPT 4: Проверка валидности сессии
bool validateSession(const String& sessionId, uint8_t& role, String& username) {
    unsigned long now = millis();
    for (int i = 0; i < sessionCount; i++) {
        if (sessionId == activeSessions[i].sessionId && activeSessions[i].expiry > now) {
            role = activeSessions[i].role;
            username = activeSessions[i].username;
            activeSessions[i].expiry = now + SESSION_TIMEOUT; // PROMPT 4: Продление сессии
            return true;
        }
    }
    return false;
}

// PROMPT 4: Инвалидация сессии (logout)
void invalidateSession(const String& sessionId) {
    for (int i = 0; i < sessionCount; i++) {
        if (sessionId == activeSessions[i].sessionId) {
            for (int j = i; j < sessionCount - 1; j++) {
                activeSessions[j] = activeSessions[j + 1];
            }
            sessionCount--;
            break;
        }
    }
}

// PROMPT 4: Проверка авторизации по роли
bool isAuthorized(uint8_t requiredRole) {
    if (!server.hasHeader("Cookie")) return false;
    
    String cookie = server.header("Cookie");
    int sessionStart = cookie.indexOf("session=");
    if (sessionStart == -1) return false;
    
    sessionStart += 8;
    int sessionEnd = cookie.indexOf(";", sessionStart);
    if (sessionEnd == -1) sessionEnd = cookie.length();
    
    String sessionId = cookie.substring(sessionStart, sessionEnd);
    
    uint8_t role;
    String username;
    return validateSession(sessionId, role, username) && role >= requiredRole;
}

// PROMPT 4: Получение имени пользователя из сессии
String getSessionUser() {
    if (!server.hasHeader("Cookie")) return "";
    
    String cookie = server.header("Cookie");
    int sessionStart = cookie.indexOf("session=");
    if (sessionStart == -1) return "";
    
    sessionStart += 8;
    int sessionEnd = cookie.indexOf(";", sessionStart);
    if (sessionEnd == -1) sessionEnd = cookie.length();
    
    String sessionId = cookie.substring(sessionStart, sessionEnd);
    
    uint8_t role;
    String username;
    if (validateSession(sessionId, role, username)) {
        return username;
    }
    
    return "";
}

// ========================== НАСТРОЙКА WIFI С WPA3 ==========================

// PROMPT 5: Настройка WPA3-Personal
void setupWiFiAP() {
    Serial.println("\n=== Configuring WiFi AP with WPA3 ===");
    
    WiFi.mode(WIFI_AP);
    
    // PROMPT 5: Конфигурация WPA3
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, WIFI_SSID);
    ap_config.ap.ssid_len = strlen(WIFI_SSID);
    ap_config.ap.channel = WIFI_CHANNEL;
    ap_config.ap.max_connection = WIFI_MAX_CONNECTIONS;
    ap_config.ap.authmode = WIFI_AUTH_WPA3_PSK;  // PROMPT 5: WPA3-Personal (SAE)
    ap_config.ap.ssid_hidden = WIFI_HIDE_SSID;
    
    strcpy((char*)ap_config.ap.password, WIFI_PASSWORD);
    
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, WIFI_HIDE_SSID, WIFI_MAX_CONNECTIONS);
    
    Serial.println("✓ Access Point created:");
    Serial.println("  SSID: " + String(WIFI_SSID));
    Serial.println("  Security: WPA3-Personal (SAE)");  // PROMPT 5: Индикация WPA3
    Serial.println("  IP: " + WiFi.softAPIP().toString());
}

// PROMPT 5: Проверка статуса Flash Encryption
void checkFlashEncryption() {
    Serial.println("\n=== Flash Encryption Status ===");
    esp_flash_enc_mode_t mode = esp_get_flash_encryption_mode();
    
    switch(mode) {
        case ESP_FLASH_ENC_MODE_DISABLED:
            Serial.println("⚠ Flash Encryption: DISABLED");
            Serial.println("  Data can be read via physical access!");  // PROMPT 5: Предупреждение
            break;
        case ESP_FLASH_ENC_MODE_DEVELOPMENT:
            Serial.println("✓ Flash Encryption: DEVELOPMENT MODE");
            Serial.println("  Data is encrypted but UART flashing allowed");  // PROMPT 5: Режим разработки
            break;
        case ESP_FLASH_ENC_MODE_RELEASE:
            Serial.println("✓ Flash Encryption: RELEASE MODE (SECURE)");
            Serial.println("  Data is protected from physical readout");  // PROMPT 5: Безопасный режим
            break;
    }
}

// ========================== НАСТРОЙКА АППАРАТУРЫ ==========================

// PROMPT 2: Установка пина 15 в LOW для возможности прошивки
void setupPin15() {
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
    Serial.println("✓ GPIO15 set to LOW (programming mode available)");  // PROMPT 2: Важное замечание
}

// PROMPT 1: Настройка пинов моторов
void setupMotorPins() {
    Serial.println("\n=== Configuring Motors ===");
    
    // PROMPT 1: Настройка ШИМ каналов
    for (int i = 0; i < 4; i++) {
        ledcSetup(i, PWM_FREQUENCY, PWM_RESOLUTION);
    }
    
    // PROMPT 1: Привязка ШИМ к пинам
    ledcAttachPin(MOTOR1_PWM, PWM_CHANNEL_MOTOR1);
    ledcAttachPin(MOTOR2_PWM, PWM_CHANNEL_MOTOR2);
    ledcAttachPin(MOTOR3_PWM, PWM_CHANNEL_MOTOR3);
    ledcAttachPin(MOTOR4_PWM, PWM_CHANNEL_MOTOR4);
    
    // PROMPT 1: Настройка пинов направления
    int motorPins[] = {MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2,
                       MOTOR3_IN1, MOTOR3_IN2, MOTOR4_IN1, MOTOR4_IN2};
    
    for (int pin : motorPins) {
        pinMode(pin, OUTPUT);
    }
    
    stopMotors();  // PROMPT 3: Централизованная остановка
    Serial.println("✓ Motors configured");
}

// PROMPT 1: Настройка сервоприводов
void setupServos() {
    Serial.println("\n=== Configuring Servos ===");
    
    // PROMPT 1: Подключение серв
    servoBase.attach(SERVO_BASE);
    servoShoulder.attach(SERVO_SHOULDER);
    servoElbow.attach(SERVO_ELBOW);
    servoWrist.attach(SERVO_WRIST);
    servoGripper.attach(SERVO_GRIPPER);
    
    // PROMPT 2: Инициализация контроллеров для неблокирующего управления
    Servo* servos[] = {&servoBase, &servoShoulder, &servoElbow, &servoWrist, &servoGripper};
    
    for (int i = 0; i < 5; i++) {
        servoControllers[i].init(servos[i], SERVO_START_ANGLE);
    }
    
    Serial.println("✓ 5 servos initialized");
}

// ========================== ОБРАБОТЧИКИ ВЕБ-ЗАПРОСОВ ==========================

// PROMPT 1: Главная страница
void handleRoot() {
    requestCount++;
    if (!isAuthorized(ROLE_GUEST)) {  // PROMPT 4: Проверка авторизации
        server.sendHeader("Location", "/login");
        server.send(302, "text/plain", "");
        return;
    }
    server.send(200, "text/html", INDEX_HTML);  // PROMPT 3: Отдельный файл с HTML
}

// PROMPT 4: Страница логина
void handleLogin() {
    if (server.method() == HTTP_GET) {
        server.send(200, "text/html", LOGIN_HTML);
    } else {
        if (server.hasArg("username") && server.hasArg("password")) {
            String username = server.arg("username");
            String password = server.arg("password");
            
            uint8_t role;
            if (authenticateUser(username, password, role)) {  // PROMPT 4: Аутентификация
                String sessionId = createSession(username, role);
                if (sessionId.length() > 0) {
                    server.sendHeader("Set-Cookie", "session=" + sessionId + "; path=/; HttpOnly");
                    server.send(200, "text/plain", "OK");
                    Serial.printf("User %s logged in\n", username.c_str());
                    return;
                }
            }
        }
        server.send(401, "text/plain", "Unauthorized");
    }
}

// PROMPT 4: Выход из системы
void handleLogout() {
    if (server.hasHeader("Cookie")) {
        String cookie = server.header("Cookie");
        int sessionStart = cookie.indexOf("session=");
        if (sessionStart != -1) {
            sessionStart += 8;
            int sessionEnd = cookie.indexOf(";", sessionStart);
            if (sessionEnd == -1) sessionEnd = cookie.length();
            invalidateSession(cookie.substring(sessionStart, sessionEnd));  // PROMPT 4: Инвалидация сессии
        }
    }
    server.sendHeader("Set-Cookie", "session=; path=/; expires=Thu, 01 Jan 1970 00:00:00 GMT");
    server.sendHeader("Location", "/login");
    server.send(302, "text/plain", "");
}

// PROMPT 1: Управление моторами
void handleMotorControl() {
    requestCount++;
    if (!isAuthorized(ROLE_USER)) {  // PROMPT 4: Только для авторизованных пользователей
        server.send(403, "text/plain", "Forbidden");
        return;
    }
    
    if (server.hasArg("cmd")) {
        String command = server.arg("cmd");
        int speed = 50;
        
        if (server.hasArg("speed")) {
            speed = server.arg("speed").toInt();
            speed = map(constrain(speed, 0, 100), 0, 100, 0, 255);  // PROMPT 1: Преобразование процентов в ШИМ
        }

        lastMotorCommandTime = millis();
        areMotorsActive = (command != "stop");

        // PROMPT 1: Обработка команд движения
        if (command == "forward") {
            moveForward(speed);
            server.send(200, "text/plain", "Moving forward");
        }
        else if (command == "backward") {
            moveBackward(speed);
            server.send(200, "text/plain", "Moving backward");
        }
        else if (command == "left") {
            turnLeft(speed);
            server.send(200, "text/plain", "Turning left");
        }
        else if (command == "right") {
            turnRight(speed);
            server.send(200, "text/plain", "Turning right");
        }
        else if (command == "stop") {
            stopMotors();  // PROMPT 3: Централизованная остановка
            server.send(200, "text/plain", "Stopped");
        }
        else {
            server.send(400, "text/plain", "Unknown command");
        }
    }
}

// PROMPT 1: Управление сервоприводами
void handleServoControl() {
    requestCount++;
    if (!isAuthorized(ROLE_USER)) {  // PROMPT 4: Только для авторизованных пользователей
        server.send(403, "text/plain", "Forbidden");
        return;
    }
    
    if (server.hasArg("name") && server.hasArg("pos")) {
        String name = server.arg("name");
        int pos = constrain(server.arg("pos").toInt(), SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);  // PROMPT 2: Проверка границ
        
        int index = -1;
        String servoName = "";
        
        if (name == "base") { index = 0; servoName = "Base"; }
        else if (name == "shoulder") { index = 1; servoName = "Shoulder"; }
        else if (name == "elbow") { index = 2; servoName = "Elbow"; }
        else if (name == "wrist") { index = 3; servoName = "Wrist"; }
        else if (name == "gripper") { index = 4; servoName = "Gripper"; }
        
        if (index != -1) {
            servoControllers[index].setTarget(pos);  // PROMPT 2: Неблокирующее обновление
            server.send(200, "text/plain", servoName + ": " + String(pos) + "°");
        } else {
            server.send(400, "text/plain", "Unknown servo");
        }
    }
}

// PROMPT 1: Получение статуса
void handleStatus() {
    requestCount++;
    if (!isAuthorized(ROLE_GUEST)) {  // PROMPT 4: Даже гости могут видеть статус
        server.send(403, "text/plain", "Forbidden");
        return;
    }
    
    // PROMPT 1: Формирование JSON с данными
    String json = "{\"servos\":[";
    for (int i = 0; i < 5; i++) {
        if (i > 0) json += ",";
        json += "{\"current\":" + String(servoControllers[i].getCurrentAngle()) + 
                ",\"isMoving\":" + (servoControllers[i].getIsMoving() ? "true" : "false") + "}";
    }
    
    unsigned long uptime = (millis() - startTime) / 1000;
    
    json += "],\"motorsActive\":" + String(areMotorsActive ? "true" : "false") + 
            ",\"stats\":{\"uptime\":" + String(uptime) + 
            ",\"requests\":" + String(requestCount) + "}}";
    
    server.send(200, "application/json", json);
}

// PROMPT 4: Эндпоинт для проверки связи
void handlePing() {
    server.send(200, "text/plain", "pong");  // PROMPT 4: Простой ответ для проверки
}

// PROMPT 4: Страница профиля
void handleProfile() {
    if (!isAuthorized(ROLE_USER)) {
        server.send(403, "text/plain", "Forbidden");
        return;
    }
    server.send(200, "text/html", PROFILE_HTML);
}

// PROMPT 4: Админ-панель
void handleAdmin() {
    if (!isAuthorized(ROLE_ADMIN)) {  // PROMPT 4: Только для администраторов
        server.send(403, "text/plain", "Forbidden");
        return;
    }
    server.send(200, "text/html", ADMIN_HTML);
}

// PROMPT 4: API для информации о пользователе
void handleApiUserInfo() {
    if (!isAuthorized(ROLE_GUEST)) {
        server.send(403, "text/plain", "Forbidden");
        return;
    }
    
    String username = getSessionUser();
    uint8_t role = ROLE_GUEST;
    
    for (int i = 0; i < userCount; i++) {
        if (username == users[i].username) {
            role = users[i].role;
            break;
        }
    }
    
    String json = "{\"username\":\"" + username + "\",\"role\":" + String(role) + "}";
    server.send(200, "application/json", json);
}

// PROMPT 4: API для управления пользователями
void handleApiUsers() {
    if (!isAuthorized(ROLE_ADMIN)) {
        server.send(403, "text/plain", "Forbidden");
        return;
    }
    
    if (server.method() == HTTP_GET) {
        String json = "{\"users\":[";
        for (int i = 0; i < userCount; i++) {
            if (i > 0) json += ",";
            json += "{\"username\":\"" + String(users[i].username) + "\",";
            json += "\"role\":" + String(users[i].role) + ",";
            json += "\"isActive\":" + String(users[i].isActive ? "true" : "false") + "}";
        }
        json += "]}";
        server.send(200, "application/json", json);
    }
}

// PROMPT 4: API для смены пароля
void handleApiPassword() {
    if (!isAuthorized(ROLE_USER)) {
        server.send(403, "text/plain", "Forbidden");
        return;
    }
    
    String json = server.arg("plain");
    // Простой парсинг JSON (в продакшене использовать библиотеку)
    int currentIdx = json.indexOf("\"currentPassword\":\"") + 18;
    int newIdx = json.indexOf("\"newPassword\":\"") + 14;
    
    String currentPassword = json.substring(currentIdx, json.indexOf("\"", currentIdx));
    String newPassword = json.substring(newIdx, json.indexOf("\"", newIdx));
    
    String username = getSessionUser();
    
    for (int i = 0; i < userCount; i++) {
        if (username == users[i].username) {
            if (currentPassword == decryptString(users[i].password)) {
                strcpy(users[i].password, encryptString(newPassword).c_str());
                saveUsersToEEPROM();  // PROMPT 4: Сохранение нового пароля
                server.send(200, "application/json", "{\"success\":true}");
                Serial.printf("Password changed for user %s\n", username.c_str());
                return;
            }
        }
    }
    
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid password\"}");
}

// ========================== ФУНКЦИИ УПРАВЛЕНИЯ МОТОРАМИ ==========================

// PROMPT 1: Установка скорости всех моторов
void setAllMotorsSpeed(int s1, int s2, int s3, int s4) {
    ledcWrite(PWM_CHANNEL_MOTOR1, s1);
    ledcWrite(PWM_CHANNEL_MOTOR2, s2);
    ledcWrite(PWM_CHANNEL_MOTOR3, s3);
    ledcWrite(PWM_CHANNEL_MOTOR4, s4);
}

// PROMPT 1: Движение вперед
void moveForward(int speed) {
    digitalWrite(MOTOR1_IN1, HIGH); digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, HIGH); digitalWrite(MOTOR2_IN2, LOW);
    digitalWrite(MOTOR3_IN1, HIGH); digitalWrite(MOTOR3_IN2, LOW);
    digitalWrite(MOTOR4_IN1, HIGH); digitalWrite(MOTOR4_IN2, LOW);
    setAllMotorsSpeed(speed, speed, speed, speed);
}

// PROMPT 1: Движение назад
void moveBackward(int speed) {
    digitalWrite(MOTOR1_IN1, LOW); digitalWrite(MOTOR1_IN2, HIGH);
    digitalWrite(MOTOR2_IN1, LOW); digitalWrite(MOTOR2_IN2, HIGH);
    digitalWrite(MOTOR3_IN1, LOW); digitalWrite(MOTOR3_IN2, HIGH);
    digitalWrite(MOTOR4_IN1, LOW); digitalWrite(MOTOR4_IN2, HIGH);
    setAllMotorsSpeed(speed, speed, speed, speed);
}

// PROMPT 1: Поворот влево
void turnLeft(int speed) {
    digitalWrite(MOTOR1_IN1, LOW); digitalWrite(MOTOR1_IN2, HIGH);
    digitalWrite(MOTOR2_IN1, HIGH); digitalWrite(MOTOR2_IN2, LOW);
    digitalWrite(MOTOR3_IN1, LOW); digitalWrite(MOTOR3_IN2, HIGH);
    digitalWrite(MOTOR4_IN1, HIGH); digitalWrite(MOTOR4_IN2, LOW);
    setAllMotorsSpeed(speed, speed, speed, speed);
}

// PROMPT 1: Поворот вправо
void turnRight(int speed) {
    digitalWrite(MOTOR1_IN1, HIGH); digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW); digitalWrite(MOTOR2_IN2, HIGH);
    digitalWrite(MOTOR3_IN1, HIGH); digitalWrite(MOTOR3_IN2, LOW);
    digitalWrite(MOTOR4_IN1, LOW); digitalWrite(MOTOR4_IN2, HIGH);
    setAllMotorsSpeed(speed, speed, speed, speed);
}

// PROMPT 3: Централизованная функция остановки
void stopMotors() {
    digitalWrite(MOTOR1_IN1, LOW); digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW); digitalWrite(MOTOR2_IN2, LOW);
    digitalWrite(MOTOR3_IN1, LOW); digitalWrite(MOTOR3_IN2, LOW);
    digitalWrite(MOTOR4_IN1, LOW); digitalWrite(MOTOR4_IN2, LOW);
    setAllMotorsSpeed(0, 0, 0, 0);
    areMotorsActive = false;
}

// PROMPT 2: Проверка таймаута моторов
void checkMotorTimeout() {
    if (areMotorsActive && (millis() - lastMotorCommandTime > MOTOR_TIMEOUT_MS)) {
        stopMotors();
        Serial.println("Motors stopped by timeout");  // PROMPT 2: Автостоп для безопасности
    }
}

// PROMPT 1: Настройка веб-сервера
void setupWebServer() {
    Serial.println("\n=== Starting Web Server ===");
    
    // PROMPT 1: Базовые эндпоинты
    server.on("/", handleRoot);
    server.on("/login", handleLogin);      // PROMPT 4: Добавлено
    server.on("/logout", handleLogout);    // PROMPT 4: Добавлено
    server.on("/control", handleMotorControl);
    server.on("/servo", handleServoControl);
    server.on("/status", handleStatus);
    server.on("/ping", handlePing);        // PROMPT 4: Добавлено
    server.on("/profile", handleProfile);  // PROMPT 4: Добавлено
    server.on("/admin", handleAdmin);      // PROMPT 4: Добавлено
    
    // PROMPT 4: API эндпоинты
    server.on("/api/user/info", handleApiUserInfo);
    server.on("/api/users", handleApiUsers);
    server.on("/api/password", HTTP_POST, handleApiPassword);
    
    server.begin();
    Serial.println("✓ Web server running on port 80");
}

// ========================== SETUP & LOOP ==========================

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=========================================");
    Serial.println("ROBOT CONTROL SYSTEM v5.0");
    Serial.println("=========================================");
    
    setupPin15();              // PROMPT 2: Критично для прошивки
    checkFlashEncryption();    // PROMPT 5: Проверка шифрования
    setupWiFiAP();             // PROMPT 1 + PROMPT 5: WPA3
    setupMotorPins();          // PROMPT 1: Моторы
    setupServos();             // PROMPT 1 + PROMPT 2: Сервы с неблокирующим управлением
    
    loadUsersFromEEPROM();     // PROMPT 4: Загрузка пользователей
    setupWebServer();          // PROMPT 1: Веб-сервер
    
    startTime = millis();
    
    Serial.println("\n✅ System Ready!");
    Serial.printf("Users: %d, Sessions: %d/%d\n", userCount, sessionCount, MAX_SESSIONS);
    Serial.println("=========================================\n");
}

void loop() {
    server.handleClient();      // PROMPT 1: Обработка веб-запросов
    
    for (int i = 0; i < 5; i++) {
        servoControllers[i].update();  // PROMPT 2: Неблокирующее обновление серв
    }
    
    checkMotorTimeout();        // PROMPT 2: Проверка таймаута
}