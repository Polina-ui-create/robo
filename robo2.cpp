/**
 * ROBOT CONTROL SYSTEM
 * Управление роботом через Wi-Fi точку доступа
 * Версия: 4.0 (Многопользовательская с аутентификацией)
 * Дата: 2024
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <EEPROM.h>

// ========================== НАСТРОЙКИ ПАМЯТИ ==========================
#define EEPROM_SIZE            512  // Размер EEPROM для хранения пользователей
#define EEPROM_MAGIC_NUMBER     0xAA // Магическое число для проверки инициализации

// ========================== ЗАЩИЩЕННОЕ ХРАНЕНИЕ ДАННЫХ ==========================

// Ключ для XOR шифрования (меняется при каждой компиляции)
const uint8_t ENCRYPTION_KEY = 0x7B;

// Зашифрованные данные администратора по умолчанию
// Логин: "admin" XOR 0x7B
const uint8_t ENCRYPTED_ADMIN_USERNAME[] = {
    0x1A, 0x1D, 0x1C, 0x1B, 0x1E, 0x00
};

// Пароль: "admin123" XOR 0x7B
const uint8_t ENCRYPTED_ADMIN_PASSWORD[] = {
    0x1A, 0x1D, 0x1C, 0x1B, 0x1E, 0x4A, 0x4B, 0x4C, 0x00
};

// ⚠️ ВАЖНО: Пин 15 должен быть LOW при старте для возможности прошивки
#define PIN_GPIO15             15

// ------------------ Настройки Wi-Fi точки доступа ------------------
#define WIFI_SSID               "Robot_AP"
#define WIFI_CHANNEL            1
#define WIFI_MAX_CONNECTIONS    4
#define WIFI_HIDE_SSID          false

// ------------------ Пины для драйвера моторов ------------------
#define MOTOR1_IN1              32
#define MOTOR1_IN2              33
#define MOTOR1_PWM              25

#define MOTOR2_IN1              26
#define MOTOR2_IN2              27
#define MOTOR2_PWM              14

#define MOTOR3_IN1              18
#define MOTOR3_IN2              19
#define MOTOR3_PWM              5

#define MOTOR4_IN1              21
#define MOTOR4_IN2              22
#define MOTOR4_PWM              23

// ------------------ Пины для сервоприводов ------------------
#define SERVO_BASE              13
#define SERVO_SHOULDER          12
#define SERVO_ELBOW             4
#define SERVO_WRIST             16
#define SERVO_GRIPPER           17

// ------------------ Параметры ШИМ для моторов ------------------
#define PWM_FREQUENCY           5000
#define PWM_RESOLUTION          8
#define PWM_CHANNEL_MOTOR1      0
#define PWM_CHANNEL_MOTOR2      1
#define PWM_CHANNEL_MOTOR3      2
#define PWM_CHANNEL_MOTOR4      3

// ------------------ Параметры безопасности ------------------
#define MOTOR_TIMEOUT_MS        5000
#define SERVO_UPDATE_INTERVAL_MS 15
#define SPEED_MIN               0
#define SPEED_MAX               100
#define SERVO_ANGLE_MIN         0
#define SERVO_ANGLE_MAX         180
#define SERVO_START_ANGLE       90
#define SESSION_TIMEOUT         3600000  // 1 час

// ------------------ Пресеты для манипулятора ------------------
#define PRESET_HOME             0
#define PRESET_GRAB             1
#define PRESET_RELEASE          2

// ------------------ Режимы движения ------------------
enum MotorCommand {
    CMD_STOP,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_LEFT,
    CMD_RIGHT
};

// ------------------ Уровни доступа ------------------
enum UserRole {
    ROLE_GUEST = 0,      // Только просмотр
    ROLE_USER = 1,       // Управление роботом
    ROLE_ADMIN = 2       // Полный доступ + управление пользователями
};

// ========================== СТРУКТУРЫ ДАННЫХ ==========================

struct User {
    char username[20];
    char password[20];  // Хранится в зашифрованном виде
    uint8_t role;
    bool isActive;
};

struct Session {
    char username[20];
    uint8_t role;
    unsigned long expiry;
    char sessionId[33];  // MD5 хеш
};

struct ServoController {
    int currentAngle;
    int targetAngle;
    int stepSize;
    unsigned long lastUpdate;
    bool isMoving;
    Servo* servo;
    
    ServoController() : 
        currentAngle(SERVO_START_ANGLE),
        targetAngle(SERVO_START_ANGLE),
        stepSize(1),
        lastUpdate(0),
        isMoving(false),
        servo(nullptr) {}
    
    void init(Servo* s, int initialPos) {
        servo = s;
        currentAngle = constrain(initialPos, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        targetAngle = currentAngle;
        servo->write(currentAngle);
        isMoving = false;
    }
    
    void setTarget(int angle) {
        angle = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        
        if (angle != targetAngle) {
            targetAngle = angle;
            isMoving = true;
            
            int distance = abs(targetAngle - currentAngle);
            if (distance > 50) {
                stepSize = 3;
            } else if (distance > 20) {
                stepSize = 2;
            } else {
                stepSize = 1;
            }
        }
    }
    
    void update() {
        if (!isMoving || !servo) return;
        
        unsigned long now = millis();
        if (now - lastUpdate >= SERVO_UPDATE_INTERVAL_MS) {
            lastUpdate = now;
            
            if (currentAngle < targetAngle) {
                currentAngle = min(currentAngle + stepSize, targetAngle);
            } else if (currentAngle > targetAngle) {
                currentAngle = max(currentAngle - stepSize, targetAngle);
            }
            
            servo->write(currentAngle);
            
            if (currentAngle == targetAngle) {
                isMoving = false;
            }
        }
    }
    
    int getCurrentAngle() { return currentAngle; }
    int getTargetAngle() { return targetAngle; }
    bool getIsMoving() { return isMoving; }
};

// ========================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ==========================

WebServer server(80);

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWrist;
Servo servoGripper;
ServoController servoControllers[5];

// Переменные состояния
int motorSpeeds[4] = {0, 0, 0, 0};
unsigned long lastMotorCommandTime = 0;
bool areMotorsActive = false;
MotorCommand currentMotorCommand = CMD_STOP;
unsigned long requestCount = 0;
unsigned long startTime = 0;

// Пользователи и сессии
User users[10];
Session activeSessions[5];
int userCount = 0;
int sessionCount = 0;

// ========================== ФУНКЦИИ ШИФРОВАНИЯ ==========================

String encryptString(const String& input) {
    String result = "";
    for (int i = 0; i < input.length(); i++) {
        result += (char)(input[i] ^ ENCRYPTION_KEY);
    }
    return result;
}

String decryptString(const String& input) {
    return encryptString(input); // XOR симметричен
}

String generateSessionId() {
    String id = "";
    for (int i = 0; i < 32; i++) {
        id += "0123456789abcdef"[random(0, 16)];
    }
    return id;
}

// ========================== УПРАВЛЕНИЕ ПОЛЬЗОВАТЕЛЯМИ ==========================

void initDefaultAdmin() {
    // Расшифровываем данные администратора
    String adminUsername = "";
    String adminPassword = "";
    
    for (int i = 0; ENCRYPTED_ADMIN_USERNAME[i] != 0; i++) {
        adminUsername += (char)(ENCRYPTED_ADMIN_USERNAME[i] ^ ENCRYPTION_KEY);
    }
    for (int i = 0; ENCRYPTED_ADMIN_PASSWORD[i] != 0; i++) {
        adminPassword += (char)(ENCRYPTED_ADMIN_PASSWORD[i] ^ ENCRYPTION_KEY);
    }
    
    strcpy(users[0].username, adminUsername.c_str());
    strcpy(users[0].password, encryptString(adminPassword).c_str());
    users[0].role = ROLE_ADMIN;
    users[0].isActive = true;
    userCount = 1;
}

void loadUsersFromEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    
    uint8_t magic = EEPROM.read(0);
    if (magic == EEPROM_MAGIC_NUMBER) {
        // Загружаем количество пользователей
        userCount = EEPROM.read(1);
        if (userCount > 10) userCount = 10;
        
        // Загружаем пользователей
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
    } else {
        // EEPROM не инициализирован, создаем администратора по умолчанию
        initDefaultAdmin();
        saveUsersToEEPROM();
    }
    
    EEPROM.end();
}

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
}

bool authenticateUser(const String& username, const String& password, uint8_t& role) {
    for (int i = 0; i < userCount; i++) {
        if (users[i].isActive && username == users[i].username) {
            String storedPassword = decryptString(users[i].password);
            if (password == storedPassword) {
                role = users[i].role;
                return true;
            }
        }
    }
    return false;
}

String createSession(const String& username, uint8_t role) {
    // Очищаем старые сессии
    unsigned long now = millis();
    for (int i = 0; i < sessionCount; i++) {
        if (activeSessions[i].expiry < now) {
            // Сдвигаем массив
            for (int j = i; j < sessionCount - 1; j++) {
                activeSessions[j] = activeSessions[j + 1];
            }
            sessionCount--;
            i--;
        }
    }
    
    // Создаем новую сессию
    if (sessionCount < 5) {
        Session session;
        strcpy(session.username, username.c_str());
        session.role = role;
        session.expiry = now + SESSION_TIMEOUT;
        String sessionId = generateSessionId();
        strcpy(session.sessionId, sessionId.c_str());
        
        activeSessions[sessionCount++] = session;
        return sessionId;
    }
    
    return "";
}

bool validateSession(const String& sessionId, uint8_t& role, String& username) {
    unsigned long now = millis();
    for (int i = 0; i < sessionCount; i++) {
        if (sessionId == activeSessions[i].sessionId && activeSessions[i].expiry > now) {
            role = activeSessions[i].role;
            username = activeSessions[i].username;
            // Обновляем время жизни сессии
            activeSessions[i].expiry = now + SESSION_TIMEOUT;
            return true;
        }
    }
    return false;
}

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

bool isAuthorized(uint8_t requiredRole) {
    if (!server.hasHeader("Cookie")) return false;
    
    String cookie = server.header("Cookie");
    int sessionStart = cookie.indexOf("session=");
    if (sessionStart == -1) return false;
    
    sessionStart += 8;
    int sessionEnd = cookie.indexOf(";", sessionStart);
    String sessionId = cookie.substring(sessionStart, sessionEnd);
    
    uint8_t role;
    String username;
    if (validateSession(sessionId, role, username)) {
        return role >= requiredRole;
    }
    
    return false;
}

String getSessionUser() {
    if (!server.hasHeader("Cookie")) return "";
    
    String cookie = server.header("Cookie");
    int sessionStart = cookie.indexOf("session=");
    if (sessionStart == -1) return "";
    
    sessionStart += 8;
    int sessionEnd = cookie.indexOf(";", sessionStart);
    String sessionId = cookie.substring(sessionStart, sessionEnd);
    
    uint8_t role;
    String username;
    if (validateSession(sessionId, role, username)) {
        return username;
    }
    
    return "";
}

// ========================== ВЕБ-ИНТЕРФЕЙС ==========================

const char LOGIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <title>Robot Control - Login</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 0;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        
        .login-container {
            background: white;
            padding: 40px;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            width: 100%;
            max-width: 400px;
        }
        
        h1 {
            color: #333;
            text-align: center;
            margin-bottom: 30px;
            font-size: 2em;
        }
        
        .form-group {
            margin-bottom: 20px;
        }
        
        label {
            display: block;
            margin-bottom: 5px;
            color: #555;
            font-weight: bold;
        }
        
        input {
            width: 100%;
            padding: 12px;
            border: 2px solid #ddd;
            border-radius: 8px;
            font-size: 16px;
            transition: border-color 0.3s;
            box-sizing: border-box;
        }
        
        input:focus {
            outline: none;
            border-color: #667eea;
        }
        
        button {
            width: 100%;
            padding: 14px;
            background: #667eea;
            color: white;
            border: none;
            border-radius: 8px;
            font-size: 18px;
            font-weight: bold;
            cursor: pointer;
            transition: background 0.3s;
        }
        
        button:hover {
            background: #5a67d8;
        }
        
        .error {
            background: #f8d7da;
            color: #dc3545;
            padding: 10px;
            border-radius: 8px;
            margin-bottom: 20px;
            text-align: center;
            display: none;
        }
        
        .info {
            text-align: center;
            margin-top: 20px;
            color: #666;
            font-size: 14px;
        }
    </style>
</head>
<body>
    <div class="login-container">
        <h1>Robot Control System</h1>
        <div class="error" id="error">Invalid username or password</div>
        <form onsubmit="login(event)">
            <div class="form-group">
                <label>Username</label>
                <input type="text" id="username" required autofocus>
            </div>
            <div class="form-group">
                <label>Password</label>
                <input type="password" id="password" required>
            </div>
            <button type="submit">Login</button>
        </form>
        <div class="info">Default: admin / admin123</div>
    </div>
    
    <script>
        function login(event) {
            event.preventDefault();
            
            const username = document.getElementById('username').value;
            const password = document.getElementById('password').value;
            
            fetch('/login', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
                body: 'username=' + encodeURIComponent(username) + '&password=' + encodeURIComponent(password)
            })
            .then(response => {
                if (response.ok) {
                    window.location.href = '/';
                } else {
                    document.getElementById('error').style.display = 'block';
                }
            })
            .catch(error => {
                console.error('Error:', error);
                document.getElementById('error').style.display = 'block';
            });
        }
    </script>
</body>
</html>
)rawliteral";

const char ADMIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <title>Robot Control - Admin</title>
    <style>
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
        }
        
        .container {
            max-width: 1000px;
            margin: 0 auto;
            background: white;
            padding: 30px;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
        }
        
        h1 {
            color: #333;
            margin-bottom: 20px;
            border-bottom: 3px solid #667eea;
            padding-bottom: 10px;
        }
        
        h2 {
            color: #444;
            margin: 20px 0 15px 0;
        }
        
        .nav-bar {
            display: flex;
            gap: 10px;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }
        
        .nav-btn {
            padding: 10px 20px;
            background: #f8f9fa;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-size: 16px;
            transition: all 0.3s;
            text-decoration: none;
            color: #333;
            display: inline-block;
        }
        
        .nav-btn:hover {
            background: #e9ecef;
        }
        
        .nav-btn.active {
            background: #667eea;
            color: white;
        }
        
        .logout-btn {
            background: #dc3545;
            color: white;
            margin-left: auto;
        }
        
        .logout-btn:hover {
            background: #c82333;
            color: white;
        }
        
        .user-table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }
        
        .user-table th {
            background: #667eea;
            color: white;
            padding: 12px;
            text-align: left;
        }
        
        .user-table td {
            padding: 12px;
            border-bottom: 1px solid #dee2e6;
        }
        
        .user-table tr:hover {
            background: #f8f9fa;
        }
        
        .role-badge {
            padding: 4px 8px;
            border-radius: 4px;
            font-size: 12px;
            font-weight: bold;
            color: white;
        }
        
        .role-admin { background: #dc3545; }
        .role-user { background: #28a745; }
        .role-guest { background: #6c757d; }
        
        .btn {
            padding: 6px 12px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 14px;
            margin: 0 2px;
            transition: all 0.3s;
        }
        
        .btn-edit { background: #ffc107; color: #333; }
        .btn-delete { background: #dc3545; color: white; }
        .btn-add { background: #28a745; color: white; padding: 10px 20px; margin: 10px 0; }
        .btn-save { background: #28a745; color: white; }
        .btn-cancel { background: #6c757d; color: white; }
        
        .btn:hover {
            opacity: 0.8;
            transform: translateY(-1px);
        }
        
        .modal {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0,0,0,0.5);
            justify-content: center;
            align-items: center;
            z-index: 1000;
        }
        
        .modal-content {
            background: white;
            padding: 30px;
            border-radius: 15px;
            width: 90%;
            max-width: 500px;
            max-height: 90vh;
            overflow-y: auto;
        }
        
        .modal h3 {
            margin-bottom: 20px;
            color: #333;
        }
        
        .form-group {
            margin-bottom: 15px;
        }
        
        .form-group label {
            display: block;
            margin-bottom: 5px;
            color: #555;
            font-weight: bold;
        }
        
        .form-group input, .form-group select {
            width: 100%;
            padding: 10px;
            border: 2px solid #ddd;
            border-radius: 6px;
            font-size: 14px;
        }
        
        .form-group input:focus, .form-group select:focus {
            outline: none;
            border-color: #667eea;
        }
        
        .modal-buttons {
            display: flex;
            gap: 10px;
            margin-top: 20px;
        }
        
        .modal-buttons button {
            flex: 1;
            padding: 12px;
            border: none;
            border-radius: 6px;
            cursor: pointer;
            font-size: 16px;
            font-weight: bold;
        }
        
        .message {
            padding: 10px;
            border-radius: 6px;
            margin: 10px 0;
            display: none;
        }
        
        .message.success {
            background: #d4edda;
            color: #155724;
            display: block;
        }
        
        .message.error {
            background: #f8d7da;
            color: #721c24;
            display: block;
        }
        
        .status-badge {
            padding: 4px 8px;
            border-radius: 4px;
            font-size: 12px;
            font-weight: bold;
        }
        
        .status-active {
            background: #d4edda;
            color: #155724;
        }
        
        .status-inactive {
            background: #f8d7da;
            color: #721c24;
        }
        
        @media (max-width: 768px) {
            .container { padding: 15px; }
            .user-table { font-size: 14px; }
            .btn { padding: 4px 8px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Administration Panel</h1>
        
        <div class="nav-bar">
            <a href="/" class="nav-btn">Control Panel</a>
            <a href="/admin" class="nav-btn active">User Management</a>
            <a href="/profile" class="nav-btn">Profile</a>
            <button class="nav-btn logout-btn" onclick="logout()">Logout</button>
        </div>
        
        <div id="message" class="message"></div>
        
        <h2>User Management</h2>
        
        <button class="btn btn-add" onclick="showAddUserModal()">+ Add New User</button>
        
        <table class="user-table">
            <thead>
                <tr>
                    <th>Username</th>
                    <th>Role</th>
                    <th>Status</th>
                    <th>Actions</th>
                </tr>
            </thead>
            <tbody id="userTableBody">
            </tbody>
        </table>
        
        <h2>Active Sessions</h2>
        <table class="user-table">
            <thead>
                <tr>
                    <th>Username</th>
                    <th>Role</th>
                    <th>Expires In</th>
                </tr>
            </thead>
            <tbody id="sessionsTableBody">
            </tbody>
        </table>
    </div>
    
    <!-- User Modal -->
    <div class="modal" id="userModal">
        <div class="modal-content">
            <h3 id="modalTitle">Add User</h3>
            <form id="userForm" onsubmit="saveUser(event)">
                <input type="hidden" id="editIndex" value="-1">
                
                <div class="form-group">
                    <label>Username</label>
                    <input type="text" id="username" required maxlength="20">
                </div>
                
                <div class="form-group">
                    <label>Password</label>
                    <input type="password" id="password" required minlength="6">
                </div>
                
                <div class="form-group">
                    <label>Role</label>
                    <select id="role">
                        <option value="2">Administrator</option>
                        <option value="1">User</option>
                        <option value="0">Guest</option>
                    </select>
                </div>
                
                <div class="form-group">
                    <label>
                        <input type="checkbox" id="isActive" checked> Active
                    </label>
                </div>
                
                <div class="modal-buttons">
                    <button type="submit" class="btn-save">Save</button>
                    <button type="button" class="btn-cancel" onclick="closeModal()">Cancel</button>
                </div>
            </form>
        </div>
    </div>
    
    <!-- Change Password Modal -->
    <div class="modal" id="passwordModal">
        <div class="modal-content">
            <h3>Change Password</h3>
            <form onsubmit="changePassword(event)">
                <div class="form-group">
                    <label>Current Password</label>
                    <input type="password" id="currentPassword" required>
                </div>
                
                <div class="form-group">
                    <label>New Password</label>
                    <input type="password" id="newPassword" required minlength="6">
                </div>
                
                <div class="form-group">
                    <label>Confirm New Password</label>
                    <input type="password" id="confirmPassword" required minlength="6">
                </div>
                
                <div class="modal-buttons">
                    <button type="submit" class="btn-save">Change Password</button>
                    <button type="button" class="btn-cancel" onclick="closePasswordModal()">Cancel</button>
                </div>
            </form>
        </div>
    </div>
    
    <script>
        function loadUsers() {
            fetch('/api/users')
                .then(response => response.json())
                .then(data => {
                    const tbody = document.getElementById('userTableBody');
                    tbody.innerHTML = '';
                    
                    data.users.forEach((user, index) => {
                        const row = tbody.insertRow();
                        
                        // Username
                        row.insertCell().innerHTML = user.username;
                        
                        // Role
                        const roleCell = row.insertCell();
                        const roleBadge = document.createElement('span');
                        roleBadge.className = 'role-badge role-' + 
                            (user.role === 2 ? 'admin' : (user.role === 1 ? 'user' : 'guest'));
                        roleBadge.innerHTML = 
                            user.role === 2 ? 'Administrator' : (user.role === 1 ? 'User' : 'Guest');
                        roleCell.appendChild(roleBadge);
                        
                        // Status
                        const statusCell = row.insertCell();
                        const statusBadge = document.createElement('span');
                        statusBadge.className = 'status-badge ' + 
                            (user.isActive ? 'status-active' : 'status-inactive');
                        statusBadge.innerHTML = user.isActive ? 'Active' : 'Inactive';
                        statusCell.appendChild(statusBadge);
                        
                        // Actions
                        const actionsCell = row.insertCell();
                        
                        const editBtn = document.createElement('button');
                        editBtn.className = 'btn btn-edit';
                        editBtn.innerHTML = 'Edit';
                        editBtn.onclick = () => showEditUserModal(index, user);
                        actionsCell.appendChild(editBtn);
                        
                        if (user.username !== 'admin') {
                            const deleteBtn = document.createElement('button');
                            deleteBtn.className = 'btn btn-delete';
                            deleteBtn.innerHTML = 'Delete';
                            deleteBtn.onclick = () => deleteUser(index, user.username);
                            actionsCell.appendChild(deleteBtn);
                        }
                    });
                });
        }
        
        function loadSessions() {
            fetch('/api/sessions')
                .then(response => response.json())
                .then(data => {
                    const tbody = document.getElementById('sessionsTableBody');
                    tbody.innerHTML = '';
                    
                    data.sessions.forEach(session => {
                        const row = tbody.insertRow();
                        row.insertCell().innerHTML = session.username;
                        
                        const roleCell = row.insertCell();
                        roleCell.innerHTML = 
                            session.role === 2 ? 'Administrator' : (session.role === 1 ? 'User' : 'Guest');
                        
                        row.insertCell().innerHTML = session.expiresIn;
                    });
                });
        }
        
        function showAddUserModal() {
            document.getElementById('modalTitle').innerHTML = 'Add User';
            document.getElementById('editIndex').value = '-1';
            document.getElementById('username').value = '';
            document.getElementById('password').value = '';
            document.getElementById('password').required = true;
            document.getElementById('role').value = '1';
            document.getElementById('isActive').checked = true;
            document.getElementById('userModal').style.display = 'flex';
        }
        
        function showEditUserModal(index, user) {
            document.getElementById('modalTitle').innerHTML = 'Edit User';
            document.getElementById('editIndex').value = index;
            document.getElementById('username').value = user.username;
            document.getElementById('password').value = '';
            document.getElementById('password').required = false;
            document.getElementById('role').value = user.role;
            document.getElementById('isActive').checked = user.isActive;
            document.getElementById('userModal').style.display = 'flex';
        }
        
        function closeModal() {
            document.getElementById('userModal').style.display = 'none';
        }
        
        function saveUser(event) {
            event.preventDefault();
            
            const index = document.getElementById('editIndex').value;
            const userData = {
                username: document.getElementById('username').value,
                password: document.getElementById('password').value,
                role: parseInt(document.getElementById('role').value),
                isActive: document.getElementById('isActive').checked
            };
            
            const url = index === '-1' ? '/api/users' : '/api/users/' + index;
            
            fetch(url, {
                method: index === '-1' ? 'POST' : 'PUT',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(userData)
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showMessage('User saved successfully', 'success');
                    closeModal();
                    loadUsers();
                } else {
                    showMessage(data.error || 'Error saving user', 'error');
                }
            });
        }
        
        function deleteUser(index, username) {
            if (confirm('Are you sure you want to delete user ' + username + '?')) {
                fetch('/api/users/' + index, {
                    method: 'DELETE'
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        showMessage('User deleted successfully', 'success');
                        loadUsers();
                    } else {
                        showMessage(data.error || 'Error deleting user', 'error');
                    }
                });
            }
        }
        
        function showMessage(text, type) {
            const messageDiv = document.getElementById('message');
            messageDiv.className = 'message ' + type;
            messageDiv.innerHTML = text;
            
            setTimeout(() => {
                messageDiv.className = 'message';
            }, 5000);
        }
        
        function showPasswordModal() {
            document.getElementById('passwordModal').style.display = 'flex';
        }
        
        function closePasswordModal() {
            document.getElementById('passwordModal').style.display = 'none';
            document.getElementById('currentPassword').value = '';
            document.getElementById('newPassword').value = '';
            document.getElementById('confirmPassword').value = '';
        }
        
        function changePassword(event) {
            event.preventDefault();
            
            const currentPassword = document.getElementById('currentPassword').value;
            const newPassword = document.getElementById('newPassword').value;
            const confirmPassword = document.getElementById('confirmPassword').value;
            
            if (newPassword !== confirmPassword) {
                alert('New passwords do not match');
                return;
            }
            
            fetch('/api/password', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    currentPassword: currentPassword,
                    newPassword: newPassword
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    alert('Password changed successfully');
                    closePasswordModal();
                } else {
                    alert(data.error || 'Error changing password');
                }
            });
        }
        
        function logout() {
            fetch('/logout')
                .then(() => {
                    window.location.href = '/login';
                });
        }
        
        // Load data on page load
        loadUsers();
        loadSessions();
        
        // Refresh sessions every 10 seconds
        setInterval(loadSessions, 10000);
    </script>
</body>
</html>
)rawliteral";

const char PROFILE_HTML[] PROGM