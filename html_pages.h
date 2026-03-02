/**
 * HTML PAGES FOR ROBOT CONTROL INTERFACE
 * 
 * ===================================================
 * PROMPTS USED:
 * 
 * 1. "Напиши код для ESP32 с веб-интерфейсом для управления роботом"
 *    - Базовая структура HTML, кнопки управления
 * 
 * 2. "Добавь в веб-интерфейс кнопки для пресетов манипулятора"
 *    - Добавлены кнопки Home/Grab/Release
 * 
 * 3. "Добавь отображение текущего IP-адреса и статуса подключения"
 *    - Добавлен статус-бар с IP, индикаторами
 * 
 * 4. "Добавь для входа на сайт логин и пароль"
 *    - Создана страница логина, профиля, админ-панели
 * 
 * 5. "Добавь во все коды промты комментариями"
 *    - Детальные комментарии в HTML/JS
 * ===================================================
 */

#ifndef HTML_PAGES_H
#define HTML_PAGES_H

// ==================== LOGIN PAGE ====================
// PROMPT 4: Страница аутентификации
const char LOGIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Robot Control - Login</title>
    <style>
        /* PROMPT 4: Стили для страницы входа */
        body {
            font-family: 'Segoe UI', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            margin: 0;
        }
        .login-container {
            background: white;
            padding: 40px;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            width: 90%;
            max-width: 400px;
        }
        h1 { text-align: center; color: #333; margin-bottom: 30px; }
        .form-group { margin-bottom: 20px; }
        label { display: block; margin-bottom: 5px; color: #555; font-weight: bold; }
        input {
            width: 100%;
            padding: 12px;
            border: 2px solid #ddd;
            border-radius: 8px;
            font-size: 16px;
            box-sizing: border-box;
        }
        input:focus { outline: none; border-color: #667eea; }
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
        }
        button:hover { background: #5a67d8; }
        .error {
            background: #f8d7da;
            color: #dc3545;
            padding: 10px;
            border-radius: 8px;
            margin-bottom: 20px;
            display: none;
            text-align: center;
        }
    </style>
</head>
<body>
    <div class="login-container">
        <h1>Robot Control System</h1>
        <!-- PROMPT 4: Элемент для отображения ошибок -->
        <div class="error" id="error">Invalid username or password</div>
        <!-- PROMPT 4: Форма входа -->
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
    </div>
    <script>
        // PROMPT 4: Функция отправки логина
        function login(event) {
            event.preventDefault();
            fetch('/login', {
                method: 'POST',
                headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                body: 'username=' + encodeURIComponent(document.getElementById('username').value) +
                      '&password=' + encodeURIComponent(document.getElementById('password').value)
            }).then(response => {
                if (response.ok) window.location.href = '/';
                else document.getElementById('error').style.display = 'block';
            });
        }
    </script>
</body>
</html>
)rawliteral";

// ==================== MAIN CONTROL PAGE ====================
// PROMPT 1: Главная страница управления
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Robot Control Panel</title>
    <style>
        /* PROMPT 1: Основные стили */
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: 'Segoe UI', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 900px;
            margin: 0 auto;
            background: white;
            padding: 30px;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
        }
        h1 {
            text-align: center;
            color: #333;
            border-bottom: 3px solid #667eea;
            padding-bottom: 10px;
            margin-bottom: 20px;
        }
        
        /* PROMPT 4: Навигационная панель */
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
            text-decoration: none;
            color: #333;
        }
        .logout-btn {
            background: #dc3545;
            color: white;
            margin-left: auto;
        }
        
        /* PROMPT 3: Статус-бар с информацией */
        .status-bar {
            background: #f8f9fa;
            border-radius: 10px;
            padding: 15px;
            margin-bottom: 20px;
            display: flex;
            justify-content: space-between;
            flex-wrap: wrap;
            gap: 10px;
            border-left: 5px solid #667eea;
        }
        .info-item {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .info-label { font-weight: bold; color: #666; }
        .info-value {
            background: #e9ecef;
            padding: 5px 10px;
            border-radius: 5px;
            font-family: monospace;
        }
        .status-led {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            display: inline-block;
            margin-right: 5px;
            background-color: #28a745;
            box-shadow: 0 0 10px #28a745;
        }
        
        /* PROMPT 1: Панели управления */
        .motor-panel, .servo-panel {
            background: #f8f9fa;
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 25px;
        }
        
        /* PROMPT 1: Сетка кнопок для моторов */
        .control-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            max-width: 400px;
            margin: 20px auto;
        }
        .control-btn {
            padding: 20px;
            font-size: 20px;
            border: none;
            border-radius: 15px;
            cursor: pointer;
            color: white;
            font-weight: bold;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        .btn-forward { background: #28a745; grid-column: 2; }
        .btn-left { background: #ffc107; grid-column: 1; }
        .btn-stop { background: #dc3545; grid-column: 2; }
        .btn-right { background: #ffc107; grid-column: 3; }
        .btn-backward { background: #28a745; grid-column: 2; }
        
        /* PROMPT 1: Слайдер скорости */
        .speed-panel {
            background: white;
            border-radius: 10px;
            padding: 15px;
            margin-top: 15px;
        }
        .speed-slider {
            width: 100%;
            height: 10px;
            border-radius: 5px;
            background: linear-gradient(90deg, #28a745, #ffc107, #dc3545);
            -webkit-appearance: none;
        }
        .speed-value {
            font-size: 24px;
            font-weight: bold;
            color: #667eea;
            text-align: center;
            margin-top: 10px;
        }
        
        /* PROMPT 1: Элементы управления сервами */
        .servo-item {
            background: white;
            border-radius: 10px;
            padding: 15px;
            margin-bottom: 15px;
        }
        .servo-header {
            display: flex;
            align-items: center;
            margin-bottom: 10px;
        }
        .servo-name { font-weight: bold; width: 100px; }
        .servo-angle {
            font-family: monospace;
            font-size: 18px;
            color: #667eea;
            font-weight: bold;
            margin: 0 15px;
        }
        .servo-slider {
            width: 100%;
            height: 8px;
            border-radius: 4px;
            background: #dee2e6;
            -webkit-appearance: none;
        }
        
        /* PROMPT 4: Пресеты для манипулятора */
        .preset-panel {
            margin: 20px 0;
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
            justify-content: center;
        }
        .preset-btn {
            padding: 15px 25px;
            border: none;
            border-radius: 10px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            flex: 1;
            min-width: 120px;
        }
        .preset-home { background: #6c757d; color: white; }
        .preset-grab { background: #28a745; color: white; }
        .preset-release { background: #ffc107; color: #333; }
        
        /* PROMPT 1: Кнопка сброса */
        .reset-btn {
            background: #6c757d;
            color: white;
            border: none;
            padding: 15px 30px;
            border-radius: 10px;
            font-size: 16px;
            cursor: pointer;
            width: 100%;
            margin-top: 15px;
            font-weight: bold;
        }
        
        /* PROMPT 2: Предупреждение о таймауте */
        .warning {
            color: #dc3545;
            font-size: 14px;
            margin-top: 10px;
            padding: 10px;
            border-radius: 5px;
            background: #f8d7da;
            display: none;
        }
        .warning.show { display: block; }
        
        /* PROMPT 3: Адаптивность для мобильных */
        @media (max-width: 600px) {
            .container { padding: 15px; }
            .control-btn { padding: 15px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot Control System</h1>
        
        <!-- PROMPT 4: Навигация -->
        <div class="nav-bar">
            <span class="nav-btn">Control Panel</span>
            <a href="/profile" class="nav-btn">Profile</a>
            <a href="/admin" class="nav-btn" id="adminLink" style="display:none;">Admin</a>
            <button class="nav-btn logout-btn" onclick="logout()">Logout</button>
        </div>
        
        <!-- PROMPT 3: Статус-бар -->
        <div class="status-bar">
            <div class="info-item">
                <span class="info-label">IP:</span>
                <span class="info-value" id="ipAddress"></span>
            </div>
            <div class="info-item">
                <span class="info-label">Status:</span>
                <span class="info-value" id="connectionStatus">
                    <span class="status-led"></span> Connected
                </span>
            </div>
            <div class="info-item">
                <span class="info-label">User:</span>
                <span class="info-value" id="currentUser"></span>
            </div>
        </div>
        
        <!-- PROMPT 1: Панель управления моторами -->
        <div class="motor-panel">
            <h2>Motion Control</h2>
            <div class="control-grid">
                <button class="control-btn btn-forward" onclick="sendCommand('forward')">▲</button>
                <button class="control-btn btn-left" onclick="sendCommand('left')">◀</button>
                <button class="control-btn btn-stop" onclick="sendCommand('stop')">●</button>
                <button class="control-btn btn-right" onclick="sendCommand('right')">▶</button>
                <button class="control-btn btn-backward" onclick="sendCommand('backward')">▼</button>
            </div>
            <div class="speed-panel">
                <h3>Motor Speed</h3>
                <input type="range" min="0" max="100" value="50" class="speed-slider" id="speedSlider" 
                       oninput="updateSpeed(this.value)">
                <div class="speed-value" id="speedValue">50%</div>
            </div>
            <div class="warning" id="motorWarning">Motors active for more than 5 seconds!</div>
        </div>
        
        <!-- PROMPT 1: Панель управления манипулятором -->
        <div class="servo-panel">
            <h2>Manipulator Control</h2>
            
            <!-- PROMPT 4: Пресеты -->
            <div class="preset-panel">
                <button class="preset-btn preset-home" onclick="applyPreset('home')">Home</button>
                <button class="preset-btn preset-grab" onclick="applyPreset('grab')">Grab</button>
                <button class="preset-btn preset-release" onclick="applyPreset('release')">Release</button>
            </div>
            
            <!-- PROMPT 1: Индивидуальные сервы -->
            <div class="servo-item">
                <div class="servo-header">
                    <span class="servo-name">Base</span>
                    <span class="servo-angle" id="baseAngle">90°</span>
                </div>
                <input type="range" min="0" max="180" value="90" class="servo-slider" id="baseSlider" 
                       onchange="sendServoCommand('base', this.value)">
            </div>
            
            <div class="servo-item">
                <div class="servo-header">
                    <span class="servo-name">Shoulder</span>
                    <span class="servo-angle" id="shoulderAngle">90°</span>
                </div>
                <input type="range" min="0" max="180" value="90" class="servo-slider" id="shoulderSlider" 
                       onchange="sendServoCommand('shoulder', this.value)">
            </div>
            
            <div class="servo-item">
                <div class="servo-header">
                    <span class="servo-name">Elbow</span>
                    <span class="servo-angle" id="elbowAngle">90°</span>
                </div>
                <input type="range" min="0" max="180" value="90" class="servo-slider" id="elbowSlider" 
                       onchange="sendServoCommand('elbow', this.value)">
            </div>
            
            <div class="servo-item">
                <div class="servo-header">
                    <span class="servo-name">Wrist</span>
                    <span class="servo-angle" id="wristAngle">90°</span>
                </div>
                <input type="range" min="0" max="180" value="90" class="servo-slider" id="wristSlider" 
                       onchange="sendServoCommand('wrist', this.value)">
            </div>
            
            <div class="servo-item">
                <div class="servo-header">
                    <span class="servo-name">Gripper</span>
                    <span class="servo-angle" id="gripperAngle">90°</span>
                </div>
                <input type="range" min="0" max="180" value="90" class="servo-slider" id="gripperSlider" 
                       onchange="sendServoCommand('gripper', this.value)">
            </div>
            
            <button class="reset-btn" onclick="resetAllServos()">Reset All Servos</button>
        </div>
    </div>

    <script>
        // PROMPT 3: Отображение IP
        let currentSpeed = 50;
        document.getElementById('ipAddress').textContent = window.location.hostname;
        
        // PROMPT 4: Получение информации о пользователе
        fetch('/api/user/info')
            .then(r => r.json())
            .then(data => {
                document.getElementById('currentUser').textContent = data.username;
                if (data.role >= 2) document.getElementById('adminLink').style.display = 'block';
            });
        
        // PROMPT 1: Отправка команд моторам
        function sendCommand(cmd) {
            fetch(`/control?cmd=${cmd}&speed=${currentSpeed}`).catch(e => console.log(e));
        }
        
        // PROMPT 1: Обновление скорости
        function updateSpeed(v) {
            currentSpeed = v;
            document.getElementById('speedValue').innerHTML = v + '%';
        }
        
        // PROMPT 1: Отправка команд сервам
        function sendServoCommand(servo, value) {
            document.getElementById(servo + 'Angle').innerHTML = value + '°';
            fetch(`/servo?name=${servo}&pos=${value}`).catch(e => console.log(e));
        }
        
        // PROMPT 4: Применение пресетов
        function applyPreset(p) {
            if (p == 'home') setAllServos(90,90,90,90,90);
            else if (p == 'grab') setAllServos(90,60,120,90,150);
            else if (p == 'release') setAllServos(90,60,120,90,30);
        }
        
        // PROMPT 4: Установка всех серв
        function setAllServos(b,s,e,w,g) {
            sendServoCommand('base', b);
            sendServoCommand('shoulder', s);
            sendServoCommand('elbow', e);
            sendServoCommand('wrist', w);
            sendServoCommand('gripper', g);
            ['base','shoulder','elbow','wrist','gripper'].forEach((n,i) => {
                document.getElementById(n + 'Slider').value = [b,s,e,w,g][i];
            });
        }
        
        // PROMPT 1: Сброс серв
        function resetAllServos() { applyPreset('home'); }
        
        // PROMPT 4: Выход
        function logout() { fetch('/logout').then(() => window.location.href = '/login'); }
        
        // PROMPT 3: Управление с клавиатуры
        document.addEventListener('keydown', e => {
            if (e.repeat || e.target.tagName == 'INPUT') return;
            e.preventDefault();
            if (e.key == 'ArrowUp') sendCommand('forward');
            else if (e.key == 'ArrowDown') sendCommand('backward');
            else if (e.key == 'ArrowLeft') sendCommand('left');
            else if (e.key == 'ArrowRight') sendCommand('right');
            else if (e.key == ' ') sendCommand('stop');
        });
        
        // PROMPT 2: Остановка при закрытии
        window.addEventListener('beforeunload', () => sendCommand('stop'));
        
        // PROMPT 2: Проверка статуса
        setInterval(() => {
            fetch('/status').then(r => r.json()).then(data => {
                document.getElementById('motorWarning').className = data.motorsActive ? 'warning show' : 'warning';
            });
        }, 1000);
    </script>
</body>
</html>
)rawliteral";

// ==================== PROFILE PAGE ====================
// PROMPT 4: Страница профиля для смены пароля
const char PROFILE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>User Profile</title>
    <style>
        /* PROMPT 4: Стили для страницы профиля */
        body {
            font-family: 'Segoe UI', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
            background: white;
            padding: 30px;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
        }
        h1 { text-align: center; color: #333; border-bottom: 3px solid #667eea; padding-bottom: 10px; }
        .nav-bar { display: flex; gap: 10px; margin: 20px 0; }
        .nav-btn {
            padding: 10px 20px;
            background: #f8f9fa;
            border: none;
            border-radius: 8px;
            text-decoration: none;
            color: #333;
        }
        .logout-btn { background: #dc3545; color: white; margin-left: auto; }
        .form-group { margin-bottom: 20px; }
        label { display: block; margin-bottom: 5px; font-weight: bold; }
        input {
            width: 100%;
            padding: 12px;
            border: 2px solid #ddd;
            border-radius: 8px;
            font-size: 16px;
            box-sizing: border-box;
        }
        button {
            width: 100%;
            padding: 14px;
            background: #667eea;
            color: white;
            border: none;
            border-radius: 8px;
            font-size: 18px;
            cursor: pointer;
        }
        .message {
            padding: 10px;
            border-radius: 8px;
            margin: 10px 0;
            display: none;
        }
        .message.success { background: #d4edda; color: #155724; display: block; }
        .message.error { background: #f8d7da; color: #721c24; display: block; }
    </style>
</head>
<body>
    <div class="container">
        <h1>User Profile</h1>
        <div class="nav-bar">
            <a href="/" class="nav-btn">Control Panel</a>
            <a href="/profile" class="nav-btn">Profile</a>
            <a href="/admin" class="nav-btn" id="adminLink" style="display:none;">Admin</a>
            <button class="nav-btn logout-btn" onclick="logout()">Logout</button>
        </div>
        
        <div id="message" class="message"></div>
        
        <h2>Change Password</h2>
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
                <label>Confirm Password</label>
                <input type="password" id="confirmPassword" required minlength="6">
            </div>
            <button type="submit">Change Password</button>
        </form>
    </div>
    
    <script>
        // PROMPT 4: Проверка прав администратора
        fetch('/api/user/info').then(r => r.json()).then(data => {
            if (data.role >= 2) document.getElementById('adminLink').style.display = 'block';
        });
        
        // PROMPT 4: Смена пароля
        function changePassword(e) {
            e.preventDefault();
            const cur = document.getElementById('currentPassword').value;
            const newP = document.getElementById('newPassword').value;
            const conf = document.getElementById('confirmPassword').value;
            
            if (newP != conf) {
                showMessage('Passwords do not match', 'error');
                return;
            }
            
            fetch('/api/password', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({currentPassword: cur, newPassword: newP})
            }).then(r => r.json()).then(data => {
                if (data.success) {
                    showMessage('Password changed', 'success');
                    document.getElementById('currentPassword').value = '';
                    document.getElementById('newPassword').value = '';
                    document.getElementById('confirmPassword').value = '';
                } else {
                    showMessage(data.error || 'Error', 'error');
                }
            });
        }
        
        function showMessage(t, type) {
            const msg = document.getElementById('message');
            msg.className = 'message ' + type;
            msg.innerHTML = t;
            setTimeout(() => msg.className = 'message', 3000);
        }
        
        function logout() { fetch('/logout').then(() => window.location.href = '/login'); }
    </script>
</body>
</html>
)rawliteral";

// ==================== ADMIN PAGE ====================
// PROMPT 4: Админ-панель для управления пользователями
const char ADMIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Admin Panel</title>
    <style>
        /* PROMPT 4: Стили для админ-панели */
        body {
            font-family: 'Segoe UI', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1000px;
            margin: 0 auto;
            background: white;
            padding: 30px;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
        }
        h1 { text-align: center; color: #333; border-bottom: 3px solid #667eea; padding-bottom: 10px; }
        .nav-bar { display: flex; gap: 10px; margin: 20px 0; }
        .nav-btn {
            padding: 10px 20px;
            background: #f8f9fa;
            border: none;
            border-radius: 8px;
            text-decoration: none;
            color: #333;
        }
        .logout-btn { background: #dc3545; color: white; margin-left: auto; }
        table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }
        th {
            background: #667eea;
            color: white;
            padding: 12px;
            text-align: left;
        }
        td {
            padding: 12px;
            border-bottom: 1px solid #dee2e6;
        }
        .btn {
            padding: 6px 12px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            margin: 0 2px;
        }
        .btn-delete { background: #dc3545; color: white; }
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
    </style>
</head>
<body>
    <div class="container">
        <h1>Admin Panel</h1>
        <div class="nav-bar">
            <a href="/" class="nav-btn">Control Panel</a>
            <a href="/profile" class="nav-btn">Profile</a>
            <a href="/admin" class="nav-btn">Admin</a>
            <button class="nav-btn logout-btn" onclick="logout()">Logout</button>
        </div>
        
        <h2>User Management</h2>
        <table>
            <thead>
                <tr><th>Username</th><th>Role</th><th>Status</th><th>Actions</th></tr>
            </thead>
            <tbody id="userTable"></tbody>
        </table>
    </div>
    
    <script>
        // PROMPT 4: Загрузка списка пользователей
        function loadUsers() {
            fetch('/api/users').then(r => r.json()).then(data => {
                let html = '';
                data.users.forEach((u, i) => {
                    let roleClass = u.role == 2 ? 'admin' : (u.role == 1 ? 'user' : 'guest');
                    let roleName = u.role == 2 ? 'Admin' : (u.role == 1 ? 'User' : 'Guest');
                    html += '<tr><td>' + u.username + '</td>';
                    html += '<td><span class="role-badge role-' + roleClass + '">' + roleName + '</span></td>';
                    html += '<td>' + (u.isActive ? 'Active' : 'Inactive') + '</td>';
                    html += '<td>';
                    if (u.username != 'admin') {
                        html += '<button class="btn btn-delete" onclick="deleteUser(' + i + ')">Delete</button>';
                    }
                    html += '</td></tr>';
                });
                document.getElementById('userTable').innerHTML = html;
            });
        }
        
        // PROMPT 4: Удаление пользователя
        function deleteUser(i) {
            if (confirm('Delete user?')) {
                fetch('/api/users/' + i, {method: 'DELETE'}).then(() => loadUsers());
            }
        }
        
        function logout() { fetch('/logout').then(() => window.location.href = '/login'); }
        loadUsers();
    </script>
</body>
</html>
)rawliteral";

#endif // HTML_PAGES_H