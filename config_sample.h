/**
 * CONFIGURATION SAMPLE FILE
 * 
 * ===================================================
 * PROMPT USED:
 * "Сохранение учётных данных в отдельном файле. Это предотвращает случайное 
 * раскрытие данных при распространении кода."
 * ===================================================
 * 
 * Этот файл является шаблоном. Скопируйте его в config.h
 * и заполните реальными данными.
 * 
 * PROMPT 6: Вынос конфиденциальных данных в отдельный файл
 * PROMPT 4: Добавление учетных данных администратора
 * PROMPT 5: Настройки WPA3 и безопасности
 */

#ifndef CONFIG_SAMPLE_H
#define CONFIG_SAMPLE_H

// ==================== WI-FI CREDENTIALS ====================
// PROMPT 1: Базовые настройки Wi-Fi
// PROMPT 5: Пароль будет использован для WPA3
// PROMPT 6: Вынесено из основного кода для безопасности
#define WIFI_SSID               "Your_Robot_AP_Name"      // PROMPT 1: Имя сети
#define WIFI_PASSWORD           "YourStrongPassword123!"  // PROMPT 1: Пароль (мин 8 символов)

// ==================== ADMIN CREDENTIALS ====================
// PROMPT 4: Учетные данные для первого входа
#define ADMIN_USERNAME          "admin"                    // PROMPT 4: Логин администратора
#define ADMIN_PASSWORD          "admin123"                 // PROMPT 4: Пароль администратора

// ==================== SECURITY SETTINGS ====================
// PROMPT 4: Настройки сессий и шифрования
#define SESSION_TIMEOUT         3600000      // PROMPT 4: 1 час в миллисекундах
#define ENCRYPTION_KEY          0x7B         // PROMPT 4: Ключ для XOR шифрования

// ==================== WI-FI SETTINGS ====================
// PROMPT 1: Дополнительные настройки Wi-Fi
#define WIFI_CHANNEL            1             // PROMPT 1: Канал Wi-Fi
#define WIFI_MAX_CONNECTIONS    4             // PROMPT 1: Макс. количество подключений
#define WIFI_HIDE_SSID          false        // PROMPT 1: Скрывать ли сеть

#endif // CONFIG_SAMPLE_H