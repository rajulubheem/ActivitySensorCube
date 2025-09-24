/**
 * Kitchen Safety Monitoring Cube
 * Version: 2.0.0
 * Author: Bheema Rajulu
 * License: MIT
 *
 * Smart IoT Kitchen Safety Monitor with BLE connectivity
 * Monitors: Gas levels, Temperature, Motion, Light, and Cooking activity
 * Features: Timer control, Multi-sensor fusion, BLE alerts, OLED display
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "BH1750.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ================== PIN DEFINITIONS ==================
#define TOUCH_SENSOR_START_PIN  2      // Start/Stop timer
#define TOUCH_SENSOR_INC_PIN    3      // Increment timer
#define TOUCH_SENSOR_DEC_PIN    4      // Decrement timer
#define TOUCH_SENSOR_RESET_PIN  5      // Reset/Alarm off
#define SDA_PIN                 21     // I2C Data
#define SCL_PIN                 22     // I2C Clock
#define MQ135_PIN               34     // Gas sensor (ADC1_CH6)
#define BUZZER_PIN              13     // Alert buzzer
#define LED_RED_PIN             12     // Red alert LED
#define LED_GREEN_PIN           14     // Green status LED

// ================== DISPLAY CONFIG ==================
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDR       0x3C

// ================== SENSOR THRESHOLDS ==================
#define GAS_NORMAL_LEVEL        400    // Normal air quality
#define GAS_WARNING_LEVEL       1000   // Warning threshold
#define GAS_DANGER_LEVEL        2000   // Danger threshold
#define GAS_CRITICAL_LEVEL      3000   // Critical - immediate evacuation

#define TEMP_NORMAL_MAX         35.0   // Normal room temperature
#define TEMP_COOKING_MAX        50.0   // Normal cooking temperature
#define TEMP_WARNING_LEVEL      60.0   // Warning temperature
#define TEMP_DANGER_LEVEL       70.0   // Danger temperature

#define MOTION_THRESHOLD        0.5    // G-force change threshold
#define VIBRATION_THRESHOLD     2.0    // Vibration detection threshold
#define LIGHT_DARK_THRESHOLD    10     // Lux - darkness detection
#define INACTIVITY_WARNING      180000 // 3 minutes no motion warning
#define INACTIVITY_DANGER       300000 // 5 minutes no motion danger

// ================== BLE CONFIGURATION ==================
#define BLE_SERVER_NAME         "KitchenSafetyCube_01"
#define SERVICE_UUID            "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID_STATUS        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_UUID_SENSOR        "a3c87cd8-7f1a-4b3e-9f9e-2b5d0f0e5a6d"
#define CHAR_UUID_ALERT         "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"
#define CHAR_UUID_COMMAND       "8d7e3c4b-2a1f-4b3e-9f9e-5c6d0f0e7a8b"

// ================== SYSTEM STATES ==================
enum SystemState {
    STATE_IDLE,
    STATE_MONITORING,
    STATE_COOKING_ACTIVE,
    STATE_TIMER_RUNNING,
    STATE_WARNING,
    STATE_DANGER,
    STATE_CRITICAL
};

enum AlertType {
    ALERT_NONE = 0,
    ALERT_GAS_WARNING = 1,
    ALERT_GAS_DANGER = 2,
    ALERT_TEMP_WARNING = 3,
    ALERT_TEMP_DANGER = 4,
    ALERT_NO_MOTION = 5,
    ALERT_TIMER_EXPIRED = 6,
    ALERT_SENSOR_FAILURE = 7,
    ALERT_MULTIPLE = 8
};

enum CookingMode {
    MODE_OFF = 0,
    MODE_STOVE_TOP = 1,
    MODE_OVEN = 2,
    MODE_MICROWAVE = 3,
    MODE_AUTO_DETECT = 4
};

// ================== DATA STRUCTURES ==================
struct SensorReadings {
    // MPU9250 data
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float temperature_mpu;

    // Environmental sensors
    float gasLevel;
    float gasBaseline;
    float lightLevel;

    // Calculated values
    float motionMagnitude;
    float vibrationLevel;
    bool motionDetected;
    bool significantMotion;
    unsigned long lastMotionTime;
    unsigned long lastSignificantMotion;

    // Sensor health
    bool mpuHealthy;
    bool lightSensorHealthy;
    bool gasSensorHealthy;
    uint8_t errorCount;
};

struct SystemStatus {
    SystemState currentState;
    SystemState previousState;
    AlertType activeAlert;
    CookingMode cookingMode;

    // Timer management
    uint32_t timerDuration;      // Total timer in seconds
    uint32_t timerRemaining;     // Remaining time in seconds
    unsigned long timerStartTime;
    bool timerActive;
    bool timerPaused;

    // Activity tracking
    unsigned long cookingStartTime;
    unsigned long lastActivityTime;
    unsigned long totalCookingTime;
    uint16_t dailyCookingCycles;

    // Alert management
    bool alarmActive;
    bool alarmMuted;
    uint8_t alertLevel;          // 0=none, 1=info, 2=warning, 3=danger, 4=critical
    unsigned long alertStartTime;
    uint8_t alertCount;

    // System health
    bool systemHealthy;
    uint32_t uptimeSeconds;
    float cpuTemperature;
    uint8_t freeHeapPercent;
};

struct UserPreferences {
    bool soundEnabled;
    bool bleEnabled;
    bool autoDetectMode;
    uint8_t displayBrightness;
    uint8_t alertSensitivity;    // 1=low, 2=medium, 3=high
    uint16_t defaultTimerMinutes;
    float gasCalibrationFactor;
};

// ================== GLOBAL OBJECTS ==================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU9250_asukiaaa mpu;
BH1750 lightMeter;

// BLE objects
BLEServer* pServer = NULL;
BLECharacteristic* pCharStatus = NULL;
BLECharacteristic* pCharSensor = NULL;
BLECharacteristic* pCharAlert = NULL;
BLECharacteristic* pCharCommand = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Global data structures
SensorReadings sensors;
SystemStatus systemStatus;
UserPreferences preferences;

// FreeRTOS handles
SemaphoreHandle_t xSensorMutex;
SemaphoreHandle_t xStatusMutex;
SemaphoreHandle_t xDisplayMutex;
QueueHandle_t xAlertQueue;
TaskHandle_t xSensorTaskHandle;
TaskHandle_t xDisplayTaskHandle;
TaskHandle_t xAlertTaskHandle;
TaskHandle_t xBLETaskHandle;
TaskHandle_t xButtonTaskHandle;
TaskHandle_t xMonitorTaskHandle;

// Timing variables
unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastBLEUpdate = 0;
unsigned long lastHealthCheck = 0;

// ================== BLE CALLBACKS ==================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client Connected");
        digitalWrite(LED_GREEN_PIN, HIGH);
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client Disconnected");
        digitalWrite(LED_GREEN_PIN, LOW);
    }
};

class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            processCommand(value);
        }
    }
};

// ================== INITIALIZATION FUNCTIONS ==================
void setupPins() {
    pinMode(TOUCH_SENSOR_START_PIN, INPUT_PULLUP);
    pinMode(TOUCH_SENSOR_INC_PIN, INPUT_PULLUP);
    pinMode(TOUCH_SENSOR_DEC_PIN, INPUT_PULLUP);
    pinMode(TOUCH_SENSOR_RESET_PIN, INPUT_PULLUP);
    pinMode(MQ135_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
}

bool setupDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("SSD1306 allocation failed"));
        return false;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Kitchen Safety Cube"));
    display.println(F("Version 2.0.0"));
    display.println(F("Initializing..."));
    display.display();
    return true;
}

bool setupSensors() {
    Wire.begin(SDA_PIN, SCL_PIN);

    // Initialize MPU9250
    mpu.setWire(&Wire);
    mpu.beginAccel();
    mpu.beginGyro();
    mpu.beginMag();

    uint8_t sensorId;
    if (mpu.readId(&sensorId) == 0) {
        Serial.println("MPU9250 ID: " + String(sensorId));
        sensors.mpuHealthy = true;
    } else {
        Serial.println("MPU9250 initialization failed!");
        sensors.mpuHealthy = false;
    }

    // Initialize BH1750 light sensor
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
        Serial.println("BH1750 initialized");
        sensors.lightSensorHealthy = true;
    } else {
        Serial.println("BH1750 initialization failed!");
        sensors.lightSensorHealthy = false;
    }

    // Calibrate gas sensor baseline
    calibrateGasSensor();

    return sensors.mpuHealthy || sensors.lightSensorHealthy;
}

void setupBLE() {
    if (!preferences.bleEnabled) return;

    BLEDevice::init(BLE_SERVER_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Status characteristic
    pCharStatus = pService->createCharacteristic(
        CHAR_UUID_STATUS,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharStatus->addDescriptor(new BLE2902());

    // Sensor data characteristic
    pCharSensor = pService->createCharacteristic(
        CHAR_UUID_SENSOR,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharSensor->addDescriptor(new BLE2902());

    // Alert characteristic
    pCharAlert = pService->createCharacteristic(
        CHAR_UUID_ALERT,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharAlert->addDescriptor(new BLE2902());

    // Command characteristic
    pCharCommand = pService->createCharacteristic(
        CHAR_UUID_COMMAND,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCharCommand->setCallbacks(new CommandCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    BLEDevice::startAdvertising();

    Serial.println("BLE Service Started");
}

void initializeSystem() {
    // Initialize system status
    systemStatus.currentState = STATE_IDLE;
    systemStatus.previousState = STATE_IDLE;
    systemStatus.activeAlert = ALERT_NONE;
    systemStatus.cookingMode = MODE_OFF;
    systemStatus.timerDuration = 0;
    systemStatus.timerRemaining = 0;
    systemStatus.timerActive = false;
    systemStatus.timerPaused = false;
    systemStatus.alarmActive = false;
    systemStatus.alarmMuted = false;
    systemStatus.alertLevel = 0;
    systemStatus.systemHealthy = true;
    systemStatus.uptimeSeconds = 0;
    systemStatus.dailyCookingCycles = 0;

    // Initialize preferences with defaults
    preferences.soundEnabled = true;
    preferences.bleEnabled = true;
    preferences.autoDetectMode = true;
    preferences.displayBrightness = 128;
    preferences.alertSensitivity = 2;  // Medium
    preferences.defaultTimerMinutes = 30;
    preferences.gasCalibrationFactor = 1.0;

    // Initialize sensor structure
    sensors.gasBaseline = 400;  // Will be calibrated
    sensors.errorCount = 0;
    sensors.lastMotionTime = millis();
}

// ================== SENSOR FUNCTIONS ==================
void calibrateGasSensor() {
    Serial.println("Calibrating gas sensor...");
    float sum = 0;
    int samples = 10;

    for (int i = 0; i < samples; i++) {
        sum += analogRead(MQ135_PIN);
        delay(100);
    }

    sensors.gasBaseline = (sum / samples) * preferences.gasCalibrationFactor;
    Serial.println("Gas baseline: " + String(sensors.gasBaseline));
}

float readGasSensor() {
    int rawValue = analogRead(MQ135_PIN);
    // Convert to PPM approximation (requires proper calibration for accuracy)
    float ratio = rawValue / sensors.gasBaseline;
    float ppm = 100 * pow(ratio, 2);  // Simplified conversion
    return constrain(ppm, 0, 5000);
}

void readMotionSensor() {
    if (!sensors.mpuHealthy) return;

    // Read accelerometer
    if (mpu.accelUpdate() == 0) {
        sensors.accelX = mpu.accelX();
        sensors.accelY = mpu.accelY();
        sensors.accelZ = mpu.accelZ();

        // Calculate motion magnitude (excluding gravity)
        float totalAccel = sqrt(sq(sensors.accelX) + sq(sensors.accelY) + sq(sensors.accelZ));
        sensors.motionMagnitude = abs(totalAccel - 1.0);  // Remove 1g gravity

        // Detect motion
        if (sensors.motionMagnitude > MOTION_THRESHOLD) {
            sensors.motionDetected = true;
            sensors.lastMotionTime = millis();

            if (sensors.motionMagnitude > VIBRATION_THRESHOLD) {
                sensors.significantMotion = true;
                sensors.lastSignificantMotion = millis();
            }
        } else if (millis() - sensors.lastMotionTime > 5000) {
            sensors.motionDetected = false;
        }
    }

    // Read gyroscope
    if (mpu.gyroUpdate() == 0) {
        sensors.gyroX = mpu.gyroX();
        sensors.gyroY = mpu.gyroY();
        sensors.gyroZ = mpu.gyroZ();

        // Calculate vibration level
        sensors.vibrationLevel = sqrt(sq(sensors.gyroX) + sq(sensors.gyroY) + sq(sensors.gyroZ));
    }

    // Read magnetometer
    if (mpu.magUpdate() == 0) {
        sensors.magX = mpu.magX();
        sensors.magY = mpu.magY();
        sensors.magZ = mpu.magZ();
    }

    // Read temperature from MPU
    mpu.beginTemp();
    if (mpu.tempUpdate() == 0) {
        sensors.temperature_mpu = mpu.temp();
    }
}

void readLightSensor() {
    if (!sensors.lightSensorHealthy) return;

    if (lightMeter.measurementReady()) {
        sensors.lightLevel = lightMeter.readLightLevel();
    }
}

// ================== SAFETY MONITORING ==================
void checkSafetyConditions() {
    if (xSemaphoreTake(xStatusMutex, portMAX_DELAY) != pdTRUE) return;

    AlertType newAlert = ALERT_NONE;
    uint8_t newAlertLevel = 0;

    // Check gas levels
    if (sensors.gasLevel > GAS_CRITICAL_LEVEL) {
        newAlert = ALERT_GAS_DANGER;
        newAlertLevel = 4;  // Critical
    } else if (sensors.gasLevel > GAS_DANGER_LEVEL) {
        newAlert = ALERT_GAS_DANGER;
        newAlertLevel = 3;  // Danger
    } else if (sensors.gasLevel > GAS_WARNING_LEVEL) {
        newAlert = ALERT_GAS_WARNING;
        newAlertLevel = 2;  // Warning
    }

    // Check temperature
    if (sensors.temperature_mpu > TEMP_DANGER_LEVEL) {
        if (newAlertLevel < 3) {
            newAlert = ALERT_TEMP_DANGER;
            newAlertLevel = 3;
        }
    } else if (sensors.temperature_mpu > TEMP_WARNING_LEVEL) {
        if (newAlertLevel < 2) {
            newAlert = ALERT_TEMP_WARNING;
            newAlertLevel = 2;
        }
    }

    // Check for inactivity during cooking
    if (systemStatus.currentState == STATE_COOKING_ACTIVE) {
        unsigned long inactivityTime = millis() - sensors.lastMotionTime;

        if (inactivityTime > INACTIVITY_DANGER) {
            if (newAlertLevel < 3) {
                newAlert = ALERT_NO_MOTION;
                newAlertLevel = 3;
            }
        } else if (inactivityTime > INACTIVITY_WARNING) {
            if (newAlertLevel < 2) {
                newAlert = ALERT_NO_MOTION;
                newAlertLevel = 2;
            }
        }
    }

    // Update system status
    systemStatus.activeAlert = newAlert;
    systemStatus.alertLevel = newAlertLevel;

    // Update system state based on conditions
    updateSystemState();

    // Trigger alarm if needed
    if (newAlertLevel >= 3 && !systemStatus.alarmMuted) {
        systemStatus.alarmActive = true;
        systemStatus.alertStartTime = millis();
        systemStatus.alertCount++;
    } else if (newAlertLevel == 0) {
        systemStatus.alarmActive = false;
    }

    xSemaphoreGive(xStatusMutex);
}

void updateSystemState() {
    SystemState newState = systemStatus.currentState;

    if (systemStatus.alertLevel >= 4) {
        newState = STATE_CRITICAL;
    } else if (systemStatus.alertLevel >= 3) {
        newState = STATE_DANGER;
    } else if (systemStatus.alertLevel >= 2) {
        newState = STATE_WARNING;
    } else if (systemStatus.timerActive) {
        newState = STATE_TIMER_RUNNING;
    } else if (detectCookingActivity()) {
        newState = STATE_COOKING_ACTIVE;
    } else if (sensors.motionDetected) {
        newState = STATE_MONITORING;
    } else {
        newState = STATE_IDLE;
    }

    if (newState != systemStatus.currentState) {
        systemStatus.previousState = systemStatus.currentState;
        systemStatus.currentState = newState;
        Serial.println("State changed: " + getStateName(newState));
    }
}

bool detectCookingActivity() {
    // Multi-factor cooking detection
    bool gasElevated = sensors.gasLevel > (sensors.gasBaseline * 1.2);
    bool tempElevated = sensors.temperature_mpu > TEMP_NORMAL_MAX;
    bool recentMotion = (millis() - sensors.lastMotionTime) < 60000;
    bool vibrationPresent = sensors.vibrationLevel > 0.5;

    // Cooking is likely if 2+ factors are present
    int factors = gasElevated + tempElevated + recentMotion + vibrationPresent;
    return factors >= 2;
}

// ================== TASK FUNCTIONS ==================
void sensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10Hz sampling

    while (1) {
        if (xSemaphoreTake(xSensorMutex, portMAX_DELAY) == pdTRUE) {
            // Read all sensors
            readMotionSensor();
            sensors.gasLevel = readGasSensor();
            readLightSensor();

            xSemaphoreGive(xSensorMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void monitorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500);  // 2Hz monitoring

    while (1) {
        // Check safety conditions
        checkSafetyConditions();

        // Update timer if active
        if (systemStatus.timerActive && !systemStatus.timerPaused) {
            unsigned long elapsed = (millis() - systemStatus.timerStartTime) / 1000;
            if (elapsed >= systemStatus.timerDuration) {
                systemStatus.timerActive = false;
                systemStatus.timerRemaining = 0;
                systemStatus.activeAlert = ALERT_TIMER_EXPIRED;
                systemStatus.alarmActive = true;
            } else {
                systemStatus.timerRemaining = systemStatus.timerDuration - elapsed;
            }
        }

        // Health check
        if (millis() - lastHealthCheck > 10000) {
            performHealthCheck();
            lastHealthCheck = millis();
        }

        // Update uptime
        systemStatus.uptimeSeconds = millis() / 1000;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void displayTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200);  // 5Hz refresh

    while (1) {
        if (xSemaphoreTake(xDisplayMutex, portMAX_DELAY) == pdTRUE) {
            updateDisplay();
            xSemaphoreGive(xDisplayMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    // Status line
    display.print(getStateName(systemStatus.currentState));
    if (deviceConnected) {
        display.setCursor(100, 0);
        display.print("BLE");
    }

    // Timer line
    if (systemStatus.timerActive) {
        display.setCursor(0, 10);
        display.print("Timer: ");
        display.print(systemStatus.timerRemaining / 60);
        display.print(":");
        display.printf("%02d", systemStatus.timerRemaining % 60);
    }

    // Sensor readings
    display.setCursor(0, 20);
    display.print("Gas: ");
    display.print((int)sensors.gasLevel);
    display.print(" ppm");

    display.setCursor(64, 20);
    display.print("T: ");
    display.print(sensors.temperature_mpu, 1);
    display.print("C");

    display.setCursor(0, 30);
    display.print("Light: ");
    display.print((int)sensors.lightLevel);
    display.print(" lux");

    display.setCursor(0, 40);
    display.print("Motion: ");
    display.print(sensors.motionDetected ? "YES" : "NO");

    // Alert message
    if (systemStatus.activeAlert != ALERT_NONE) {
        display.setCursor(0, 50);
        display.setTextSize(1);
        display.print(getAlertMessage(systemStatus.activeAlert));
    }

    // Activity indicator
    display.setCursor(120, 56);
    display.print(millis() / 1000 % 2 ? "*" : " ");

    display.display();
}

void alertTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(250);  // 4Hz for alerts
    bool buzzerState = false;
    uint8_t ledPattern = 0;

    while (1) {
        if (systemStatus.alarmActive && preferences.soundEnabled) {
            // Generate alert pattern based on severity
            switch (systemStatus.alertLevel) {
                case 4:  // Critical - continuous
                    digitalWrite(BUZZER_PIN, HIGH);
                    digitalWrite(LED_RED_PIN, HIGH);
                    break;

                case 3:  // Danger - fast beep
                    buzzerState = !buzzerState;
                    digitalWrite(BUZZER_PIN, buzzerState);
                    digitalWrite(LED_RED_PIN, buzzerState);
                    break;

                case 2:  // Warning - slow beep
                    if (millis() % 1000 < 500) {
                        digitalWrite(BUZZER_PIN, HIGH);
                        digitalWrite(LED_RED_PIN, HIGH);
                    } else {
                        digitalWrite(BUZZER_PIN, LOW);
                        digitalWrite(LED_RED_PIN, LOW);
                    }
                    break;

                default:
                    digitalWrite(BUZZER_PIN, LOW);
                    digitalWrite(LED_RED_PIN, LOW);
            }
        } else {
            digitalWrite(BUZZER_PIN, LOW);

            // LED status indication
            if (systemStatus.currentState == STATE_COOKING_ACTIVE) {
                digitalWrite(LED_RED_PIN, millis() % 2000 < 100);  // Brief flash
            } else {
                digitalWrite(LED_RED_PIN, LOW);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void bleTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // 1Hz BLE updates

    while (1) {
        if (deviceConnected) {
            sendBLEUpdates();
        }

        // Handle connection changes
        if (!deviceConnected && oldDeviceConnected) {
            delay(500);
            pServer->startAdvertising();
            Serial.println("Start advertising");
            oldDeviceConnected = deviceConnected;
        }

        if (deviceConnected && !oldDeviceConnected) {
            oldDeviceConnected = deviceConnected;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void sendBLEUpdates() {
    // Send status update
    if (pCharStatus) {
        String status = "{";
        status += "\"state\":" + String(systemStatus.currentState) + ",";
        status += "\"alert\":" + String(systemStatus.activeAlert) + ",";
        status += "\"timer\":" + String(systemStatus.timerRemaining) + ",";
        status += "\"uptime\":" + String(systemStatus.uptimeSeconds);
        status += "}";

        pCharStatus->setValue(status.c_str());
        pCharStatus->notify();
    }

    // Send sensor data
    if (pCharSensor) {
        String sensorData = "{";
        sensorData += "\"gas\":" + String(sensors.gasLevel, 0) + ",";
        sensorData += "\"temp\":" + String(sensors.temperature_mpu, 1) + ",";
        sensorData += "\"light\":" + String(sensors.lightLevel, 0) + ",";
        sensorData += "\"motion\":" + String(sensors.motionDetected) + ",";
        sensorData += "\"vibration\":" + String(sensors.vibrationLevel, 2);
        sensorData += "}";

        pCharSensor->setValue(sensorData.c_str());
        pCharSensor->notify();
    }

    // Send alert if active
    if (systemStatus.activeAlert != ALERT_NONE && pCharAlert) {
        String alert = "{";
        alert += "\"type\":" + String(systemStatus.activeAlert) + ",";
        alert += "\"level\":" + String(systemStatus.alertLevel) + ",";
        alert += "\"message\":\"" + getAlertMessage(systemStatus.activeAlert) + "\"";
        alert += "}";

        pCharAlert->setValue(alert.c_str());
        pCharAlert->notify();
    }
}

void buttonTask(void *pvParameters) {
    static unsigned long lastDebounce[4] = {0, 0, 0, 0};
    const unsigned long debounceDelay = 50;
    static bool lastState[4] = {HIGH, HIGH, HIGH, HIGH};

    while (1) {
        // Check Start/Stop button
        checkButton(TOUCH_SENSOR_START_PIN, 0, lastState, lastDebounce, []() {
            toggleTimer();
        });

        // Check Increment button
        checkButton(TOUCH_SENSOR_INC_PIN, 1, lastState, lastDebounce, []() {
            adjustTimer(60);  // Add 1 minute
        });

        // Check Decrement button
        checkButton(TOUCH_SENSOR_DEC_PIN, 2, lastState, lastDebounce, []() {
            adjustTimer(-60);  // Remove 1 minute
        });

        // Check Reset/Mute button
        checkButton(TOUCH_SENSOR_RESET_PIN, 3, lastState, lastDebounce, []() {
            handleReset();
        });

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void checkButton(int pin, int index, bool lastState[], unsigned long lastDebounce[],
                 void (*action)()) {
    bool currentState = digitalRead(pin);

    if (currentState != lastState[index]) {
        lastDebounce[index] = millis();
    }

    if ((millis() - lastDebounce[index]) > 50) {
        if (currentState == LOW && lastState[index] == HIGH) {
            action();
        }
    }

    lastState[index] = currentState;
}

// ================== BUTTON ACTIONS ==================
void toggleTimer() {
    if (xSemaphoreTake(xStatusMutex, portMAX_DELAY) == pdTRUE) {
        if (systemStatus.timerActive) {
            // Stop timer
            systemStatus.timerActive = false;
            systemStatus.timerPaused = true;
            Serial.println("Timer stopped");
        } else {
            // Start timer
            if (systemStatus.timerDuration == 0) {
                systemStatus.timerDuration = preferences.defaultTimerMinutes * 60;
            }
            systemStatus.timerActive = true;
            systemStatus.timerPaused = false;
            systemStatus.timerStartTime = millis();
            systemStatus.timerRemaining = systemStatus.timerDuration;
            Serial.println("Timer started: " + String(systemStatus.timerDuration) + " seconds");
        }
        xSemaphoreGive(xStatusMutex);
    }
}

void adjustTimer(int seconds) {
    if (xSemaphoreTake(xStatusMutex, portMAX_DELAY) == pdTRUE) {
        if (!systemStatus.timerActive) {
            systemStatus.timerDuration += seconds;
            if (systemStatus.timerDuration < 0) {
                systemStatus.timerDuration = 0;
            } else if (systemStatus.timerDuration > 7200) {  // Max 2 hours
                systemStatus.timerDuration = 7200;
            }
            Serial.println("Timer set to: " + String(systemStatus.timerDuration) + " seconds");
        }
        xSemaphoreGive(xStatusMutex);
    }
}

void handleReset() {
    if (xSemaphoreTake(xStatusMutex, portMAX_DELAY) == pdTRUE) {
        if (systemStatus.alarmActive) {
            // Mute alarm
            systemStatus.alarmActive = false;
            systemStatus.alarmMuted = true;
            Serial.println("Alarm muted");

            // Auto-unmute after 5 minutes
            xTaskCreate([](void* p) {
                vTaskDelay(pdMS_TO_TICKS(300000));
                systemStatus.alarmMuted = false;
                vTaskDelete(NULL);
            }, "UnmuteTask", 1024, NULL, 1, NULL);
        } else {
            // Reset timer
            systemStatus.timerDuration = 0;
            systemStatus.timerRemaining = 0;
            systemStatus.timerActive = false;
            systemStatus.timerPaused = false;
            Serial.println("Timer reset");
        }
        xSemaphoreGive(xStatusMutex);
    }
}

// ================== UTILITY FUNCTIONS ==================
void processCommand(std::string command) {
    if (command == "RESET") {
        handleReset();
    } else if (command == "MUTE") {
        systemStatus.alarmMuted = true;
    } else if (command == "UNMUTE") {
        systemStatus.alarmMuted = false;
    } else if (command.substr(0, 6) == "TIMER:") {
        int minutes = std::stoi(command.substr(6));
        systemStatus.timerDuration = minutes * 60;
    } else if (command == "CALIBRATE") {
        calibrateGasSensor();
    }
}

void performHealthCheck() {
    systemStatus.systemHealthy = true;

    // Check sensor health
    if (!sensors.mpuHealthy) {
        Serial.println("WARNING: MPU9250 not responding");
        systemStatus.systemHealthy = false;
    }

    if (!sensors.lightSensorHealthy) {
        Serial.println("WARNING: Light sensor not responding");
        systemStatus.systemHealthy = false;
    }

    // Check memory
    systemStatus.freeHeapPercent = (ESP.getFreeHeap() * 100) / ESP.getHeapSize();
    if (systemStatus.freeHeapPercent < 10) {
        Serial.println("WARNING: Low memory");
        systemStatus.systemHealthy = false;
    }

    // Check CPU temperature (simulated)
    systemStatus.cpuTemperature = temperatureRead();
    if (systemStatus.cpuTemperature > 80) {
        Serial.println("WARNING: High CPU temperature");
        systemStatus.systemHealthy = false;
    }
}

String getStateName(SystemState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_MONITORING: return "MONITORING";
        case STATE_COOKING_ACTIVE: return "COOKING";
        case STATE_TIMER_RUNNING: return "TIMER ON";
        case STATE_WARNING: return "WARNING!";
        case STATE_DANGER: return "DANGER!!";
        case STATE_CRITICAL: return "CRITICAL!!!";
        default: return "UNKNOWN";
    }
}

String getAlertMessage(AlertType alert) {
    switch (alert) {
        case ALERT_GAS_WARNING: return "Gas detected!";
        case ALERT_GAS_DANGER: return "EVACUATE-GAS!";
        case ALERT_TEMP_WARNING: return "High temp!";
        case ALERT_TEMP_DANGER: return "FIRE RISK!";
        case ALERT_NO_MOTION: return "Check stove!";
        case ALERT_TIMER_EXPIRED: return "Timer done!";
        case ALERT_SENSOR_FAILURE: return "Sensor fail!";
        case ALERT_MULTIPLE: return "MULTI-ALERT!";
        default: return "";
    }
}

// ================== MAIN SETUP & LOOP ==================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("    Kitchen Safety Monitoring Cube");
    Serial.println("         Version 2.0.0");
    Serial.println("          By: Bheema Rajulu");
    Serial.println("========================================\n");

    // Initialize system
    initializeSystem();
    setupPins();

    // Initialize hardware
    if (!setupDisplay()) {
        Serial.println("ERROR: Display initialization failed!");
    }

    if (!setupSensors()) {
        Serial.println("ERROR: Sensor initialization failed!");
    }

    setupBLE();

    // Create mutexes and queues
    xSensorMutex = xSemaphoreCreateMutex();
    xStatusMutex = xSemaphoreCreateMutex();
    xDisplayMutex = xSemaphoreCreateMutex();
    xAlertQueue = xQueueCreate(10, sizeof(AlertType));

    // Create tasks with proper core assignment
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 3, &xSensorTaskHandle, 1);
    xTaskCreatePinnedToCore(monitorTask, "MonitorTask", 4096, NULL, 2, &xMonitorTaskHandle, 1);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, &xDisplayTaskHandle, 1);
    xTaskCreatePinnedToCore(alertTask, "AlertTask", 2048, NULL, 4, &xAlertTaskHandle, 0);
    xTaskCreatePinnedToCore(bleTask, "BLETask", 4096, NULL, 1, &xBLETaskHandle, 0);
    xTaskCreatePinnedToCore(buttonTask, "ButtonTask", 2048, NULL, 2, &xButtonTaskHandle, 0);

    Serial.println("System initialization complete!");
    Serial.println("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
    Serial.println("CPU frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");

    // Flash LEDs to indicate ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_GREEN_PIN, HIGH);
        delay(100);
        digitalWrite(LED_GREEN_PIN, LOW);
        delay(100);
    }
}

void loop() {
    // Main loop is empty - all work done in tasks
    // Keep alive and monitor for system health
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Periodic status print (every 30 seconds)
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 30000) {
        Serial.println("\n--- System Status ---");
        Serial.println("State: " + getStateName(systemStatus.currentState));
        Serial.println("Gas: " + String(sensors.gasLevel) + " ppm");
        Serial.println("Temp: " + String(sensors.temperature_mpu) + " C");
        Serial.println("Motion: " + String(sensors.motionDetected ? "Yes" : "No"));
        Serial.println("Uptime: " + String(systemStatus.uptimeSeconds) + " seconds");
        Serial.println("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        Serial.println("-------------------\n");
        lastStatusPrint = millis();
    }
}