#include <WiFi.h>
#include <WiFiClient.h>
#include "EspMQTTClient.h"
#include "secrets.h"

EspMQTTClient client (
  WIFI_SSID, // WiFi SSID
  WIFI_PASSWORD, // WiFi password
  MQTT_BROKER_IP, // MQTT Broker server IP
  MQTT_USERNAME, // MQTT Broker username
  MQTT_PASSWORD, // MQTT broker password
  "esp32_fan_controller" // MQTT Client ID
);

// ========== MQTT TOPICS ==========
const char* TOPIC_TEMP = "fan_controller/temperature";
const char* TOPIC_FRONT_RPM = "fan_controller/front_fan/rpm";
const char* TOPIC_BACK_RPM = "fan_controller/back_fan/rpm";
const char* TOPIC_FAN_SPEED = "fan_controller/fan_speed_percent";
const char* TOPIC_FRONT_DOOR = "fan_controller/front_door/open";
const char* TOPIC_BACK_DOOR = "fan_controller/back_door/open";
const char* TOPIC_SENSOR_STATUS = "fan_controller/sensor_status";

// ========== FAN CONFIGURATION ==========
const int FAN_MAX_RPM = 5200;
const int PWM_FREQ = 25000;  // 25 kHz frequency for computer fans
const int PWM_RESOLUTION = 8;
const unsigned long TACH_SAMPLE_TIME = 1000;  // 1 second
const int MIN_FAN_SPEED_PERCENT = 30;  // Minimum speed to prevent fan stall
const int MIN_PWM_VALUE = (MIN_FAN_SPEED_PERCENT * 255) / 100;  // = 77
const int MAX_PWM_VALUE = 255;

// ========== TEMPERATURE CONFIGURATION ==========
const int TEMP_PIN = A10;  // WROOM D4 = ADC10
const int MIN_SPEED_TEMP = 15;
const int MAX_SPEED_TEMP = 40;
const float MIN_VALID_TEMP = -10.0;  // Minimum realistic temperature (°C)
const float MAX_VALID_TEMP = 80.0;   // Maximum realistic temperature (°C)
const int MAX_SENSOR_ERRORS = 5;     // Max consecutive errors before failsafe
const int FAILSAFE_SPEED_PERCENT = 100;  // Run fans at 100% on sensor failure

// ========== STATUS LED ==========
const int STATUS_LED_PIN = 2;

// ========== STRUCT DEFINITIONS ==========
struct Fan {
  uint8_t pwmPin;
  uint8_t tachPin;
  volatile unsigned long pulseCount;

  void initialize(void (*isr)(), uint32_t initialSpeed) {
    ledcAttach(pwmPin, PWM_FREQ, PWM_RESOLUTION);
    pinMode(tachPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(tachPin), isr, FALLING);
    ledcWrite(pwmPin, initialSpeed);
  }

  unsigned long getRPM() {
    return (pulseCount * 60000) / (TACH_SAMPLE_TIME * 2);
  }

  void resetPulseCount() {
    pulseCount = 0;
  }

  void setSpeed(uint8_t speed) {
    ledcWrite(pwmPin, speed);
  }
};

struct DoorSensor {
  uint8_t sensorPin;
  uint8_t ledPin;
  volatile bool isOpen;

  void initialize() {
    pinMode(sensorPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
  }

  void update() {
    bool doorOpenNow = digitalRead(sensorPin) == LOW;
    if (isOpen != doorOpenNow) {
      isOpen = doorOpenNow;
      digitalWrite(ledPin, isOpen ? LOW : HIGH);
    }
  }
};

// ========== GLOBAL INSTANCES ==========
Fan frontFan = {32, 33, 0};  // PWM: GPIO32, TACH: GPIO33
Fan backFan = {25, 27, 0};   // PWM: GPIO25, TACH: GPIO27

DoorSensor frontDoor = {19, 21, false};  // Sensor: GPIO19, LED: GPIO21
DoorSensor backDoor = {5, 18, false};    // Sensor: GPIO5, LED: GPIO18

// ========== GLOBAL STATE ==========
unsigned long _lastTachTime = 0;
int _fanSpeed = MIN_PWM_VALUE;  // Start at minimum speed
float _currentTemp = 0.0;
float _lastValidTemp = 20.0;  // Last known good temperature
int _sensorErrorCount = 0;     // Consecutive sensor read errors
bool _sensorFailed = false;    // Sensor in failed state
bool _lastSensorFailedState = false;  // Last published sensor state
bool _lastFrontDoorState = false;
bool _lastBackDoorState = false;

// ========== MESSAGE BUFFERS ==========
char _mqttBuffer[16];    // Buffer for formatting MQTT messages
char _serialBuffer[128]; // Buffer for formatting serial output


// ========== INTERRUPT SERVICE ROUTINES ==========
void IRAM_ATTR _frontTachISR() {
  frontFan.pulseCount = frontFan.pulseCount + 1;
}

void IRAM_ATTR _backTachISR() {
  backFan.pulseCount = backFan.pulseCount + 1;
}


// ========== TEMPERATURE FUNCTIONS ==========
float getTemperature() {
  int tmp36_Value = analogRead(TEMP_PIN);
  float voltage = tmp36_Value * 3.3;
  voltage /= 4096.0;  // ESP32 has 12-bit ADC (0-4095)
  float tempC = (voltage - 0.5) * 100; // convert to celsius

  // Validate temperature reading
  if (tempC >= MIN_VALID_TEMP && tempC <= MAX_VALID_TEMP) {
    // Valid reading - reset error counter
    _sensorErrorCount = 0;
    _lastValidTemp = tempC;

    // Recovery from failed state
    if (_sensorFailed) {
      _sensorFailed = false;
      Serial.println("Temperature sensor recovered!");
    }

    return tempC;
  } else {
    // Invalid reading
    _sensorErrorCount++;
    Serial.print("Warning: Invalid temperature reading: ");
    Serial.print(tempC);
    Serial.print(" °C (reading #");
    Serial.print(_sensorErrorCount);
    Serial.println(")");

    // Enter failed state if too many consecutive errors
    if (_sensorErrorCount >= MAX_SENSOR_ERRORS && !_sensorFailed) {
      _sensorFailed = true;
      Serial.println("ERROR: Temperature sensor failed! Entering failsafe mode.");
    }

    // Return last valid temperature
    return _lastValidTemp;
  }
}


// ========== SERIAL LOGGING ==========
void printStatus(bool frontDoor, bool backDoor, float temp, bool sensorFailed,
                 int pwm, int speedPercent, unsigned long targetRpm, unsigned long actualRpm) {
  // Format: FD:X BD:X Temp:XX.XX°C PWM:XXX Spd:XX% Tgt:XXXX Act:XXXX [FAIL]
  snprintf(_serialBuffer, sizeof(_serialBuffer),
           "FD:%d BD:%d Temp:%6.2f°C%s PWM:%3d Spd:%3d%% Tgt:%4lu Act:%4lu",
           frontDoor, backDoor, temp,
           sensorFailed ? " [FAIL]" : "       ",
           pwm, speedPercent, targetRpm, actualRpm);
  Serial.println(_serialBuffer);
}


// ========== MQTT CALLBACK ==========
void onConnectionEstablished() {
  Serial.println("MQTT Connected!");

  // Publish online status
  client.publish("fan_controller/status", "online", true);

  // Publish initial values
  client.publish(TOPIC_TEMP, "0");
  client.publish(TOPIC_FRONT_RPM, "0");
  client.publish(TOPIC_BACK_RPM, "0");
  client.publish(TOPIC_FAN_SPEED, "0");
  client.publish(TOPIC_FRONT_DOOR, "false");
  client.publish(TOPIC_BACK_DOOR, "false");
  client.publish(TOPIC_SENSOR_STATUS, "ok");
}


// ========== SETUP ==========
void setup() {
  // Start serial monitor
  Serial.begin(115200);
  Serial.println("Simple Fan Control Starting...");

  // Enable MQTT debug output
  client.enableDebuggingMessages();
  client.enableHTTPWebUpdater();
  client.enableLastWillMessage("fan_controller/status", "offline");

  // Initialize fans
  frontFan.initialize(_frontTachISR, _fanSpeed);
  backFan.initialize(_backTachISR, _fanSpeed);

  // Initialize door sensors
  frontDoor.initialize();
  backDoor.initialize();

  // Initialize status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(100);
  digitalWrite(STATUS_LED_PIN, LOW);

  _lastTachTime = millis();

  Serial.println("Fan controller ready!");
  Serial.println("---");
  Serial.println("FD=Front Door, BD=Back Door, Temp=Temperature, PWM=Fan PWM, Spd=Speed %, Tgt=Target RPM, Act=Actual RPM");
}


// ========== MAIN LOOP ==========
void loop() {
  // Process MQTT
  client.loop();

  // Update door sensors
  frontDoor.update();
  backDoor.update();

  // Publish door status changes to MQTT (with retain flag)
  if (client.isConnected()) {
    if (frontDoor.isOpen != _lastFrontDoorState) {
      client.publish(TOPIC_FRONT_DOOR, frontDoor.isOpen ? "true" : "false", true);
      _lastFrontDoorState = frontDoor.isOpen;
    }
    if (backDoor.isOpen != _lastBackDoorState) {
      client.publish(TOPIC_BACK_DOOR, backDoor.isOpen ? "true" : "false", true);
      _lastBackDoorState = backDoor.isOpen;
    }
  }

  // Check if it's time to update fan speed
  unsigned long currentTime = millis();
  if (currentTime - _lastTachTime >= TACH_SAMPLE_TIME) {

    // Get current RPM readings
    unsigned long frontRpm = frontFan.getRPM();
    unsigned long backRpm = backFan.getRPM();

    // Read temperature and calculate fan speed
    _currentTemp = getTemperature();

    // Check if sensor has failed - use failsafe speed
    if (_sensorFailed) {
      _fanSpeed = MAX_PWM_VALUE;  // Run at 100% for safety
    } else {
      _fanSpeed = map((long)_currentTemp, MIN_SPEED_TEMP, MAX_SPEED_TEMP, MIN_PWM_VALUE, MAX_PWM_VALUE);
      _fanSpeed = constrain(_fanSpeed, MIN_PWM_VALUE, MAX_PWM_VALUE);
    }

    int speedPercent = map(_fanSpeed, MIN_PWM_VALUE, MAX_PWM_VALUE, MIN_FAN_SPEED_PERCENT, 100);
    unsigned long targetRPM = map(speedPercent, 0, 100, 0, FAN_MAX_RPM);

    // Apply new fan speed
    frontFan.setSpeed(_fanSpeed);
    backFan.setSpeed(_fanSpeed);

    // Display status (fixed-width columns for easy reading)
    printStatus(frontDoor.isOpen, backDoor.isOpen, _currentTemp, _sensorFailed,
                _fanSpeed, speedPercent, targetRPM, frontRpm);

    // Publish to MQTT (with retain flag so subscribers get latest values)
    if (client.isConnected()) {
      // Temperature (format as float with 2 decimal places)
      dtostrf(_currentTemp, 4, 2, _mqttBuffer);
      client.publish(TOPIC_TEMP, _mqttBuffer, true);

      // Front fan RPM
      sprintf(_mqttBuffer, "%lu", frontRpm);
      client.publish(TOPIC_FRONT_RPM, _mqttBuffer, true);

      // Back fan RPM
      sprintf(_mqttBuffer, "%lu", backRpm);
      client.publish(TOPIC_BACK_RPM, _mqttBuffer, true);

      // Fan speed percentage
      sprintf(_mqttBuffer, "%d", speedPercent);
      client.publish(TOPIC_FAN_SPEED, _mqttBuffer, true);

      // Publish sensor status if it changed
      if (_sensorFailed != _lastSensorFailedState) {
        client.publish(TOPIC_SENSOR_STATUS, _sensorFailed ? "failed" : "ok", true);
        _lastSensorFailedState = _sensorFailed;
      }
    }

    // Reset counters (disable interrupts to prevent race condition)
    noInterrupts();
    frontFan.resetPulseCount();
    backFan.resetPulseCount();
    interrupts();

    _lastTachTime = currentTime;
  }

  delay(50);
}
