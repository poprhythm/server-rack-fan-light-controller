#include <WiFi.h>
#include <WiFiClient.h>
#include "EspMQTTClient.h"
#include "secrets.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"
#include <Preferences.h>

EspMQTTClient client (
  WIFI_SSID,
  WIFI_PASSWORD,
  MQTT_BROKER_IP,
  MQTT_USERNAME,
  MQTT_PASSWORD,
  "server_cabinet_fan_light_controller"
);

// ========== MQTT TOPICS ==========
const char* TOPIC_TEMP = "fan_controller/temperature";
const char* TOPIC_FRONT_RPM = "fan_controller/front_fan/rpm";
const char* TOPIC_BACK_RPM = "fan_controller/back_fan/rpm";
const char* TOPIC_FAN_SPEED = "fan_controller/fan_speed_percent";
const char* TOPIC_FRONT_DOOR = "fan_controller/front_door/open";
const char* TOPIC_BACK_DOOR = "fan_controller/back_door/open";
const char* TOPIC_SENSOR_STATUS = "fan_controller/sensor_status";
const char* TOPIC_FRONT_FAN_STALLED = "fan_controller/front_fan/stalled";
const char* TOPIC_BACK_FAN_STALLED = "fan_controller/back_fan/stalled";

// ========== FAN CONFIGURATION ==========
const int FAN_MAX_RPM = 5200;
const int PWM_FREQ = 25000;
const int PWM_RESOLUTION = 8;
const unsigned long TACH_SAMPLE_TIME = 1000;
const int MIN_FAN_SPEED_PERCENT = 30;
const int MIN_PWM_VALUE = (MIN_FAN_SPEED_PERCENT * 255) / 100;
const int MAX_PWM_VALUE = 255;
const unsigned long FAN_STALL_GRACE_PERIOD = 60000;  // Allow 60s for fans to spin up after boot
const int MIN_RPM_THRESHOLD = 200;  // Below this RPM is considered stalled

// ========== TEMPERATURE CONFIGURATION ==========
const int TEMP_PIN = A10;
const int MIN_SPEED_TEMP = 20;
const int MAX_SPEED_TEMP = 35;
const float MIN_VALID_TEMP = -10.0;
const float MAX_VALID_TEMP = 80.0;
const int MAX_SENSOR_ERRORS = 5;
const int FAILSAFE_SPEED_PERCENT = 100;
const float TEMP_CALIBRATION_OFFSET = 0.0;  // Adjust if temperature reads consistently high/low

// ========== PIN ASSIGNMENTS ==========
const int STATUS_LED_PIN = 2;

// ========== STRUCT DEFINITIONS ==========
struct Fan {
  uint8_t pwmPin;
  uint8_t tachPin;
  const char* rpmTopic;
  const char* stalledTopic;
  volatile unsigned long pulseCount;
  bool stalled;
  bool lastPublishedStalledState;

  void initialize(void (*isr)(), uint32_t initialSpeed) {
    ledcAttach(pwmPin, PWM_FREQ, PWM_RESOLUTION);
    pinMode(tachPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(tachPin), isr, FALLING);
    ledcWrite(pwmPin, initialSpeed);
    stalled = false;
    lastPublishedStalledState = false;
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

  void checkStall(bool gracePeriodExpired, int speedPercent, int minSpeedPercent, int minRPM) {
    if (gracePeriodExpired && speedPercent > minSpeedPercent) {
      unsigned long rpm = getRPM();
      bool isStalled = (rpm < minRPM);

      if (isStalled != stalled) {
        stalled = isStalled;
        Serial.print("WARNING: Fan on pin ");
        Serial.print(pwmPin);
        Serial.print(" ");
        Serial.println(isStalled ? "STALLED!" : "recovered");
      }
    }
  }

  void publishRPM(EspMQTTClient& client, char* buffer) {
    if (client.isConnected()) {
      unsigned long rpm = getRPM();
      sprintf(buffer, "%lu", rpm);
      client.publish(rpmTopic, buffer, true);
    }
  }

  void publishStallStatus(EspMQTTClient& client) {
    if (client.isConnected() && stalled != lastPublishedStalledState) {
      client.publish(stalledTopic, stalled ? "true" : "false", true);
      lastPublishedStalledState = stalled;
    }
  }
};

struct DoorSensor {
  uint8_t sensorPin;
  uint8_t ledPin;
  const char* mqttTopic;
  volatile bool isOpen;
  bool lastPublishedState;

  void initialize() {
    pinMode(sensorPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    lastPublishedState = false;
  }

  void update() {
    bool doorOpenNow = digitalRead(sensorPin) == HIGH;
    if (isOpen != doorOpenNow) {
      isOpen = doorOpenNow;
      digitalWrite(ledPin, isOpen ? HIGH : LOW);
    }
  }

  void publishMQTT(EspMQTTClient& client) {
    if (client.isConnected() && isOpen != lastPublishedState) {
      client.publish(mqttTopic, isOpen ? "true" : "false", true);
      lastPublishedState = isOpen;
      Serial.print("MQTT: Door ");
      Serial.print(mqttTopic);
      Serial.print(" ");
      Serial.println(isOpen ? "opened" : "closed");
    }
  }
};

struct TemperatureSensor {
  uint8_t pin;
  float calibrationOffset;
  float minValidTemp;
  float maxValidTemp;
  int maxSensorErrors;
  float currentTemp;
  float lastValidTemp;
  int sensorErrorCount;
  bool sensorFailed;
  bool lastPublishedFailedState;
  const char* tempTopic;
  const char* statusTopic;
  Preferences* prefsPtr;

  void initialize(uint8_t adcPin, float offset, float minTemp, float maxTemp, int maxErrors, Preferences& prefs) {
    pin = adcPin;
    calibrationOffset = offset;
    minValidTemp = minTemp;
    maxValidTemp = maxTemp;
    maxSensorErrors = maxErrors;
    currentTemp = 0.0;
    sensorErrorCount = 0;
    sensorFailed = false;
    lastPublishedFailedState = false;
    prefsPtr = &prefs;

    // Load last saved temperature from flash, or use default
    lastValidTemp = loadTemperature(prefs);
    Serial.print("Loaded last temperature from flash: ");
    Serial.print(lastValidTemp);
    Serial.println(" °C");

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    analogSetPinAttenuation(pin, ADC_11db);
  }

  float read(bool useWifiWorkaround) {
    if (useWifiWorkaround) {
      Serial.println("Reading accurate temperature (WiFi will disconnect briefly)...");
      esp_wifi_stop();
      delay(50);
    }

    const int numSamples = 10;
    long sum = 0;
    for (int i = 0; i < numSamples; i++) {
      sum += analogRead(pin);
      delayMicroseconds(100);
    }

    if (useWifiWorkaround) {
      esp_wifi_start();
      delay(100);
      Serial.println("WiFi restarting (MQTT will reconnect automatically)...");
    }

    int tmp36_Value = sum / numSamples;
    float voltage = tmp36_Value * 3.3 / 4096.0;
    float tempC = ((voltage - 0.5) * 100) + calibrationOffset;

    if (tempC >= minValidTemp && tempC <= maxValidTemp) {
      sensorErrorCount = 0;
      lastValidTemp = tempC;
      currentTemp = tempC;

      // Save temperature to flash only during accurate calibration readings
      if (useWifiWorkaround) {
        saveTemperature(tempC, *prefsPtr);
      }

      if (sensorFailed) {
        sensorFailed = false;
        Serial.println("Temperature sensor recovered!");
      }

      return tempC;
    } else {
      if (useWifiWorkaround) {
        sensorErrorCount++;
        Serial.print("Warning: Invalid temperature reading: ");
        Serial.print(tempC);
        Serial.print(" °C (error #");
        Serial.print(sensorErrorCount);
        Serial.println(")");

        if (sensorErrorCount >= maxSensorErrors && !sensorFailed) {
          sensorFailed = true;
          Serial.println("ERROR: Temperature sensor failed! Entering failsafe mode.");
        }
      }

      currentTemp = lastValidTemp;
      return lastValidTemp;
    }
  }

  bool isFailed() const {
    return sensorFailed;
  }

  float getLastValid() const {
    return lastValidTemp;
  }

  void publishMQTT(EspMQTTClient& client, char* buffer) {
    if (client.isConnected()) {
      dtostrf(currentTemp > 0 ? currentTemp : lastValidTemp, 4, 2, buffer);
      client.publish(tempTopic, buffer, true);

      if (sensorFailed != lastPublishedFailedState) {
        client.publish(statusTopic, sensorFailed ? "failed" : "ok", true);
        lastPublishedFailedState = sensorFailed;
      }
    }
  }

  float loadTemperature(Preferences& prefs) {
    prefs.begin("fan-ctrl", true);  // read-only mode
    float savedTemp = prefs.getFloat("lastTemp", 20.0);  // default to 20°C if not found
    prefs.end();

    // Validate the loaded temperature
    if (savedTemp >= minValidTemp && savedTemp <= maxValidTemp) {
      return savedTemp;
    } else {
      return 20.0;  // Return default if saved value is out of range
    }
  }

  void saveTemperature(float temp, Preferences& prefs) {
    prefs.begin("fan-ctrl", false);  // read-write mode
    prefs.putFloat("lastTemp", temp);
    prefs.end();
  }
};

// ========== GLOBAL INSTANCES ==========
Fan frontFan = {32, 33, TOPIC_FRONT_RPM, TOPIC_FRONT_FAN_STALLED, 0, false, false};
Fan backFan = {25, 27, TOPIC_BACK_RPM, TOPIC_BACK_FAN_STALLED, 0, false, false};
DoorSensor frontDoor = {19, 21, TOPIC_FRONT_DOOR, false, false};
DoorSensor backDoor = {5, 18, TOPIC_BACK_DOOR, false, false};
TemperatureSensor tempSensor;
Preferences preferences;

// ========== GLOBAL STATE ==========
unsigned long _lastTachTime = 0;
unsigned long _lastTempCalibrationTime = 0;
unsigned long _lastMqttPublishTime = 0;
unsigned long _bootTime = 0;
unsigned long _mqttConnectedTime = 0;
int _fanSpeed = MIN_PWM_VALUE;

const unsigned long MQTT_STABILIZE_TIME = 10000;
const unsigned long TEMP_CALIBRATION_INTERVAL = 300000;  // 5 minutes
const unsigned long MQTT_PUBLISH_INTERVAL = 60000;  // 60 seconds
const int WATCHDOG_TIMEOUT_SECONDS = 30;

// ========== MESSAGE BUFFERS ==========
char _mqttBuffer[16];
char _serialBuffer[128];

// ========== INTERRUPT SERVICE ROUTINES ==========
void IRAM_ATTR _frontTachISR() {
  frontFan.pulseCount++;
}

void IRAM_ATTR _backTachISR() {
  backFan.pulseCount++;
}

// ========== SERIAL LOGGING ==========
void printStatus(bool frontDoor, bool backDoor, float temp, bool sensorFailed,
                 int pwm, int speedPercent, unsigned long targetRpm, unsigned long actualRpm) {
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
  _mqttConnectedTime = millis();

  // Publish online status and current state
  client.publish("fan_controller/status", "online", true);

  // Use struct methods to publish current state
  tempSensor.publishMQTT(client, _mqttBuffer);
  frontDoor.publishMQTT(client);
  backDoor.publishMQTT(client);
  frontFan.publishStallStatus(client);
  backFan.publishStallStatus(client);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Serial.println("Fan Controller Starting...");

  // Initialize hardware BEFORE WiFi to avoid conflicts with GPIO25
  tempSensor.initialize(TEMP_PIN, TEMP_CALIBRATION_OFFSET, MIN_VALID_TEMP, MAX_VALID_TEMP, MAX_SENSOR_ERRORS, preferences);
  tempSensor.tempTopic = TOPIC_TEMP;
  tempSensor.statusTopic = TOPIC_SENSOR_STATUS;

  frontFan.initialize(_frontTachISR, _fanSpeed);
  backFan.initialize(_backTachISR, _fanSpeed);

  // Reset pulse counters and set initial timing immediately after fan init
  noInterrupts();
  frontFan.resetPulseCount();
  backFan.resetPulseCount();
  interrupts();
  _lastTachTime = millis();
  _bootTime = millis();

  // Initialize watchdog timer
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  Serial.print("Watchdog timer enabled (");
  Serial.print(WATCHDOG_TIMEOUT_SECONDS);
  Serial.println("s timeout)");

  // Configure WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  // Enable MQTT features
  client.enableHTTPWebUpdater();
  client.enableLastWillMessage("fan_controller/status", "offline");

  frontDoor.initialize();
  backDoor.initialize();

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(100);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.println("Fan controller ready!");
  Serial.println("FD=Front Door, BD=Back Door, Temp=Temperature, PWM=Fan PWM, Spd=Speed %, Tgt=Target RPM, Act=Actual RPM");
}

// ========== MAIN LOOP ==========
void loop() {
  client.loop();

  // Update door sensors and publish changes immediately
  frontDoor.update();
  backDoor.update();
  frontDoor.publishMQTT(client);
  backDoor.publishMQTT(client);

  // Update fan speed every second
  unsigned long currentTime = millis();
  if (currentTime - _lastTachTime >= TACH_SAMPLE_TIME) {

    unsigned long frontRpm = frontFan.getRPM();
    unsigned long backRpm = backFan.getRPM();

    // Check if it's time for temperature calibration
    bool mqttStable = client.isConnected() && _mqttConnectedTime > 0 &&
                      (currentTime - _mqttConnectedTime) >= MQTT_STABILIZE_TIME;
    bool timeForCalibration = (currentTime - _lastTempCalibrationTime) >= TEMP_CALIBRATION_INTERVAL;
    bool useWifiWorkaround = mqttStable && timeForCalibration;

    if (useWifiWorkaround) {
      _lastTempCalibrationTime = currentTime;
    }

    tempSensor.read(useWifiWorkaround);

    // Calculate fan speed
    if (tempSensor.isFailed()) {
      _fanSpeed = MAX_PWM_VALUE;
    } else {
      _fanSpeed = map((long)tempSensor.currentTemp, MIN_SPEED_TEMP, MAX_SPEED_TEMP, MIN_PWM_VALUE, MAX_PWM_VALUE);
      _fanSpeed = constrain(_fanSpeed, MIN_PWM_VALUE, MAX_PWM_VALUE);
    }

    int speedPercent = map(_fanSpeed, MIN_PWM_VALUE, MAX_PWM_VALUE, MIN_FAN_SPEED_PERCENT, 100);
    unsigned long targetRPM = map(speedPercent, 0, 100, 0, FAN_MAX_RPM);

    frontFan.setSpeed(_fanSpeed);
    backFan.setSpeed(_fanSpeed);

    // Check for fan stall (after grace period on boot)
    bool gracePeriodExpired = (currentTime - _bootTime) > FAN_STALL_GRACE_PERIOD;
    frontFan.checkStall(gracePeriodExpired, speedPercent, MIN_FAN_SPEED_PERCENT, MIN_RPM_THRESHOLD);
    backFan.checkStall(gracePeriodExpired, speedPercent, MIN_FAN_SPEED_PERCENT, MIN_RPM_THRESHOLD);

    printStatus(frontDoor.isOpen, backDoor.isOpen, tempSensor.currentTemp, tempSensor.isFailed(),
                _fanSpeed, speedPercent, targetRPM, frontRpm);

    // Publish fan stall status changes immediately
    frontFan.publishStallStatus(client);
    backFan.publishStallStatus(client);

    // Publish to MQTT every 60 seconds
    bool timeToPublish = (currentTime - _lastMqttPublishTime) >= MQTT_PUBLISH_INTERVAL;

    if (client.isConnected() && timeToPublish) {
      tempSensor.publishMQTT(client, _mqttBuffer);
      frontFan.publishRPM(client, _mqttBuffer);
      backFan.publishRPM(client, _mqttBuffer);

      sprintf(_mqttBuffer, "%d", speedPercent);
      client.publish(TOPIC_FAN_SPEED, _mqttBuffer, true);

      _lastMqttPublishTime = currentTime;
    }

    noInterrupts();
    frontFan.resetPulseCount();
    backFan.resetPulseCount();
    interrupts();

    _lastTachTime = currentTime;
  }

  // Reset watchdog timer to prevent reboot
  esp_task_wdt_reset();

  delay(200);
}
