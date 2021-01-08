#include "credentials.h"
#include "CircularBuffer.h"

// This #include statement was automatically added by the Particle IDE.
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include <ArduinoJson.h>

// This #include statement was automatically added by the Particle IDE.
#include <PublishManager.h>

// This #include statement was automatically added by the Particle IDE.
#include <SparkFunMAX17043.h>

// This #include statement was automatically added by the Particle IDE.
#include <MQTT.h>

#include "math.h"
#include <stdarg.h>

// Header Like Section
#define DOOR_OPEN   1
#define DOOR_CLOS   2
#define DOOR_UNKN   3

struct doorcontact_t {
    const uint8_t   nc_pin;
    const uint8_t   no_pin;
    uint8_t         state;
    uint8_t         stateLast;
    uint32_t        count;
    unsigned long   lastDebounceTime;
};

struct doorevent_t {
    bool        set = false;
    uint8_t     state;
    uint32_t    count;
};

struct batteryevent_t {
    bool    set = false;
    double  voltage;
    double  soc;
    bool    alert;
    bool    charging;
};

struct wifievent_t {
    bool    set = false;
    int8_t  rssi;
    float   strength;
    float   quality;
};

struct mqttevent_t {
    time_t                  timestamp = 0UL;
    struct doorevent_t      doorEvent; 
    struct batteryevent_t   batteryEvent;
    struct wifievent_t      wifiEvent;
};

bool mqttConnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool readDoorState(struct doorcontact_t* doorState);
uint8_t computeDoorState(uint8_t nc, uint8_t no);
bool shouldConnect();
void addDevice(DynamicJsonDocument* doc);
void publishDiscovery();
void updateBatteryStatus();
void sendBatteryStatus();
void updateWifiStatus();
void sendWifiStatus();
void updateDoorState();
void sendMqttEvents();
bool maybeSleep();
char* jsptf(const char * format, ...);
void publishJson(const char* topic, DynamicJsonDocument* doc, bool retain);


static const bool MQTT_TESTING              = false;     // Set to true to prefix ALL MQTT topics with TEST/
static const char* DEVICE_NAME              = "library_monitor-E"; // TODO get this from cloud and store in EEPROM
static const char* MQTT_DEVICE_NAME         = "little_free_library";
static const char* HA_FRIENDLY_NAME         = "Little Free Library";
static const char* HA_DEVICE_MODEL          = "photon";

// Build device description
static const char* MQTT_HA_DISCOVERY_TOPIC  = "homeassistant";
static const char* HA_BATTERY_VOLTAGE_ID    = "battery_voltage";
static const char* HA_BATTERY_SOC_ID        = "battery_soc";
static const char* HA_BATTERY_CHARGING_ID   = "battery_charging";
static const char* HA_BATTERY_LOW_ID        = "battery_low";
static const char* HA_DOOR_ID               = "door";
static const char* HA_WIFI_STRENGTH_ID      = "wifi_strength";
static const char* HA_WIFI_QUALITY_ID       = "wifi_quality";
static const char* HA_NEXT_WAKE_ID          = "next_wake";
static const char* HA_BATTERY_TOPIC         = "particle/ha/%s/battery";
static const char* HA_DOOR_TOPIC            = "particle/ha/%s/door";
static const char* HA_WIFI_TOPIC            = "particle/ha/%s/wifi";
static const char* HA_SLEEP_TOPIC           = "particle/ha/%s/sleep";
static const char* SLEEP_DELAY_TOPIC        = "particle/config/%s";


static const int BATTERY_UPDATE_PCT_THRESHOLD       = 20;               // 20 == 00.20% point change
static const std::chrono::duration<int, std::milli> EVENT_RATE_LIMIT_MILLIS = 250ms;
static const std::chrono::duration<int> SLEEP_DELAY         = 5s;       // How long to wait after an event to sleep
static const int SHORT_SLEEP_SOC_THRESHOLD                  = 70;       // at 70% start increasing
static const std::chrono::duration<int> SHORT_SLEEP_DURATION= 5min;     // How long to sleep when SOC is above SHORT_SLEEP_SOC_THRESHOLD
static const std::chrono::duration<int> LONG_SLEEP_DURATION = 60min;    // How long to sleep
static const std::chrono::duration<int> CONNECT_INTERVAL    = 60s;      // How long to wait before ensuring MQTT server is connected
static const std::chrono::duration<int> EVENT_REFRESH_INTRV = 5min;     // Repeat event if it hasn't happened in this time

// How long to wait for the open/close relay to stabilze before reporting
static const std::chrono::duration<int, std::milli> DEBOUNCE_DELAY_MILLIS = 500ms;

// How many open/close events to queue at most
static const size_t MAX_EVENT_QUEUE = 20;

static const double EMA_ALPHA = 0.80;


LEDStatus doorOpenLED(RGB_COLOR_RED, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);
LEDStatus doorClosedLED(RGB_COLOR_GREEN, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);
LEDStatus sensorErrorLED(RGB_COLOR_ORANGE, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);


// Device Config
SYSTEM_THREAD(ENABLED);
STARTUP(WiFi.selectAntenna(ANT_AUTO));
STARTUP(Particle.publishVitals(LONG_SLEEP_DURATION));


// MQTT Client Setup
MQTT mqttClient(MQTT_SERVER, MQTT_PORT, mqttCallback, 512);
bool mqttDiscoveryPublished = false;


// Setup logging
SerialLogHandler logHandler;

// Particle Variables
double voltage = 0; // LiPo voltage
double soc = 0; //  LiPo state-of-charge (SOC)
bool alert; // True of SOC alert is triggered
bool doorClosed; // True if door closed

char configTopic[128];

// local state
unsigned long lastWifiEvent = 0UL;
unsigned long lastBatteryEvent = 0UL;
double lastEventSoc = 0;
double lastEventVoltage = 0;
unsigned long lastDoorEvent = 0UL;
unsigned long lastWake = 0UL;
unsigned long lastEvent = 0UL;
unsigned long lastConnect = 0UL;
unsigned long sleepDelayOverride = 0UL;
SystemSleepWakeupReason wakeReason = SystemSleepWakeupReason::BY_BLE; // using BLE as photon has no BLE so should never happen
doorcontact_t doorState = { D2, D3, DOOR_UNKN, DOOR_UNKN, 0, 0UL };
CircularBuffer<mqttevent_t> eventQueue(MAX_EVENT_QUEUE);

char formatBuffer[128];
char* jsptf(const char * format, ...) {
    va_list va;
    va_start (va, format);
    vsnprintf(formatBuffer, sizeof(formatBuffer), format, va);
    va_end (va);

    return formatBuffer;
}

// Global variable to hold the watchdog object pointer
ApplicationWatchdog *wd;
PublishManager<15> publishManager;

void watchdogHandler() {
  // In 2.0.0 and later, RESET_NO_WAIT prevents notifying the cloud of a pending reset
  System.reset(RESET_NO_WAIT);
}

void setup() {
    snprintf(configTopic, sizeof(configTopic), SLEEP_DELAY_TOPIC, MQTT_DEVICE_NAME);

    // After 60s of no loop completion reset the device
    wd = new ApplicationWatchdog(60s, watchdogHandler);

    Serial.begin(9600);

    Particle.variable("voltage", voltage);
    Particle.variable("soc", soc);
    Particle.variable("alert", alert);
    Particle.variable("doorClosed", doorClosed);
    Particle.variable("sleepDelayOverride", sleepDelayOverride);

    // Init door sensor pins
    pinMode(doorState.nc_pin, INPUT_PULLDOWN);
    pinMode(doorState.no_pin, INPUT_PULLDOWN);

    // Init "keep awake" pin
    pinMode(D7, INPUT_PULLDOWN);


    // Set up the MAX17043 LiPo fuel gauge:
    lipo.begin(); // Initialize the MAX17043 LiPo fuel gauge

    // Quick start restarts the MAX17043 in hopes of getting a more accurate
    // guess for the SOC.
    lipo.quickStart();

    // We can set an interrupt to alert when the battery SoC gets too low.
    // We can alert at anywhere between 1% - 32%:
    lipo.setThreshold(5);

    delay(2000);
    
    waitFor(Time.isValid, 30000);
    
    lastWake = Time.now();
}

void loop() {
    publishManager.process();
    updateDoorState();
    
    if (shouldConnect()) {
        // Ensure we connect to the MQTT server periodically to check for updates
        mqttConnect();
    } else {
        // Always keep mqtt connected IF already connected
        mqttClient.loop();
    }

    publishDiscovery();

    updateBatteryStatus();
    updateWifiStatus();
    sendMqttEvents();

    if (!maybeSleep()) {
        delay(7);
    }
}

bool shouldConnect() {
    return Time.now() > (lastConnect + CONNECT_INTERVAL.count());
}

/**
 * Called by MQTT library when a subscribed topic is updated.
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;

    publishManager.publish("mqtt/callback", topic);
    Log.info("MQTT: %s\n%s", topic, p);

    // TODO cache this in a global?    
    if (strcmp(configTopic, topic) == 0) {
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, payload);

        // Test if parsing succeeds.
        if (error) {
            Log.info("deserializeJson() failed: %s", error.c_str());
            // treat parse failure as an empty doc and still update config
        }

        sleepDelayOverride = doc["sleep_delay"];
        Log.info("Sleep Delay Override: %d", sleepDelayOverride);
    }
}

/**
 * Call loop, if that fails attempt to connect to MQTT server.
 * 
 * @return true if MQTT server connected, false if not.
 */
bool mqttConnect() {
    // Call loop, return if successful
    if (mqttClient.loop()) {
        lastConnect = Time.now();
        return true;
    }

    // Short circuit if there is no cloud connection
    if (!Particle.connected()) {
        return false;
    }

    // connect to the server
    Log.info("MQTT: Start Connect");
    mqttClient.connect(DEVICE_NAME + System.deviceID(), MQTT_USERNAME, MQTT_PASSWORD);
    if (!mqttClient.isConnected()) {
        // connection failed
        // TODO need to rate limit these in case of a bad connection
        publishManager.publish("mqtt/log", "connection failed");
        Log.error("MQTT: Connect Failed");
        return false;
    }

    mqttClient.subscribe(configTopic);
    Log.info("MQTT: Subscribed - %s", configTopic);

    lastConnect = Time.now();
    publishManager.publish("mqtt/connection", "established");
    Log.info("MQTT: Connected");
    return true;
}

void addDevice(DynamicJsonDocument* doc) {
    JsonObject device = (*doc).createNestedObject("device");
    device["name"] = HA_FRIENDLY_NAME;
    device["model"] = HA_DEVICE_MODEL;
    device["manufacturer"] = "edalquist";
    JsonArray identifiers = device.createNestedArray("identifiers");
    identifiers.add(DEVICE_NAME);
    identifiers.add(System.deviceID());
}

/**
 * Publish HA MQTT discovery docs
 */
void publishDiscovery() {
    // TODO add a way to reset mqttDiscoveryPublished remotely
    if (mqttDiscoveryPublished) {
        return;
    }
    if (!mqttConnect()) {
        return;
    }
    
    auto expiration = (LONG_SLEEP_DURATION * 4).count();

    DynamicJsonDocument doc(512);

    // Build device descriptions
    doc.clear();
    addDevice(&doc);
    doc["name"] = jsptf("%s Battery V", HA_FRIENDLY_NAME);
    doc["unique_id"] = jsptf("%s_%s", DEVICE_NAME, HA_BATTERY_VOLTAGE_ID);
    doc["device_class"] = "voltage";
    doc["unit_of_measurement"] = "V";
    doc["state_topic"] = jsptf(HA_BATTERY_TOPIC, MQTT_DEVICE_NAME);
    doc["value_template"] = "{{ value_json.voltage }}";
    doc["expire_after"] = expiration;
    publishJson(jsptf("%s/sensor/%s/%s/config", MQTT_HA_DISCOVERY_TOPIC, MQTT_DEVICE_NAME, HA_BATTERY_VOLTAGE_ID), &doc, true);

    doc.clear();
    addDevice(&doc);
    doc["name"] = jsptf("%s Battery %%", HA_FRIENDLY_NAME);
    doc["unique_id"] = jsptf("%s_%s", DEVICE_NAME, HA_BATTERY_SOC_ID);
    doc["device_class"] = "battery";
    doc["unit_of_measurement"] = "%";
    doc["state_topic"] = jsptf(HA_BATTERY_TOPIC, MQTT_DEVICE_NAME);
    doc["value_template"] = "{{ value_json.soc }}";
    doc["expire_after"] = expiration;
    publishJson(jsptf("%s/sensor/%s/%s/config", MQTT_HA_DISCOVERY_TOPIC, MQTT_DEVICE_NAME, HA_BATTERY_SOC_ID), &doc, true);

    doc.clear();
    addDevice(&doc);
    doc["name"] = jsptf("%s Battery Charging", HA_FRIENDLY_NAME);
    doc["unique_id"] = jsptf("%s_%s", DEVICE_NAME, HA_BATTERY_CHARGING_ID);
    doc["device_class"] = "battery_charging";
    doc["state_topic"] = jsptf(HA_BATTERY_TOPIC, MQTT_DEVICE_NAME);
    doc["value_template"] = "{{ value_json.charging }}";
    doc["expire_after"] = expiration;
    publishJson(jsptf("%s/binary_sensor/%s/%s/config", MQTT_HA_DISCOVERY_TOPIC, MQTT_DEVICE_NAME, HA_BATTERY_CHARGING_ID), &doc, true);

    doc.clear();
    addDevice(&doc);
    doc["name"] = jsptf("%s Battery Low", HA_FRIENDLY_NAME);
    doc["unique_id"] = jsptf("%s_%s", DEVICE_NAME, HA_BATTERY_LOW_ID);
    doc["device_class"] = "battery";
    doc["state_topic"] = jsptf(HA_BATTERY_TOPIC, MQTT_DEVICE_NAME);
    doc["value_template"] = "{{ value_json.alert }}";
    doc["expire_after"] = expiration;
    publishJson(jsptf("%s/binary_sensor/%s/%s/config", MQTT_HA_DISCOVERY_TOPIC, MQTT_DEVICE_NAME, HA_BATTERY_LOW_ID), &doc, true);

    doc.clear();
    addDevice(&doc);
    doc["name"] = jsptf("%s Door", HA_FRIENDLY_NAME);
    doc["unique_id"] = jsptf("%s_%s", DEVICE_NAME, HA_DOOR_ID);
    doc["device_class"] = "door";
    doc["state_topic"] = jsptf(HA_DOOR_TOPIC, MQTT_DEVICE_NAME);
    doc["value_template"] = "{{ value_json.state }}";
    doc["expire_after"] = expiration;
    publishJson(jsptf("%s/binary_sensor/%s/%s/config", MQTT_HA_DISCOVERY_TOPIC, MQTT_DEVICE_NAME, HA_DOOR_ID), &doc, true);

    doc.clear();
    addDevice(&doc);
    doc["name"] = jsptf("%s WiFi Strength", HA_FRIENDLY_NAME);
    doc["unique_id"] = jsptf("%s_%s", DEVICE_NAME, HA_WIFI_STRENGTH_ID);
    doc["device_class"] = "signal_strength";
    doc["unit_of_measurement"] = "dBm";
    doc["state_topic"] = jsptf(HA_WIFI_TOPIC, MQTT_DEVICE_NAME);
    doc["value_template"] = "{{ value_json.strength }}";
    doc["expire_after"] = expiration;
    publishJson(jsptf("%s/sensor/%s/%s/config", MQTT_HA_DISCOVERY_TOPIC, MQTT_DEVICE_NAME, HA_WIFI_STRENGTH_ID), &doc, true);

    doc.clear();
    addDevice(&doc);
    doc["name"] = jsptf("%s WiFi Quality", HA_FRIENDLY_NAME);
    doc["unique_id"] = jsptf("%s_%s", DEVICE_NAME, HA_WIFI_QUALITY_ID);
    doc["device_class"] = "signal_strength";
    doc["unit_of_measurement"] = "dBm";
    doc["state_topic"] = jsptf(HA_WIFI_TOPIC, MQTT_DEVICE_NAME);
    doc["value_template"] = "{{ value_json.quality }}";
    doc["expire_after"] = expiration;
    publishJson(jsptf("%s/sensor/%s/%s/config", MQTT_HA_DISCOVERY_TOPIC, MQTT_DEVICE_NAME, HA_WIFI_QUALITY_ID), &doc, true);

    doc.clear();
    addDevice(&doc);
    doc["name"] = jsptf("%s Next Wake", HA_FRIENDLY_NAME);
    doc["unique_id"] = jsptf("%s_%s", DEVICE_NAME, HA_NEXT_WAKE_ID);
    doc["device_class"] = "timestamp";
    doc["unit_of_measurement"] = "ms";
    doc["state_topic"] = jsptf(HA_SLEEP_TOPIC, MQTT_DEVICE_NAME);
    doc["value_template"] = "{{ value_json.next_wake | timestamp_local }}";
    doc["expire_after"] = expiration;
    publishJson(jsptf("%s/sensor/%s/%s/config", MQTT_HA_DISCOVERY_TOPIC, MQTT_DEVICE_NAME, HA_NEXT_WAKE_ID), &doc, true);
    
    // TODO add last update timestamp topic

    mqttDiscoveryPublished = true;
    Log.info("MQTT: Published Discovery");
}

void updateDoorState() {
    // Get door state, update status LED
    if (!readDoorState(&doorState) && Time.now() <= (lastDoorEvent + EVENT_REFRESH_INTRV.count())) {
        return;
    }

    mqttevent_t mqttEvent;
    mqttEvent.timestamp = Time.now();
    mqttEvent.doorEvent = { true, doorState.state, doorState.count };
    eventQueue.put(mqttEvent);
    lastDoorEvent = mqttEvent.timestamp;

    if (doorState.state == DOOR_OPEN) {
        doorOpenLED.setActive(true);
        doorClosed = false;
    } else if (doorState.state == DOOR_CLOS) {
        doorClosedLED.setActive(true);
        doorClosed = true;
    } else {
        sensorErrorLED.setActive(true);
    }
}

bool readDoorState(struct doorcontact_t* doorState) {
    const unsigned long millisRef = millis();

    uint8_t nc = digitalRead((*doorState).nc_pin);
    uint8_t no = digitalRead((*doorState).no_pin);
    uint8_t state = computeDoorState(nc, no);

    // On first iteration force a state update
    if ((*doorState).lastDebounceTime == 0UL) {
        (*doorState).state = state;
        (*doorState).lastDebounceTime = millisRef;
        return true;
    }

    // Nothing changed, short-circuit
    if ((*doorState).state == state && (*doorState).stateLast == state) {
        return false;
    }

    if (state != (*doorState).stateLast) {
        // new state is different than last read, reset debounce timer
        (*doorState).lastDebounceTime = millisRef;
    } else if ((millisRef - (*doorState).lastDebounceTime) > DEBOUNCE_DELAY_MILLIS.count()) {
        // Been longer than our debounce delay, update stored state
        (*doorState).state = state;
        (*doorState).count++; // TODO increment persistent counter
        return true;
    }

    (*doorState).stateLast = state;

    return false;
}

uint8_t computeDoorState(uint8_t nc, uint8_t no) {
    if (nc == no) {
        Log.info("nc: %d", nc);
        Log.info("no: %d", no);
        return DOOR_UNKN;
    } else if (nc == LOW) {
        return DOOR_OPEN;
    } else if (no == LOW) {
        return DOOR_CLOS;
    } else {
        Log.info("nc: %d", nc);
        Log.info("no: %d", no);
        return DOOR_UNKN;
    }
}

bool charging = false;
void updateBatteryStatus() {
    // voltage = voltageAvg.add(lipo.getVoltage());
    voltage = EMA_ALPHA * lipo.getVoltage() + (1 - EMA_ALPHA) * voltage; // lipo.getVoltage() returns a voltage value (e.g. 3.93)
    soc = lipo.getSOC(); // lipo.getSOC() returns the estimated state of charge (e.g. 79%)
    alert = lipo.getAlert(); // lipo.getAlert() returns a 0 or 1 (0=alert not triggered)

    // Clear alerts when soc is back above 15%
    if (alert && soc > 15) {
        lipo.clearAlert();
    }

    // Send if SOC changes by threshold OR if lastBatteryEvent is outside of refresh interval
    if (abs((lastEventSoc - soc) * 100) > BATTERY_UPDATE_PCT_THRESHOLD || Time.now() > (lastBatteryEvent + EVENT_REFRESH_INTRV.count())) {
        if (lastEventSoc != 0 && soc > lastEventSoc) {
            charging = true;
        } else if (lastEventSoc == 0 || soc < lastEventSoc) {
            charging = false;
        }

        lastEventSoc = soc;
        lastEventVoltage = voltage;
    
        mqttevent_t mqttEvent;
        mqttEvent.timestamp = Time.now();
        mqttEvent.batteryEvent = { true, voltage, soc, alert, charging };
        eventQueue.put(mqttEvent);
        lastBatteryEvent = mqttEvent.timestamp;
    }
}

void updateWifiStatus() {
    // Short circuit if there is no cloud connection OR not within refresh interval
    if (!Particle.connected() || Time.now() <= (lastWifiEvent + EVENT_REFRESH_INTRV.count())) {
        return;
    }

    WiFiSignal sig = WiFi.RSSI();
    
    mqttevent_t mqttEvent;
    mqttEvent.timestamp = Time.now();
    mqttEvent.wifiEvent = { true, (int8_t) WiFi.RSSI(),  sig.getStrengthValue(), sig.getQualityValue() };
    eventQueue.put(mqttEvent);
    lastWifiEvent = mqttEvent.timestamp;
}

time_t lastEventTimestamp = 0UL;
void sendMqttEvents() {
    // Short-circuit if queue empty, no mqtt connect, or rate limited
    if (eventQueue.empty() || !mqttConnect() || System.millis() <= (lastEvent + EVENT_RATE_LIMIT_MILLIS.count())) {
        return;
    }

    // Don't send next event until the same amount of time has passed as between the original events OR 10s has passed as a fail-safe
    if (lastEvent > 0
            && lastEventTimestamp > 0
            && !(System.millis() <= (lastEvent + 10000))
            && (System.millis() - lastEvent) / 1000 > eventQueue.peek().timestamp - lastEventTimestamp) {
        return;
    }
    Log.info("Event Queue: %d", eventQueue.size());
    mqttevent_t nextEvent = eventQueue.get();

    if (nextEvent.doorEvent.set) {
        // Get door state and compose state string
        auto doorStateName = "UNKNOWN";
        if (nextEvent.doorEvent.state == DOOR_OPEN) {
            doorStateName = "ON";
        } else if (nextEvent.doorEvent.state == DOOR_CLOS) {
            doorStateName = "OFF";
        }

        DynamicJsonDocument doc(256);
        doc["state"] = doorStateName;
        doc["count"] = nextEvent.doorEvent.count;
        publishJson(jsptf(HA_DOOR_TOPIC, MQTT_DEVICE_NAME), &doc, true);
    } else if (nextEvent.batteryEvent.set) {
        DynamicJsonDocument doc(256);
        doc["voltage"] = nextEvent.batteryEvent.voltage;
        doc["soc"] = nextEvent.batteryEvent.soc;
        doc["alert"] = nextEvent.batteryEvent.alert ? "ON" : "OFF";
        doc["charging"] = nextEvent.batteryEvent.charging ? "ON" : "OFF";
        publishJson(jsptf(HA_BATTERY_TOPIC, MQTT_DEVICE_NAME), &doc, true);
    } else if (nextEvent.wifiEvent.set) {
        DynamicJsonDocument doc(256);
        doc["rssi"] = nextEvent.wifiEvent.rssi;
        doc["strength"] = nextEvent.wifiEvent.strength;
        doc["quality"] = nextEvent.wifiEvent.quality;
        publishJson(jsptf(HA_WIFI_TOPIC, MQTT_DEVICE_NAME), &doc, true);
    } else {
        Log.error("UNKNOWN MQTT EVENT");
    }

    lastEvent = System.millis();
    
    // Capture previous event timestamp and then zero out global var to signal we need a new event
    lastEventTimestamp = nextEvent.timestamp;
    nextEvent.timestamp = 0UL;
}

bool emptyPublishQueue() {
    publishManager.process();
    return publishManager.cacheSize() <= 0;
}

bool maybeSleep() {
    // Log previous wake reason
    if (Serial.isConnected() && wakeReason != SystemSleepWakeupReason::BY_BLE) {
        // TODO do we need special wakeup handling logic if it was door contact or RTC?
        if (wakeReason == SystemSleepWakeupReason::BY_GPIO) {
            Log.info("wake::GPIO");
        } else if (wakeReason == SystemSleepWakeupReason::BY_RTC) {
            Log.info("wake::RTC");
        } else {
            Log.info("wake::%d", wakeReason);
        }
        wakeReason = SystemSleepWakeupReason::BY_BLE;
    }

    auto now = Time.now();
    if (digitalRead(D7) != HIGH // D7 is the force-awake pin
            && eventQueue.empty() // Nothing in the event queue
            && !shouldConnect() // No pending MQTT connection
            && emptyPublishQueue() // publish queue is empty
            && now > (doorState.lastDebounceTime + SLEEP_DELAY.count()) // at least N since last state change
            && now > (lastWake + SLEEP_DELAY.count() + sleepDelayOverride)) { // at least N since last wakeup
        
        // reset sleep delay override, it will get re-read if still on the MQTT server after wakeup
        sleepDelayOverride = 0UL;
        
        // If soc is below SHORT_SLEEP_SOC_THRESHOLD start lengthing sleep duration with a max of LONG_SLEEP_DURATION
        std::chrono::duration<int> sleepDuration = SHORT_SLEEP_DURATION;
        if (soc < SHORT_SLEEP_SOC_THRESHOLD) {
            auto sleepRatio = (LONG_SLEEP_DURATION - SHORT_SLEEP_DURATION) * (1 - (soc / SHORT_SLEEP_SOC_THRESHOLD));
            sleepDuration = std::chrono::duration_cast<std::chrono::seconds>(SHORT_SLEEP_DURATION + sleepRatio);
        }
        
        auto nextWake = Time.now() + sleepDuration.count();
        auto nextWakeStr = Time.format(nextWake, TIME_FORMAT_ISO8601_FULL);
        
        // Publish sleep data
        DynamicJsonDocument doc(256);
        doc["next_wake"] = nextWake;
        doc["next_wake_str"] = nextWakeStr.c_str();
        publishJson(jsptf(HA_SLEEP_TOPIC, MQTT_DEVICE_NAME), &doc, true);

        // Publish sleep event and wait for it to be published
        String sleepMsg = String::format("until %s (%ds), at %s, after %ds", 
                nextWakeStr.c_str(),
                sleepDuration,
                Time.format(Time.now(), TIME_FORMAT_ISO8601_FULL).c_str(),
                now - lastWake);
        Log.info("sleep::STOP %s", sleepMsg);
        Particle.publish("sleep::STOP", sleepMsg);

        SystemSleepConfiguration config;
        config
            .mode(SystemSleepMode::STOP)
            .gpio(D2, RISING)
            .gpio(D3, RISING)
            .gpio(D7, RISING)
            .duration(sleepDuration);
        SystemSleepResult result = System.sleep(config);

        // SLEEPING

        wakeReason = result.wakeupReason();
        lastWake = Time.now();
        
        // For RTC wakeups force battery and wifi updates
        if (wakeReason == SystemSleepWakeupReason::BY_RTC) {
            lastBatteryEvent = 0UL;
            lastWifiEvent = 0UL;
        }
        
        return true;
    }

    return false;
}

/**
 * Publish JSON to MQTT
 */
void publishJson(const char* topic, DynamicJsonDocument* doc, bool retain) {
    String formattedTopic;
    if (MQTT_TESTING) {
        formattedTopic = String::format("TEST/%s", topic);
    } else {
        formattedTopic = String(topic);
    }

    char output[measureJson(*doc) + 1];
    serializeJson(*doc, output, sizeof(output));

    Log.info("MQTT: %s\t%s", formattedTopic, output);
    mqttClient.publish(formattedTopic, output, retain);
    publishManager.publish("mqtt/publishJson", formattedTopic);
}
