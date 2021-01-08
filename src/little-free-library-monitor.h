#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include <ArduinoJson.h>

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
    uint64_t        lastDebounceTime;
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
    uint64_t                timestamp = 0UL;
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
static const std::chrono::duration<uint64_t, std::milli> DEBOUNCE_DELAY_MILLIS = 500ms;

// How many open/close events to queue at most
static const size_t MAX_EVENT_QUEUE = 20;

static const double EMA_ALPHA = 0.80;


static LEDStatus doorOpenLED(RGB_COLOR_RED, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);
static LEDStatus doorClosedLED(RGB_COLOR_GREEN, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);
static LEDStatus sensorErrorLED(RGB_COLOR_ORANGE, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);
