#include <Arduino.h>
#include <RadioLib.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "kinetic_helpers.h"

// Enable polling the devices
#define ENABLE_POLLING
// Polling interval in seconds (multiplied by number of devices for per-device poll interval)
#define POLL_INTERVAL_S 45

//#define DEBUG_SERIAL

// Radio Config
#define PACKET_LENGTH 12 // bytes
#define MIN_RSSI -85 // dBm
#define CARRIER_FREQUENCY 433.3 // MHz
#define BIT_RATE 100.0 // kbps
#define FREQUENCY_DEVIATION 50.0 // kHz
#define RX_BANDWIDTH 135.0 // kHz
#define OUTPUT_POWER 10 // dBm
#define PREAMBLE_LENGTH 32 // bits 

// IotWebConf Config
#define CONFIG_PARAM_MAX_LEN 128
#define CONFIG_VERSION "mqt4"
#define CONFIG_PIN 2 // D4
#define STATUS_PIN 16 // D0
#define MAX_DEVICE_IDS 32
#define DEVICE_ID_LENGTH 9  // 8 hex chars + null terminator

#define DEBOUNCE_MILLIS 300

// CC1101 pins (adjust as needed)
#define CS_PIN 15
#define GDO0_PIN 5
#define GDO2_PIN 4

// CC1101 module
CC1101 radio = new Module(CS_PIN, GDO0_PIN, RADIOLIB_NC, GDO2_PIN);

// Set up IotWebConf
const char deviceName[] = "kinetic2mqtt";
const char apPassword[] = "EMWhP56Q"; // Default password for SSID and configuration page, must be changed after first boot

void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

// Storage for device IDs
char deviceIDValues[MAX_DEVICE_IDS][DEVICE_ID_LENGTH] = {0};

DNSServer dnsServer;
WebServer server(80);
WiFiClient net;
PubSubClient mqttClient(net);

char mqttServerValue[CONFIG_PARAM_MAX_LEN];
char mqttUserNameValue[CONFIG_PARAM_MAX_LEN];
char mqttUserPasswordValue[CONFIG_PARAM_MAX_LEN];
char mqttTopicValue[CONFIG_PARAM_MAX_LEN];

IotWebConf iotWebConf(deviceName, &dnsServer, &server, apPassword, CONFIG_VERSION);
IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfParameterGroup deviceIdGroup = IotWebConfParameterGroup("deviceids", "Receiver Device IDs");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, CONFIG_PARAM_MAX_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, CONFIG_PARAM_MAX_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, CONFIG_PARAM_MAX_LEN);
IotWebConfTextParameter mqttTopicParam = IotWebConfTextParameter("MQTT Topic", "mqttTopic", mqttTopicValue, CONFIG_PARAM_MAX_LEN);

bool needReset = false;

// Radio TX/RX state management
int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false;
volatile bool txInProgress = false;
volatile bool receivedFlag = false;
volatile bool rxInProgress = false;

// Create device ID parameters array
IotWebConfTextParameter* deviceIDParams[MAX_DEVICE_IDS];

// parametrically initialise device ID parameters
void initializeDeviceIDParams() {
  for (int i = 0; i < MAX_DEVICE_IDS; i++) {
    char* paramId = new char[16];
    char* paramLabel = new char[32];
    
    sprintf(paramId, "devId%d", i);
    sprintf(paramLabel, "Device ID %d", i + 1);
    
    deviceIDParams[i] = new IotWebConfTextParameter(
      paramLabel,
      paramId,
      deviceIDValues[i],
      DEVICE_ID_LENGTH
    );
  }
}

// Global State Variables
unsigned long lastPollTime = 0;
uint8_t lastPolledIndex = 0;
bool mqttJustConnected = false;

// Device state tracking
struct DeviceState {
  uint32_t deviceID;
  uint8_t state;
  int rssi;
  unsigned long lastUpdate;
  unsigned long lastMessageTime;
};

DeviceState deviceStates[MAX_DEVICE_IDS];

// MQTT Callback for handling incoming messages
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  #ifdef DEBUG_SERIAL
  Serial.print("MQTT message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  #endif

  // Extract device ID from topic (format: kinetic/set/DEVICEID)
  String topicStr(topic);
  int lastSlash = topicStr.lastIndexOf('/');
  String deviceIDStr = topicStr.substring(lastSlash + 1);
  
  // Parse JSON payload
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    #ifdef DEBUG_SERIAL
    Serial.println("JSON parse error");
    #endif
    return;
  }
  
  if (!doc.containsKey("state")) {
    #ifdef DEBUG_SERIAL
    Serial.println("Missing 'state' field");
    #endif
    return;
  }
  
  uint32_t deviceID = strtoul(deviceIDStr.c_str(), NULL, 16);
  uint8_t newState = doc["state"];
  bool desiredState = (newState == 1);
  
  #ifdef DEBUG_SERIAL
  Serial.print("Setting device ");
  Serial.print(deviceID, HEX);
  Serial.print(" to state ");
  Serial.println(desiredState);
  #endif
  
  // Transmit command
  rxInProgress = false;
  radio.finishReceive();
  
  uint8_t txBuf[12];
  encodeForTransmission(txBuf, deviceID, SET, desiredState);
  
  txInProgress = true;
  transmissionState = radio.startTransmit(txBuf, PACKET_LENGTH);
  if (transmissionState != RADIOLIB_ERR_NONE) {
    txInProgress = false;
    #ifdef DEBUG_SERIAL
    Serial.print("TX failed, code ");
    Serial.println(transmissionState);
    #endif
  }
}


void setup() {
  #ifdef DEBUG_SERIAL
  Serial.begin(115200);
  #endif
  pinMode(CONFIG_PIN, INPUT_PULLUP);

  // Initialise and configure CC1101
  #ifdef DEBUG_SERIAL
  Serial.print("Initializing Radio... ");
  #endif

  int state = radio.begin(CARRIER_FREQUENCY, BIT_RATE, FREQUENCY_DEVIATION, RX_BANDWIDTH, OUTPUT_POWER, PREAMBLE_LENGTH);
  radio.setCrcFiltering(false);
  radio.fixedPacketLengthMode(PACKET_LENGTH);
  radio.setPreambleLength(PREAMBLE_LENGTH, 0);
  radio.setPacketSentAction(setTxFlag);
  radio.setPacketReceivedAction(setRxFlag);

  // Sync word: only 2 bytes supported, bitshift 0x21a4 to the right so we can add the extra 0 in
  uint8_t syncWord[] = {0x10, 0xd2};
  radio.setSyncWord(syncWord, 2);

  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    #ifdef DEBUG_SERIAL
    Serial.println("done!");
    #endif
    rxInProgress = true;
  } else {
    #ifdef DEBUG_SERIAL
    Serial.print("failed, code ");
    Serial.println(state);
    #endif
    while (true);
  }

  // Initialise IOTWebConf
  #ifdef DEBUG_SERIAL
  Serial.println("Initialising IotWebConf... ");
  #endif

  // Initialize device ID parameters
  initializeDeviceIDParams();
  
  // Add all device ID parameters to the group
  for (int i = 0; i < MAX_DEVICE_IDS; i++) {
    deviceIdGroup.addItem(deviceIDParams[i]);
  }
  
  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttUserNameParam);
  mqttGroup.addItem(&mqttUserPasswordParam);
  mqttGroup.addItem(&mqttTopicParam);

  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.addParameterGroup(&mqttGroup);
  iotWebConf.addParameterGroup(&deviceIdGroup);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);

  bool validConfig = iotWebConf.init();
  if (!validConfig) {
    mqttServerValue[0] = '\0';
    mqttUserNameValue[0] = '\0';
    mqttUserPasswordValue[0] = '\0';
    mqttTopicValue[0] = '\0';
  }

  // This will disable the device sitting in AP mode for 30s on startup
  // Not required due to presence of reset button to manually enable AP mode
  //iotWebConf.setApTimeoutMs(0);

  server.on("/", []{ iotWebConf.handleConfig(); });
  server.onNotFound([](){ iotWebConf.handleNotFound(); });

  // Initialise MQTT
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setServer(mqttServerValue, 1883);

  lastPollTime = millis();
}

// TX/RX interrupt handlers
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setTxFlag(void) {
  if(txInProgress)
    transmittedFlag = true;
}

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setRxFlag(void) {
  if(rxInProgress)
    receivedFlag = true;
}

void loop() {
  iotWebConf.doLoop();

  if (needReset) {
    #ifdef DEBUG_SERIAL
    Serial.println("Rebooting after 1 second.");
    #endif
    iotWebConf.delay(1000);
    ESP.restart();
  }

  // If WiFi is connected but MQTT is not, establish MQTT connection
  if ((iotWebConf.getState() == iotwebconf::OnLine) && !mqttClient.connected()) {
    connectMqtt();
  }
  mqttClient.loop();

  // Poll all devices after we connect to mqtt
  if (mqttJustConnected && rxInProgress) {
    int deviceCount = getFilledDeviceIDCount();
    if (millis()- lastPollTime > 250) {
      // Put a short interval between the initial polls so the rest of the code can keep up, otherwise we only catch the final poll
      if (deviceCount > 0 && lastPolledIndex < deviceCount) {
        uint32_t deviceID = getDeviceID(lastPolledIndex);
        if (deviceID != 0) {
          pollDevice(deviceID);
        }
        lastPolledIndex++;
        lastPollTime = millis();
      } else {
        // Finished polling all devices
        mqttJustConnected = false;
        lastPolledIndex = 0;
      }
    }
  }

  // Poll devices periodically (technically unnecessary as all devices send a message when switched, but some situations these messages can be missed)
  #ifdef ENABLE_POLLING
  if(millis() - lastPollTime >= (POLL_INTERVAL_S * 1000)) {
    int deviceCount = getFilledDeviceIDCount();
    if (deviceCount > 0) {
      uint32_t deviceID = getDeviceID(lastPolledIndex);
      if (deviceID != 0) {
        pollDevice(deviceID);
      }
      lastPolledIndex = (lastPolledIndex + 1) % deviceCount;
    }
    lastPollTime = millis();
  }
  #endif

  // Handle transmission completion
  if(transmittedFlag) {
    transmittedFlag = false;
    
    if(transmissionState == RADIOLIB_ERR_NONE) {
      #ifdef DEBUG_SERIAL
      Serial.println("TX success!");
      #endif
    } else {
      #ifdef DEBUG_SERIAL
      Serial.print("TX failed, code ");
      Serial.println(transmissionState);
      #endif
    }
    
    int err = radio.finishTransmit();
    if(err != RADIOLIB_ERR_NONE) {
      #ifdef DEBUG_SERIAL
      Serial.print("TX cleanup failed, code ");
      Serial.println(err);
      #endif
    }
    
    txInProgress = false;
    rxInProgress = true;
    radio.startReceive();
  }

  // Handle received messages
  if(receivedFlag) {
    receivedFlag = false;
    rxInProgress = false;
    
    byte byteArr[PACKET_LENGTH];
    int state = radio.readData(byteArr, PACKET_LENGTH);
    
    if (state == RADIOLIB_ERR_NONE) {
      if (radio.getRSSI() > MIN_RSSI) {
        // Decode the transmission
        uint8_t dataBuf[10];
        unshiftTransmission(dataBuf, byteArr);

        #ifdef DEBUG_SERIAL
        char data[41] = "";
        bytesToHexString(dataBuf, 10, data);
        Serial.println(data);
        #endif
        
        struct kineticMessage *msg;
        msg = decodeTransmission(dataBuf);
        
        if (msg->devType != UNKNOWN) {
          handleReceivedMessage(msg);
        } else {
          #ifdef DEBUG_SERIAL
          Serial.print(msg->devType);

          Serial.println("Decoded message is invalid or unknown type");
          #endif
        }
      }
    } else {
      // An error occurred receiving the message
      #ifdef DEBUG_SERIAL
      Serial.print("RadioLib error: ");
      Serial.println(state);
      #endif
    }
    
    // Resume receiving
    rxInProgress = true;
    radio.startReceive();
  }
}

// Handle received kinetic message and publish to MQTT
void handleReceivedMessage(struct kineticMessage* msg) {
  char deviceIDStr[9];
  sprintf(deviceIDStr, "%08X", msg->deviceID);
  
  unsigned long currentTime = millis();
  
  // Find or initialize device state
  int deviceIndex = -1;
  for (int i = 0; i < MAX_DEVICE_IDS; i++) {
    if (deviceStates[i].deviceID == msg->deviceID) {
      deviceIndex = i;
      break;
    }
    // Track first empty slot for new devices
    if (deviceStates[i].deviceID == 0 && deviceIndex == -1) {
      deviceIndex = i;
    }
  }
  
  // If we found or allocated a slot, check debouncing
  if (deviceIndex >= 0) {
    // Debounce switch-only devices (they repeatedly transmit until they run out of energy)
    if (msg->devType == SWITCH_ONLY) {
      // Skip if same message received within debounce window and state unchanged
      if ((currentTime - deviceStates[deviceIndex].lastMessageTime) < DEBOUNCE_MILLIS && 
          deviceStates[deviceIndex].state == msg->state) {
        #ifdef DEBUG_SERIAL
        Serial.println("Debouncing");
        #endif
        return;
      }
    }
    
    // Update device state tracking
    deviceStates[deviceIndex].deviceID = msg->deviceID;
    deviceStates[deviceIndex].state = msg->state;
    deviceStates[deviceIndex].rssi = (int)radio.getRSSI();
    deviceStates[deviceIndex].lastUpdate = currentTime;
    deviceStates[deviceIndex].lastMessageTime = currentTime;
    
    #ifdef DEBUG_SERIAL
    Serial.println("Updated device state");
    #endif
  }
  
  if (mqttClient.connected()) {
    // Build topic: mqttTopicValue/state/DEVICEID
    char topic[strlen(mqttTopicValue) + 18] = "";
    strcat(topic, mqttTopicValue);
    strcat(topic, "/state/");
    strcat(topic, deviceIDStr);
    
    // Build JSON payload
    StaticJsonDocument<256> doc;
    doc["state"] = msg->state;
    doc["type"] = msg->msgType;
    doc["rssi"] = radio.getRSSI();
    doc["device_type"] = msg->devType;
    
    char payload[256];
    serializeJson(doc, payload, sizeof(payload));
    
    mqttClient.publish(topic, payload);
    #ifdef DEBUG_SERIAL
    Serial.print("Published state for device ");
    Serial.print(deviceIDStr);
    Serial.print(" to ");
    Serial.println(topic);
    #endif
  }
}

// Poll a specific device for its current state
void pollDevice(uint32_t deviceID) {
  #ifdef DEBUG_SERIAL
  Serial.print("Polling device ");
  Serial.println(deviceID, HEX);
  #endif
  
  if (!rxInProgress) {
    #ifdef DEBUG_SERIAL
    Serial.println("WARNING: RX not in progress, cannot poll");
    #endif
    return;
  }
  
  rxInProgress = false;
  radio.finishReceive();
  
  uint8_t txBuf[12];
  encodeForTransmission(txBuf, deviceID, POLL, false);
  
  txInProgress = true;
  transmissionState = radio.startTransmit(txBuf, PACKET_LENGTH);
  if (transmissionState != RADIOLIB_ERR_NONE) {
    txInProgress = false;
    #ifdef DEBUG_SERIAL
    Serial.print("Poll TX failed, code ");
    Serial.println(transmissionState);
    #endif
  }
}

// Publish system status to MQTT
void publishSystemStatus(const char* status) {
  if (mqttClient.connected()) {
    char topic[strlen(mqttTopicValue) + 8] = "";
    strcat(topic, mqttTopicValue);
    strcat(topic, "/system");
    
    StaticJsonDocument<128> doc;
    doc["status"] = status;
    
    char payload[256];
    serializeJson(doc, payload, sizeof(payload));
    
    mqttClient.publish(topic, payload);
  }
}

// Establish an MQTT connection and subscribe to control topics
void connectMqtt() {
  // Loop until we're reconnected
  #ifdef DEBUG_SERIAL
  Serial.println("Attempting MQTT connection...");
  #endif

  // Create a random client ID
  String clientId = iotWebConf.getThingName();
  clientId += "-";
  clientId += String(random(0xffff), HEX);

  #ifdef DEBUG_SERIAL
  Serial.println(clientId);
  #endif

  // Attempt to connect
  if (connectMqttOptions()) {
    #ifdef DEBUG_SERIAL
    Serial.println("MQTT connected");
    #endif
    
    // Publish online status
    publishSystemStatus("online");
    
    // Subscribe to set commands for all devices: kinetic/set/+
    char setTopic[strlen(mqttTopicValue) + 6] = "";
    strcat(setTopic, mqttTopicValue);
    strcat(setTopic, "/set/+");
    
    #ifdef DEBUG_SERIAL
    Serial.print("Subscribing to: ");
    Serial.println(setTopic);
    #endif
    mqttClient.subscribe(setTopic);
    
    // Set flag to poll devices in main loop (non-blocking)
    mqttJustConnected = true;
    lastPollTime = 0; // Force immediate polling
  } else {
    #ifdef DEBUG_SERIAL
    Serial.print("MQTT Connection Failed, state: ");
    Serial.println(mqttClient.state());
    Serial.println("Retrying in 5 seconds");
    #endif
    // Wait 5 seconds before retrying
    iotWebConf.delay(5000);
  }
}

bool connectMqttOptions()
{
  bool result;
  if (mqttUserPasswordValue[0] != '\0')
  {
    String userName = mqttUserNameValue;
    String password = mqttUserPasswordValue;
    result = mqttClient.connect(iotWebConf.getThingName(), userName.c_str(), password.c_str());
  }
  else
  {
    result = mqttClient.connect(iotWebConf.getThingName());
  }
  return result;
}

// Convert array of bytes into a string containing the HEX representation of the array
void bytesToHexString(byte array[], unsigned int len, char buffer[]) {
    for (unsigned int i = 0; i < len; i++) {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

// Convert a single byte into a string containing the HEX representation
void byteToHexString(byte mybyte, char buffer[]) {
    byte nib1 = (mybyte >> 4) & 0x0F;
    byte nib2 = (mybyte >> 0) & 0x0F;
    buffer[0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    buffer[2] = '\0';
}

// If configuration is saved in IOTWebConf, reboot the device
void configSaved() {
  #ifdef DEBUG_SERIAL
  Serial.println("Configuration was updated.");
  #endif
  needReset = true;
}

// Validate the data entered into the IOTWebConf configuration page
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper) {
  #ifdef DEBUG_SERIAL
  Serial.println("Validating form.");
  #endif
  bool valid = true;

  int l = webRequestWrapper->arg(mqttServerParam.getId()).length();
  if (l < 3) {
    mqttServerParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }

  l = webRequestWrapper->arg(mqttTopicParam.getId()).length();
  if (l < 3) {
    mqttTopicParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }

  // Validate Device IDs
  // Check that IDs are in proper format (8 hex characters) and there are no empty IDs in between filled ones
  int lastFilledIndex = -1;
  for (int i = 0; i < MAX_DEVICE_IDS; i++) {
    String devIdStr = webRequestWrapper->arg(deviceIDParams[i]->getId());
    
    if (devIdStr.length() > 0) {
      // ID is filled - validate format
      if (devIdStr.length() != 8) {
        deviceIDParams[i]->errorMessage = "Must be exactly 8 hex characters (32-bit)!";
        valid = false;
      } else {
        // Check all characters are valid hex
        bool isValidHex = true;
        for (char c : devIdStr) {
          if (!isxdigit(c)) {
            isValidHex = false;
            break;
          }
        }
        if (!isValidHex) {
          deviceIDParams[i]->errorMessage = "Must be valid hexadecimal (0-9, A-F)!";
          valid = false;
        }
      }
      
      // Check chaining: previous ID must be filled if current ID is filled
      if (i > 0 && lastFilledIndex != i - 1) {
        deviceIDParams[i]->errorMessage = "Previous Device ID must be filled first!";
        valid = false;
      }
      
      lastFilledIndex = i;
    }
  }

  return valid;
}

// Get the number of filled device IDs
int getFilledDeviceIDCount() {
  int count = 0;
  for (int i = 0; i < MAX_DEVICE_IDS; i++) {
    if (strlen(deviceIDValues[i]) > 0) {
      count++;
    } else {
      break;  // Chain breaks here
    }
  }
  return count;
}

// Get device ID at index as a uint32_t, or 0 if index is out of range or ID is not filled in the web form
const uint32_t getDeviceID(int index) {
  if (index >= 0 && index < MAX_DEVICE_IDS && strlen(deviceIDValues[index]) > 0) {
    return strtoul(deviceIDValues[index], 0, 16);
  }
  return 0;
}
