// PowMr mqtt modbus bridge

#include <FS.h> //this needs to be first, or it all crashes and burns...
#include "LittleFS.h"
#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#include <PubSubClient.h> // MQTT client

#include <ModbusMaster.h>
ModbusMaster node;

// R232 level shifter connections

#define SS_TX_PIN D8 // GPIO15
#define SS_RX_PIN D7 // GPIO13
#include <SoftwareSerial.h>
SoftwareSerial Ser1(SS_RX_PIN, SS_TX_PIN);

#define VERSION 1.04

#define DEBUG(x) Serial.println(x);

ADC_MODE(ADC_VCC);
int vdd;

// stay alive
bool stay_alive = false;

// flag for saving data
bool shouldSaveConfig = false;

// MQTT
char conf_server_ip[32] = "mqtt.local";
char conf_server_port[6] = "1883";
uint16_t conf_server_port_int;

#define MAX_MSG 50
WiFiClient espClient;
PubSubClient client(espClient);
char msg[MAX_MSG + 1];

const char *publishTopic = "/iot/node/powmr/log/modbus";
const char *publishTopicLog = "/iot/node/powmr/log/console";
const char *mqtt_topic_cmd = "/iot/node/powmr/c/+";

/**
 * @fn int strend(const char *s, const char *t)
 * @brief Searches the end of string s for string t
 * @param s the string to be searched
 * @param t the substring to locate at the end of string s
 * @return one if the string t occurs at the end of the string s, and zero otherwise
 */
int strend(const char *s, const char *t)
{
  size_t ls = strlen(s); // find length of s
  size_t lt = strlen(t); // find length of t
  if (ls >= lt)          // check if t can fit in s
  {
    // point s to where t should start and compare the strings from there
    return (0 == memcmp(t, s + (ls - lt), lt));
  }
  return 0; // t was longer than s
}

// callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void alivePrint()
{
  Serial.print(".");
}

void hexDump(const uint8_t *b, int len)
{
  // #ifdef DEBUG_PRINTS
  Serial.println();
  for (int i = 0; i < len; i = i + 16)
  {
    Serial.print("           ");
    for (int x = 0; x < 16 && (x + i) < len; x++)
    {
      if (b[i + x] <= 0xf)
        Serial.print("0");
      Serial.print(b[i + x], HEX);
      Serial.print(" ");
    }
    Serial.print(" ");
    for (int x = 0; x < 16 && (x + i) < len; x++)
    {
      if (b[i + x] <= 32 || b[i + x] >= 126)
      {
        Serial.print(".");
      }
      else
        Serial.print((char)b[i + x]);
    }
    Serial.print("\n");
  }
  Serial.print("                   Length: ");
  Serial.println(len);
  //  #endif
}

void falltosleep()
{
  DEBUG("Sleep...\n");
  ESP.deepSleep(60e6); // 60 sec
  // RF_NO_CAL
  // ESP.deepSleepInstant(microseconds, mode); // mode WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED
}

// mqtt subscribe callback / command topic
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("\nMessage arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strend(topic, "/txpow"))
  {
    Serial.println("TX power set");
    payload[length] = '\0'; // might be unsafe
    float txpow = atof((char *)payload);
    WiFi.setOutputPower(txpow); // float 0 - 20.5 ->>> 4.0 * val (0-82)
  }

  if (strend(topic, "/reboot"))
  {
    Serial.println("Topic Reboot...");
    Serial.flush();
    delay(1000);
    ESP.reset();
    delay(5000);
  }

#define TOKEN_SEP " "

  if (strend(topic, "/modbus_write_single"))
  {
    uint16_t reg_n;
    uint16_t reg_val;
    char *ptr;
    ptr = (char *)payload;
    static char *seq_tok_last; // last char in tokenizer

    *(payload + length) = '\0';
    ptr = strtok_r(ptr, TOKEN_SEP, &seq_tok_last);

    if (ptr != NULL)
    {
      reg_n = atoi(ptr);
      ptr = strtok_r(NULL, TOKEN_SEP, &seq_tok_last);
      if (ptr != NULL)
      {
        reg_val = atoi(ptr);
        uint8_t res = node.writeSingleRegister(reg_n, reg_val);
        char buf[10];
        snprintf(buf, 9, "%u", res);
        client.publish(publishTopicLog, buf);
      }
    }
  }

  if (strend(topic, "/modbus_read_single"))
  {
    *(payload + length) = '\0';
    uint16_t reg_n = atoi((char *)payload);

    uint8_t res = node.readHoldingRegisters(reg_n, 1);
    uint16_t response;
    if (res == node.ku8MBSuccess)
    {
      response = node.getResponseBuffer(0);
      char buf[10];
      char topic_buf[50];
      snprintf(buf, 9, "%u", response);
      snprintf(topic_buf, 49, "%s/%u", publishTopic, reg_n);
      client.publish(topic_buf, buf);
    }
    else
    {
      client.publish(publishTopicLog, "error reading from holding register");
    }
  }

  if (strend(topic, "/modbus_read_range"))
  {
    *(payload + length) = '\0';
    char *delimiter = strchr((char *)payload, ',');
    if (delimiter == nullptr)
    {
      client.publish(publishTopicLog, "invalid payload format");
      return;
    }

    // Split payload into start register and number of registers
    *delimiter = '\0';
    uint16_t start_reg = atoi((char *)payload);
    uint16_t num_regs = atoi(delimiter + 1);

    // Read registers
    uint8_t res = node.readHoldingRegisters(start_reg, num_regs);
    if (res == node.ku8MBSuccess)
    {
      char buf[50];
      char topic_buf[50];
      for (uint16_t i = 0; i < num_regs; i++)
      {
        uint16_t response = node.getResponseBuffer(i);
        snprintf(buf, sizeof(buf), "%u", response);
        snprintf(topic_buf, sizeof(topic_buf), "%s/%u", publishTopic, start_reg + i);
        client.publish(topic_buf, buf);
      }
    }
    else
    {
      client.publish(publishTopicLog, "error reading from holding registers");
    }
  }
}

void reconnect()
{
  // Loop until we're reconnected
  if (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      client.subscribe(mqtt_topic_cmd);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" ");
      // should we retry ???
      // Wait 5 seconds before retrying
      // delay(5000);
      // timerId = timer.setTimeout(5000,reconnect);
    }
  }
}

int getRSSI()
{
  // print the received signal strength:
  int rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
  return (rssi);
}

void publish_float(const char *topic, float var)
{
  char buf[11];
  snprintf(buf, 10, "%3.1f", var);
  client.publish(topic, buf);
}

void publish_float4(const char *topic, float var)
{
  char buf[11];
  snprintf(buf, 10, "%3.4f", var);
  client.publish(topic, buf);
}

// runs when we are waiting for modbus data
void idle()
{
  ArduinoOTA.handle();

  if (!client.connected())
  {
    reconnect();
  }

  client.loop();

  yield();
}

void setup()
{
  // unsigned long StartTime = millis();

  // pinMode(holdPin, OUTPUT);  // sets GPIO 0 to output
  // digitalWrite(holdPin, HIGH);  // sets GPIO 0 to high (this holds CH_PD high even if the PIR output goes low)

  // Serial.begin(2400); // looks like it's a main speed
  Serial.begin(250000);
  Ser1.begin(9600);
  Ser1.enableIntTx(false);
  // Ser1.enableRxGPIOPullUp(true);

  delay(10);
  Serial.printf("\n\nSDK version:%s\n\r", system_get_sdk_version());
  Serial.print("Firmware version: ");
  Serial.println(VERSION);

  Serial.println("Booting...");

  pinMode(LED_BUILTIN, OUTPUT);    // Initialize the BUILTIN_L
  digitalWrite(LED_BUILTIN, HIGH); // turn off

  // clean FS, for testing
  // SPIFFS.format();
  // read configuration from FS json
  Serial.println("mounting FS");

  if (LittleFS.begin())
  {
    Serial.println("mounted file system");
    if (LittleFS.exists("/config.json"))
    {
      // file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success())
        {
          Serial.println("\nparsed json");
          if (json["server_ip"])
          {
            strcpy(conf_server_ip, json["server_ip"]);
            strcpy(conf_server_port, json["server_port"]);
          }
        }
        else
        {
          Serial.println("failed to load json config");
        }
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }
  // end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter c_conf_server_ip("server_ip", "MQTT server ip", conf_server_ip, 32);
  WiFiManagerParameter c_conf_server_port("port", "MQTT server port", conf_server_port, 6);

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // add all your parameters here
  wifiManager.addParameter(&c_conf_server_ip);
  wifiManager.addParameter(&c_conf_server_port);

  // reset settings - for testing
  // wifiManager.resetSettings();

  // set minimu quality of signal so it ignores AP's under that quality
  // defaults to 8%
  // wifiManager.setMinimumSignalQuality();

  // sets timeout until configuration portal gets turned off
  // useful to make it all retry or go to sleep
  // in seconds
  wifiManager.setTimeout(180);

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("ESP-POWMR", "XXXXXXXX"))
  {
    Serial.println("failed to connect and hit timeout...");
  }

  // if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // read updated parameters
  strcpy(conf_server_ip, c_conf_server_ip.getValue());
  strcpy(conf_server_port, c_conf_server_port.getValue());
  conf_server_port_int = atoi(conf_server_port);

  // save the custom parameters to FS
  if (shouldSaveConfig)
  {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();

    json["server_ip"] = conf_server_ip;
    json["server_port"] = conf_server_port;

    json["ip"] = WiFi.localIP().toString();
    json["gateway"] = WiFi.gatewayIP().toString();
    json["subnet"] = WiFi.subnetMask().toString();

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile)
    {
      Serial.println("failed to open config file for writing");
    }
    else
    {
      json.printTo(Serial);
      json.printTo(configFile);
      configFile.close();
      // end save
    }
  }

  // Serial.print("\nlocal ip: ");
  // Serial.println(WiFi.localIP());

  // WiFi.printDiag(Serial);

  Serial.print("DNS Lookup ...");

  IPAddress mqttServerIP;

  if (!WiFi.hostByName(conf_server_ip, mqttServerIP))
  { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting...");
    // falltosleep();
    delay(1000);
    // ESP.reset();
  }
  Serial.print("MQTT server IP:\t");
  Serial.println(mqttServerIP);

  /// end Of WifiManager portal

  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //   Serial.println("Connection Failed! Rebooting...");
  //   delay(5000);
  //   ESP.restart();
  // }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("foxyfox3");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
                     {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("\nOTA: Start updating " + type); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer((const char *)conf_server_ip, conf_server_port_int);
  client.setCallback(callback);
  // client.setBufferSize(MQTT_MAX_PACKET_SIZE);

  DEBUG(conf_server_ip);
  DEBUG(conf_server_port_int);

  if (!client.connected())
  {
    reconnect();
  }

  Serial.println("Publishing to mqtt");
  vdd = ESP.getVcc();
  snprintf(msg, MAX_MSG, "%ld", vdd);
  client.publish(publishTopicLog, msg);

  // communicate with Modbus slave ID 1 over Serial
  node.begin(1, Ser1);
  node.idle(idle);

  Serial.println("To the main loop...");
}

#define TIME_OUT 15
// 512
#define MAX_MBUS_PKT 500

void loop()
{

  ArduinoOTA.handle();

  if (!client.connected())
  {
    reconnect();
  }

  client.loop();

  static unsigned long timeLastInput = 0;
  unsigned long now = millis();
  static char buffer[MAX_MBUS_PKT + 1];
  static int index = 0;

  // this part also used to sniff communication with the original dongle

  if (Serial.available() > 0)
  {
    char x = Serial.read();
    timeLastInput = now;

    if (index < MAX_MBUS_PKT)
    {
      buffer[index++] = x;
    }
  }

  if (now - timeLastInput > TIME_OUT || index >= MAX_MBUS_PKT)
  {
    if (index > 0)
    {
      // Serial.println("Time out");
      // hexDump((uint8_t *) buffer, index);

      static uint8_t charArr[2 * MAX_MBUS_PKT + 1]; // Note there needs to be 1 extra space for this to work as snprintf null terminates.
      uint8_t *myPtr;
      myPtr = charArr;

      for (uint16_t i = 0; i < index; i++)
      {
        snprintf((char *)myPtr, 3, "%02x", buffer[i]); // convert a byte to character string, and save 2 characters (+null) to charArr;
        myPtr += 2;                                    // increment the pointer by two characters in charArr so that next time the null from the previous go is overwritten.
      }

      client.publish(publishTopic, charArr, index * 2);

      index = 0;
      timeLastInput = now;
    }
    else
    {
      // Serial.print(".");
    }
  }
}