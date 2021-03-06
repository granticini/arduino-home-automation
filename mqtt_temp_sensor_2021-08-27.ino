/*
 MQTT Test Bed 
 27 August 2021

 Connect to WiFi
 Connect to MQTT

 Every x seconds it reads the temperature sensor value and publishes it to MQTT
 
 
  - publishes WiFi strength to mqtt topic

  - subscribes to the topic "arduino/in"
    printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 
*/

#include <SPI.h>
#include <WiFiNINA.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <OneWire.h>

#include "arduino_secrets.h" 


#define APP_VERSION  "0.3.0"
#define APP_AUTHOR   "Grant Phillips"
#define APP_CREATED  "17 Aug 2021"
#define APP_BUILT    "28 Aug 2021"


// ------------------------
#define SERIAL_BAUD          9600
#define SLEEPY_TIME_1_SECOND 1000
#define SLEEPY_TIME_MINI      100
#define SLEEPY_TIME_TINY       10

// publish sensor readings every minute
#define SENSOR_PUBLISH_FREQUENCY  60000

#define ON  HIGH
#define OFF LOW

#define BUTTON_PRESSED   LOW
#define BUTTON_UNPRESSED HIGH

// sensitive data is in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;       // your network SSID (name)
char pass[] = SECRET_PASS;       // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status

#define MQTT_BROKER_IP_NUMBER "10.0.0.234"
#define MQTT_BROKER_PORT      1883

#define MQTT_MY_ID                        "arduino_office"
#define MQTT_TOPIC_ARDUINO_IN             "arduino/in"
#define MQTT_TOPIC_ARDUINO_CONNECTED      "arduino/connected"
#define MQTT_TOPIC_ARDUINO_WIFI_STRENGTH  "arduino/wifi/strength"
#define MQTT_TOPIC_ARDUINO_TEMP           "arduino/temp"
#define MQTT_TOPIC_ARDUINO_STATUS         "arduino/status"

#define MY_LOCATION                       "Office"

// OneWire stuff for the Temperature Sensor
//const int PIN_LED =  7;         // the number of the LED pin
const int PIN_TEMP_SENSOR = 10; // the number of the pin the Temp Sensor is attached to

int ledPin = PIN_LED;

OneWire  ds(PIN_TEMP_SENSOR);

const int buttonPin = 2;   // This is the buttons read pin
int buttonState = BUTTON_UNPRESSED;     // Variable for current button reading
int lastButtonState = BUTTON_UNPRESSED; // Variable for last know button reading
unsigned long lastDebounceTime = 0; //last time the pin was toggled, used to keep track of time
unsigned long debounceDelay = 50;   //the debounce time which user sets prior to run
#define MAX_PIN 24
int lastButtonStates[MAX_PIN];
char macBuffer[20];
char hexChars[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};


long tickCounter = 0;
char buffer[20];
char jsonBuffer[300];
char myipString[20];
char tempSensorChip[10];
char location[30];
long wifiStrength = 0;
float tempReading = -999;
IPAddress myip;

unsigned long previousTime;
bool ledState = false;

void callback(char* topic, byte* payload, unsigned int length) 
{
  flashLed(3);
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) 
  {
    Serial.print((char)payload[i]);
  }
  
  Serial.println();
}

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.println("Attempting MQTT connection...");
    flashLed(2);

    // Attempt to connect
    Serial.print("client.connect ... ");
    if (client.connect(MQTT_MY_ID, MQTT_USERNAME, MQTT_PASSWORD)) 
    {
      Serial.println("connected");
      
      // Once connected, publish an announcement...
      Serial.print("publish to ");
      Serial.print(MQTT_TOPIC_ARDUINO_CONNECTED);
      Serial.print(" ... ");
      client.publish(MQTT_TOPIC_ARDUINO_CONNECTED, "1");
      Serial.println("done.");
    
      // ... and resubscribe
      Serial.print("subscribe to ");
      Serial.print(MQTT_TOPIC_ARDUINO_IN);
      Serial.print(" ... ");
      client.subscribe(MQTT_TOPIC_ARDUINO_IN);
      Serial.println("done.");
    }
    else
    {
      Serial.println("failed");
      Serial.print("rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
    
      // Wait 5 seconds before retrying
      snooze(5);
    }
  }
  Serial.println("reconnected to MQTT ok");

  flashLed(10);
}

void snooze(int seconds)
{
  // Waitx5 seconds before retrying
  for(int i=0; i < seconds; i++)
  {
    delay(SLEEPY_TIME_1_SECOND);
    Serial.print(".");
  }
  Serial.println();
}

void setup()
{
  //Initialize serial and wait for port to open:
  Serial.begin(SERIAL_BAUD);
  while (!Serial) 
  {
    Serial.print(".");
    delay(SLEEPY_TIME_MINI);
    // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Debug ok");

  for (int i=0; i < sizeof(tempSensorChip); i++)
  {
    tempSensorChip[i] = '\0';
  }

  for (int i=0; i < sizeof(macBuffer); i++)
  {
    macBuffer[i] = '\0'; 
  }

  strcpy(location, MY_LOCATION);

  // ----------------------------
  // initialise the array that tracks the Button states
  int pin;
  for (pin = 0; pin < MAX_PIN; pin++)
  {
    lastButtonStates[pin] = BUTTON_UNPRESSED;
  }
    
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);

  // ----------------------------
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) 
  {
    Serial.println("Communication with WiFi module failed!");
     
    // don't continue
    while (true);
  }

  tickCounter = 0;
  previousTime = millis();

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) 
  {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) 
  {
    Serial.print("Attempting to connect to WiFi: ");
    Serial.println(ssid);
    
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    snooze(5);
  }

  // you're connected now, so print out the data:
  Serial.println("You are connected to the network.");
  printCurrentNet();
  printWifiData();
  
  // --------
  Serial.print("MQTT client.setClient(wifiClient) ... ");
  client.setClient(wifiClient);
  Serial.println("done.");

  Serial.print("client.setServer(");
  Serial.print(MQTT_BROKER_IP_NUMBER);
  Serial.print(", ");
  Serial.print(MQTT_BROKER_PORT);
  Serial.print(") ... ");
  client.setServer(MQTT_BROKER_IP_NUMBER, MQTT_BROKER_PORT);
  Serial.println("done.");

  Serial.print("client.setCallback() ... ");
  client.setCallback(callback);
  Serial.println(" done.");

  flashLed(10);
  
  // Allow the hardware to sort itself out
  snooze(2);

    if (!client.connected()) 
  {
    reconnect();
  }

  Serial.println("-------------------------");
  Serial.println("Ready.");
}

void loop() 
{
  delay(SLEEPY_TIME_MINI);

  if (!client.connected()) 
  {
    reconnect();
  }

  // ------------------------------
  // read the button state (Pressed or Unpressed)
  int lastButtonState = lastButtonStates[buttonPin];
  int buttonState = readButtonState(buttonPin);
  if (buttonState != lastButtonState)
  {
    // button state has changed
    if (buttonState == BUTTON_PRESSED)
    {
      Serial.println("Button has been pressed");
      ledState = ON;
    }
    else
    {
      Serial.println("Button has been un-pressed");
      ledState = OFF;
    }
  }
  
  // set the LED
  digitalWrite(ledPin, ledState);

  // ------------------------------
  unsigned long currentTime = millis();
  
  if ((tickCounter == 0) or 
      (buttonState == BUTTON_PRESSED) or
      (currentTime < previousTime) or 
      (currentTime - previousTime >= SENSOR_PUBLISH_FREQUENCY))
  {
    // sleepytime has been reached
    ledState = !ledState;
    previousTime = currentTime;

    publishSensorData();

    tickCounter = tickCounter + 1;
  }

  client.loop();
}

int readButtonState(int buttonPin)
{
  // read the button pin, if pressed will be high, if not pressed will be low
  int thisButtonState = lastButtonStates[buttonPin];
    
  int reading = digitalRead(buttonPin);
  //Serial.print("button reading is ");
  //Serial.println(reading);

  // check the difference between current time and last registered button press time,
  // if it's greater than user defined delay then change the LED state as it's not a bounce
  if ((millis() - lastDebounceTime) > debounceDelay) 
  {
    if (reading != lastButtonStates[buttonPin]) 
    {
      thisButtonState = reading;
      lastButtonStates[buttonPin] = reading;
      lastDebounceTime = millis();

      //Serial.print("buttonState changed to ");
      //if (thisButtonState == BUTTON_PRESSED) Serial.println("Pressed");
      //else Serial.println("Un-pressed");
    }
  }

  return thisButtonState;
}

void flashLed(int flashes)
{
  for (int i=0; i < flashes; i++)
  {
      delay(200);
      digitalWrite(ledPin, ON);
      delay(200);
      digitalWrite(ledPin, OFF);
  }
}

void publishSensorData()
{
  flashLed(2); 
  publishWiFiStrength();

  flashLed(2); 
  publishTemp();  

  flashLed(2); 
  publishStatus();
}

void publishWiFiStrength()
{
    // get the WiFi signal strength:
    wifiStrength = WiFi.RSSI();
    
    // Serial.print("signal strength (RSSI):");
    // Serial.println(wifiStrength);
  
    ltoa(wifiStrength, buffer, 10);
 
    // Publish my Wifi status/strength
    Serial.print("publish to ");
    Serial.print(MQTT_TOPIC_ARDUINO_WIFI_STRENGTH);
    Serial.print(", ");
    Serial.print(buffer);
    Serial.print(" ... ");
    client.publish(MQTT_TOPIC_ARDUINO_WIFI_STRENGTH, buffer);
    Serial.println("done.");
}

void publishTemp()
{
    Serial.println("Read Temp from sensor:");
    tempReading = readSensor();

    if (tempReading <= -100)
    {
      Serial.print("No temp reading available, ");
      Serial.println(tempReading);
      return;
    }
    
    String tempString = String(tempReading, 1);
    tempString.toCharArray(buffer, sizeof(buffer));
 
    // Publish the temperature from the sensor
    Serial.print("publish to ");
    Serial.print(MQTT_TOPIC_ARDUINO_TEMP);
    Serial.print(", ");
    Serial.print(buffer);
    Serial.print(" ... ");
    client.publish(MQTT_TOPIC_ARDUINO_TEMP, buffer);
    Serial.println("done.");
}

void publishStatus()
{
    Serial.println("publishStatus:");
    
    String tempString = String(tempReading, 1);
    tempString.toCharArray(buffer, sizeof(buffer));

    //Serial.println(tempString);
    
    /*
     * 
     * this is too big
     * max length is around 230 characters
{
  "client_name": "Arduino Office",
  "app_version": "0.2.0",
  "app_built": "28 Aug 2021",
  "ip": "10.0.0.212",
  "mac": "F0:08:D1:CB:51:D0",
  "wifi_strength": "-64",
  "temperature_celsius": "18.6",
  "temperature_chip": "DS18B20",
  "tick_counter": 75,
  "led_state": "on",
  "button_state": "pressed",
  "pin_temp_sensor": 3,
  "pin_button": 2,
  "pin_led": 7,
  "mqtt_broker_host": "10.0.0.234",
  "mqtt_broker_port": 1883
}

    */ 
    Serial.println(MQTT_MY_ID);
    Serial.println(APP_VERSION);
    Serial.println(APP_BUILT);
    Serial.println(location);
    Serial.println(myipString);
    Serial.println(macBuffer);
    Serial.println(wifiStrength);
    Serial.println(buffer);
    Serial.println(tempSensorChip);
    Serial.println(tickCounter);
    Serial.println();
    Serial.println("sprintf ...");
    sprintf(jsonBuffer, "{\"name\": \"%s\",\"version\": \"%s\",\"built\": \"%s\",\"loc\": \"%s\",\"ip\": \"%s\",\"mac\": \"%s\",\"wifi\": \"%d\",\"temp\": \"%s\",\"sensor\": \"%s\",\"ticks\": \"%d\",\"led\": \"%s\",\"btn\": \"%s\"}",
      MQTT_MY_ID,
      APP_VERSION,
      APP_BUILT,
      location,
      myipString,
      macBuffer,
      wifiStrength,
      buffer,
      tempSensorChip,
      tickCounter,
      ledState ? "on" : "off",
      buttonState == BUTTON_PRESSED ? "pressed" : "unpressed"
    );

    // Publish the temperature from the sensor
    Serial.print("publish to ");
    Serial.println(MQTT_TOPIC_ARDUINO_STATUS);
    Serial.println(strlen(jsonBuffer));
    Serial.println(jsonBuffer);
    client.publish(MQTT_TOPIC_ARDUINO_STATUS, jsonBuffer);
    Serial.println("Done.");
}

void printWifiData() 
{
  // print your board's IP address:
  myip = WiFi.localIP();
  sprintf(myipString, "%d.%d.%d.%d", myip[0], myip[1], myip[2], myip[3]);
  
  Serial.print("IP Address: ");
  Serial.println(myip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  
  int macBufferIndex = 0;
  for (int i = sizeof(mac)-1; i >= 0; i--) 
  {
    byte x = mac[i] >> 4;
    byte y = mac[i] % 16;

    macBuffer[macBufferIndex++] = hexChars[x];
    macBuffer[macBufferIndex++] = hexChars[y];
    if (i > 0) 
    {
      macBuffer[macBufferIndex++] = ':';
    }
  }
  
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) 
{
  for (int i = 5; i >= 0; i--) 
  {
    if (mac[i] < 16) 
    {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) 
    {
      Serial.print(":");
    }
  }
  Serial.println();
}

float readSensor()
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius; //, fahrenheit;
  
  Serial.println("Read Temp Sensor");
  if ( !ds.search(addr)) 
  {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return -100;
  }
  
  Serial.print("  ROM =");
  for( i = 0; i < 8; i++) 
  {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
  Serial.println();

  if (OneWire::crc8(addr, 7) != addr[7]) 
  {
      Serial.println("CRC is not valid!");
      return -101;
  }
 
  // the first ROM byte indicates which chip
  switch (addr[0]) 
  {
    case 0x10:
      if (tempSensorChip[0] == '\0')
      {
        strcpy(tempSensorChip, "DS18S20");
        Serial.println("  Chip = DS18S20");  // or old DS1820
      }
      type_s = 1;
      break;
      
    case 0x28:
      if (tempSensorChip[0] == '\0')
      {
        strcpy(tempSensorChip, "DS18B20");
        Serial.println("  Chip = DS18B20");
      }
      type_s = 0;
      break;
      
    case 0x22:
      if (tempSensorChip[0] == '\0')
      {
        strcpy(tempSensorChip, "DS1822");
        Serial.println("  Chip = DS1822");
      }
      type_s = 0;
      break;
      
    default:
      if (tempSensorChip[0] == '\0')
      {
        strcpy(tempSensorChip, "Unknown");
        Serial.println("Device is not a DS18x20 family device.");
      }
      return -102;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(SLEEPY_TIME_1_SECOND);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  // we need 9 bytes
  for ( i = 0; i < 9; i++) 
  {
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) 
  {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) 
    {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  }
  else
  {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  
  celsius = (float)raw / 16.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.println(" Celsius");

  return celsius;
}