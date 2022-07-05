/*
 * Another go at DCC for Arduino
 * Requirements:
 * - Avoid bitbang overhead
 *  - solution direction: use PIO to create a DCC interface. 
 * - Communicate settings and monitor values via JSON formated messages
 * - Use of wifi for easy connectivity
 * - Use local actuators to change speed
 */

#include "rtos.h"  
#include "mbed.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <hardware/timer.h>
#include <hardware/irq.h>
#include <hardware/adc.h>
#include <hardware/pwm.h>
#include "hardware/pio.h"
#include "structs/pio.h"
//#include <ArduinoMqttClient.h>
#include <PubSubClient.h>

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2) || defined(ARDUINO_NANO_RP2040_CONNECT)
  #include <WiFiNINA.h>
  
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ESP8266_ESP12)
  #include <ESP8266WiFi.h>
#endif

#include <ArduinoJson.h>
#include "arduino_secrets.h"

NinaPin ledR = LEDR;
NinaPin ledG = LEDG;
NinaPin ledB = LEDB;

//-------------------------------------------------------------------------------------

// This file uses the header file generated with the pico asm tool, see the sheel script asm.sh
extern "C" {
#include "DCC_tx.pio.h"
};

#define LED_BUILTIN 6

using namespace rtos; // we will be using rtos::ThisThread  
using namespace mbed;

//**************************************************************************
// global variables
//**************************************************************************

#define maxDCCpacketBytes 3
#define MAXTRAINS 10


//**************************************************************************
// Wifi and MQTT variables
//**************************************************************************
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status

// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate 
//    flashed in the WiFi module.

volatile  int countMQTTfail = 0;
volatile bool wifiOK = 0;
volatile bool mqttOK = 0;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Callback function header
void callback(char* topic, byte* payload, unsigned int length);
// const char broker[] = "test.mosquitto.org";
const String broker = "192.168.178.132";
int        port        = 1883;
IPAddress server(192, 168, 178, 132);


const char willTopic[] = "dcc-controller-1/will";
const char inTopic[]   = "dcc-controller-1/trains_set";
const char outTopic[]  = "dcc-controller-1/trains_status";

const long interval = 1000;
unsigned long previousMillis = 0;

int messageSeqID = 0;
//**************************************************************************
// PIO0 global variables
//**************************************************************************

/*------------------------------------------
We're going to use PIO to output DCC signals
FIFO encoding
 |  15:9  |         8         |  7:0 |
 | future | !start / preamble | data |
*/

const byte SPEED = A2; // potR setting
const uint PIN_TX = 25; // DCC signal 
const uint PIN_Test = 7; // scope trigger
const uint PIN_enable = 3; // Enable DCC_power

PIO pio = pio0;
uint sm = 0;
uint offset = pio_add_program(pio, &DCC_tx_program);
char packetState = 1;
bool toggle;

//**************************************************************************
// DCC global vars
//**************************************************************************

// This is period counter.
// DCC one bit -> one period High, one period low
// DCC zreo bit -> two periods High, two perios low
float Tperiod;
const uint SERIAL_BAUD = 0.5*1/0.000058; // half bit per 58usec
float pioStepPeriod;


// idle packet
#define idle_length 2
char idlePacket[3] = {2,0xFF,0}; //idle packet, address = all, speed=0

/* Mail, to communicate between a thread and the ISR
 *  
 */
typedef struct {
  byte packetLen;
  byte mailByte[4]; // DCC bytes in order of standard definitions
} mail_t;


// Train datastructure

class Train {
  private:
    boolean active;
  public:
    byte address;
    byte trainSpeed; // in DCC byte format
    int8_t commsSpeed; // signed -100..100%
    int8_t analogSpeed;
    int8_t sollSpeed;
    boolean setPerComms; // select comms or analog speed setting
    
  void initTrain() {
    active = true;
    address = 38;
    trainSpeed = 0;
    setPerComms = true;
  }
  void activate() {
    active = true;
  }
  void deactivate() {
    active = false;
  }
  boolean state() {
    return active;
  }
};

Train trains[MAXTRAINS -1]; // objects, maximum of trains on the track


//##########################################################################
// ISR
//##########################################################################
// Globals shared with the ISR
volatile char DCCbyte[4];
volatile byte i;
volatile uint8_t errorByte; 
Mail<mail_t,16> mail_box; // mailbox to hold 16 messages in the queue at max


mail_t *mail; // local message workspace

static void on_pio0_irq0() {
  uint16_t bp;
osEvent evt; 
  uint8_t packetLen;   
  switch (packetState) {
    case 0:
      toggle =!toggle;
      digitalWrite(PIN_Test, toggle);    
      // get ptr to mail
      mail = mail_box.try_get();
      if (mail != NULL){ //(evt.status == osEventMail) {
      // mail message recieved 
        packetLen  = mail->mailByte[0];
        DCCbyte[1] = mail->mailByte[1];
        DCCbyte[2] = mail->mailByte[2];
        mail_box.free(mail);       
      }
      else { //idle packet      
        packetLen  = idlePacket[0];        
        DCCbyte[1] = idlePacket[1];
        DCCbyte[2] = idlePacket[2]; 
      }      
      bp = 1<<8; // set preamble bit[8] to yes
      bp = bp | DCCbyte[1]; //bits[7..0] into the PIO register
      errorByte = DCCbyte[1];
      i = 1;
      packetState = 1;
    break;
    case 1:
      i++;
      bp = DCCbyte[i];
      errorByte ^= DCCbyte[i];
      if (i >= packetLen){ 
        packetState = 2;
      }
    break;      
    case 2:
      bp = errorByte;
      packetState = 0;
    break;          
  }
  DCC_tx_program_to_pio(pio, sm, bp);
  /* debug
  Serial.print(bp);
  Serial.print("\n");    
  */  
}

//##########################################################################
// Declaring the Thread objects  
// NOTE:  threads hang until serial monitor is attached
//##########################################################################
Thread th1;
Thread th2;

//##########################################################################
void task_blinkLedBuiltIn() {  
// Thread function pin toggle
// Thread functions DO NOT exit
  while(1){
    gpio_put(LED_BUILTIN, 1);
    ThisThread::sleep_for(250);
    gpio_put(LED_BUILTIN, 0);
    ThisThread::sleep_for(250);
  }
}

//##########################################################################
// Actions taken after Mqtt message received event
//##########################################################################
void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  int set_speed;
  byte id;
  deserializeJson(doc, payload, length);
  id = doc["ID"];
  set_speed = doc["set_speed"];
  if (set_speed <= 100) {
    trains[id].commsSpeed = set_speed;
  }
  trains[id].address = doc["address"];

  Serial.print("Mqtt trainNumber: ");
  Serial.println(id);
  Serial.print("Mqtt comms speed: ");
  Serial.println(trains[id].commsSpeed);


}

void publishState( byte id )
{
  StaticJsonDocument<256> doc2;
  
  // Train JSON datafields to the PC via MQTT
  Serial.println(outTopic);
  if (mqttClient.connected()) {
    doc2["ID"] = id;   
    doc2["name"] = "Small_black";   
    doc2["address"] = trains[id].address;
    doc2["speed"]   = trains[id].sollSpeed;
    doc2["units"] = "%";
    doc2["seqID"] = messageSeqID++;

    char buffer[256];
    size_t n = serializeJson(doc2, buffer);
      if (mqttOK) mqttClient.publish(outTopic, buffer, n+1);

    Serial.print("Mqtt message sent, ID: ");
    Serial.println(messageSeqID);
  }
  else {
    Serial.println("Waiting for MQTT conection.");    
  }
}

//##########################################################################
void task_setTrainSpeed() {
// Thread function read local input from potentiometer
// Thread functions DO NOT exit
  #define NORMED 0.14 // 14 is max speed, sollSpeed range is +/-100%
  uint8_t val;
  int8_t sollSpeed;
  double normalized;
  byte id = 0;

  while(1){
    // sollSpeed must be given in +/- 100%

    if (trains[id].setPerComms==true) {
      trains[id].sollSpeed = trains[id].commsSpeed;
    }    
    else{
      // read potentiometer setting
      // analog resolution is 1024
      trains[id].analogSpeed = byte(analogRead(SPEED)/5.0) - 100;
      trains[id].sollSpeed = trains[id].analogSpeed;
    }
    
    if (trains[id].sollSpeed > 100) {
      trains[id].sollSpeed = 100;
    }
    else if (trains[id].sollSpeed <-100) {
      trains[id].sollSpeed = -100;
    }


    normalized = NORMED*trains[id].sollSpeed;
    val = byte(abs(normalized));
    val &= B00011111; 
    
    if (trains[id].sollSpeed > 0) {
      trains[id].trainSpeed = val | B01100000; // =96 + value; set forward, 01DCSSSS instruction -> D=forward 
    }
    else {
      trains[id].trainSpeed = val | B01000000; // =64 + value; set backward
    }

    // "mail" to ISR
    mail_t *mail = mail_box.alloc();  // alloc next mail message memory
    mail->mailByte[0] = 2;  // packet Length
    mail->mailByte[1] = 38; // address   
    mail->mailByte[2] = trains[id].trainSpeed; // speed
  /*debug     
    Serial.println(mail->mailByte[0]);    
    Serial.println(mail->mailByte[1]);    
    Serial.println(mail->mailByte[2],BIN);    
*/     
    if (mail_box.full()==false) {  
      mail_box.put(mail);
    }
     
    ThisThread::sleep_for(100);    
  } // while 1
}

//##########################################################################
void connect_checkWIFI()
{
  String message;
  if (WiFi.status() == WL_CONNECTED)
  {
    //Serial.println("[WIFI] WiFi OK");
    wifiOK = 1;
    digitalWrite(ledG, HIGH);
    digitalWrite(ledR, LOW);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    wifiOK = 0;
    mqttOK = 0;
    digitalWrite(ledG, LOW);
    digitalWrite(ledR, HIGH);
    // Start connection to WLAN router and print a status value
    Serial.println("[WIFI] Trying to connect to WLAN router");
    WiFi.disconnect();
    mqttClient.disconnect();
    delay(1000); // wait for hw to reset, before starting reconnect again

    status = WiFi.begin(ssid, pass);
    // WL_IDLE_STATUS     = 0
    // WL_NO_SSID_AVAIL   = 1
    // WL_SCAN_COMPLETED  = 2
    // WL_CONNECTED       = 3
    // WL_CONNECT_FAILED  = 4
    // WL_CONNECTION_LOST = 5
    // WL_DISCONNECTED    = 6
    ThisThread::sleep_for(10000);
    Serial.print("[WIFI] Wifi status = ");
    switch (status) {
      case 0: message = "WL_IDLE_STATUS"; break;
      case 1: message = "WL_NO_SSID_AVAIL"; break;
      case 2: message = "WL_SCAN_COMPLETED"; break;
      case 3: message = "WL_CONNECTED"; break;
      case 4: message = "WL_CONNECT_FAILED"; break;
      case 5: message = "WL_CONNECTION_LOST"; break;
      case 6: message = "WL_DISCONNECTED"; break;
    }
    Serial.println (message);
    
    if (status == WL_CONNECTED)
    {
        wifiOK = 1;
        Serial.println("[WIFI] Connection to WLAN router successful");
        printCurrentNet();
        printWifiData();
    }
  }
  CheckMQTT();
}

void CheckMQTT()
{
  if (mqttClient.connected() && wifiOK)
  {
    mqttOK = 1;
    mqttClient.loop();
    //Serial.println("MQTT OK");
  }

  if (mqttClient.connected() && !wifiOK)
  {
    mqttOK = 0;
  }

  if (!mqttClient.connected() && !wifiOK)
  {
    mqttOK = 0;
  }
  
  if (!mqttClient.connected() && wifiOK)
  {
    countMQTTfail++;
    if (countMQTTfail > 10) {
      countMQTTfail = 0;
      mqttClient.disconnect();
      delay(10);        
      Serial.print("[MQTT] Attempting MQTT connection...");
      // Create a random client ID
      String clientId = "dcc-controller-1";
      if (mqttClient.connect(clientId.c_str())) 
      {
        Serial.println("connected");
        // Once connected, publish an announcement...
        mqttClient.publish(outTopic, "(re)Subscribing to inTopic");
        // ... and resubscribe
        mqttClient.subscribe(inTopic);
      } else 
      {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in  seconds");
        mqttClient.disconnect();
        delay(10);        
      }
    }
  }
}
//##########################################################################
void setup() {
  trains[0].initTrain();
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); // prevents usb driver crash on startup, do not omit this    
  while (!Serial) {
  }
  Serial.begin(9600);
  delay(1000); // prevents usb driver crash on startup, do not omit this    
  while (!Serial) {
  }
  
  // Declare i/o - on board LED
  _gpio_init(LED_BUILTIN) ;
  gpio_set_dir(LED_BUILTIN, GPIO_OUT);
 
  Serial.println("Led turned on!");
  

//***************************************************************************
// setup MQTT communication
  mqttClient.setServer(server, 1883);
  mqttClient.setCallback(callback);
  // Allow the hardware to sort itself out
  delay(1500);


//* Start Mbed Threads ******************************************************
  th1.start(task_blinkLedBuiltIn);  
  th2.start(task_setTrainSpeed);


//* Start ISR ***************************************************************
// start PIO0 DCC interface
  float div = (float)clock_get_hz(clk_sys) / (6 * SERIAL_BAUD); // 6 pio ticks per DCC-one period

  Tperiod = 1000000.0 / ((float)clock_get_hz(clk_sys)/div);
  pioStepPeriod = Tperiod; // pio step period
  // put your setup code here, to run once:
  pinMode (LED_BUILTIN, OUTPUT);
  pinMode (ledG, OUTPUT);
  pinMode (ledR, OUTPUT);
  
  pinMode (PIN_Test, OUTPUT);  
  pinMode (PIN_enable, OUTPUT);
  digitalWrite(PIN_enable, HIGH);

  DCC_tx_program_init(pio, sm, offset, PIN_TX, div, on_pio0_irq0);
  delay(100);
  Serial.println("Start PIO DCC interface");
  Serial.print("Period T:");
  Serial.print(pioStepPeriod);  
  Serial.println(" usec");    

  Serial.print(char(idlePacket[0]));
  Serial.print(char(idlePacket[1]));
  Serial.print(idlePacket[2]);
  Serial.println();    
 
} // setup()


void loop() {
  // to avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    connect_checkWIFI();
    publishState(0);  
  }
}

//************* other functions ***************************
void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("[WIFI] IP Address: ");
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("[WIFI] MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("[WIFI] SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("[WIFI] BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("[WIFI] signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println("dBm");
  
  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("[WIFI] Encryption Type:");
  Serial.println(encryption, HEX);
  //Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
