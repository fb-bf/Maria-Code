/*==========================================================================
 * Project: sprintf functionality
 * Author:  Scott C
 * Date created: 06 May 2019
 * Arduino IDE version: 1.8.5
 * Operating System: Windows 10 Pro
 * Tutorial Link: https://arduinobasics.blogspot.com/2019/05/sprintf-function.html
 * 
 * Acknowledgements:
 * The following resource was a key element of this tutorial: http://www.cplusplus.com/reference/cstdio/printf/
 * Another useful resource can be found here: https://en.wikipedia.org/wiki/C_data_types
 * 
 *------------------------------------------------------------------------------
 * Code Explanation
 * -----------------------------------------------------------------------------
 * Begin serial communication at a baud rate of 9600
 * Wait until Serial communication has established before continuing
 * 
 * The sprintf function will write the formatting string and the variables into the "data" character array.
 * You provide a formatting string template, that contains placeholders for variables that you plan to insert.
 * These placeholders have a percentage sign (%) prefix.  Eg.  %s, %d, %f etc.
 * The number of placeholders must match the number of variables.
 * The variables are inserted at the placeholder position, and formatted based on the type of placeholder used.
 * %d = signed integer               %f = floating point number  
 * %s = string                     %.1f = float to 1 decimal place
 * %c = character                  %.3f = float to 3 decimal places
 * %e = scientific notation          %g = shortest representation of %e or %f                
 * %u = unsigned integer             %o = unsigned octal
 * %x = unsigned hex (lowercase)     %X = unsigned hex (uppercase)
 * %hd = short int                  %ld = long int
 * %lld = long long int
 * =============================================================================  */

//#include <Servo.h> //use functions define in this library to control the 6 inch valve actuator
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
// REMOTE UPLOAD HEADER
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
///////////////////////

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
//#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#define i2c_Address 0x3c  //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define i2c_Relay1 0x27  // i2c slave address for relay module 1
#define i2c_Relay2 0x26  // i2c slave address for relay module 2

byte Set_Valve_15_Percent_Open = 23;   // 15% (not linear voltage) of 100, sets PWM duration.
byte Set_Valve_50_Percent_Open = 50;   // 50% of 100, sets PWM duration.
byte Set_Valve_100_Percent_Open = 99;  // 100% of 100, sets PWM duration.

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

byte VFD_30_HP_Pump = 0;
byte VFD_Conveyor_Sluice = 0;

byte VFD_Concentrator1_Low_Speed = 0;   // Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator1_High_Speed = 0;  // (On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator2_Low_Speed = 0;   //Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator2_High_Speed = 0;  // (On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator3_Low_Speed = 0;   //  Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator3_High_Speed = 0;  //(On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator4_Low_Speed = 0;   //  Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator4_High_Speed = 0;  // (On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator5_Low_Speed = 0;   //  Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator5_High_Speed = 0;  // (On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator6_Low_Speed = 0;   //  Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator6_High_Speed = 0;  // (On - Signal HIGH) or Signal LOW is off )

//byte VFD_Booster_Pump_15HP_Low_Speed = 0;  //not used, but this relay could be later on
byte VFD_Table = 0;

byte VFD_Starboard_Sump = 0;
byte VFD_Port_Screen = 0;
byte VFD_3_Tsurumi = 0;
byte VFD_2_Tsurumi = 0;
byte VFD_Tails_Pump = 0;    //Also has rheostat connected directly to the VFD
byte VFD_Starboard_Screen = 0;          // Set pin 43 to control the 60 hp pump
byte VFD_Port_Sump = 0;          // //Also has rheostat connected directly to the VFD
byte VFD_Riven_Pumps = 0;  //Also has rheostat connected directly to the VFD
byte VFD_Sluice_Pumps = 0;


byte Ball_Valve1 = Set_Valve_15_Percent_Open;  // These 3 pins support PWM and we'll control the valves this way.
byte Ball_Valve2 = Set_Valve_15_Percent_Open;
byte Ball_Valve3 = Set_Valve_15_Percent_Open;
byte Ball_Valve4 = Set_Valve_15_Percent_Open;
byte Ball_Valve5 = Set_Valve_15_Percent_Open;
byte Ball_Valve6 = Set_Valve_15_Percent_Open;

byte Conc1_1in_Valve = 0;  // 0 is off, 1 is on
byte Conc2_1in_Valve = 0;
byte Conc3_1in_Valve = 0;
byte Conc4_1in_Valve = 0;
byte Conc5_1in_Valve = 0;
byte Conc6_1in_Valve = 0;

byte Conc1_Air_Valve = 0;
byte Conc2_Air_Valve = 0;
byte Conc3_Air_Valve = 0;
byte Conc4_Air_Valve = 0;
byte Conc5_Air_Valve = 0;
byte Conc6_Air_Valve = 0;



// initialize the library with the numbers of the interface pins


// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
//uint8_t broadcastAddress1[] = {0x84, 0xCC, 0xA8, 0x83, 0x6A, 0x85};
uint8_t broadcastAddress1[] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
//uint8_t broadcastAddress2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
//uint8_t broadcastAddress3[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
/* Below is the structure used to comunicate with remote concentrator module
   This module will control the 1in and air valves through relays and the
   ball valves are controlled by values passed to the PWM module to set a
   a volatage between 0 and 10 volts. We are using 12 relays out of the 16
   possible relays */
typedef struct Valve_Contoller {
  uint16_t valve_relay_states;
  uint8_t Ball_valve_1;
  uint8_t Ball_valve_2;
  uint8_t Ball_valve_3;
  uint8_t Ball_valve_4;
  uint8_t Ball_valve_5;
  uint8_t Ball_valve_6;
} Valve_Contoller;
Valve_Contoller Data_1;
/* For the next remote modlue we'll only be talking to the relays
 * so it's stucture is just one word of information. Since we have
 * more than 16 VFD states to control we'll use two of these modules
 */
typedef struct VFD_Contoller {
  uint16_t VFD_relay_states;
} VFD_Contoller;
VFD_Contoller Data_2;

typedef struct VFD_Contoller2 {
  uint16_t VFD2_relay_states;
} VFD_Contoller2;
VFD_Contoller2 Data_3;

byte New_Event = 1;
byte Valve_Problem = 0;  // 0 means we connected with it, 1 means we failed to connect.
byte VFD1_Problem = 0;
byte VFD2_Problem = 0;
byte Valve_retry_counter = 0;
byte VFD1_retry_counter = 0;
byte VFD2_retry_counter = 0;



byte Valve_Module_Status = 0;  // 0 means we connected with it, 1 means we failed to connect.
byte VFD1_Module_Status = 0;
byte VFD2_Module_Status = 0;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  //Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print(macStr);
  //Serial.print(" send status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (mac_addr[4] == 0x01) {
    if (status == 1) {
      Valve_Problem = 1;
    } else {
      Valve_Problem = 0;
    }
  }
  
}

// Replace with your network credentials
const char *ssid = "Gold_Plant";
const char *password = "123456789";

// Set web server port number to 80
WiFiServer server(80);
String header;
TaskHandle_t Web;
//TaskHandle_t Plant;
uint16_t Invertword = 65535;
uint16_t BitInvert =0;
void pf575_write(uint16_t data, int i2c_address) 
{
  //for the new 16 channel relay cards we need to inver the bits
  //BitInvert = data ^ Invertword;
  // *******************************
  Wire.beginTransmission(i2c_address);
  Wire.write(lowByte(data));
  Wire.write(highByte(data));
  Wire.endTransmission();
}
// ***************************************************************
void setup() {

  //Initiate Serial communication.
  Serial.begin(115200);

  // before we push the start button.
  // The state of the pins can be defined as HIGH or LOW. So that is what you'll see throughout the program.
  Wire.begin();
  // Set all ports as output
  pf575_write(word(B11111111,B11111111), i2c_Relay1);
  pf575_write(word(B11111111,B11111111), i2c_Relay2);

  
  // WiFi.mode(WIFI_AP_STA);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);  // register the callback function
  // register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); //This is added for use with newest IDE
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer1");
    Valve_Problem = 1;
    return;
  }
 
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  ArduinoOTA 
      .onStart([]() { 
        String type; 
        if (ArduinoOTA.getCommand() == U_FLASH) 
          type = "sketch"; 
        else // U_SPIFFS 
          type = "filesystem"; 
 
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end() 
        Serial.println("Start updating " + type); 
      }) 
      .onEnd([]() { 
        Serial.println("\nEnd"); 
      }) 
      .onProgress([](unsigned int progress, unsigned int total) { 
        Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
      }) 
      .onError([](ota_error_t error) { 
        Serial.printf("Error[%u]: ", error); 
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed"); 
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed"); 
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed"); 
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed"); 
        else if (error == OTA_END_ERROR) Serial.println("End Failed"); 
      }); 
 
    ArduinoOTA.begin(); 
  /////

  server.begin();
  //set all relays to off and ball valves to 15% flow
  Data_1.valve_relay_states = 0;
  Data_1.Ball_valve_1 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_2 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_3 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_4 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_5 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_6 = Set_Valve_15_Percent_Open;
  esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *)&Data_1, sizeof(Valve_Contoller));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
display.setFont(&FreeSans9pt7b);
  display.begin(i2c_Address, true); // Address 0x3C default
  //display.setFont(&FreeMono9pt7b);
  
  
  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,12);
  display.println("Relay Tester");
  
  display.display();
  delay(5000);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Web1",    /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Web,      /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}  // end of setup

void Task1code(void *pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    Deal_With_client();
    vTaskDelay(20);
  }
}

void loop() {

  // Below updates changes that occur from the control person
  // Since we refresh every 5 seconds, check to see if this is
  // a real change from the controller
  //First time through loop this will try to talk to all remote modules
  //REMOTE UPLOAD 
  ArduinoOTA.handle(); 
  //////
  if (New_Event == 1) {
    New_Event = 0;
    Serial.println("Saw New_Event");
    // Update all of the relay modules because we know something should change
    Update_Valve_Module();
    delay(500);
    Update_VFD1_Module();
    delay(500);
    Update_VFD2_Module();
    delay(500);
  }
  delay(1000);
  //Deal_With_client();
}  //end loop


void Deal_With_client() {
  WiFiClient client = server.available();  // Listen for incoming clients
  //int Time_in_Sec = Time_in_Seconds ;
  String Upper_Content = "";
  String Lower_Content = "";
  String Stopstart_Content = "";
  if (client) {  // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");                                             // print a message out in the serial port
    String currentLine = "";                                                   // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {  // if there's bytes to read from the client,
        char c = client.read();  // read a byte, then
        //Serial.write(c);         // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          delay(1);
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            Serial.println("HTTP/1.1 200 OK");
            // turns the GPIOs on and off

            if (header.indexOf("GET /Refresh") >= 0) {
              Upper_Content = Upper();
              client.println(Upper_Content);
              //Plant_Run_State = Plant_stopped;
              Serial.println("Saw refresh");
              goto END;  //Don't need to refresh the entire page
            }


            if (header.indexOf("GET /Conc_1") >= 0) {
              if (VFD_Concentrator1_High_Speed == 0 & VFD_Concentrator1_Low_Speed == 0) {
                VFD_Concentrator1_High_Speed = 1;
                VFD_Concentrator1_Low_Speed = 0;
              } else if (VFD_Concentrator1_High_Speed == 1 & VFD_Concentrator1_Low_Speed == 0) {
                VFD_Concentrator1_High_Speed = 1;
                VFD_Concentrator1_Low_Speed = 1;
              } else if (VFD_Concentrator1_High_Speed == 1 & VFD_Concentrator1_Low_Speed == 1) {
                VFD_Concentrator1_High_Speed = 0;
                VFD_Concentrator1_Low_Speed = 0;
              }
              Update_VFD1_Module();
            }

            if (header.indexOf("GET /Conc_2") >= 0) {
              if (VFD_Concentrator2_High_Speed == 0 & VFD_Concentrator2_Low_Speed == 0) {
                VFD_Concentrator2_High_Speed = 1;
                VFD_Concentrator2_Low_Speed = 0;
              } else if (VFD_Concentrator2_High_Speed == 1 & VFD_Concentrator2_Low_Speed == 0) {
                VFD_Concentrator2_High_Speed = 1;
                VFD_Concentrator2_Low_Speed = 1;
              } else if (VFD_Concentrator2_High_Speed == 1 & VFD_Concentrator2_Low_Speed == 1) {
                VFD_Concentrator2_High_Speed = 0;
                VFD_Concentrator2_Low_Speed = 0;
              }
              Update_VFD1_Module();
            }

            if (header.indexOf("GET /Conc_3") >= 0) {
              if (VFD_Concentrator3_High_Speed == 0 & VFD_Concentrator3_Low_Speed == 0) {
                VFD_Concentrator3_High_Speed = 1;
                VFD_Concentrator3_Low_Speed = 0;
              } else if (VFD_Concentrator3_High_Speed == 1 & VFD_Concentrator3_Low_Speed == 0) {
                VFD_Concentrator3_High_Speed = 1;
                VFD_Concentrator3_Low_Speed = 1;
              } else if (VFD_Concentrator3_High_Speed == 1 & VFD_Concentrator3_Low_Speed == 1) {
                VFD_Concentrator3_High_Speed = 0;
                VFD_Concentrator3_Low_Speed = 0;
              }
              Update_VFD1_Module();
            }

            if (header.indexOf("GET /Conc_4") >= 0) {
              if (VFD_Concentrator4_High_Speed == 0 & VFD_Concentrator4_Low_Speed == 0) {
                VFD_Concentrator4_High_Speed = 1;
                VFD_Concentrator4_Low_Speed = 0;
              } else if (VFD_Concentrator4_High_Speed == 1 & VFD_Concentrator4_Low_Speed == 0) {
                VFD_Concentrator4_High_Speed = 1;
                VFD_Concentrator4_Low_Speed = 1;
              } else if (VFD_Concentrator4_High_Speed == 1 & VFD_Concentrator4_Low_Speed == 1) {
                VFD_Concentrator4_High_Speed = 0;
                VFD_Concentrator4_Low_Speed = 0;
              }
              Update_VFD1_Module();
            }

            if (header.indexOf("GET /Conc_5") >= 0) {
              if (VFD_Concentrator5_High_Speed == 0 & VFD_Concentrator5_Low_Speed == 0) {
                VFD_Concentrator5_High_Speed = 1;
                VFD_Concentrator5_Low_Speed = 0;
              } else if (VFD_Concentrator5_High_Speed == 1 & VFD_Concentrator5_Low_Speed == 0) {
                VFD_Concentrator5_High_Speed = 1;
                VFD_Concentrator5_Low_Speed = 1;
              } else if (VFD_Concentrator5_High_Speed == 1 & VFD_Concentrator5_Low_Speed == 1) {
                VFD_Concentrator5_High_Speed = 0;
                VFD_Concentrator5_Low_Speed = 0;
              }
              Update_VFD1_Module();
            }

            if (header.indexOf("GET /Conc_6") >= 0) {
              if (VFD_Concentrator6_High_Speed == 0 & VFD_Concentrator6_Low_Speed == 0) {
                VFD_Concentrator6_High_Speed = 1;
                VFD_Concentrator6_Low_Speed = 0;
              } else if (VFD_Concentrator6_High_Speed == 1 & VFD_Concentrator6_Low_Speed == 0) {
                VFD_Concentrator6_High_Speed = 1;
                VFD_Concentrator6_Low_Speed = 1;
              } else if (VFD_Concentrator6_High_Speed == 1 & VFD_Concentrator6_Low_Speed == 1) {
                VFD_Concentrator6_High_Speed = 0;
                VFD_Concentrator6_Low_Speed = 0;
              }
              Update_VFD1_Module();
            }

            if (header.indexOf("GET /Ball_Valve1") >= 0) {
              if (Ball_Valve1 == Set_Valve_15_Percent_Open) {
                Ball_Valve1 = Set_Valve_50_Percent_Open;
              } else if (Ball_Valve1 == Set_Valve_50_Percent_Open) {
                Ball_Valve1 = Set_Valve_100_Percent_Open;
              } else if (Ball_Valve1 == Set_Valve_100_Percent_Open) {
                Ball_Valve1 = Set_Valve_15_Percent_Open;
              }
              Update_Valve_Module();
            }

            if (header.indexOf("GET /Ball_Valve2") >= 0) {
              if (Ball_Valve2 == Set_Valve_15_Percent_Open) {
                Ball_Valve2 = Set_Valve_50_Percent_Open;
              } else if (Ball_Valve2 == Set_Valve_50_Percent_Open) {
                Ball_Valve2 = Set_Valve_100_Percent_Open;
              } else if (Ball_Valve2 == Set_Valve_100_Percent_Open) {
                Ball_Valve2 = Set_Valve_15_Percent_Open;
              }
              Update_Valve_Module();
            }

            if (header.indexOf("GET /Ball_Valve3") >= 0) {
              if (Ball_Valve3 == Set_Valve_15_Percent_Open) {
                Ball_Valve3 = Set_Valve_50_Percent_Open;
              } else if (Ball_Valve3 == Set_Valve_50_Percent_Open) {
                Ball_Valve3 = Set_Valve_100_Percent_Open;
              } else if (Ball_Valve3 == Set_Valve_100_Percent_Open) {
                Ball_Valve3 = Set_Valve_15_Percent_Open;
              }
              Update_Valve_Module();
            }

            if (header.indexOf("GET /Ball_Valve4") >= 0) {
              if (Ball_Valve4 == Set_Valve_15_Percent_Open) {
                Ball_Valve4 = Set_Valve_50_Percent_Open;
              } else if (Ball_Valve4 == Set_Valve_50_Percent_Open) {
                Ball_Valve4 = Set_Valve_100_Percent_Open;
              } else if (Ball_Valve4 == Set_Valve_100_Percent_Open) {
                Ball_Valve4 = Set_Valve_15_Percent_Open;
              }
              Update_Valve_Module();
            }

            if (header.indexOf("GET /Ball_Valve5") >= 0) {
              if (Ball_Valve5 == Set_Valve_15_Percent_Open) {
                Ball_Valve5 = Set_Valve_50_Percent_Open;
              } else if (Ball_Valve5 == Set_Valve_50_Percent_Open) {
                Ball_Valve5 = Set_Valve_100_Percent_Open;
              } else if (Ball_Valve5 == Set_Valve_100_Percent_Open) {
                Ball_Valve5 = Set_Valve_15_Percent_Open;
              }
              Update_Valve_Module();
            }

            if (header.indexOf("GET /Ball_Valve6") >= 0) {
              if (Ball_Valve6 == Set_Valve_15_Percent_Open) {
                Ball_Valve6 = Set_Valve_50_Percent_Open;
              } else if (Ball_Valve6 == Set_Valve_50_Percent_Open) {
                Ball_Valve6 = Set_Valve_100_Percent_Open;
              } else if (Ball_Valve6 == Set_Valve_100_Percent_Open) {
                Ball_Valve6 = Set_Valve_15_Percent_Open;
              }
              Update_Valve_Module();
            }

            if (header.indexOf("GET /Conc1_1in") >= 0) {
              if (Conc1_1in_Valve == 1) {
                Conc1_1in_Valve = 0;
              } else {
                Conc1_1in_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc2_1in") >= 0) {
              if (Conc2_1in_Valve == 1) {
                Conc2_1in_Valve = 0;
              } else {
                Conc2_1in_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc3_1in") >= 0) {
              if (Conc3_1in_Valve == 1) {
                Conc3_1in_Valve = 0;
              } else {
                Conc3_1in_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc4_1in") >= 0) {
              if (Conc4_1in_Valve == 1) {
                Conc4_1in_Valve = 0;
              } else {
                Conc4_1in_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc5_1in") >= 0) {
              if (Conc5_1in_Valve == 1) {
                Conc5_1in_Valve = 0;
              } else {
                Conc5_1in_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc6_1in") >= 0) {
              if (Conc6_1in_Valve == 1) {
                Conc6_1in_Valve = 0;
              } else {
                Conc6_1in_Valve = 1;
              }
              Update_Valve_Module();
            }

            if (header.indexOf("GET /Conc1_Air") >= 0) {
              if (Conc1_Air_Valve == 1) {
                Conc1_Air_Valve = 0;
              } else {
                Conc1_Air_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc2_Air") >= 0) {
              if (Conc2_Air_Valve == 1) {
                Conc2_Air_Valve = 0;
              } else {
                Conc2_Air_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc3_Air") >= 0) {
              if (Conc3_Air_Valve == 1) {
                Conc3_Air_Valve = 0;
              } else {
                Conc3_Air_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc4_Air") >= 0) {
              if (Conc4_Air_Valve == 1) {
                Conc4_Air_Valve = 0;
              } else {
                Conc4_Air_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc5_Air") >= 0) {
              if (Conc5_Air_Valve == 1) {
                Conc5_Air_Valve = 0;
              } else {
                Conc5_Air_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Conc6_Air") >= 0) {
              if (Conc6_Air_Valve == 1) {
                Conc6_Air_Valve = 0;
              } else {
                Conc6_Air_Valve = 1;
              }
              Update_Valve_Module();
            }
            if (header.indexOf("GET /Port_Sump") >= 0) {
              if (VFD_Port_Sump == 0) {
                VFD_Port_Sump = 1;
              } else {
                VFD_Port_Sump = 0;
              }
              Update_VFD2_Module();
            }

            if (header.indexOf("GET /Sluice") >= 0) {
              if (VFD_Conveyor_Sluice == 0) {
                VFD_Conveyor_Sluice = 1;
              } else {
                VFD_Conveyor_Sluice = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /Starboard_Sump") >= 0) {
              if (VFD_Starboard_Sump == 0) {
                VFD_Starboard_Sump = 1;
              } else {
                VFD_Starboard_Sump = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /Tails_Pump") >= 0) {
              if (VFD_Tails_Pump == 0) {
                VFD_Tails_Pump = 1;
              } else {
                VFD_Tails_Pump = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /Port_Screen") >= 0) {
              if (VFD_Port_Screen == 0) {
                VFD_Port_Screen = 1;
              } else {
                VFD_Port_Screen = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /(3)_Tsurumi") >= 0) {
              if (VFD_3_Tsurumi == 0) {
                VFD_3_Tsurumi = 1;
              } else {
                VFD_3_Tsurumi = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /(2)_Tsurumi") >= 0) {
              if (VFD_2_Tsurumi == 0) {
                VFD_2_Tsurumi = 1;
              } else {
                VFD_2_Tsurumi = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /Starboard_Screen") >= 0) {
              if (VFD_Starboard_Screen == 0) {
                VFD_Starboard_Screen = 1;
              } else {
                VFD_Starboard_Screen = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /Riven_Pumps") >= 0) {
              if (VFD_Riven_Pumps == 0) {
                VFD_Riven_Pumps = 1;
              } else {
                VFD_Riven_Pumps = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /Sluice_Pumps") >= 0) {
              if (VFD_Sluice_Pumps == 0) {
                VFD_Sluice_Pumps = 1;
              } else {
                VFD_Sluice_Pumps = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /Table") >= 0) {
              if (VFD_Table == 0) {
                VFD_Table = 1;
              } else {
                VFD_Table = 0;
              }
              Update_VFD2_Module();
            }
            /*if (header.indexOf("GET /Transfer") >= 0) {
              if (VFD_20HP_Transfer_Pump == 0) {
                VFD_20HP_Transfer_Pump = 1;
              } else {
                VFD_20HP_Transfer_Pump = 0;
              }
              Update_VFD2_Module();
            }
            if (header.indexOf("GET /Booster") >= 0) {
              if (VFD_Booster_Pump_15HP == 0) {
                VFD_Booster_Pump_15HP = 1;
              } else {
                VFD_Booster_Pump_15HP = 0;
              }
              Update_VFD2_Module();
            }*/
            if (header.indexOf("GET /Update") >= 0) {
              Serial.println("Saw Update");
              Lower_Content = Lower();
              client.println(Lower_Content);
              goto END;  //Don't need to refresh the entire page
            }            // end of Update



            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            //client.println("<meta http-equiv='Refresh' content='5;url=http://192.168.4.1/Refresh'/>");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons

            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: 2px solid black; color: white; width: 15vw;");
            client.println("text-decoration: none; height:7vw; font-size: 2vw; margin: 0px; vertical-align:middle; cursor: pointer;}");
            client.println(".btn_off {background-color: red; text-align: center;}");
            client.println(".btn_slow {background-color: blue; text-align: center;}");
            client.println(".btn_50 {background-color: yellow; text-align: center;}");
            client.println(".btn_dis {border-radius: 12px; cursor: not-allowed; border: 2px solid white; text-align: center;}");
            client.println(".btn_large {width: 30vw; height:14vw; font-size: 5vw;}");
            client.println(".btn_Med {width: 18vw; height:9vw; font-size: 2vw;}");
            client.println("</style></head>");
            client.println("<body>");
            // The upper division contains the messages and active buttons
            client.println("<div id='upper'>");
            Upper_Content = Upper();
            client.println(Upper_Content);
            client.println("</div>");

            Lower_Content = Lower();
            client.println(Lower_Content);
            client.println("</div>");
            /*
            client.println("<script>");
            client.println("setInterval(function ( ) {");
            client.println("var xhttp = new XMLHttpRequest();");
            client.println("xhttp.onreadystatechange = function() {");
            client.println("if (this.readyState == 4 && this.status == 200) {");
            client.println("document.getElementById('states').innerHTML = this.responseText; } };");
            client.println("xhttp.open('GET', '/Update', true);");
            client.println("xhttp.send();");
            client.println("},1333 ) ;");
            client.println("</script>");

            client.println("<script>");
              client.println("setInterval(function ( ) {");
              client.println("var xhttp = new XMLHttpRequest();");
              client.println("xhttp.onreadystatechange = function() {");
              client.println("if (this.readyState == 4 && this.status == 200) {");
              client.println("document.getElementById('upper').innerHTML = this.responseText; } };");
              client.println("xhttp.open('GET', '/Refresh', true);");
              client.println("xhttp.send();");
              client.println("}, 10000 ) ;");
            client.println("</script>");

            client.println("<script>");
              client.println("setInterval(function ( ) {");
              client.println("var xhttp = new XMLHttpRequest();");
              client.println("xhttp.onreadystatechange = function() {");
              client.println("if (this.readyState == 4 && this.status == 200) {");
              client.println("document.getElementById('Remaining_time').innerHTML = this.responseText; } };");
              client.println("xhttp.open('GET', '/Remaining_time', true);");
              client.println("xhttp.send();");
              client.println("}, 10000 ) ;");
            client.println("</script>");  */

            client.println("</body>");
            //}
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop

END:
            break;
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}  //end of deal with client
String Lower() {
  String Value = "";

  if (VFD_Concentrator1_High_Speed == 0)
    Value += ("<a href=\"/Conc_1\"><button class=\"button btn_off\">Conc 1 on</button>");
  if (VFD_Concentrator1_High_Speed == 1 & VFD_Concentrator1_Low_Speed == 0)
    Value += ("<a href=\"/Conc_1\"><button class=\"button btn_on\">Conc 1 slow</button>");
  if (VFD_Concentrator1_High_Speed == 1 & VFD_Concentrator1_Low_Speed == 1)
    Value += ("<a href=\"/Conc_1\"><button class=\"button btn_slow\">Conc 1 off</button>");

  if (VFD_Concentrator2_High_Speed == 0)
    Value += ("<a href=\"/Conc_2\"><button class=\"button btn_off\">Conc 2 on</button>");
  if (VFD_Concentrator2_High_Speed == 1 & VFD_Concentrator2_Low_Speed == 0)
    Value += ("<a href=\"/Conc_2\"><button class=\"button btn_on\">Conc 2 slow</button>");
  if (VFD_Concentrator2_High_Speed == 1 & VFD_Concentrator2_Low_Speed == 1)
    Value += ("<a href=\"/Conc_2\"><button class=\"button btn_slow\">Conc 2 off</button>");

  if (VFD_Concentrator3_High_Speed == 0)
    Value += ("<a href=\"/Conc_3\"><button class=\"button btn_off\">Conc 3 on</button>");
  if (VFD_Concentrator3_High_Speed == 1 & VFD_Concentrator3_Low_Speed == 0)
    Value += ("<a href=\"/Conc_3\"><button class=\"button btn_on\">Conc 3 slow</button>");
  if (VFD_Concentrator3_High_Speed == 1 & VFD_Concentrator3_Low_Speed == 1)
    Value += ("<a href=\"/Conc_3\"><button class=\"button btn_slow\">Conc 3 off</button>");

  if (VFD_Concentrator4_High_Speed == 0)
    Value += ("<a href=\"/Conc_4\"><button class=\"button btn_off\">Conc 4 on</button>");
  if (VFD_Concentrator4_High_Speed == 1 & VFD_Concentrator4_Low_Speed == 0)
    Value += ("<a href=\"/Conc_4\"><button class=\"button btn_on\">Conc 4 slow</button>");
  if (VFD_Concentrator4_High_Speed == 1 & VFD_Concentrator4_Low_Speed == 1)
    Value += ("<a href=\"/Conc_4\"><button class=\"button btn_slow\">Conc 4 off</button>");

  if (VFD_Concentrator5_High_Speed == 0)
    Value += ("<a href=\"/Conc_5\"><button class=\"button btn_off\">Conc 5 on</button>");
  if (VFD_Concentrator5_High_Speed == 1 & VFD_Concentrator5_Low_Speed == 0)
    Value += ("<a href=\"/Conc_5\"><button class=\"button btn_on\">Conc 5 slow</button>");
  if (VFD_Concentrator5_High_Speed == 1 & VFD_Concentrator5_Low_Speed == 1)
    Value += ("<a href=\"/Conc_5\"><button class=\"button btn_slow\">Conc 5 off</button>");

  if (VFD_Concentrator6_High_Speed == 0)
    Value += ("<a href=\"/Conc_6\"><button class=\"button btn_off\">Conc 6 on</button>");
  if (VFD_Concentrator6_High_Speed == 1 & VFD_Concentrator6_Low_Speed == 0)
    Value += ("<a href=\"/Conc_6\"><button class=\"button btn_on\">Conc 6 slow</button>");
  if (VFD_Concentrator6_High_Speed == 1 & VFD_Concentrator6_Low_Speed == 1)
    Value += ("<a href=\"/Conc_6\"><button class=\"button btn_slow\">Conc 6 off</button>");

  if (Ball_Valve1 == Set_Valve_100_Percent_Open)
    Value += "<a href=\"/Ball_Valve1\"><button class=\"button btn_on\">Ball_Valve1 15%</button>";
  if (Ball_Valve1 == Set_Valve_50_Percent_Open)
    Value += "<a href=\"/Ball_Valve1\"><button class=\"button btn_off\">Ball_Valve1 100%</button>";
  if (Ball_Valve1 == Set_Valve_15_Percent_Open)
    Value += "<a href=\"/Ball_Valve1\"><button class=\"button btn_slow\">Ball_Valve1 50%</button>";

  if (Ball_Valve2 == Set_Valve_100_Percent_Open)
    Value += "<a href=\"/Ball_Valve2\"><button class=\"button btn_on\">Ball_Valve2 15%</button>";
  if (Ball_Valve2 == Set_Valve_50_Percent_Open)
    Value += "<a href=\"/Ball_Valve2\"><button class=\"button btn_off\">Ball_Valve2 100%</button>";
  if (Ball_Valve2 == Set_Valve_15_Percent_Open)
    Value += "<a href=\"/Ball_Valve2\"><button class=\"button btn_slow\">Ball_Valve2 50%</button>";

  if (Ball_Valve3 == Set_Valve_100_Percent_Open)
    Value += "<a href=\"/Ball_Valve3\"><button class=\"button btn_on\">Ball_Valve3 15%</button>";
  if (Ball_Valve3 == Set_Valve_50_Percent_Open)
    Value += "<a href=\"/Ball_Valve3\"><button class=\"button btn_off\">Ball_Valve3 100%</button>";
  if (Ball_Valve3 == Set_Valve_15_Percent_Open)
    Value += "<a href=\"/Ball_Valve3\"><button class=\"button btn_slow\">Ball_Valve3 50%</button>";

  if (Ball_Valve4 == Set_Valve_100_Percent_Open)
    Value += "<a href=\"/Ball_Valve4\"><button class=\"button btn_on\">Ball_Valve4 15%</button>";
  if (Ball_Valve4 == Set_Valve_50_Percent_Open)
    Value += "<a href=\"/Ball_Valve4\"><button class=\"button btn_off\">Ball_Valve4 100%</button>";
  if (Ball_Valve4 == Set_Valve_15_Percent_Open)
    Value += "<a href=\"/Ball_Valve4\"><button class=\"button btn_slow\">Ball_Valve4 50%</button>";

  if (Ball_Valve5 == Set_Valve_100_Percent_Open)
    Value += "<a href=\"/Ball_Valve5\"><button class=\"button btn_on\">Ball_Valve5 15%</button>";
  if (Ball_Valve5 == Set_Valve_50_Percent_Open)
    Value += "<a href=\"/Ball_Valve5\"><button class=\"button btn_off\">Ball_Valve5 100%</button>";
  if (Ball_Valve5 == Set_Valve_15_Percent_Open)
    Value += "<a href=\"/Ball_Valve5\"><button class=\"button btn_slow\">Ball_Valve5 50%</button>";

  if (Ball_Valve6 == Set_Valve_100_Percent_Open)
    Value += "<a href=\"/Ball_Valve6\"><button class=\"button btn_on\">Ball_Valve6 15%</button>";
  if (Ball_Valve6 == Set_Valve_50_Percent_Open)
    Value += "<a href=\"/Ball_Valve6\"><button class=\"button btn_off\">Ball_Valve6 100%</button>";
  if (Ball_Valve6 == Set_Valve_15_Percent_Open)
    Value += "<a href=\"/Ball_Valve6\"><button class=\"button btn_slow\">Ball_Valve6 50%</button>";

  if (Conc1_1in_Valve == 1) {
    Value += "<a href=\"/Conc1_1in\"><button class=\"button btn_on\">#1 1in Valve off</button>";
  } else {
    Value += "<a href=\"/Conc1_1in\"><button class=\"button btn_off\">#1 1in Valve on</button>";
  }
  if (Conc2_1in_Valve == 1) {
    Value += "<a href=\"/Conc2_1in\"><button class=\"button btn_on\">#2 1in Valve off</button>";
  } else {
    Value += "<a href=\"/Conc2_1in\"><button class=\"button btn_off\">#2 1in Valve on</button>";
  }
  if (Conc3_1in_Valve == 1) {
    Value += "<a href=\"/Conc3_1in\"><button class=\"button btn_on\">#3 1in Valve off</button>";
  } else {
    Value += "<a href=\"/Conc3_1in\"><button class=\"button btn_off\">#3 1in Valve on</button>";
  }
  if (Conc4_1in_Valve == 1) {
    Value += "<a href=\"/Conc4_1in\"><button class=\"button btn_on\">#4 1in Valve off</button>";
  } else {
    Value += "<a href=\"/Conc4_1in\"><button class=\"button btn_off\">#4 1in Valve on</button>";
  }
  if (Conc5_1in_Valve == 1) {
    Value += "<a href=\"/Conc5_1in\"><button class=\"button btn_on\">#5 1in Valve off</button>";
  } else {
    Value += "<a href=\"/Conc5_1in\"><button class=\"button btn_off\">#5 1in Valve on</button>";
  }
  if (Conc6_1in_Valve == 1) {
    Value += "<a href=\"/Conc6_1in\"><button class=\"button btn_on\">#6 1in Valve off</button>";
  } else {
    Value += "<a href=\"/Conc6_1in\"><button class=\"button btn_off\">#6 1in Valve on</button>";
  }

  if (Conc1_Air_Valve == 1) {
    Value += "<a href=\"/Conc1_Air\"><button class=\"button btn_on\">#1 Air Valve off</button>";
  } else {
    Value += "<a href=\"/Conc1_Air\"><button class=\"button btn_off\">#1 Air Valve on</button>";
  }
  if (Conc2_Air_Valve == 1) {
    Value += "<a href=\"/Conc2_Air\"><button class=\"button btn_on\">#2 Air Valve off</button>";
  } else {
    Value += "<a href=\"/Conc2_Air\"><button class=\"button btn_off\">#2 Air Valve on</button>";
  }
  if (Conc3_Air_Valve == 1) {
    Value += "<a href=\"/Conc3_Air\"><button class=\"button btn_on\">#3 Air Valve off</button>";
  } else {
    Value += "<a href=\"/Conc3_Air\"><button class=\"button btn_off\">#3 Air Valve on</button>";
  }
  if (Conc4_Air_Valve == 1) {
    Value += "<a href=\"/Conc4_Air\"><button class=\"button btn_on\">#4 Air Valve off</button>";
  } else {
    Value += "<a href=\"/Conc4_Air\"><button class=\"button btn_off\">#4 Air Valve on</button>";
  }
  if (Conc5_Air_Valve == 1) {
    Value += "<a href=\"/Conc5_Air\"><button class=\"button btn_on\">#5 Air Valve off</button>";
  } else {
    Value += "<a href=\"/Conc5_Air\"><button class=\"button btn_off\">#5 Air Valve on</button>";
  }
  if (Conc6_Air_Valve == 1) {
    Value += "<a href=\"/Conc6_Air\"><button class=\"button btn_on\">#6 Air Valve off</button>";
  } else {
    Value += "<a href=\"/Conc6_Air\"><button class=\"button btn_off\">#6 Air Valve on</button>";
  }

  if (VFD_Starboard_Sump == 1) {
    Value += "<a href=\"/Starboard_Sump\"><button class=\"button btn_on\">Starboard Sump Off</button>";
  } else {
    Value += "<a href=\"/Starboard_Sump\"><button class=\"button btn_off\">Starboard Sump On</button>";
  }

  if (VFD_Starboard_Screen == 1) {
    Value += "<a href=\"/Starboard_Screen\"><button class=\"button btn_on\">Starboard Screen Off</button>";
  } else {
    Value += "<a href=\"/Starboard_Screen\"><button class=\"button btn_off\">Starboard Screen On</button>";
  }

  if (VFD_Port_Screen == 1) {
    Value += "<a href=\"/Port_Screen\"><button class=\"button btn_on\">Port Screen off</button>";
  } else {
    Value += "<a href=\"/Port_Screen\"><button class=\"button btn_off\">Port Screen on</button>";
  }
  if (VFD_Tails_Pump == 1) {
    Value += "<a href=\"/Tails_Pump\"><button class=\"button btn_on\">Tails Pump off</button>";
  } else {
    Value += "<a href=\"/Tails_Pump\"><button class=\"button btn_off\">Tails Pump on</button>";
  }
  if (VFD_3_Tsurumi == 1) {
    Value += "<a href=\"/(3)_Tsurumi\"><button class=\"button btn_on\">(3)_Tsurumi off</button>";
  } else {
    Value += "<a href=\"/(3)_Tsurumi\"><button class=\"button btn_off\">(3)_Tsurumi on</button>";
  }
  if (VFD_2_Tsurumi == 1) {
    Value += "<a href=\"/(2)_Tsurumi\"><button class=\"button btn_on\">(2)_Tsurumi off</button>";
  } else {
    Value += "<a href=\"/(2)_Tsurumi\"><button class=\"button btn_off\">(2)_Tsurumi on</button>";
  }
  if (VFD_Port_Sump == 1) {
    Value += "<a href=\"/Port_Sump\"><button class=\"button btn_on\">Port Sump off</button>";
  } else {
    Value += "<a href=\"/Port_Sump\"><button class=\"button btn_off\">Port Sump on</button>";
  }
  if (VFD_Riven_Pumps == 1) {
    Value += "<a href=\"/Riven_Pumps\"><button class=\"button btn_on\">Riven Pumps off</button>";
  } else {
    Value += "<a href=\"/Riven_Pumps\"><button class=\"button btn_off\">Riven Pumps on</button>";
  }
  if (VFD_Sluice_Pumps == 1) {
    Value += "<a href=\"/Sluice_Pumps\"><button class=\"button btn_on\">Sluice Pumps off</button>";
  } else {
    Value += "<a href=\"/Sluice_Pumps\"><button class=\"button btn_off\">Sluice Pumps on</button>";
  }
  if (VFD_Table == 1) {
    Value += "<a href=\"/Table\"><button class=\"button btn_on\">Table off</button>";
  } else {
    Value += "<a href=\"/Table\"><button class=\"button btn_off\">Table on</button>";
  }
  /*
  if (VFD_60_ft_Conveyor == 1) {
    Value += "<a href=\"/60_ft\"><button class=\"button btn_on\">60ft Conveyor off</button>";
  } else {
    Value += "<a href=\"/60_ft\"><button class=\"button btn_off\">60ft Conveyor on</button>";
  }
  if (VFD_Booster_Pump_15HP == 1) {
    Value += "<a href=\"/Booster\"><button class=\"button btn_on\">Booster pump off</button>";
  } else {
    Value += "<a href=\"/Booster\"><button class=\"button btn_off\">Booster pump on</button>";
  }*/
  return Value;
}  // end of Lower function

String Upper() {
  Serial.print("Valve_Problem ");
  Serial.print(Valve_Problem);
  Serial.print(" VFD1_Problem ");
  Serial.print(VFD1_Problem);
  Serial.print(" VFD2_Problem ");
  Serial.println(VFD2_Problem);
  String Value = ("<h1>Test the concentrator relay control</h1>");
  if ((VFD2_Problem + VFD1_Problem + Valve_Problem) > 0) {
    // one or more of the modules isn't communicating

    if (Valve_Problem > 0) {
      Value += "<p>Check the Valve control module before starting</p>";
    }
    if (VFD1_Problem > 0) {
      Value += "<p>Check the 1st relay control module before starting</p>";
    }
    if (VFD2_Problem > 0) {
      Value += "<p>Check the 2nd relay control module before starting</p>";
    }
    //client.println("</div>");
    delay(100);  //wait for someone to press start button, stop button
                 // or for the plant to finish initializing or stopping
  }
  return Value;
}

void Update_Valve_Module() {

    Data_1.valve_relay_states &= 0b1111111111111110; Data_1.valve_relay_states |= Conc1_Air_Valve;
    Data_1.valve_relay_states &= 0b1111111111111101; Data_1.valve_relay_states |= Conc2_Air_Valve<<1;
    Data_1.valve_relay_states &= 0b1111111111111011; Data_1.valve_relay_states |= Conc3_Air_Valve<<2;
    Data_1.valve_relay_states &= 0b1111111111110111; Data_1.valve_relay_states |= Conc4_Air_Valve<<3;
    Data_1.valve_relay_states &= 0b1111111111101111; Data_1.valve_relay_states |= Conc5_Air_Valve<<4;
    Data_1.valve_relay_states &= 0b1111111111011111; Data_1.valve_relay_states |= Conc6_Air_Valve<<5;
//    Data_1.valve_relay_states &= 0b1111111110111111; Data_1.valve_relay_states |= Decanter_Air_Valve<<6;
    Data_1.valve_relay_states &= 0b1111111011111111; Data_1.valve_relay_states |= Conc1_1in_Valve<<8;
    Data_1.valve_relay_states &= 0b1111110111111111; Data_1.valve_relay_states |= Conc2_1in_Valve<<9;
    Data_1.valve_relay_states &= 0b1111101111111111; Data_1.valve_relay_states |= Conc3_1in_Valve<<10;
    Data_1.valve_relay_states &= 0b1111011111111111; Data_1.valve_relay_states |= Conc4_1in_Valve<<11;
    Data_1.valve_relay_states &= 0b1110111111111111; Data_1.valve_relay_states |= Conc5_1in_Valve<<12;
    Data_1.valve_relay_states &= 0b1101111111111111; Data_1.valve_relay_states |= Conc6_1in_Valve<<13;
    //Data_1.valve_relay_states &= 0b1011111111111111; Data_1.valve_relay_states |= VFD_Conveyor_Sluice<<14;
    //Data_1.valve_relay_states &= 0b0111111111111111; Data_1.valve_relay_states |= Conc2_Air_Valve<<15;
    
    Data_1.Ball_valve_1 = Ball_Valve1; // Ball_valveX will contain value to send to 0-10 module
    Data_1.Ball_valve_2 = Ball_Valve2;
    Data_1.Ball_valve_3 = Ball_Valve3;
    Data_1.Ball_valve_4 = Ball_Valve4;
    Data_1.Ball_valve_5 = Ball_Valve5;
    Data_1.Ball_valve_6 = Ball_Valve6;
  esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *)&Data_1, sizeof(Valve_Contoller));
  if (result == ESP_OK) {
    Serial.println("Sent with success 0x01");
  } else {
    Serial.println("Error sending the data");
  }
  delay(100);
  while (Valve_Problem == 1) {
    Serial.print("Valve_receive_error ");
    Valve_retry_counter++;
    if (Valve_retry_counter > 5) {
      Valve_retry_counter = 0;
      
      return;
    }
    esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *)&Data_1, sizeof(Valve_Contoller));
    delay(10);
  }
  Valve_retry_counter = 0;
  Valve_Problem = 0;
  //Deal_With_client();
}

void Update_VFD1_Module() {

  Data_2.VFD_relay_states &= 0b1111111011111111;
  Data_2.VFD_relay_states |= VFD_Concentrator1_High_Speed << 8;
  Data_2.VFD_relay_states &= 0b1111110111111111;
  Data_2.VFD_relay_states |= VFD_Concentrator1_Low_Speed << 9;
  Data_2.VFD_relay_states &= 0b1111101111111111;
  Data_2.VFD_relay_states |= VFD_Concentrator2_High_Speed << 10;
  Data_2.VFD_relay_states &= 0b1111011111111111;
  Data_2.VFD_relay_states |= VFD_Concentrator2_Low_Speed << 11;
  Data_2.VFD_relay_states &= 0b1110111111111111;
  Data_2.VFD_relay_states |= VFD_Concentrator3_High_Speed << 12;
  Data_2.VFD_relay_states &= 0b1101111111111111;
  Data_2.VFD_relay_states |= VFD_Concentrator3_Low_Speed << 13;

  Data_2.VFD_relay_states &= 0b1011111111111111;
  Data_2.VFD_relay_states |= VFD_Concentrator4_High_Speed << 14;
  Data_2.VFD_relay_states &= 0b0111111111111111;
  Data_2.VFD_relay_states |= VFD_Concentrator4_Low_Speed << 15;
  Data_2.VFD_relay_states &= 0b1111111111111110;
  Data_2.VFD_relay_states |= VFD_Concentrator5_High_Speed;
  Data_2.VFD_relay_states &= 0b1111111111111101;
  Data_2.VFD_relay_states |= VFD_Concentrator5_Low_Speed << 1;
  Data_2.VFD_relay_states &= 0b1111111111111011;
  Data_2.VFD_relay_states |= VFD_Concentrator6_High_Speed << 2;
  Data_2.VFD_relay_states &= 0b1111111111110111;
  Data_2.VFD_relay_states |= VFD_Concentrator6_Low_Speed << 3;

  BitInvert = Data_2.VFD_relay_states ^ Invertword;
  Wire.beginTransmission(i2c_Relay1);
  Wire.write(lowByte(BitInvert));
  Wire.write(highByte(BitInvert));
  Wire.endTransmission();
}  // end Update_VFD1_Module
void Update_VFD2_Module() {

  Data_3.VFD2_relay_states &= 0b1111111011111111; Data_3.VFD2_relay_states |= VFD_Starboard_Sump<<8;
  Data_3.VFD2_relay_states &= 0b1111110111111111; Data_3.VFD2_relay_states |= VFD_Port_Sump<<9;
  Data_3.VFD2_relay_states &= 0b1111101111111111; Data_3.VFD2_relay_states |= VFD_Starboard_Screen<<10;
  Data_3.VFD2_relay_states &= 0b1111011111111111; Data_3.VFD2_relay_states |= VFD_Port_Screen<<11;
  Data_3.VFD2_relay_states &= 0b1110111111111111; Data_3.VFD2_relay_states |= VFD_3_Tsurumi<<12;
  Data_3.VFD2_relay_states &= 0b1101111111111111; Data_3.VFD2_relay_states |= VFD_2_Tsurumi<<13;
  Data_3.VFD2_relay_states &= 0b1011111111111111; Data_3.VFD2_relay_states |= VFD_Tails_Pump<<14;
  Data_3.VFD2_relay_states &= 0b0111111111111111; Data_3.VFD2_relay_states |= VFD_Riven_Pumps<<15;
  
  Data_3.VFD2_relay_states &= 0b1111111111111110; Data_3.VFD2_relay_states |= VFD_Sluice_Pumps;
  Data_3.VFD2_relay_states &= 0b1111111111111101; Data_3.VFD2_relay_states |= VFD_Table<<1;
  //Data_3.VFD2_relay_states &= 0b1111111111111011; Data_3.VFD2_relay_states |= VFD_30_HP_Pump<<2;
  //Data_3.VFD2_relay_states &= 0b1111111111110111; Data_3.VFD2_relay_states |= VFD_50_HP_Pump<<3;
  //Data_3.VFD2_relay_states &= 0b1111111111101111; Data_3.VFD2_relay_states |= VFD_60_ft_Conveyor<<4;
  //Data_3.VFD2_relay_states &= 0b1111111111011111; Data_3.VFD2_relay_states |= VFD_60_ft_Conveyor<<5;
  //Data_3.VFD2_relay_states &= 0b1111111110111111; Data_3.VFD2_relay_states |= VFD_Submersible_Pump<<6;

  BitInvert = Data_3.VFD2_relay_states ^ Invertword;
    Wire.beginTransmission(i2c_Relay2);
    Wire.write(lowByte(BitInvert));
    Wire.write(highByte(BitInvert));
    Wire.endTransmission();
}
