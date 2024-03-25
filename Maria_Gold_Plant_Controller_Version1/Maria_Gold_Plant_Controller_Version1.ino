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


#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
//#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
//U8G2_SSD1306_128X64_ALT0_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#define i2c_Relay1 0x27   // i2c slave address for relay module 1
#define i2c_Relay2 0x26   // i2c slave address for relay module 2

byte Set_Valve_15_Percent_Open = 30; // 15% (not linear voltage) of 100, sets PWM duration. 
byte Set_Valve_50_Percent_Open = 60; // 50% of 100, sets PWM duration.
byte Set_Valve_79_Percent_Open = 79; // New value to use on Concentrators 4,5,6.
byte Set_Valve_80_Percent_Open = 80; // New value to try better rinsing.

byte Set_Valve_100_Percent_Open = 99; // 100% of 100, sets PWM duration.

int Time_Between_Rinses = A0;  // This is the input voltage from the pot 5 volts is 60 minutes
// Start delay time variables, please use values divisible by 5000.

int One_Inch_Valve_Open_Delay = 10000;
int One_Inch_Valve_Close_Delay = 15000;
int Conveyor_Sluice_Clean_Time = 58; //This can't be greater than 59 seconds DAD
int Sequencer_Time; // Value between 0 and 1024;
String RinseTime = "11:00";
// Brute force Conveyor sluice clean and stop times
// These are set up for a 22 minute concentrator rinse cycle
// Since the concentrator rinse cycle takes about 2 minutes
// and we'll leave the conveyor stopped for that time, I'm using 6 minutes
// of conveyor stop time between the 2 minute rinse cycles to even this out.


/*byte sluice_on1 = 20; //after the first minute of the new 22 minute rinse cycle
                     // rotate sluice for 2 minutes;
byte sluice_off1 = 18; // turn off the sluice for next 6 minutes
byte sluice_on2 = 12;
byte sluice_off2 = 10; // at 11 minute mark turn off
byte sluice_on3 = 4;  // at 5 minute mark turn on again
byte sluice_off3 = 2; */
// New cycle values for September 2022 attempt at 11 minute cycles
byte sluice_on1 = 10; //after the first minute of the new 22 minute rinse cycle
                     // rotate sluice for 2 minutes;
byte sluice_off1 = 9; // turn off the sluice for next 6 minutes
byte sluice_on2 = 6;
byte sluice_off2 = 5; // at 11 minute mark turn off
byte sluice_on3 = 2;  // at 5 minute mark turn on again
byte sluice_off3 = 1; 


byte Use_Short_time =0;

byte Start_Timer =0;
// End delay time variables
// Set up the pins on the Arduino.

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

byte VFD_30_HP_Pump = 0;
byte VFD_Conveyor_Sluice = 0;

byte VFD_Concentrator1_Low_Speed = 0; // Signal HIGH sets concentrator to  panel preset low speed 
byte VFD_Concentrator1_High_Speed = 0; // (On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator2_Low_Speed = 0; //Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator2_High_Speed = 0; // (On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator3_Low_Speed = 0; //  Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator3_High_Speed = 0; //(On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator4_Low_Speed = 0; //  Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator4_High_Speed = 0; // (On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator5_Low_Speed = 0; //  Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator5_High_Speed = 0; // (On - Signal HIGH) or Signal LOW is off )

byte VFD_Concentrator6_Low_Speed = 0; //  Signal HIGH sets concentrator to  panel preset low speed
byte VFD_Concentrator6_High_Speed = 0; // (On - Signal HIGH) or Signal LOW is off )

byte VFD_Booster_Pump_15HP_Low_Speed = 0; //not used, but this relay could be later on
byte VFD_Booster_Pump_15HP = 0;

byte VFD_Tails_Conveyor = 0;
byte VFD_Screen = 0;
byte VFD_Primary_VT40 = 0;
byte VFD_Secondary_VT40 = 0;
byte VFD_Submersible_Pump = 0; //Also has rheostat connected directly to the VFD
byte VFD_60_HP_Pump = 0; // Set pin 43 to control the 60 hp pump
byte VFD_50_HP_Pump = 0; // //Also has rheostat connected directly to the VFD
byte VFD_Hopper = 0; //Also has rheostat connected directly to the VFD
byte VFD_20HP_Transfer_Pump = 0; //Also has rheostat connected directly to the VFD
byte VFD_60_ft_Conveyor = 0;


byte Ball_Valve1 = Set_Valve_15_Percent_Open; // These 3 pins support PWM and we'll control the valves this way.
byte Ball_Valve2 = Set_Valve_15_Percent_Open;
byte Ball_Valve3 = Set_Valve_15_Percent_Open;
byte Ball_Valve4 = Set_Valve_15_Percent_Open;
byte Ball_Valve5 = Set_Valve_15_Percent_Open;
byte Ball_Valve6 = Set_Valve_15_Percent_Open;

byte Conc1_1in_Valve = 0; // 0 is off, 1 is on
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

// Global variable definitions so we can use between functions
byte Concentrator_A_Low_Speed;
byte Concentrator_A_High_Speed;
byte Concentrator_B_Low_Speed; 
byte Concentrator_B_High_Speed;
byte Concentrator_C_Low_Speed;
byte Concentrator_C_High_Speed;
      
byte Concentrator_A_Air_Valve;
byte Concentrator_B_Air_Valve;
byte Concentrator_C_Air_Valve;
      
byte Concentrator_A_1p5in_Valve;
byte Concentrator_B_1p5in_Valve;
byte Concentrator_C_1p5in_Valve;
    
byte Concentrator_A_1in_Valve;
byte Concentrator_B_1in_Valve;
byte Concentrator_C_1in_Valve;

String Line1a = " Line 1 ";
String Line2a = " Line 2 ";
String Line3a = " Line 3 ";
String Line4a = " Line 4 ";
String Line5a = " Line 5 ";
String Line1b = " Line 6 ";
int CycleCount = 2; //start with 2 so we can use the remander function to switch between
                    // A, B, and C cycles
 
// initialize the library with the numbers of the interface pins



// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
//uint8_t broadcastAddress1[] = {0x84, 0xCC, 0xA8, 0x83, 0x6A, 0x85};
uint8_t broadcastAddress1[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
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
byte First_Time = 0; //signals to the web page to do the initial setup
byte New_Event = 1;
byte Valve_Problem = 0; // 0 means we connected with it, 1 means we failed to connect.
byte VFD1_Problem = 0;
byte VFD2_Problem = 0;
byte Valve_retry_counter = 0;
byte VFD1_retry_counter = 0;
byte VFD2_retry_counter = 0;

byte Plant_Run_State = 5; // 5 is plant off, 2 is plant starting, 3 is plant running, 4 is plant stopping
byte Last_State = 5;
byte Conveyor_sluice_manual_off = 0; //1 means were stopping the sluice for now, 0 means run the normal sluice sequence
const int Waiting_for_command = 1;
const int Initialize_plant = 2;
const int Plant_running = 3;
const int Plant_stopping = 4;
const int Plant_stopped = 5;

byte Valve_Module_Status = 0; // 0 means we connected with it, 1 means we failed to connect.
byte VFD1_Module_Status = 0;
byte VFD2_Module_Status = 0;
byte Valve_module_last_Cycle_Type = 7;
byte VFD1_module_last_Cycle_Type = 7;

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
  if (mac_addr[4] == 0x01){
    if(status == 1){
      Valve_Problem = 1;
    }
    else{
     Valve_Problem = 0; 
    }
  }
 
}
 
 // Replace with your network credentials
 const char* ssid     = "Gold";
 const char* password = "123456789";
 const int First_Cycle = 1;
 const int Second_Cycle = 2;
 const int Third_Cycle = 3;
 const int First_Cycle_firstpass = 1;
 const int Second_Cycle_firstpass = 2;
 const int Third_Cycle_firstpass = 3;
 const int First_Cycle_secondpass = 4;
 const int Second_Cycle_secondpass = 5;
 const int Third_Cycle_secondpass = 6;
 const int Normal_Control = 7;
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
  // get the time to wait betweeen Concentrator rinses
  // Sequencer_Time = analogRead(Time_Between_Rinses); // Get Pot value only once
  //Sequencer_Time = 358; // little greater than 21 minutes 1024*21/60
  
  //cut rinse times to 11 minutes ****************
  Sequencer_Time = 190; // little greater than 11 minutes 1024*21/60
  // *************************************
  
  //Initiate Serial communication.
  Serial.begin(115200);
  
  // before we push the start button.
  // The state of the pins can be defined as HIGH or LOW. So that is what you'll see throughout the program.
  Wire.begin();
  // Set all ports as output
  pf575_write(word(B11111111,B11111111), i2c_Relay1);
  pf575_write(word(B11111111,B11111111), i2c_Relay2);
  
 
 WiFi.mode(WIFI_AP_STA);
  //WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent); // register the callback function
  // register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); //This is added for use with newest arduino IDE
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
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
  
  server.begin();
  //set all relays to off and ball valves to 15% flow
  Data_1.valve_relay_states = 0;
  Data_1.Ball_valve_1 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_2 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_3 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_4 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_5 = Set_Valve_15_Percent_Open;
  Data_1.Ball_valve_6 = Set_Valve_15_Percent_Open;
  esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *) &Data_1, sizeof(Valve_Contoller));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
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
  display.println("Hello, world!");
  display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
  display.println(3.141592);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.print("0x"); display.println(0xDEADBEEF, HEX);
  display.display();
  delay(5000);
 //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Web1",     /* name of task. */
                    50000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Web,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */                  
  delay(500);
} // end of setup
 
void Task1code(void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    Deal_With_client();
    vTaskDelay(20);
  } 
}  

void loop() {
int Stop_Plant = 0;
int Cycle_Type = 0;


    switch (Plant_Run_State) {
      case Waiting_for_command:
        delay(1000); //wait for someone to press start button, stop button
                     // or for the plant to finish initializing or stopping
      break;
    case Initialize_plant:
      display.clearDisplay();
      //display.setTextSize(1);
      display.setCursor(0,12);
      display.print("Maria Control");
      display.setCursor(0,30);
      display.println("Initializing ");
      display.display();
      Turn_On_Plant();
      Last_State = Initialize_plant;
      Plant_Run_State = Waiting_for_command; //wait to finish initialization
      display.clearDisplay();
      //display.setTextSize(1);
      display.setCursor(0,12);
      display.print("Maria Control");
      display.setCursor(0,30);
      display.println("  Ready to ");
      display.println("  Start ");
      display.display();
      First_Time = 1; 
      break;
    case Plant_running:
    // This loop goes forever and runs every X minutes set by the pot
    // The range of delay is between 1 and 60 minutes
    // The time can be changed without re-starting the sequencer
    VFD_Hopper = 1 ;  // Turn on, Also has rheostat conected directly to the VFD
    Update_VFD2_Module();
    Last_State = Plant_running; 
      while (Plant_Run_State == Plant_running) {
      CycleCount++; //increment the cycle count
      Delay_Between_Rinses(CycleCount); //This function reads the pot and produces the delay
      // **** function that does the actual transition between each cycles
      if(Plant_Run_State == Plant_stopping) break;
      Begin_Rinse_Cycle(CycleCount);  
      }
      break;
    case Plant_stopping:
      Stop_Plant_And_Rinse_All_Concentrators();
      Plant_Run_State = Plant_stopped; //wait to do something else
      First_Time = 1;
      break;
   case Plant_stopped:
      delay(1000);
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,12);
      display.print("Maria Control");
      display.setCursor(0,26);
      display.print("   Waiting to ");
      display.setCursor(0,40);
      display.println("   Initialize");
      display.display();
      
      Last_State = Plant_stopped;
      // The web page will ask for someone to initialize the plant
      // This will just loop here untill that happens
      // There will be messages to tell operators if the remote
      // modules are comunicating.
      break;     
    } 
  
  // Below updates changes that occur from the control person
  // Since we refresh every 5 seconds, check to see if this is 
  // a real change from the controller
  //First time through loop this will try to talk to all remote modules
 if (New_Event == 1){
    New_Event = 0;
    Serial.println("Saw New_Event");
    // Update all of the relay modules because we know something should change
    Update_Valve_Module(7);
    delay(500);
    Update_VFD1_Module(7);
    delay(500);
    Update_VFD2_Module();
    delay(500);
 }
 //Deal_With_client();
} //end loop

  void Delay_Between_Rinses(int Count) {
    
  // int Sequencer_Time; // Value between 0 and 1024;
  // used to read every time through the loop, but we never adjusted this so it's read only once now in setup
  int Number_of_button_pushes = 0;
  int Time_in_Minutes;
  int TM;
  int Time_in_Seconds = 60;
  String temp;
  
  unsigned long lastTime;
  unsigned long lastTime2;
 
  // perform the delay between rinses *******
  Time_in_Minutes = (Sequencer_Time * 60) / 1024; //
  if(Count == 3)  Time_in_Minutes = Time_in_Minutes/2; //First time thru cut time in half

  RinseTime = String(Time_in_Minutes) +":"+ String(Time_in_Seconds);
  display.clearDisplay();
  display.setCursor(0, 12);
  display.print("  Rinse in: ");
  display.setCursor(0, 28);
  display.print("     " + RinseTime);
  display.setCursor(0, 44);
  display.println(" Count = " + String(Count));
  display.display();
    lastTime = millis();
   while (Time_in_Minutes >= 0) {
    while (Time_in_Seconds >= 1) {
      while (millis() - lastTime <= 1000) {
      }
      lastTime = millis();
      Time_in_Seconds -= 1; //Subtract 1 second each time through
      display.setCursor(0, 28);
      display.setTextColor(SH110X_BLACK, SH110X_WHITE); // 'inverted' text
      display.print("     " + RinseTime);
      display.display();
      
      temp = String(Time_in_Seconds);
      temp = Time_in_Seconds < 10 ? "0" + temp : temp;
      RinseTime = String(Time_in_Minutes) +":"+ temp;
      display.setCursor(0, 28);
      display.setTextColor(SH110X_WHITE, SH110X_BLACK); // 
      display.print("     " + RinseTime);
      display.display();
      if (Plant_Run_State == Plant_stopping)
      return;
    } 
    // This section runs once every minute
  Time_in_Minutes -= 1; //Subtract 1 minute each time through
  New_Event = 1; // check the remote modules to see if they are still talking
  // New code for controlling the sluice on and off timing ***************
  TM = Time_in_Minutes;
  if (Conveyor_sluice_manual_off == 0){ 
    if (TM == sluice_on1 || TM == sluice_on2 || TM == sluice_on3 ){
      VFD_Conveyor_Sluice = 1;
      display.setCursor(0, 63);
      display.print("Conv Sluice on");
      Update_VFD2_Module();
    }
  }
  if ( TM == sluice_off1 || TM == sluice_off2 || TM == sluice_off3){
    VFD_Conveyor_Sluice = 0;
    display.setCursor(0, 63);
    display.print("Conv Sluice off");
    Update_VFD2_Module();
  }
  //if (TM == 16 ){ This was used for 21 minute cycles
  if (TM == 8 ){ // Use this for 11 minute cycles
    if (Ball_Valve1 == Set_Valve_15_Percent_Open) Ball_Valve1 = Set_Valve_80_Percent_Open;
    if (Ball_Valve2 == Set_Valve_15_Percent_Open) Ball_Valve2 = Set_Valve_80_Percent_Open;
    if (Ball_Valve3 == Set_Valve_15_Percent_Open) Ball_Valve3 = Set_Valve_80_Percent_Open;
    if (Ball_Valve4 == Set_Valve_15_Percent_Open) Ball_Valve4 = Set_Valve_80_Percent_Open;
    if (Ball_Valve5 == Set_Valve_15_Percent_Open) Ball_Valve5 = Set_Valve_80_Percent_Open;
    if (Ball_Valve6 == Set_Valve_15_Percent_Open) Ball_Valve6 = Set_Valve_80_Percent_Open;
    Update_Valve_Module(7); // 7 means we're not in the a,b,c cycles
  }
  //if (TM == 15 ){ This was used for 21 minute cycles
  if (TM == 7 ){  
    if (Ball_Valve1 == Set_Valve_80_Percent_Open) Ball_Valve1 = Set_Valve_15_Percent_Open;
    if (Ball_Valve2 == Set_Valve_80_Percent_Open) Ball_Valve2 = Set_Valve_15_Percent_Open;
    if (Ball_Valve3 == Set_Valve_80_Percent_Open) Ball_Valve3 = Set_Valve_15_Percent_Open;
    if (Ball_Valve4 == Set_Valve_80_Percent_Open) Ball_Valve4 = Set_Valve_15_Percent_Open;
    if (Ball_Valve5 == Set_Valve_80_Percent_Open) Ball_Valve5 = Set_Valve_15_Percent_Open;
    if (Ball_Valve6 == Set_Valve_80_Percent_Open) Ball_Valve6 = Set_Valve_15_Percent_Open;
    Update_Valve_Module(7); // 7 means we're not in the a,b,c cycles
  }
  // ***********************************************************  
  Time_in_Seconds = 60;
  }
  return;
  
}

 
void Turn_On_Plant() {
 
 
// Air Valve A Open, Air Valve B Open, Air Valve C Close
// These valves are normally closed, so we need 4 closed at any time
// So 4 should be low at any time
 Conc1_Air_Valve = 0;
 Conc2_Air_Valve = 0;
 Conc3_Air_Valve = 1;
 Conc4_Air_Valve = 0;
 Conc5_Air_Valve = 0;
 Conc6_Air_Valve = 1;
 
// set all rinse valves off
 Conc1_1in_Valve = 0;
 Conc2_1in_Valve = 0;
 Conc3_1in_Valve = 0;
 Conc4_1in_Valve = 0;
 Conc5_1in_Valve = 0;
 Conc6_1in_Valve = 0;
 
// 1 1/2 Valve A 100%,  1 1/2 Valve B 100%, 1 1/2 Valve C 15% 
 Ball_Valve1 = Set_Valve_100_Percent_Open;
 Ball_Valve2 = Set_Valve_100_Percent_Open;
 Ball_Valve3 = Set_Valve_15_Percent_Open;
 //Ball_Valve4 = Set_Valve_100_Percent_Open;
 //Ball_Valve5 = Set_Valve_100_Percent_Open;
 Ball_Valve4 = Set_Valve_79_Percent_Open; //East side of Cape values
 Ball_Valve5 = Set_Valve_79_Percent_Open;
 Ball_Valve6 = Set_Valve_15_Percent_Open;
 // tell the concentrator valve module to update the valves
  Update_Valve_Module(7); // 7 means we're not in the a,b,c cycles
 // Conc A Speed 'Operate, Conc B Speed 'Operate',Conc C Speed '0'
 
  VFD_Concentrator1_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator1_High_Speed = 1; //Turn on (high speed)
  
  VFD_Concentrator2_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator2_High_Speed = 1; //Turn on (high speed)
   
  VFD_Concentrator3_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator3_High_Speed = 0; //Set to off
   
  VFD_Concentrator4_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator4_High_Speed = 1; //Turn on (high speed)
  
  VFD_Concentrator5_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator5_High_Speed = 1; //Turn on (high speed)
   
  VFD_Concentrator6_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator6_High_Speed = 0; //Set to off
  Update_VFD1_Module(7); // 7 means we're not in the a,b,c cycles
  
  //VFD_Conveyor_Sluice = 1; //Turn on Conveyor_Sluice
  VFD_Tails_Conveyor = 1; // Turn on Tails Conveyor
  Update_VFD2_Module();
    delay(5000); // 5 second delay
  VFD_Screen = 1; // Turn on Screen
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  VFD_Primary_VT40 = 1; // Turn on Primary VT40
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  VFD_Secondary_VT40 = 1; // Turn on Secondary VT40
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  VFD_Submersible_Pump = 1;  // Turn on, Also has rheostat conected directly to the VFD
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  VFD_Booster_Pump_15HP = 1;
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  VFD_20HP_Transfer_Pump = 1; // Turn on, Also has rheostat conected directly to the VFD
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  VFD_60_HP_Pump = 1; // Turn on
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  VFD_30_HP_Pump = 1; // Turn on
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  VFD_50_HP_Pump = 1; // Turn on
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  VFD_60_ft_Conveyor = 1; // Turn on
  Update_VFD2_Module();
  delay(5000); // 5 second delay
  //digitalWrite(VFD_Hopper_On,HIGH);  // Turn on, Also has rheostat conected directly to the VFD
  
 return;
}
void Begin_Rinse_Cycle(int Count) {
  // Definitions for the Switch Cases
  int Conc_Control =0;
  int Cycle_Type = 0;
  Cycle_Type = (Count % 3) + 1; //Do our own MOD function. remander of count/3 +1
  
  display.clearDisplay();
  display.setCursor(0, 12);
  display.print("   Rinsing Plant   ");
  display.setCursor(0, 26); // Line 3, space 1
  display.print("cycletype "+String(Cycle_Type) );
  display.display();
    switch (Cycle_Type) {
      case First_Cycle:
        Conc_Control = First_Cycle_firstpass; 
      break;
    case Second_Cycle:
        Conc_Control = Second_Cycle_firstpass;
      break;
    case Third_Cycle:
      Conc_Control = Third_Cycle_firstpass;
      break;
    }
   
// Turn 1st concentrator C to operate
  Concentrator_A_High_Speed = 1; // need to do this here because we left it off
  Concentrator_C_Low_Speed = 0; // Initialize low speed logic
  Concentrator_C_High_Speed = 1; //Set to on
  Update_VFD1_Module(Conc_Control); //send VDF states to module
  delay(10000); // delay 10 seconds to ramp to speed
  
  // Open C 1-1/2 Valve to 100%
 
  Concentrator_C_1p5in_Valve = Set_Valve_100_Percent_Open;
  // open A and C water valves, close B water valve
  // Rubber valves are open when there is no air pressure
  // The air valves are normally closed (LOW)
  Concentrator_A_Air_Valve = 1; //open air valve
  Concentrator_B_Air_Valve = 0; //close air valve
  Concentrator_C_Air_Valve = 0; //close air valve
  Update_Valve_Module(Conc_Control); //Send valve states
  delay(1000);
  
  // Turn Concentrator A speed to rinse (low)
  
  Concentrator_A_Low_Speed = 1; // Set to low speed
  Update_VFD1_Module(Conc_Control); //send VDF states to module
  delay(500);
  
  //delay(2000); // wait 2 seconds
  
   Concentrator_A_1p5in_Valve = Set_Valve_50_Percent_Open; //Set A 1-1/2 valve to 50% open
   Update_Valve_Module(Conc_Control); //Send valve states  
   delay(One_Inch_Valve_Open_Delay); // wait 10 seconds
   Concentrator_A_1in_Valve = 1; // Open A rinse valve
   Update_Valve_Module(Conc_Control); //Send valve states  
   delay(One_Inch_Valve_Close_Delay); //wait 15 seconds
   Concentrator_A_1in_Valve= 0; // Close A rinse valve
   Update_Valve_Module(Conc_Control); //Send valve states  
   Concentrator_A_1p5in_Valve = Set_Valve_15_Percent_Open; //set A 1-1/2 valve to 15% open
   Update_Valve_Module(Conc_Control); //Send valve states
    
   Concentrator_A_High_Speed = 0; // Turn off A concentrator
   Concentrator_A_Low_Speed = 0; // Set A concentrator to high speed for whe we turn it on again
   Update_VFD1_Module(Conc_Control); //send VDF states to module
   delay(3000);
 
   // Now do it again for the second "A" concentrator
    switch (Cycle_Type) {
      case First_Cycle:
        Conc_Control = First_Cycle_secondpass; 
      break;
      case Second_Cycle:
        Conc_Control = Second_Cycle_secondpass;
      break;
      case Third_Cycle:
        Conc_Control = Third_Cycle_secondpass;
      break;
    }
   
// Turn 1st concentrator C to operate
  Concentrator_A_High_Speed = 1; // need to do this here because we left it off
  Concentrator_C_Low_Speed = 0; // Initialize low speed logic
  Concentrator_C_High_Speed = 1; //Set to on
 
  Update_VFD1_Module(Conc_Control); //send VDF states to module
  delay(10000); // delay 10 seconds to ramp to speed
  
  // Open C 1-1/2 Valve to 79%
  //********* New value to try for East side of Cape pay
  Concentrator_C_1p5in_Valve = Set_Valve_79_Percent_Open;

  // open A and C water valves, close B water valve
  // Rubber valves are open when there is no air pressure
  // The air valves are normally closed (LOW)
  Concentrator_A_Air_Valve = 1; //open air valve
  Concentrator_B_Air_Valve = 0; //close air valve
  Concentrator_C_Air_Valve = 0; //close air valve
  Update_Valve_Module(Conc_Control); //Send valve states
  delay(1000);
  
  // Turn Concentrator A speed to rinse (low)

  Concentrator_A_Low_Speed = 1; // Set to low speed
  Update_VFD1_Module(Conc_Control); //send VDF states to module
  delay(500);
 
  //delay(2000); // wait 2 seconds
  
   Concentrator_A_1p5in_Valve = Set_Valve_50_Percent_Open; //Set A 1-1/2 valve to 50% open
   Update_Valve_Module(Conc_Control); //Send valve states  
   delay(One_Inch_Valve_Open_Delay); // wait 10 seconds
   Concentrator_A_1in_Valve = 1; // Open A rinse valve
   Update_Valve_Module(Conc_Control); //Send valve states  
   delay(One_Inch_Valve_Close_Delay); //wait 15 seconds
   Concentrator_A_1in_Valve= 0; // Close A rinse valve
   Update_Valve_Module(Conc_Control); //Send valve states  
   Concentrator_A_1p5in_Valve = Set_Valve_15_Percent_Open; //set A 1-1/2 valve to 15% open
   Update_Valve_Module(Conc_Control); //Send valve states

   Concentrator_A_High_Speed = 0; // Turn off A concentrator
   Concentrator_A_Low_Speed = 0; // Set A concentrator to high speed for whe we turn it on again
   Update_VFD1_Module(Conc_Control); //send VDF states to module
   delay(3000);

  return;
} // end Begin_Rinse_Cycle

void Stop_Plant_And_Rinse_All_Concentrators() {
 // Stop Feed hopper
   display.clearDisplay();
   //lcd.clear();
   display.setCursor(0, 15);
   display.print("  Stopping Plant ");
   display.display();  
   
 
 VFD_Hopper = 0;  // Turn off
 Update_VFD2_Module();
 display.setCursor(0, 30);
 display.print(" Hopper off"); //
 display.setCursor(0, 45);
 display.print("Waiting 20 sec ");
 display.display();
 delay(20000); //wait 20 seconds
 
 VFD_60_ft_Conveyor = 0; // Turn off
 Update_VFD2_Module();
 display.clearDisplay();
 display.setCursor(0, 15);
 display.print("  Stopping Plant ");
 display.setCursor(0, 30);
 display.print("60ft Conv off"); //
 display.setCursor(0, 45);
 display.print("Waiting 10 sec ");
 display.display();
 
 delay(10000); //wait 10 seconds
 
 VFD_Screen = 0; // Turn off Screen
 VFD_Tails_Conveyor = 0; // Turn off Tails Conveyor
 Update_VFD2_Module();
 //digitalWrite(VFD_Conveyor_Sluice_High_Speed,HIGH); //Turn on Conveyor_Sluice
 display.clearDisplay();
 display.setCursor(0, 12);
 display.print("  Screen off");
  display.setCursor(0, 26);
 display.print("Conveyor Sluice");
 display.setCursor(0, 40);
 display.print("To high Speed ");
 display.display();
 delay(2000);
// New delay added on the running the conveyor sluice DAD
 //delay(20000); //wait 20 seconds then turn off the conveyor sluice.
 VFD_Conveyor_Sluice = 1; //Turn on Conveyor_Sluice (Frank 5/6/2022)
 Update_VFD2_Module();
  display.clearDisplay();
  display.setCursor(0, 12);
  display.print(" A Rinse cycle"); 
  display.setCursor(0, 25);
  display.print(" Running"); 
  display.display();
  Conc1_1in_Valve = 0; // 0 is off, 1 is on
  Conc2_1in_Valve = 0;
  Conc3_1in_Valve = 0;
  Conc4_1in_Valve = 0;
  Conc5_1in_Valve = 0;
  Conc6_1in_Valve = 0;
  Update_Valve_Module(7);

 // All concentrators go to 100% speed
 
  VFD_Concentrator1_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator1_High_Speed = 1; //Turn on (high speed)
  
  VFD_Concentrator2_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator2_High_Speed = 1; //Turn on (high speed)
   
  VFD_Concentrator3_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator3_High_Speed = 1; //Turn on (high speed)
   
  VFD_Concentrator4_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator4_High_Speed = 1; //Turn on (high speed)
  
  VFD_Concentrator5_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator5_High_Speed = 1; //Turn on (high speed)
   
  VFD_Concentrator6_Low_Speed = 0; // Initialize low speed logic
  VFD_Concentrator6_High_Speed = 1; //Turn on (high speed)
  Update_VFD1_Module(7);
  // 1 1/2 Valve A 100%,  1 1/2 Valve B 100%, 1 1/2 Valve C 20% 
  Ball_Valve1 = Set_Valve_100_Percent_Open;
  Ball_Valve2 = Set_Valve_100_Percent_Open;
  Ball_Valve3 = Set_Valve_100_Percent_Open;
  Ball_Valve4 = Set_Valve_100_Percent_Open;
  Ball_Valve5 = Set_Valve_100_Percent_Open;
  Ball_Valve6 = Set_Valve_100_Percent_Open;
  Update_Valve_Module(7);
 delay(5001); //Wait 8 seconds after spinning up all concentrators and opening
 delay(3000);
              // all 1.5in valves
 // open all pinch valves
  Conc1_Air_Valve = 0;
  Conc2_Air_Valve = 0;
  Conc3_Air_Valve = 0;
  Conc4_Air_Valve = 0;
  Conc5_Air_Valve = 0;
  Conc6_Air_Valve = 0;
  Update_Valve_Module(7);
  Begin_Rinse_Cycle(3); //Do the "A" Rinse cycle
  display.clearDisplay();
  display.setCursor(0, 12);
  display.print(" B Rinse cycle"); 
  display.setCursor(0, 25);
  display.print(" Running"); //lcd.setCursor(0,1);
  display.display();
  Begin_Rinse_Cycle(4); //Do the "B" Rinse cycle
  display.clearDisplay();
  display.setCursor(0, 12);
  display.print(" C Rinse cycle"); 
  display.setCursor(0, 25);
  display.print(" Running"); //lcd.setCursor(0,1);
  display.display();
  Begin_Rinse_Cycle(5); //Do the "C" Rinse cycle
 
   /*lcd2.setCursor(0,0);
  lcd2.print("  15hp Booster off"); //
  lcd2.setCursor(0, 1);
  lcd2.print("Submersible pump off"); //
  lcd2.setCursor(0,2);
  lcd2.print(" 50hp pump off  "); //
  lcd2.setCursor(0, 3);
  lcd2.print(" Waiting 10 sec "); */
  VFD_Conveyor_Sluice = 0; //Turn off Conveyor_Sluice (Frank 5/6/2022)
  VFD_Booster_Pump_15HP = 0; // Turn off Booster pump
  VFD_Submersible_Pump = 0;  // Turn off Submersible pump
  VFD_50_HP_Pump = 0; // Turn off 50hp Transfer pump
  Update_VFD2_Module();
  delay(10000); //wait 10 seconds
  /*lcd2.clear();
  lcd2.setCursor(0,0);
  lcd2.print("  20hp pump off  "); //
  lcd2.setCursor(0, 1);
  lcd2.print("Waiting 10 sec "); //
  
  lcd2.setCursor(0,0);
  lcd2.print(" 60hp pump off  "); //
  lcd2.setCursor(0,0);
  lcd2.print(" 30hp pump off  "); //
  lcd2.setCursor(0, 1);
  lcd2.print(" All concentrators"); //
  lcd2.setCursor(0, 2);
  lcd2.print(" off, 5 sec delays"); //
  lcd2.setCursor(0, 3);
  lcd2.print("Then  20 sec delay"); */
  VFD_20HP_Transfer_Pump = 0; // Turn off 20hp Transfer pump
  Update_VFD2_Module();
  delay(10000); //wait 10 seconds
  VFD_60_HP_Pump = 0; // Turn off tails pump
  Update_VFD2_Module();
  delay(5000); //wait 5 seconds
  VFD_30_HP_Pump = 0; // Turn off pump
  Update_VFD2_Module();
  // Turn off all Concentrators
  VFD_Concentrator1_High_Speed = 0; // Turn off
  Update_VFD1_Module(7);
  delay(5000); // 5 second delay
  VFD_Concentrator2_High_Speed = 0; // Turn off
  Update_VFD1_Module(7);
  delay(5000); // 5 second delay
  VFD_Concentrator3_High_Speed = 0; // Turn off
  Update_VFD1_Module(7);
  delay(5000); // 5 second delay
  VFD_Concentrator4_High_Speed = 0; // Turn off
  Update_VFD1_Module(7);
  delay(5000); // 5 second delay
  VFD_Concentrator5_High_Speed = 0; // Turn off
  Update_VFD1_Module(7);
  delay(5000); // 5 second delay
  VFD_Concentrator6_High_Speed = 0; // Turn off
  Update_VFD1_Module(7);
  delay(20000); // wait 20 seconds

  /*lcd2.clear();
  lcd2.setCursor(0,0);
  lcd2.print(" VT-40 1 off"); //
  lcd2.setCursor(0, 1);
  lcd2.print("  VT-40 2 off"); //
  lcd2.setCursor(0, 2);
  lcd2.print(" 1.5in valves open"); //
  lcd2.setCursor(0, 3);
  lcd2.print("Then 10 sec delay"); */
  VFD_Primary_VT40 = 0; // Turn off Primary VT40
  VFD_Secondary_VT40 = 0; // Turn off Secondary VT40
  Update_VFD2_Module();
  
// open all 1.5in valves at the end of the day
  Ball_Valve1 = Set_Valve_100_Percent_Open;
  Ball_Valve2 = Set_Valve_100_Percent_Open;
  Ball_Valve3 = Set_Valve_100_Percent_Open;
  Ball_Valve4 = Set_Valve_100_Percent_Open;
  Ball_Valve5 = Set_Valve_100_Percent_Open;
  Ball_Valve6 = Set_Valve_100_Percent_Open;
  Update_Valve_Module(7);

 delay(10000); // delay 10 seconds to let valves open
 // turn off all air valves
  Conc1_Air_Valve = 1;
  Conc2_Air_Valve = 1;
  Conc3_Air_Valve = 1;
  Conc4_Air_Valve = 1;
  Conc5_Air_Valve = 1;
  Conc6_Air_Valve = 1;
  
  Conc1_1in_Valve = 0; // turn all 1 in valves off
  Conc2_1in_Valve = 0;
  Conc3_1in_Valve = 0;
  Conc4_1in_Valve = 0;
  Conc5_1in_Valve = 0;
  Conc6_1in_Valve = 0;
  Update_Valve_Module(7);
  return;
}
void Deal_With_client() {
  WiFiClient client = server.available();   // Listen for incoming clients
 //int Time_in_Sec = Time_in_Seconds ;
 String Upper_Content = "";
 String Lower_Content = "";
 String Stopstart_Content = "";
  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            Serial.println("HTTP/1.1 200 OK");
            // turns the GPIOs on and off
            
          if (header.indexOf("GET /Refresh") >= 0){
            Upper_Content = Upper();
            client.println(Upper_Content);
            Serial.println("Saw refresh");
            goto END;   //Don't need to refresh the entire page
            First_Time = 1;
            }
            if (header.indexOf("GET /ReStart") >= 0){
              Plant_Run_State = Plant_stopped;
              New_Event = 1; // run through the remote devices again
              First_Time = 1;
            }
            if (header.indexOf("GET /Start") >= 0){
              if(Last_State == Initialize_plant) Plant_Run_State = Plant_running;
              New_Event = 1; // run through the remote devices again
              //goto END;
              First_Time = 1;
            }
            if (header.indexOf("GET /Initialize") >= 0){
              if(Last_State == Plant_stopped) Plant_Run_State = Initialize_plant;
              New_Event = 0; // No intervention from main loop required
              //goto END;
              First_Time = 1;
            }
            if (header.indexOf("GET /Stop") >= 0){
              if(Last_State == Plant_running) Plant_Run_State = Plant_stopping;
              New_Event = 0; // No intervention from main loop required
              First_Time = 1;
            }
            if (header.indexOf("GET /Hopper_stop") >= 0){
              VFD_Hopper = 0;
              //New_Event = 1; // run through the remote devices again
              Update_VFD2_Module();
              First_Time = 1;
            }
            if (header.indexOf("GET /Hopper_start") >= 0){
              VFD_Hopper = 1;
              //New_Event = 1; // run through the remote devices again
              Update_VFD2_Module();
              First_Time = 1;
            }
            if (header.indexOf("GET /60ft_stop") >= 0){
              VFD_60_ft_Conveyor = 0;
              //New_Event = 1; // run through the remote devices again
              Update_VFD2_Module();
              First_Time = 1;
            }
            if (header.indexOf("GET /60ft_start") >= 0){
              VFD_60_ft_Conveyor = 1;
              //New_Event = 1; // run through the remote devices again
              Update_VFD2_Module();
              First_Time = 1;
            }
            if (header.indexOf("GET /Tails_stop") >= 0){
              VFD_Tails_Conveyor = 0;
              //New_Event = 1; // run through the remote devices again
              Update_VFD2_Module();
              First_Time = 1;
            }
            if (header.indexOf("GET /Tails_start") >= 0){
              VFD_Tails_Conveyor = 1;
              //New_Event = 1; // run through the remote devices again
              Update_VFD2_Module();
              First_Time = 1;
            }
            if (header.indexOf("GET /Screen_stop") >= 0){
              VFD_Screen = 0;
              //New_Event = 1; // run through the remote devices again
              Update_VFD2_Module();
              First_Time = 1;
            }
            if (header.indexOf("GET /Screen_start") >= 0){
              VFD_Screen = 1;
              //New_Event = 1; // run through the remote devices again
              Update_VFD2_Module();
              First_Time = 1;
            }
            if (header.indexOf("GET /Sluice_stop") >= 0){
              Conveyor_sluice_manual_off = 1;
              //New_Event = 1; // run through the remote devices again
              First_Time = 1;
            }
            if (header.indexOf("GET /Sluice_start") >= 0){
              Conveyor_sluice_manual_off = 0;
              //New_Event = 1; // run through the remote devices again
              First_Time = 1;
            }
            if (header.indexOf("GET /Update") >= 0){
              Serial.println("Saw Update");
              Stopstart_Content = Stop_Buttons();
              client.println(Stopstart_Content);
              Lower_Content = Lower();
              client.println(Lower_Content);
              goto END;   //Don't need to refresh the entire page
              First_Time = 1;
            } // end of Update
            
            if (header.indexOf("GET /Remaining_time") >= 0){
              Serial.println("Saw time_remaining");
              //client.println("HTTP/1.1 200 OK");
              //client.println("Content-type:text/html");
              Serial.println(RinseTime);
              client.println(RinseTime);
              client.println();
              goto END;
              First_Time = 1;
            }
 
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
            
            //if(Plant_Run_State == Plant_running)
            //client.println("<p>Rinse cycle starts in <span id='Remaining_time'>22:00</span> minutes!</p>");
            /*client.println("<div id='stopstart'>");
            Stopstart_Content = Stop_Buttons();
            client.println(Stopstart_Content);
            client.println("</div>");*/
              
            client.println("<div id='states'>");
            Stopstart_Content = Stop_Buttons();
            client.println(Stopstart_Content);
            Lower_Content = Lower();
            client.println(Lower_Content);
            client.println("</div>");
            
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
            client.println("</script>");
          
            client.println("</body>");
            //}
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            
 END:     break;
          } else { // if you got a newline, then clear currentLine
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
} //end of deal with client
String Lower() {
  String Value ="";
  if (VFD_Concentrator1_High_Speed == 0)
  Value = ("<p><button class=\"button btn_off btn_dis\">Conc 1 off</button>");
  if (VFD_Concentrator1_High_Speed == 1 & VFD_Concentrator1_Low_Speed == 0)
  Value = ("<p><button class=\"button btn_on btn_dis\">Conc 1 on</button>");
  if (VFD_Concentrator1_High_Speed == 1 & VFD_Concentrator1_Low_Speed == 1)
  Value = ("<p><button class=\"button btn_slow btn_dis\">Conc 1 slow</button>");

  if (VFD_Concentrator2_High_Speed == 0)
  Value += "<a><button class=\"button btn_off btn_dis\">Conc 2 off</button>";
  if (VFD_Concentrator2_High_Speed == 1 & VFD_Concentrator2_Low_Speed == 0)
  Value += "<a><button class=\"button btn_on btn_dis\">Conc 2 on</button>";
  if (VFD_Concentrator2_High_Speed == 1 & VFD_Concentrator2_Low_Speed == 1)
  Value += "<a><button class=\"button btn_slow btn_dis\">Conc 2 slow</button>";
  
  if (VFD_Concentrator3_High_Speed == 0)
  Value += "<a><button class=\"button btn_off btn_dis\">Conc 3 off</button>";
  if (VFD_Concentrator3_High_Speed == 1 & VFD_Concentrator3_Low_Speed == 0)
  Value += "<a><button class=\"button btn_on btn_dis\">Conc 3 on</button>";
  if (VFD_Concentrator3_High_Speed == 1 & VFD_Concentrator3_Low_Speed == 1)
  Value += "<a><button class=\"button btn_slow btn_dis\">Conc 3 slow</button>";
 
  if (VFD_Concentrator4_High_Speed == 0)
  Value += "<a><button class=\"button btn_off btn_dis\">Conc 4 off</button>";
  if (VFD_Concentrator4_High_Speed == 1 & VFD_Concentrator4_Low_Speed == 0)
  Value += "<a><button class=\"button btn_on btn_dis\">Conc 4 on</button>";
  if (VFD_Concentrator4_High_Speed == 1 & VFD_Concentrator4_Low_Speed == 1)
  Value += "<a><button class=\"button btn_slow btn_dis\">Conc 4 slow</button>";

  if (VFD_Concentrator5_High_Speed == 0)
  Value += "<a><button class=\"button btn_off btn_dis\">Conc 5 off</button>";
  if (VFD_Concentrator5_High_Speed == 1 & VFD_Concentrator5_Low_Speed == 0)
  Value += "<a><button class=\"button btn_on btn_dis\">Conc 5 on</button>";
  if (VFD_Concentrator5_High_Speed == 1 & VFD_Concentrator5_Low_Speed == 1)
  Value += "<a><button class=\"button btn_slow btn_dis\">Conc 5 slow</button>";;

  if (VFD_Concentrator6_High_Speed == 0)
  Value += "<a><button class=\"button btn_off btn_dis\">Conc 6 off</button>";
  if (VFD_Concentrator6_High_Speed == 1 & VFD_Concentrator6_Low_Speed == 0)
  Value += "<a><button class=\"button btn_on btn_dis\">Conc 6 on</button>";
  if (VFD_Concentrator6_High_Speed == 1 & VFD_Concentrator6_Low_Speed == 1)
  Value += "<a><button class=\"button btn_slow btn_dis\">Conc 6 slow</button>";

  if (Ball_Valve1 == Set_Valve_100_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve1 100%</button>";
  if (Ball_Valve1 == Set_Valve_80_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve1 80%</button>";
  if (Ball_Valve1 == Set_Valve_79_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve1 79%</button>";
  if (Ball_Valve1 == Set_Valve_50_Percent_Open)
  Value += "<a><button class=\"button btn_50 btn_dis\">Ball_Valve1 50%</button>";
  if (Ball_Valve1 == Set_Valve_15_Percent_Open)
  Value += "<a><button class=\"button btn_slow btn_dis\">Ball_Valve1 15%</button>";

  if (Ball_Valve2 == Set_Valve_100_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve2 100%</button>";
  if (Ball_Valve2 == Set_Valve_80_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve2 80%</button>";
  if (Ball_Valve2 == Set_Valve_79_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve2 79%</button>";
  if (Ball_Valve2 == Set_Valve_50_Percent_Open)
  Value += "<a><button class=\"button btn_50 btn_dis\">Ball_Valve2 50%</button>";
  if (Ball_Valve2 == Set_Valve_15_Percent_Open)
  Value += "<a><button class=\"button btn_slow btn_dis\">Ball_Valve2 15%</button>";

  if (Ball_Valve3 == Set_Valve_100_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve3 100%</button>";
  if (Ball_Valve3 == Set_Valve_80_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve3 80%</button>";
  if (Ball_Valve3 == Set_Valve_79_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve3 79%</button>";
  if (Ball_Valve3 == Set_Valve_50_Percent_Open)
  Value += "<a><button class=\"button btn_50 btn_dis\">Ball_Valve3 50%</button>";
  if (Ball_Valve3 == Set_Valve_15_Percent_Open)
  Value += "<a><button class=\"button btn_slow btn_dis\">Ball_Valve3 15%</button>";

  if (Ball_Valve4 == Set_Valve_100_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve4 100%</button>";
  if (Ball_Valve4 == Set_Valve_80_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve4 80%</button>";
  if (Ball_Valve4 == Set_Valve_79_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve4 79%</button>";
  if (Ball_Valve4 == Set_Valve_50_Percent_Open)
  Value += "<a><button class=\"button btn_50 btn_dis\">Ball_Valve4 50%</button>";
  if (Ball_Valve4 == Set_Valve_15_Percent_Open)
  Value += "<a><button class=\"button btn_slow btn_dis\">Ball_Valve4 15%</button>";

  if (Ball_Valve5 == Set_Valve_100_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve5 100%</button>";
  if (Ball_Valve5 == Set_Valve_80_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve5 80%</button>";
  if (Ball_Valve5 == Set_Valve_79_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve5 79%</button>";
  if (Ball_Valve5 == Set_Valve_50_Percent_Open)
  Value += "<a><button class=\"button btn_50 btn_dis\">Ball_Valve5 50%</button>";
  if (Ball_Valve5 == Set_Valve_15_Percent_Open)
  Value += "<a><button class=\"button btn_slow btn_dis\">Ball_Valve5 15%</button>";

  if (Ball_Valve6 == Set_Valve_100_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve6 100%</button>";
  if (Ball_Valve6 == Set_Valve_80_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve6 80%</button>";
  if (Ball_Valve6 == Set_Valve_79_Percent_Open)
  Value += "<a><button class=\"button btn_on btn_dis\">Ball_Valve6 79%</button>";
  if (Ball_Valve6 == Set_Valve_50_Percent_Open)
  Value += "<a><button class=\"button btn_50 btn_dis\">Ball_Valve6 50%</button>";
  if (Ball_Valve6 == Set_Valve_15_Percent_Open)
  Value += "<a><button class=\"button btn_slow btn_dis\">Ball_Valve6 15%</button>";
  
  if(Conc1_1in_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#1 1in Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#1 1in Valve off</button>"; 
  }
  if(Conc2_1in_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#2 1in Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#2 1in Valve off</button>"; 
  }
  if(Conc3_1in_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#3 1in Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#3 1in Valve off</button>"; 
  }
  if(Conc4_1in_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#4 1in Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#4 1in Valve off</button>"; 
  }
  if(Conc5_1in_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#5 1in Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#5 1in Valve off</button>"; 
  }
  if(Conc6_1in_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#6 1in Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#6 1in Valve off</button>"; 
  }
  if(Conc1_Air_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#1 Air Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#1 Air Valve off</button>"; 
  }
  if(Conc2_Air_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#2 Air Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#2 Air Valve off</button>"; 
  }
  if(Conc3_Air_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#3 Air Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#3 Air Valve off</button>"; 
  }
  if(Conc4_Air_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#4 Air Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#4 Air Valve off</button>"; 
  }
  if(Conc5_Air_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#5 Air Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#5 Air Valve off</button>"; 
  }
  if(Conc6_Air_Valve == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">#6 Air Valve on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">#6 Air Valve off</button>"; 
  }
  
  
  if(VFD_Hopper == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">Hopper on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">Hopper off</button>"; 
  }
  if(VFD_Conveyor_Sluice == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">Conveyor Sluice on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">Conveyor Sluice off</button>"; 
  }
  if(VFD_Screen == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">Screen on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">Screen off</button>"; 
  }
  if(VFD_Tails_Conveyor == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">Tails Convey on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">Tails Convey off</button>"; 
  }
  if(VFD_Primary_VT40 == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">VT40_1 on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">VT40_1 off</button>"; 
  }
  if(VFD_Secondary_VT40 == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">VT40_2 on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">VT40_2 off</button>"; 
  }
  if(VFD_Submersible_Pump == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">Submers Pump on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">Submers Pump off</button>"; 
  }
  if(VFD_60_HP_Pump == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">60HP Pump on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">60HP Pump off</button>"; 
  }
  if(VFD_50_HP_Pump == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">50HP Pump on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">50HP Pump off</button>"; 
  }
  if(VFD_30_HP_Pump == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">30HP Pump on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">30HP Pump off</button>"; 
  }
  if(VFD_20HP_Transfer_Pump == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">Transfer Pump on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">Transfer Pump off</button>"; 
  }
  if(VFD_60_ft_Conveyor == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">60ft Conveyor on</button>"; 
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">60ft Conveyor off</button>"; 
  }
  if (VFD_Booster_Pump_15HP == 1){
   Value += "<a><button class=\"button btn_on btn_dis\">Booster pump on</button>";
  }
  else{
   Value += "<a><button class=\"button btn_off btn_dis\">Booster pump off</button>"; 
  }
  return Value; 
} // end of Lower function
    
String Upper() {
  Serial.print("Valve_Problem ");
  Serial.print(Valve_Problem);
  Serial.print(" VFD1_Problem ");
  Serial.print(VFD1_Problem);
  Serial.print(" VFD2_Problem ");
  Serial.println(VFD2_Problem);
  String Value =("<h1>Riven Control System</h1>");
  if ( (VFD2_Problem+VFD1_Problem+Valve_Problem)> 0){
    // one or more of the modules isn't communicating
    switch (Plant_Run_State) {
      case Waiting_for_command:
        if (Valve_Problem > 0){
          Value += "<p>Check the Valve control module before starting</p>";  
        }
        if (VFD1_Problem > 0){
          Value += "<p>Check the 1st relay control module before starting</p>";  
        }
        if (VFD2_Problem > 0){
          Value += "<p>Check the 2nd relay control module before starting</p>";  
        }
        //client.println("</div>");
        delay(100); //wait for someone to press start button, stop button
                     // or for the plant to finish initializing or stopping
      break;
      case Initialize_plant:
        if (Valve_Problem > 0){
          Value += "<p>Check the Valve control module before starting</p>";  
        }
        if (VFD1_Problem > 0){
          Value += "<p>Check the 1st relay control module before starting</p>";  
        }
        if (VFD2_Problem > 0){
          Value += "<p>Check the 2nd relay control module before starting</p>";  
        }
       // client.println("</div>");
      break;
      case Plant_running:
        // This loop goes forever and runs every X minutes set by the pot
        // The range of delay is between 1 and 60 minutes
        // The time can be changed without re-starting the sequencer
        if (Valve_Problem > 0){
          Value += "<p>Check the Valve control module before starting</p>";  
        }
        if (VFD1_Problem > 0){
          Value += "<p>Check the 1st relay control module before starting</p>";  
        }
        if (VFD2_Problem > 0){
          Value += "<p>Check the 2nd relay control module before starting</p>";  
        }
        //client.println("</div>");
      break;
      case Plant_stopping:
        if (Valve_Problem > 0){
          Value += "<p>Check the Valve control module before starting</p>";  
        }
        if (VFD1_Problem > 0){
          Value += "<p>Check the 1st relay control module before starting</p>";  
        }
        if (VFD2_Problem > 0){
          Value += "<p>Check the 2nd relay control module before starting</p>";  
        }
        //client.println("</div>");
        break;
      case Plant_stopped:
        if (Valve_Problem > 0){
          Value += "<p>Check the Valve control module before starting</p>";  
        }
        if (VFD1_Problem > 0){
          Value += "<p>Check the 1st relay control module before starting</p>";  
        }
        if (VFD2_Problem > 0){
          Value += "<p>Check the 2nd relay control module before starting</p>";  
        }
         Value += "<p><a href=\"/ReStart\"><button class=\"button btn_on\">Try Again</button></a>";
        //client.println("</div>");
       break;     
     }
  
  }
  else{
    switch (Plant_Run_State) {
      case Waiting_for_command:
        Value += "<p><a href=\"/Start\"><button class=\"button btn_on btn_large\">Start Plant</button></a></p>";
        
        delay(100); //wait for someone to press start button, stop button
                     // or for the plant to finish initializing or stopping
      break;
      case Initialize_plant:
        Value += "<p style='font-size:30px;'>Please wait for the plant to initialize</p>";
      break;
      case Plant_running:
        // This loop goes forever and runs every X minutes set by the pot
        // The range of delay is between 1 and 60 minutes
        // The time can be changed without re-starting the sequencer
        Value += "<p>Rinse cycle starts in ";
        Value += RinseTime;
        Value +=" minutes!</p>";
        Value += "<p><a href=\"/Stop\"><button class=\"button btn_on btn_large\">Stop Plant</button></a></p>";
        
                      
        
        
      break;
      case Plant_stopping:
        Value += "<p style='font-size:30px;'>Please wait for the plant to shut down</p>";
        
        break;
     case Plant_stopped:
        Value += "<p><a href=\"/Initialize\"><button class=\"button btn_on btn_large\">Inititialize Plant</button></a></p>";
        // The web page will ask for someone to initialize the plant
        // This will just loop here untill that happens
        // There will be messages to tell operators if the remote
        // modules are comunicating.
        break;     
    }
    
  }// end of if problem
    return Value; 
}           
String Stop_Buttons() {
  String Value =("<p>Emergency Stop Buttons</p>");
  if(VFD_Hopper == 1){
   Value += "<p><a href=\"/Hopper_stop\"><button class=\"button btn_on btn_Med\">Hopper Stop</button>"; 
  }
  else{
   Value += "<a href=\"/Hopper_start\"><button class=\"button btn_off btn_Med\">Hopper Start</button>"; 
  }
  if(VFD_60_ft_Conveyor == 1){
   Value += "<a href=\"/60ft_stop\"><button class=\"button btn_on btn_Med\">60ft Conveyor Stop</button>"; 
  }
  else{
   Value += "<a href=\"/60ft_start\"><button class=\"button btn_off btn_Med\">60ft Conveyor Start</button>"; 
  }
  if(VFD_Screen == 1){
   Value += "<a href=\"/Screen_stop\"><button class=\"button btn_on btn_Med\">Screen Stop</button>"; 
  }
  else{
   Value += "<a href=\"/Screen_start\"><button class=\"button btn_off btn_Med\">Screen Start</button>"; 
  }
  if(VFD_Tails_Conveyor == 1){
   Value += "<a href=\"/Tails_stop\"><button class=\"button btn_on btn_Med\">Tails Conveyor Stop</button>"; 
  }
  else{
   Value += "<a href=\"/Tails_start\"><button class=\"button btn_off btn_Med\">Tails Conveyor Start</button>"; 
  }
  if(Conveyor_sluice_manual_off == 0){
   Value += "<a href=\"/Sluice_stop\"><button class=\"button btn_on btn_Med\">Conveyor Sluice Stop</button>"; 
  }
  else{
   Value += "<a href=\"/Sluice_start\"><button class=\"button btn_off btn_Med\">Conveyor Sluice Start</button>"; 
  }
  return Value; 
}
void Update_Valve_Module(int Cycle_Type){
  // Cycle_type will be a 1 for A firstpass, 2 for B firstpass, 3 for C firstpass
  //  4 for A secondpass, 5 for B secondpass, 6 for C secondpass, 7 for Normal operation
  
   switch (Cycle_Type) {
    case First_Cycle_firstpass:
      Conc1_Air_Valve = Concentrator_A_Air_Valve;
      Conc2_Air_Valve = Concentrator_B_Air_Valve;
      Conc3_Air_Valve = Concentrator_C_Air_Valve;
      Ball_Valve1 = Concentrator_A_1p5in_Valve;
      Ball_Valve3 = Concentrator_C_1p5in_Valve;
      Conc1_1in_Valve = Concentrator_A_1in_Valve;
    break;
    case Second_Cycle_firstpass:
      Conc2_Air_Valve = Concentrator_A_Air_Valve;
      Conc3_Air_Valve = Concentrator_B_Air_Valve;
      Conc1_Air_Valve = Concentrator_C_Air_Valve;
      Ball_Valve2 = Concentrator_A_1p5in_Valve;
      Ball_Valve1 = Concentrator_C_1p5in_Valve;
      Conc2_1in_Valve = Concentrator_A_1in_Valve;
    break;
    case Third_Cycle_firstpass:
      Conc3_Air_Valve = Concentrator_A_Air_Valve;
      Conc1_Air_Valve = Concentrator_B_Air_Valve;
      Conc2_Air_Valve = Concentrator_C_Air_Valve;
      Ball_Valve3 = Concentrator_A_1p5in_Valve;
      Ball_Valve2 = Concentrator_C_1p5in_Valve;
      Conc3_1in_Valve = Concentrator_A_1in_Valve;
    break;
    case First_Cycle_secondpass:
      Conc4_Air_Valve = Concentrator_A_Air_Valve;
      Conc5_Air_Valve = Concentrator_B_Air_Valve;
      Conc6_Air_Valve = Concentrator_C_Air_Valve;
      Ball_Valve4 = Concentrator_A_1p5in_Valve;
      Ball_Valve6 = Concentrator_C_1p5in_Valve;
      Conc4_1in_Valve = Concentrator_A_1in_Valve;
    break;
    case Second_Cycle_secondpass:
      Conc5_Air_Valve = Concentrator_A_Air_Valve;
      Conc6_Air_Valve = Concentrator_B_Air_Valve;
      Conc4_Air_Valve = Concentrator_C_Air_Valve;
      Ball_Valve5 = Concentrator_A_1p5in_Valve;
      Ball_Valve4 = Concentrator_C_1p5in_Valve;
      Conc5_1in_Valve = Concentrator_A_1in_Valve;
    break;
    case Third_Cycle_secondpass:
      Conc6_Air_Valve = Concentrator_A_Air_Valve;
      Conc4_Air_Valve = Concentrator_B_Air_Valve;
      Conc5_Air_Valve = Concentrator_C_Air_Valve;
      Ball_Valve6 = Concentrator_A_1p5in_Valve;
      Ball_Valve5 = Concentrator_C_1p5in_Valve;
      Conc6_1in_Valve = Concentrator_A_1in_Valve;
    break;
    case  Normal_Control:
    // No translation required before controlling valves
    break;
   }
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
    Data_1.valve_relay_states &= 0b1011111111111111; Data_1.valve_relay_states |= VFD_Conveyor_Sluice<<14;
    //Data_1.valve_relay_states &= 0b0111111111111111; Data_1.valve_relay_states |= Conc2_Air_Valve<<15;
    
    Data_1.Ball_valve_1 = Ball_Valve1; // Ball_valveX will contain value to send to 0-10 module
    Data_1.Ball_valve_2 = Ball_Valve2;
    Data_1.Ball_valve_3 = Ball_Valve3;
    Data_1.Ball_valve_4 = Ball_Valve4;
    Data_1.Ball_valve_5 = Ball_Valve5;
    Data_1.Ball_valve_6 = Ball_Valve6;  
  esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *) &Data_1, sizeof(Valve_Contoller));
  if (result == ESP_OK) {
    Serial.println("Sent with success 0x01");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(100);
  while( Valve_Problem == 1) {
    Serial.print("Valve_receive_error ");
    Valve_retry_counter++;
    if (Valve_retry_counter > 5){
      Valve_retry_counter = 0;
      //lcd.setCursor(0, 1); // Line 2, space 1
      //lcd.print("  Valve_Problem " );
      return;
    }
    esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *) &Data_1, sizeof(Valve_Contoller));
    delay(10);
  }
  Valve_retry_counter = 0;
  Valve_Problem = 0;
  //lcd.setCursor(0, 1); // Line 2, space 1
  ///lcd.print("                    " );
  //Deal_With_client();
}

void Update_VFD1_Module(int Cycle_Type){
  switch (Cycle_Type) {
    case First_Cycle_firstpass:
      VFD_Concentrator1_Low_Speed = Concentrator_A_Low_Speed;
      VFD_Concentrator1_High_Speed = Concentrator_A_High_Speed;
      VFD_Concentrator3_Low_Speed = Concentrator_C_Low_Speed;
      VFD_Concentrator3_High_Speed = Concentrator_C_High_Speed;
    break;
    case Second_Cycle_firstpass:
      VFD_Concentrator2_Low_Speed = Concentrator_A_Low_Speed;
      VFD_Concentrator2_High_Speed = Concentrator_A_High_Speed;
      VFD_Concentrator1_Low_Speed = Concentrator_C_Low_Speed;
      VFD_Concentrator1_High_Speed = Concentrator_C_High_Speed;
    break;
    case Third_Cycle_firstpass:
      VFD_Concentrator3_Low_Speed = Concentrator_A_Low_Speed;
      VFD_Concentrator3_High_Speed = Concentrator_A_High_Speed;
      VFD_Concentrator2_Low_Speed = Concentrator_C_Low_Speed;
      VFD_Concentrator2_High_Speed = Concentrator_C_High_Speed;
    break;
    case First_Cycle_secondpass:
      VFD_Concentrator4_Low_Speed = Concentrator_A_Low_Speed;
      VFD_Concentrator4_High_Speed = Concentrator_A_High_Speed;
      VFD_Concentrator6_Low_Speed = Concentrator_C_Low_Speed;
      VFD_Concentrator6_High_Speed = Concentrator_C_High_Speed;
    break;
    case Second_Cycle_secondpass:
      VFD_Concentrator5_Low_Speed = Concentrator_A_Low_Speed;
      VFD_Concentrator5_High_Speed = Concentrator_A_High_Speed;
      VFD_Concentrator4_Low_Speed = Concentrator_C_Low_Speed;
      VFD_Concentrator4_High_Speed = Concentrator_C_High_Speed;
    break;
    case Third_Cycle_secondpass:
      VFD_Concentrator6_Low_Speed = Concentrator_A_Low_Speed;
      VFD_Concentrator6_High_Speed = Concentrator_A_High_Speed;
      VFD_Concentrator5_Low_Speed = Concentrator_C_Low_Speed;
      VFD_Concentrator5_High_Speed = Concentrator_C_High_Speed;
    break;
    case  Normal_Control:
    // No translation required before controlling valves
    break;
   }
  Data_2.VFD_relay_states &= 0b1111111011111111; Data_2.VFD_relay_states |= VFD_Concentrator1_High_Speed<<8;
  Data_2.VFD_relay_states &= 0b1111110111111111; Data_2.VFD_relay_states |= VFD_Concentrator1_Low_Speed<<9;
  Data_2.VFD_relay_states &= 0b1111101111111111; Data_2.VFD_relay_states |= VFD_Concentrator2_High_Speed<<10;
  Data_2.VFD_relay_states &= 0b1111011111111111; Data_2.VFD_relay_states |= VFD_Concentrator2_Low_Speed<<11;
  Data_2.VFD_relay_states &= 0b1110111111111111; Data_2.VFD_relay_states |= VFD_Concentrator3_High_Speed<<12;
  Data_2.VFD_relay_states &= 0b1101111111111111; Data_2.VFD_relay_states |= VFD_Concentrator3_Low_Speed<<13;
  
  Data_2.VFD_relay_states &= 0b1011111111111111; Data_2.VFD_relay_states |= VFD_Concentrator4_High_Speed<<14;
  Data_2.VFD_relay_states &= 0b0111111111111111; Data_2.VFD_relay_states |= VFD_Concentrator4_Low_Speed<<15;
  Data_2.VFD_relay_states &= 0b1111111111111110; Data_2.VFD_relay_states |= VFD_Concentrator5_High_Speed;
  Data_2.VFD_relay_states &= 0b1111111111111101; Data_2.VFD_relay_states |= VFD_Concentrator5_Low_Speed<<1;
  Data_2.VFD_relay_states &= 0b1111111111111011; Data_2.VFD_relay_states |= VFD_Concentrator6_High_Speed<<2;
  Data_2.VFD_relay_states &= 0b1111111111110111; Data_2.VFD_relay_states |= VFD_Concentrator6_Low_Speed<<3;

  //pf575_write(Data_2.VFD_relay_states); // Writes to one of the relay cards attached through the I2C bus
  
  BitInvert = Data_2.VFD_relay_states ^ Invertword;
  Wire.beginTransmission(i2c_Relay1);
  Wire.write(lowByte(BitInvert));
  Wire.write(highByte(BitInvert));
  Wire.endTransmission();
} // end Update_VFD1_Module

void Update_VFD2_Module(){

    Data_3.VFD2_relay_states &= 0b1111111011111111; Data_3.VFD2_relay_states |= VFD_Hopper<<8;
    Data_3.VFD2_relay_states &= 0b1111110111111111; Data_3.VFD2_relay_states |= VFD_Conveyor_Sluice<<9;
    Data_3.VFD2_relay_states &= 0b1111101111111111; Data_3.VFD2_relay_states |= VFD_Tails_Conveyor<<10;
    Data_3.VFD2_relay_states &= 0b1111011111111111; Data_3.VFD2_relay_states |= VFD_Screen<<11;
    Data_3.VFD2_relay_states &= 0b1110111111111111; Data_3.VFD2_relay_states |= VFD_Primary_VT40<<12;
    Data_3.VFD2_relay_states &= 0b1101111111111111; Data_3.VFD2_relay_states |= VFD_Secondary_VT40<<13;
    Data_3.VFD2_relay_states &= 0b1011111111111111; Data_3.VFD2_relay_states |= VFD_Submersible_Pump<<14;
    Data_3.VFD2_relay_states &= 0b0111111111111111; Data_3.VFD2_relay_states |= VFD_Booster_Pump_15HP<<15;
    
    Data_3.VFD2_relay_states &= 0b1111111111111110; Data_3.VFD2_relay_states |= VFD_20HP_Transfer_Pump;
    Data_3.VFD2_relay_states &= 0b1111111111111101; Data_3.VFD2_relay_states |= VFD_60_HP_Pump<<1;
    Data_3.VFD2_relay_states &= 0b1111111111111011; Data_3.VFD2_relay_states |= VFD_30_HP_Pump<<2;
    Data_3.VFD2_relay_states &= 0b1111111111110111; Data_3.VFD2_relay_states |= VFD_50_HP_Pump<<3;
    Data_3.VFD2_relay_states &= 0b1111111111101111; Data_3.VFD2_relay_states |= VFD_60_ft_Conveyor<<4;
    //Data_3.VFD2_relay_states &= 0b1111111111011111; Data_3.VFD2_relay_states |= VFD_60_ft_Conveyor<<5;
    //Data_3.VFD2_relay_states &= 0b1111111110111111; Data_3.VFD2_relay_states |= VFD_Submersible_Pump<<6;
    
    //pf575_write(Data_3.VFD2_relay_states); // Writes to one of the relay cards attached through the I2C bus 
    BitInvert = Data_3.VFD2_relay_states ^ Invertword;
    Wire.beginTransmission(i2c_Relay2);
    Wire.write(lowByte(BitInvert));
    Wire.write(highByte(BitInvert));
    Wire.endTransmission();
}
