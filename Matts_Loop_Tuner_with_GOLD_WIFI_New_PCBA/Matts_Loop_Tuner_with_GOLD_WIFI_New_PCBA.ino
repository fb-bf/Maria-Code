#include <ESP32Encoder.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>
#include <Fonts/FreeSans9pt7b.h>
//#include <U8g2lib.h> //Library for oled display using characters
#include "SparkFun_TCA9534.h"

//#include <Adafruit_PWMServoDriver.h>
//Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
#include "DFRobot_GP8403.h"

// REMOTE UPLOAD HEADER
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
///////////////////////

DFRobot_GP8403 dac1(&Wire,0x5D);
DFRobot_GP8403 dac2(&Wire,0x5E);
DFRobot_GP8403 dac3(&Wire,0x5F);

Adafruit_SSD1327 display(128, 128, &Wire, -1, 800000);
//U8G2_SSD1327_EA_W128128_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
TCA9534 myGPIO; // define the Sparkfun Gpio module
ESP32Encoder boomencoder; // encoder for the boom
ESP32Encoder trolleyencoder; // encoder for the trolley
ESP32Encoder dragflowencoder;
ESP32Encoder hosereelencoder;
ESP32Encoder hoseguideencoder;
hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;
Preferences pref; // create pref for using the EEPROM section to store latest Control Loop values
int t1 = 0;
int t2 = 0;
bool UpdateDisplay = false; // used to limit the number of times we run through the display method
#define NUM_GPIO 8
bool currentPinMode[NUM_GPIO] = {GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN};
bool gpioStatus[NUM_GPIO];
const int potPin = 35;
int potValue = 0;
float PotScale;
int MyCounter = 0;
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


portMUX_TYPE timer1Mux = portMUX_INITIALIZER_UNLOCKED;// Used to keep multiple cpus from accessing the interupt at same time??  
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;
//volatile bool interruptbool1 = false;
bool interruptbool2 = false;
volatile int interrupt1Counter = 0;
volatile int interrupt2Counter = 0;
//enum vtlStates {AMBER, RED, RA, GREEN}; // the states a single traffic light can be in
// the above line makes a variable data type "vtlStates" and assigns values 0, 1, 2, 3 to AMBER, RED, RA, GREEN
//vtlStates tlState; 
// Global Control Loop variables
// Below are used in parsing the reply from the client tuning page
int32_t positionValue = 0;
float gainValue = 0;
float diffValue = 0;
float extraValue1 = 0;
float extraValue2 = 0;
float extraValue3 = 0;
float extraValue4 = 0;
int clip1Value = 0;
int clip2Value = 0;
int Newlooptimer = 0;
bool RunLoop = true;
uint16_t looptimer = 2000; //This is a divider used to set loop interupt time
// a value of 1000 makes the interupt time 10 msec (100 times a second)
// We'll start with using 2000 so 50 times a second.
// *******************

int32_t boomPosition = 0;
int boomVelocity = 0;
int32_t boomLastPosition = 0;
int boomServoOutput = 0;
int LastboomServoOutput = 0;
int32_t boomCurrentPosition = 0;
float Bencoderperdeg = 1131.7; // Overwritten when Saved variables are read
float boomControlCommand;
float boomControlCommand1;
float minBoomangle = 0; // Probably home position
float maxBoomangle = 180;
float BoomG = 3;
float BoomI = 0;
float BoomD = 0;
int BoomClipRight = 2047; // We don't know the direction yet, so go with this.
int BoomClipLeft = -2047;
int Boomdb = 0;

int32_t trolleyPosition = 0;
int trolleyVelocity = 0;
int trolleyLastPosition = 0;
int trolleyServoOutput = 0;
int LasttrolleyServoOutput = 0;
int maxtrolleylocation = 90;
int32_t trolleyCurrentPosition = 0;
int32_t TrolleyError = 0;
float Trencoderperft = 7639.; // Overwritten when Saved variables are read 
float trolleyControlCommand;
float TrolleyG = .0268;
float TrolleyI = 0;
float TrolleyD = 0;
int TrolleyClipOut = 2047; // in the model, out is a negative direction
int TrolleyClipIn = -2047;
int Trolleydb = 0;
int goToHere = 3000;

int32_t hoseReelPosition = 0;
int hoseReelVelocity = 0;
int movingOutVel = 75;  // This is about .5 ft/sec in encodercounts/sec
int movingInVel = -10; // Used to chech if the trolley or dredge is moving in
int hoseReelStall = -5; // checks to see if the hosereel has stalled pulling in the pipe
// The above are used to check if the hose reel is controlling the pipe correctly

int hoseReelLastPosition = 0;
int hoseReelServoOutput = 0;
int LasthoseReelServoOutput = 0;
int32_t hoseReelCurrentPosition = 0;
int32_t HoseReelError = 0;
int hoseReeladjustment = 0;
float hoseReelControlCommand;
float HoseReelG = 0;
float HoseReelI = 0;
float HoseReelD = 0;
float Boomanglecomp = 1.;
float Hrencoderperft = 7381.;
float Hose2drag_ratio = .9661;// .9661
int HoseReelClipOut = 2047;
int HoseReelClipIn = -2047;
int HoseReeldb = 0;

int32_t hoseGuidePosition = 0;
int hoseGuideVelocity = 0;
int hoseGuideLastPosition = 0;
int hoseGuideServoOutput = 0;
int LasthoseGuideServoOutput = 0;
int32_t hoseGuideCurrentPosition = 0;
int32_t HoseGuideError = 0;
float hoseGPosScale = .0245;// Overwritten when Saved variables are read 
float hoseGuideControlCommand;
float HoseGuideG = 0;
float HoseGuideI = 0;
float HoseGuideD = 0;
int HoseGuideClipOut = 2047;
int HoseGuideClipIn = -2047;
int HoseGuidedb = 0;

int32_t dragflowPosition = 0;
float dragflowheight = 0;
int dragflowVelocity = 0;
int dragflowLastPosition = 0;
int dragflowServoOutput = 0;
int LastdragflowServoOutput = 0;
int32_t dragflowCurrentPosition = 0;
int32_t DragflowError = 0;
float Dfencoderperft = 7639.; // Overwritten when Saved variables are read
float dragflowControlCommand;
float DragflowG = .0268;
float DragflowI = 0;
float DragflowD = 0;
int DragflowClipUp = 2047;
int DragflowClipDown = -2047;
int Dragflowdb = 0;
float MaxDdepthchange = 2.0;
float MaxBAngchange = .5;
float MaxTrollchange = 2.0;
int LoopCounter = -1;
float Spare1; //Spare variables we save in Flash just in case
float Spare2;
float Spare3;
float Spare4;
float Spare5;
int TrollyP = 0;
  int BoomP = 0;
  int DragP = 0;
  int HoseP = 0;
  int HoseGP = 0;
// variables needed to check for wireless limit switches
byte Trigger = 0;
typedef struct Test_str {
  uint8_t Trigger;
} Test_str;
Test_str Trigger_State;
String Limitswitchstatus = "Okay";
byte New_Message = 0;
bool Hose_reel_guide_right_switch = false; //#10
bool Hose_reel_guide_left_switch = false; //#11
bool Hose_reel_upper_switch = false; //#13
bool Hose_reel_lower_switch = false; //#14
bool Trolley_out_switch = false; //#19
bool Trolley_in_switch = false; //#20
bool Boom_low_angle_switch = false; //#16
bool Boom_high_angle_switch = false; //#17
bool Dragflow_up_switch = false; //#22
bool Dragflow_timer_switch = false; //#21
bool Dragflow_switch_notreporting = false;
bool Hose_reel_guide_timer_switch = false; //#9
bool Hose_reel_guide_switch_notreporting = false;
bool Hose_reel_timer_switch = false; //#12
bool Hose_reel_switch_notreporting = false;
bool Trolley_timer_switch = false; //#18
bool Trolley_switch_notreporting = false;
bool Boom_angle_timer_switch = false; //#15
bool Boom_angle_switch_notreporting = false;


uint8_t newMACAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;// Used to keep multiple cpus from accessing the interupt at same time?? 
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 200;
String header;

void IRAM_ATTR onTimer1() {
  portENTER_CRITICAL_ISR(&timer1Mux);
  //Serial.println("timer1 interupt");
  interrupt1Counter++;
  portEXIT_CRITICAL_ISR(&timer1Mux);
}       


void IRAM_ATTR onTimer2() {
  portENTER_CRITICAL_ISR(&timer2Mux);
    interrupt2Counter++;
  //interruptbool2 = true; // update this here quickley, We'll look for change in main loop and then 
                      // check to see if all of the limit switch boxes have report in during the time period
 portEXIT_CRITICAL_ISR(&timer2Mux);
}   

//callback function that will be executed when data is received from limit switches
void OnDataRecv(const uint8_t *mac,const uint8_t*incomingData, int len) {
  memcpy(&Trigger_State, incomingData, sizeof(Trigger_State));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Trigger Value ");
  Serial.println(Trigger_State.Trigger);
  //digitalWrite(NETWORK_LED, HIGH);
  New_Message = 1;
}
int32_t IIRFilter(int32_t Input){
//This will use the "Input" from a encoder, and return the output of the filter as an integer.
// These coef are supposed to be used with the following equation
// y(n) = a0x(n) + a1x(n-1) + a2x(n-2) – b1y(n-1) – b2y(n-2))
  static int32_t In1; // last input
  static int32_t In2; //input from 2 periods ago
  static int32_t Out; //Last output of filter
  static int32_t Out1; //Last output of filter
  static int32_t Out2; //Output from 2 periods ago

  //example of 9 hz notch filter at 50 hz sample rate.
  //use Biquad filter attempt file to generate your filter coef.
  static float a0 = 0.60979104;
  static float a1 = -0.51927274;
  static float a2 = 0.60979104;
  static float b1 = -0.51927274;
  static float b2 = 0.21958211;
  Out = a0*Input + a1*In1 + a2*In2 - b1*Out1 - b2*Out2;
  In2 = In1;
  In1 = Input;
  Out2 = Out1;
  Out1 = Out;
  return Out;
}

void ControlLoop() {  
 if (RunLoop){ // Checks to see if the loop is disabled
    BoomControl();
    TrolleyControl();
    DragflowControl();
    HoseReelControl();
    HoseGuideControl();
    HoseReelOverride(); // This checks to see if the hose reel is keeping the proper amount of hose out
    }
    else{ // shut down all of the systems untill this is figured out
     boomServoOutput = 5000;
     dragflowServoOutput = 5000;
     trolleyServoOutput = 5000;
     hoseReelServoOutput = 5000;
     hoseGuideServoOutput = 5000;     
    }
    //interruptbool1 = false;
}
void BoomControl(){
  //LoopCounter++;
  // counts per foot at 90 ft radius will be 1131
  // Boom errors could be 20,000 or larger!!
  // If BoomG = 3, BoomControlCommand will be 60,000, and we need values of +/- 50
  // If we want max speed with a 10 foot error, then Gain = 1/(10*1131/5000) = .442
  // We'll then clip the max control effort to 5000 for moves greater than 10 ft. 
  // when we're dealing with PWM width.
   boomCurrentPosition = (int32_t)boomencoder.getCount();
   //boomCurrentPosition = IIRFilter(boomCurrentPosition); // use if you need to filter the input to controller
   int32_t BoomError = boomPosition - boomCurrentPosition;
   if(abs(BoomError) > Boomdb){ 
   boomVelocity =  boomLastPosition - boomCurrentPosition;
   boomControlCommand = (float)(BoomError * BoomG) - (float)(boomVelocity *BoomD);
   if(boomControlCommand > BoomClipRight) boomControlCommand = BoomClipRight;
   if(boomControlCommand < BoomClipLeft) boomControlCommand = BoomClipLeft;
   boomServoOutput = 5000 - boomControlCommand;
   }
   else{
    boomServoOutput = 5000;
   }
   if(boomServoOutput != LastboomServoOutput){
        //pwm1.setPWM(0,1,boomServoOutput);
        dac1.setDACOutVoltage(boomServoOutput, 0);
     }  
     LastboomServoOutput = boomServoOutput;
   boomLastPosition = boomCurrentPosition;
   //digitalWrite(5, LOW);
   
}
void TrolleyControl(){
  // Trolley motion will be 7639 counts per ft, if we use SingleEdge with the encoder.
  // If we want max speed with a 10 foot error, then Gain = 1/(10*7639/2048) = .0268
  // We'll then clip the max control effort to 2048 for moves greater than 10 ft.
  // Velocity at 1ft/sec will be 7639/50 = 152 if interupt time is 20 ms.
  // Trolley error is defined as positive when desired position is farther out than current position
   trolleyCurrentPosition = (int32_t)trolleyencoder.getCount();
     TrolleyError = trolleyPosition - trolleyCurrentPosition;
     if(abs(TrolleyError) > Trolleydb){
     trolleyVelocity = trolleyCurrentPosition - trolleyLastPosition ; // create a positve velocity term when trolley is moving out
     trolleyControlCommand = (float)(TrolleyError * TrolleyG) - (float)(trolleyVelocity * TrolleyD);
     // positive control efforts means we're pulling the trolley in
     if(trolleyControlCommand > TrolleyClipIn) trolleyControlCommand = TrolleyClipIn;
     // negative control efforts means we're pushing the trolley out
     if(trolleyControlCommand < TrolleyClipOut) trolleyControlCommand = TrolleyClipOut;
     trolleyServoOutput = 5000 - trolleyControlCommand;
     }
     else{
        trolleyServoOutput = 5000; 
     }
     if(trolleyServoOutput != LasttrolleyServoOutput){
        //pwm1.setPWM(1,1,trolleyServoOutput);
        dac1.setDACOutVoltage(trolleyServoOutput, 1);
     }  
     LasttrolleyServoOutput = trolleyServoOutput;
     trolleyLastPosition = trolleyCurrentPosition;
      
   dragflowPosition = trolleyCurrentPosition - dragflowheight*Dfencoderperft;
   // dragflowheight is defined as 0 when at home height, and negative when below home height
   // It's units are in feet
}
void DragflowControl(){
  // Dragflow motion will be 7639 counts per ft, if we use SingleEdge with the encoder.
  // If we want max speed with a 10 foot error, then Gain = 1/(10*7639/2048) = .0268
  // We'll then clip the max control effort to 2048 for moves greater than 10 ft.
  // DragflowError is designed to be positive when the pump is higher than it should be
  // and the winch needs to move outwards to correct for that
  // dragflowvelocity is positive when the winch is moving out
  
   dragflowCurrentPosition = (int32_t)dragflowencoder.getCount();
   DragflowError = dragflowPosition - dragflowCurrentPosition;
   if(abs(DragflowError) > Dragflowdb){
   dragflowVelocity =  dragflowLastPosition - dragflowCurrentPosition;
   dragflowControlCommand = (float)(DragflowError * DragflowG) - (float)(dragflowVelocity * DragflowD);
   if(dragflowControlCommand > DragflowClipUp) dragflowControlCommand = DragflowClipUp;
   if(dragflowControlCommand < DragflowClipDown) dragflowControlCommand = DragflowClipDown;
   
   dragflowServoOutput = 5000 - dragflowControlCommand;
   }
   else{
     dragflowServoOutput = 5000; 
   }
   if(dragflowServoOutput != LastdragflowServoOutput){
        //pwm1.setPWM(2,1,dragflowServoOutput);
        dac2.setDACOutVoltage(dragflowServoOutput, 0);
     }  
   LastdragflowServoOutput = dragflowServoOutput;
   dragflowLastPosition = dragflowCurrentPosition;
      //digitalWrite(5, LOW);
   
}
void HoseReelControl(){
  // HoseReel diameter = 12 feet+ .5 pipe diameter, counts/ft = 7381
  // Max gain at 10 feet request gives gain of .0277. This servos from trolley
  // distance + dragflow height, and takes into account the boom angle.
  // Dragflowposition already takes the trolley location and dragflow height into
  // account. Hose2drag_ratio = .966, Boomanglecomp = ???
  // HoseReelError is defined as positive if hoseReelPosition is farther out than Current position
  
   //hoseReelPosition = dragflowCurrentPosition * Hose2drag_ratio + Boomanglecomp * boomCurrentPosition + hoseReeladjustment;
   
   //hoseReelPosition =  hoseReeladjustment;
   hoseReelCurrentPosition = (int32_t)hosereelencoder.getCount();
   int32_t HoseReelError = hoseReelPosition - hoseReelCurrentPosition;
   if(abs(HoseReelError) > HoseReeldb){
   hoseReelVelocity =  hoseReelLastPosition - hoseReelCurrentPosition;
   hoseReelControlCommand = (float)(HoseReelError * HoseReelG) + (float)(hoseReelVelocity * HoseReelD);
   if(hoseReelControlCommand > HoseReelClipOut) hoseReelControlCommand = HoseReelClipOut;
   if(hoseReelControlCommand < HoseReelClipIn) hoseReelControlCommand = HoseReelClipIn;
   hoseReelServoOutput = 5000 - hoseReelControlCommand;
   }
   else{
     hoseReelServoOutput = 5000; 
   }   
   if(hoseReelServoOutput != LasthoseReelServoOutput){
        //pwm1.setPWM(3,1,hoseReelServoOutput);
        dac2.setDACOutVoltage(hoseReelServoOutput, 1);
     }  
   LasthoseReelServoOutput = hoseReelServoOutput;
   hoseReelLastPosition = hoseReelCurrentPosition;
   //digitalWrite(5, LOW);   
}
void HoseReelOverride(){
  // This function will check to see if we need to adjust the reel position based on the
  // inputs from the limit switches located abov and below the hose loop that happens
  // just after leaving the reel.
  // Both the trolley velocity and the dragflowvelocity are positive when the winches are moving out
  if (Hose_reel_upper_switch){ // when true we saw the limit switch go off.
    // We might need to adjust the hosereel desired position by 1 foot out.
    if( trolleyVelocity > movingOutVel|| dragflowVelocity > movingOutVel ) { // this is .5 ft /second
      // This means we have not let enough hose out to keep up with trolley motion moving out
      hoseReeladjustment = hoseReeladjustment + Hrencoderperft; //Let out 1 extra foot of hose
    }
    if ( trolleyVelocity <  movingInVel || dragflowVelocity < movingInVel ) { // trolley moving in, or pump moving up
      if ( hoseReelVelocity > hoseReelStall) { //It seems the hosereel has stalled trying to pull in the hose
        hoseReeladjustment = hoseReeladjustment + Hrencoderperft; //Let out 1 extra foot of hose 
      }
    }
  }
  if (Hose_reel_lower_switch) { // when true we have too much hose let out
    hoseReeladjustment = hoseReeladjustment - Hrencoderperft; //pull in 1 extra foot of hose  
  }
}
void HoseGuideControl(){
  // The hose guide should only require a location that is related to the amount of
  // hose that is out of the reel. So a multiplier will be used to scale the location
  // of the guide encoder to the location of the reel encoder.
  //For every 1 revolution of the the hose reel, the guide needs to move 10 inches.
  // This is about 5.3 degrees (7073 encoder counts) scale is then .0245
  
   hoseGuidePosition = hoseReelCurrentPosition * hoseGPosScale;
   hoseGuideCurrentPosition = (int32_t)hoseguideencoder.getCount();
   int32_t HoseGuideError = hoseGuidePosition - hoseGuideCurrentPosition;
   if(abs(HoseReelError) > HoseReeldb){
   hoseGuideVelocity =  hoseGuideLastPosition - hoseGuideCurrentPosition;
   hoseGuideControlCommand = (float)(HoseGuideError * HoseGuideG) + (float)(hoseGuideVelocity * HoseGuideD);
   if(hoseGuideControlCommand > HoseGuideClipOut) hoseGuideControlCommand = HoseGuideClipOut;
   if(hoseGuideControlCommand < HoseGuideClipIn) hoseGuideControlCommand = HoseGuideClipIn;
   hoseGuideServoOutput = 5000 - hoseGuideControlCommand;
   }
   else{
     hoseGuideServoOutput = 5000; 
   }   
   if(hoseGuideServoOutput != LasthoseGuideServoOutput){
        //pwm1.setPWM(4,1,hoseGuideServoOutput);
        dac3.setDACOutVoltage(hoseGuideServoOutput, 0);
     }  
   LasthoseGuideServoOutput = hoseGuideServoOutput;
   hoseGuideLastPosition = hoseGuideCurrentPosition;
   //digitalWrite(5, LOW);
}
// Below parses the html response from the loop tuning client page
void parseLoopValues(){
  int EqualSignLoc = 0;
  int SpaceFollowing = 0;
  String returnValue = "";
  const char * temp;
EqualSignLoc = header.indexOf("=");
  SpaceFollowing = header.indexOf("&",EqualSignLoc);
  returnValue = header.substring(EqualSignLoc+1,SpaceFollowing);
  temp = returnValue.c_str();
  sscanf(temp, "%"SCNd32, &positionValue);
  //positionValue = returnValue.toInt();
  //Serial.println(positionValue);
  EqualSignLoc = header.indexOf("=",SpaceFollowing);
  SpaceFollowing = header.indexOf("&",EqualSignLoc);
  returnValue = header.substring(EqualSignLoc+1,SpaceFollowing);
  temp = returnValue.c_str();
  gainValue = std::atof(temp);
  EqualSignLoc = header.indexOf("=",SpaceFollowing);
  SpaceFollowing = header.indexOf("&",EqualSignLoc);
  returnValue = header.substring(EqualSignLoc+1,SpaceFollowing);
  temp = returnValue.c_str();
  diffValue = std::atof(temp);
  EqualSignLoc = header.indexOf("=",SpaceFollowing);
  SpaceFollowing = header.indexOf("&",EqualSignLoc);
  returnValue = header.substring(EqualSignLoc+1,SpaceFollowing);
  clip1Value = returnValue.toInt();
  EqualSignLoc = header.indexOf("=",SpaceFollowing);
  SpaceFollowing = header.indexOf(" ",EqualSignLoc);
  returnValue = header.substring(EqualSignLoc+1,SpaceFollowing);
  clip2Value = returnValue.toInt();  
}
void parseExtraValues(){
  int EqualSignLoc = 0;
  int SpaceFollowing = 0;
  String returnValue = "";
  const char * temp;
  EqualSignLoc = header.indexOf("=");
  SpaceFollowing = header.indexOf("&",EqualSignLoc);
  returnValue = header.substring(EqualSignLoc+1,SpaceFollowing);
  temp = returnValue.c_str();
  extraValue1 = std::atof(temp);
  EqualSignLoc = header.indexOf("=",SpaceFollowing);
  SpaceFollowing = header.indexOf("&",EqualSignLoc);
  returnValue = header.substring(EqualSignLoc+1,SpaceFollowing);
  temp = returnValue.c_str();
  extraValue2 = std::atof(temp);
 // EqualSignLoc = header.indexOf("=",SpaceFollowing);
  //SpaceFollowing = header.indexOf("&",EqualSignLoc);
  //returnValue = header.substring(EqualSignLoc+1,SpaceFollowing);
 // Newlooptimer = returnValue.toInt();
  EqualSignLoc = header.indexOf("=",SpaceFollowing);
  SpaceFollowing = header.indexOf(" ",EqualSignLoc);
  returnValue = header.substring(EqualSignLoc+1,SpaceFollowing);
  Newlooptimer = returnValue.toInt();  
}
 const char* ssid     = "Gold";
 const char* password = "123456789";
 
// Set web server port number to 80
WiFiServer server(80);


TaskHandle_t Web;  
    
 void SaveEEpromValues(){
  timerAlarmDisable(timer1);
  pref.begin("ControlValues", false);
  pref.putFloat("EEBoomG",BoomG);
  pref.putFloat("EEBoomI",BoomI);
  pref.putFloat("EEBoomD",BoomD);
  pref.putFloat("EEBEncodperdeg",Bencoderperdeg);
  pref.putFloat("EEMinBoomAngle",minBoomangle);
  pref.putFloat("EEMaxBoomAngle",maxBoomangle);
  
  pref.putFloat("EETrolleyG",TrolleyG);
  pref.putFloat("EETrolleyI",TrolleyI);
  pref.putFloat("EETrolleyD",TrolleyD);
  pref.putFloat("EETEncodperft",Trencoderperft);
  
  pref.putFloat("EEDragflowG",DragflowG);
  pref.putFloat("EEDragflowI",DragflowI);
  pref.putFloat("EEDragflowD",DragflowD);
  pref.putFloat("EEDencodperft",Dfencoderperft);
  
  pref.putFloat("EEHoseReelG",HoseReelG);
  pref.putFloat("EEHoseReelI",HoseReelI);
  pref.putFloat("EEHoseReelD",HoseReelD);
  pref.putFloat("EEHencodperft",Hrencoderperft);
  pref.putFloat("EEHoseDragR",Hose2drag_ratio);
  pref.putFloat("EEBoomAComp",Boomanglecomp);
  
  pref.putFloat("EEHoseGuideG",HoseGuideG);
  pref.putFloat("EEHoseGuideI",HoseGuideI);
  pref.putFloat("EEHoseGuideD",HoseGuideD);
  pref.putFloat("EEHoseGPosScale",hoseGPosScale);
  
  pref.putShort("EEBoomClipRight",BoomClipRight);
  pref.putShort("EEBoomClipLeft",BoomClipLeft);
  pref.putShort("EEBoomdb",Boomdb);
  pref.putShort("EETrollClipOut",TrolleyClipOut);
  pref.putShort("EETrollClipIn",TrolleyClipIn);
  pref.putShort("EETrolleydb",Trolleydb);
  pref.putShort("EEHoseClipOut",HoseReelClipOut);
  pref.putShort("EEHoseClipIn",HoseReelClipIn);
  pref.putShort("EEHoseRealdb",HoseReeldb);
  pref.putShort("EEHoseGClipOut",HoseGuideClipOut);
  pref.putShort("EEHoseGClipIn",HoseGuideClipIn);
  pref.putShort("EEHoseGuidedb",HoseGuidedb);
  pref.putShort("EEDragfClipUp",DragflowClipUp);
  pref.putShort("EEDragfClipDn",DragflowClipDown);
  pref.putShort("EEDragflowdb",Dragflowdb);
  pref.putUShort("EELooptimer",looptimer);
  pref.putShort("EEMovingOutVel",movingOutVel);
  pref.putShort("EEMovingInVel",movingInVel); 
  pref.putShort("EEHRStall",hoseReelStall);
  pref.putFloat("EEDrDepthChange",MaxDdepthchange);
  pref.putFloat("EEBAngChange",MaxBAngchange);
  pref.putFloat("EETrolChange",MaxTrollchange);
  pref.putFloat("EESpare1",Spare1);
  pref.putFloat("EESpare2",Spare2);
  pref.putFloat("EESpare3",Spare3);
  pref.putFloat("EESpare4",Spare4);
  pref.putFloat("EESpare5",Spare5);
  
  Serial.println("Done Writing to EEPROM");
  pref.end();
  timerAlarmEnable(timer1);   
 }
 void HandleGPIOModule(){
   uint8_t portValue = myGPIO.digitalReadPort(gpioStatus);
   //Serial.print("GPIO port value ");
   //Serial.println(portValue);
   if(!gpioStatus[0]) { //these pins are pulled high so should be true unless triggered
    Gpio0();
   }
   if(!gpioStatus[1]) { //these pins are pulled high so should be true unless triggered
    Gpio1();
   }
   if(!gpioStatus[2]) { //these pins are pulled high so should be true unless triggered
    Gpio2();
   }
   if(!gpioStatus[3]) { //these pins are pulled high so should be true unless triggered
    Gpio3();
   }
}
void Gpio0(){ //Placeholder for possible future use
  Serial.println("GPIO port 0 went low");
}
void Gpio1(){ //Placeholder for possible future use
  Serial.println("GPIO port 1 went low"); 
}
void Gpio2(){ //Placeholder for possible future use
  Serial.println("GPIO port 2 went low"); 
}
void Gpio3(){ //Placeholder for possible future use
  Serial.println("GPIO port 3 went low"); 
}   
void HandleLimitSwitches(){
  bool All_reporting;
  Serial.println("Saw a trigger");
  New_Message = 0;
  Limitswitchstatus = "Okay";
  if(Trigger_State.Trigger == 21 ) Dragflow_timer_switch = true;
  else if(Trigger_State.Trigger == 9 ) Hose_reel_guide_timer_switch = true;
  else if(Trigger_State.Trigger == 15 ) Boom_angle_timer_switch = true;
  else if(Trigger_State.Trigger == 18 ) Trolley_timer_switch = true;
  else if(Trigger_State.Trigger == 12 ) Hose_reel_timer_switch = true;
  else if(Trigger_State.Trigger == 13 ) Hose_reel_upper_switch = true;
  else if(Trigger_State.Trigger == 14 ) Hose_reel_lower_switch = true;
  else { // if we get to here, we are seeing a true limit stitch event!!!
    Limitswitchstop(); // One of the limit switches has been detected
  }
  if ( interruptbool2 ){ // check to see that all switch boxes reported in during the last 4 hours
    interruptbool2 = false;
    if ( Hose_reel_guide_timer_switch == false){
      Serial.print("Hose_reel_guide_switch failed to report ");
      Hose_reel_guide_switch_notreporting = true;
      All_reporting = false;
    }
    else{
      Hose_reel_guide_switch_notreporting = false;
    }
    if ( Hose_reel_timer_switch == false){
      Serial.print("Hose_reel_switch failed to report ");
      Hose_reel_switch_notreporting = true;
      All_reporting = false;
    }
    else{
      Hose_reel_switch_notreporting = false;
    }
    if ( Trolley_timer_switch == false){
      Serial.print("Trolley_switch failed to report ");
      Trolley_switch_notreporting = true;
      All_reporting = false;
    }
    else{
      Trolley_switch_notreporting = false;
    }
    if ( Boom_angle_timer_switch == false){
      Serial.print("Boom_angle_switch failed to report ");
      Boom_angle_switch_notreporting = true;
      All_reporting = false;
    }
    else{
      Boom_angle_switch_notreporting = false;
    }
    if ( Dragflow_timer_switch == false){
      Serial.print("Dragflow_switch failed to report ");
      Dragflow_switch_notreporting = true;
      All_reporting = false;
    }
    else{
      Dragflow_switch_notreporting = false;
    }
    if(All_reporting) Serial.print("All limit switches reporting ");
    // reset all timer switch states
    Dragflow_timer_switch = false;
    Hose_reel_guide_timer_switch = false;
    Boom_angle_timer_switch = false;
    Trolley_timer_switch = false;
    Hose_reel_timer_switch = false;     
  } //end of interruptbool2 = true;
}
void Limitswitchstop(){
  RunLoop = false; // This will stop the loop from running until it is reset
  if(Trigger_State.Trigger == 10 ) Limitswitchstatus = "Hose_reel_guide_right_switch Tripped";
  if(Trigger_State.Trigger == 11 ) Limitswitchstatus = "Hose_reel_guide_left_switch Tripped";
  if(Trigger_State.Trigger == 19 ) Limitswitchstatus = "Trolley_out_switch Tripped";
  if(Trigger_State.Trigger == 20 ) Limitswitchstatus = "Trolley_in_switch Tripped";
  if(Trigger_State.Trigger == 16 ) Limitswitchstatus = "Boom_low_angle_switch Tripped";
  if(Trigger_State.Trigger == 17 ) Limitswitchstatus = "Boom_high_angle_switch";
  if(Trigger_State.Trigger == 22 ) Limitswitchstatus = "Dragflow_up_switch Tripped";
  Serial.println("A Limit switch has triggered");
}  
void setup() {
  Serial.begin(115200);


  WiFi.mode(WIFI_STA);
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid);
  esp_wifi_set_channel( 11, WIFI_SECOND_CHAN_NONE);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();
  server.setNoDelay(true);
  
  //REMOTE UPLOAD
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


  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  Serial.print("[NEW] ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  Serial.print("Wifi channel: "); Serial.println(WiFi.channel());
  //Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv); 

  
  // Read last stored conrol loop values from eeprom
  pref.begin("ControlValues", false);
  BoomG = pref.getFloat("EEBoomG",0);
  BoomI = pref.getFloat("EEBoomI",0);
  BoomD = pref.getFloat("EEBoomD",0);
  Bencoderperdeg = pref.getFloat("EEBEncodperdeg",0);
  minBoomangle = pref.getFloat("EEMinBoomAngle",0);
  maxBoomangle = pref.getFloat("EEMaxBoomAngle",0);
  
  TrolleyG = pref.getFloat("EETrolleyG",0);
  TrolleyI = pref.getFloat("EETrolleyI",0);
  TrolleyD = pref.getFloat("EETrolleyD",0);
  Trencoderperft = pref.getFloat("EETEncodperft",0);
  
  DragflowG = pref.getFloat("EEDragflowG",0);
  DragflowI = pref.getFloat("EEDragflowI",0);
  DragflowD = pref.getFloat("EEDragflowD",0);
  Dfencoderperft = pref.getFloat("EEDencodperft",0);

  HoseReelG = pref.getFloat("EEHoseReelG",0);
  HoseReelI = pref.getFloat("EEHoseReelI",0);
  HoseReelD = pref.getFloat("EEHoseReel",0);
  Hrencoderperft = pref.getFloat("EEHencodperft",0);
  Hose2drag_ratio = pref.getFloat("EEHoseDragR",0);
  Boomanglecomp = pref.getFloat("EEBoomAComp",0);
 
  HoseGuideG = pref.getFloat("EEHoseGuideG",0);
  HoseGuideI = pref.getFloat("EEHoseGuideI",0);
  HoseGuideD = pref.getFloat("EEHoseGuideD",0);
  hoseGPosScale =  pref.getFloat("EEHoseGPosScale",0);
  
  BoomClipRight = pref.getShort("EEBoomClipRight",0);
  BoomClipLeft = pref.getShort("EEBoomClipLeft",0);
  TrolleyClipOut = pref.getShort("EETrollClipOut",0);
  TrolleyClipIn = pref.getShort("EETrollClipIn",0);
  HoseReelClipOut = pref.getShort("EEHoseClipOut",0);
  HoseReelClipIn = pref.getShort("EEHoseClipIn",0);
  HoseGuideClipOut = pref.getShort("EEHoseGClipOut",0);
  HoseGuideClipIn = pref.getShort("EEHoseGClipIn",0);
  DragflowClipUp = pref.getShort("EEDragfClipUp",0);
  DragflowClipDown = pref.getShort("EEDragfClipDn",0);
  
  Boomdb = pref.getShort("EEBoomdb",0);  
  Trolleydb = pref.getShort("EETrolleydb",0); 
  HoseReeldb = pref.getShort("EEHoseRealdb",0);
  HoseGuidedb = pref.getShort("EEHoseGuidedb",0); 
  Dragflowdb = pref.getShort("EEDragflowdb",0);
  looptimer = pref.getUShort("EELooptimer",0);
  movingOutVel = pref.getShort("EEMovingOutVel",0);
  movingInVel = pref.getShort("EEMovingInVel",0); 
  hoseReelStall = pref.getShort("EEHRStall",0);
  MaxDdepthchange = pref.getFloat("EEDrDepthChange",0);
  MaxBAngchange = pref.getFloat("EEBAngChange",0);
  MaxTrollchange = pref.getFloat("EETrolChange",0);
  Spare1 = pref.getFloat("EESpare1",0);
  Spare2 = pref.getFloat("EESpare2",0);
  Spare3 = pref.getFloat("EESpare3",0);
  Spare4 = pref.getFloat("EESpare4",0);
  Spare5 = pref.getFloat("EESpare5",0);
  pref.end();
  Wire.begin(); //start the I2C bus
  myGPIO.begin(); //Start the gpio module
  myGPIO.pinMode(currentPinMode); // Defined pinmode map
  /*pwm1.begin();
  pwm1.setPWMFreq(900);  // This is the maximum PWM frequency
  pwm1.setPWM(0,1,2048); // set servo valves to off (5volts)
  pwm1.setPWM(1,1,2048); // set servo valves to off (5volts)
  pwm1.setPWM(2,1,2048);
  pwm1.setPWM(3,1,2048);
  pwm1.setPWM(4,1,2048);*/
  //The original module used a pwm to voltage control where the 5 volt range was 2048
  //The new PCB uses a 0-10 module that scales 5 volts to 5000.
  //To use the same gain terms for each of the control loops we should multiply
  //the original saved gain values by 5000/2048 = 2.44;
  //We'll do that here so we're not doing it everytime we go through the control loops

  BoomG = 2.44*BoomG;
  TrolleyG = 2.44*TrolleyG;
  DragflowG = 2.44*DragflowG;
  HoseReelG = 2.44*HoseReelG;
  HoseGuideG = 2.44*HoseGuideG;

  while(dac1.begin()!=0){
    Serial.println("init error");
    delay(1000);
   }
   while(dac2.begin()!=0){
    Serial.println("init error");
    delay(1000);
   }
   while(dac3.begin()!=0){
    Serial.println("init error");
    delay(1000);
   }
  Serial.println("init succeed");
  //Set DAC output range
  dac3.setDACOutRange(dac3.eOutputRange10V);
  delay(1);
  dac1.setDACOutRange(dac1.eOutputRange10V);
  delay(1);
  dac2.setDACOutRange(dac2.eOutputRange10V);
  // These modules have 2 channels 0,1
  // Setting them to a value of 2 sets both channels to the same value
  dac1.setDACOutVoltage(5000, 2); //Set all outputs to 5 volts to start with
  dac2.setDACOutVoltage(5000, 2);
  dac3.setDACOutVoltage(5000, 2);
    
  timer1 = timerBegin(0, 800, true); //Begin timer with 100,000Hz frequency (80MHz/800)
  // use timer0, divide clock by 80 to get 1 mhz, true means count up.
  timerAttachInterrupt(timer1, &onTimer1, true);   //Attach the interrupt to timer1
                                                   // When interupt goes off we'll call the function controlLoop
                                                   
  //looptimer = 2000;
  Serial.print("Current looptimer value ="); Serial.println(looptimer);
  timerAlarmWrite(timer1, looptimer, true);
  timerAlarmEnable(timer1);

  timer2 = timerBegin(1, 8000, true); // run at .01 mhz max divider is 65,536
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 1.45e8, true); //This should be a little over 4 hours and is used for checking the limit switches
  timerAlarmEnable(timer2);
  pinMode(5, OUTPUT);

     

  
 // Set up the encoders *******************************

  ESP32Encoder::useInternalWeakPullResistors=UP;

  //.attachFullQuad()
  //.attachHalfQuad()
  //.attachSingleEdge
  boomencoder.attachFullQuad(17, 16);
  trolleyencoder.attachSingleEdge(35, 34);
  dragflowencoder.attachSingleEdge(33, 32);
  hosereelencoder.attachFullQuad(26, 25);
  hoseguideencoder.attachSingleEdge(14, 13);  
  // set starting count value after attaching
  boomencoder.setFilter(255);
  trolleyencoder.setFilter(255);
  dragflowencoder.setFilter(255);
  hosereelencoder.setFilter(255);
  hoseguideencoder.setFilter(255);
  
    
  boomencoder.setCount(0);
  trolleyencoder.setCount(0);
  dragflowencoder.setCount(0);
  hosereelencoder.setCount(0);
  hoseguideencoder.setCount(0);  
  Serial.println("setup the encoders");
  display.setFont(&FreeSans9pt7b);
  display.begin(0x3C, true); // Address 0x3C default
  //display.begin();
  //display.setFont(&FreeMono9pt7b);
  
  Serial.println("setting up OLED Display");
  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(SSD1327_WHITE);
  display.setCursor(0,12);
  display.println("Loop Tuner");
  //display.setTextColor(SSD1327_BLACK, SSD1327_WHITE); // 'inverted' text
  //display.println(3.141592);
  //display.setTextSize(1);
  //display.setTextColor(SSD1327_WHITE);
  display.print("number: "); display.println(looptimer);
  display.display();

   
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   // Task function. 
                    "Web1",      // name of task.
                    10000,       // Stack size of task 
                    NULL,        // parameter of the task 
                    2,           // priority of the task 
                    &Web,        // Task handle to keep track of created task 
                    0);          // pin task to core 1
    portENTER_CRITICAL(&timer1Mux);
    interrupt1Counter = 0; // Set this to 0 before we start the main loop where we call the control loop
    portEXIT_CRITICAL(&timer1Mux);                

 Serial.println("setup Task1code on core 0");
                 
} // end of setup

void Task1code(void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    Deal_With_client();
    //DisplayOled();
    vTaskDelay(5);
  }
} 
void DisplayOled(){
  if (UpdateDisplay) {
    display.display();
    UpdateDisplay = false;
  }
}  

void loop() {
  //REMOTE UPLOAD
  ArduinoOTA.handle();
  //////
  int Yloc;
  //Serial.println(interruptbool1);
  if(New_Message == 1) HandleLimitSwitches(); // we've seen somthing from one of the remote limit switches
  if (interrupt1Counter > 0) {
    portENTER_CRITICAL(&timer1Mux);
    interrupt1Counter--;
    portEXIT_CRITICAL(&timer1Mux);
    ControlLoop();
  }
  if (interrupt2Counter > 0) {
    portENTER_CRITICAL(&timer2Mux);
    interrupt2Counter--;
    portEXIT_CRITICAL(&timer2Mux);
    HandleLimitSwitches();
    interruptbool2 = true; // Our 4 hour timer is up and we need to see if all switches have reported
  }
  

  //Because we're triggering the control loop by observing the interruptbool1 in the main loop
  //we must be careful not to take too much time to come back through this step in the loop
  //we can tell if we're not getting done in time by seeing if the interrupt counters get equal to or greater than 1
  if (interrupt1Counter >= 1){
    Serial.print("Interupt 1 Counter = ");
    Serial.println(interrupt1Counter); 
  }
  /*if (interrupt2Counter => 1){
    Serial.print("Interupt 2 Counter = ");
    Serial.println(interrupt2Counter); 
  }*/
  //HandleGPIOModule(); // Check the Gpio ports for a change
  if (trolleyCurrentPosition != TrollyP){
  Serial.print("Trolley Position =");
  Serial.println(trolleyCurrentPosition);
  TrollyP = trolleyCurrentPosition;
  UpdateDisplay = true;
  }

  if (boomCurrentPosition != BoomP){
  Serial.print("Boom Position =");
  Serial.println(boomCurrentPosition);
  Serial.print("Boom Servo Output =");
  Serial.println(boomServoOutput);
  Serial.print("Boom Gain =");
  Serial.println(BoomG);
  Serial.print("Boom Control Command =");
  Serial.println(boomControlCommand);
  Serial.print("Run Loop = ");
  Serial.println(RunLoop);
  BoomP = boomCurrentPosition;
  UpdateDisplay = true;
 }
  if (dragflowCurrentPosition != DragP){
  Serial.print("DragFlow Position =");
  Serial.println(dragflowCurrentPosition);
  DragP = dragflowCurrentPosition;
  UpdateDisplay = true;
  }
  
  if (hoseReelCurrentPosition != HoseP){
  Serial.print("Hose Reel Position =");
  Serial.println(hoseReelCurrentPosition);
  HoseP = hoseReelCurrentPosition;
  UpdateDisplay = true;
  }
  if (hoseGuideCurrentPosition != HoseGP){
  Serial.print("Hose Guide Position =");
  Serial.println(hoseGuideCurrentPosition);
  HoseGP = hoseGuideCurrentPosition;
  UpdateDisplay = true;
  }
  if (UpdateDisplay){
  display.clearDisplay();
  // text display tests
  //display.setTextSize(1);
  //display.setTextColor(SSD1327_WHITE);
  display.setCursor(0,15);
  display.print("TrollyP= ");display.println(TrollyP);
  display.print("BoomP= ");display.println(BoomP);
  display.print("DragP= ");display.println(DragP);
  display.print("HoseP= ");display.println(HoseP);
  display.print("HoseGP= ");display.println(HoseGP);
  DisplayOled();
  }
  
}
  
void Deal_With_client() {
 WiFiClient client = server.available();   // Listen for incoming clients
 //int Time_in_Sec = Time_in_Seconds ;
 
 String HTML_Content = "";
 String Upper_Content = "";
 String Lower_Content = "";
 String Form_Content = "";
 //String returnValue = "";
 //int EqualSignLoc = 0;
 //int SpaceFollowing = 0;
 const char* PARAM_INPUT_1 = "New_Location";
 const char* PARAM_INPUT_2 = "input2";
 const char* PARAM_INPUT_3 = "input3";
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
        //Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          //delay(1);
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            Serial.println("HTTP/1.1 200 OK");
            
           
            if (header.indexOf("GET /Update_Status") >= 0){
              //Form_Content = Form();
              //This will effectively refresh the page   //Don't need to refresh the entire page
              Serial.println("Saw Udate Status");
            }
            else if (header.indexOf("GET /Save_To_EEprom") >= 0){
              SaveEEpromValues();
              //client.println(); // The HTTP response ends with another blank line
              //break;
              //goto END;
            }
            else if (header.indexOf("GET /Stop_Loop") >= 0){
              RunLoop = false;
              
              Serial.println("Stopping all loops");
            }
            else if (header.indexOf("GET /Start_Loop") >= 0){
              RunLoop = true;
              
            }
            else if (header.indexOf("GET /form/submit?New_Boom_Location") >= 0) {
              parseLoopValues();
              boomPosition = positionValue;
              BoomG = gainValue;
              BoomD = diffValue;
              BoomClipRight = clip1Value;
              BoomClipLeft = clip2Value;
              Serial.println(boomPosition);
              Serial.println(BoomG);
            }
            else if (header.indexOf("GET /form/submit?New_Trolley_Location") >= 0) {
              parseLoopValues();
              trolleyPosition = positionValue;
              TrolleyG = gainValue;
              TrolleyD = diffValue;
              TrolleyClipOut = clip1Value;
              TrolleyClipIn = clip2Value;
              Serial.println(trolleyPosition);
              Serial.println(TrolleyG);
            }
            else if (header.indexOf("GET /form/submit?New_Dragflow_Location") >= 0) {
              parseLoopValues();
              dragflowheight = positionValue;
              DragflowG = gainValue;
              DragflowD = diffValue;
              DragflowClipUp = clip1Value;
              DragflowClipDown = clip2Value;
              Serial.println(dragflowheight);
              Serial.println(DragflowG);
            }
            else if (header.indexOf("GET /form/submit?New_HoseReel_Location") >= 0) {
              parseLoopValues();
              hoseReelPosition = positionValue; 
              HoseReelG = gainValue;
              HoseReelD = diffValue;
              HoseReelClipOut = clip1Value;
              HoseReelClipIn = clip2Value;
              Serial.println(hoseReelPosition);
              Serial.println(HoseReelG);
            }
            else if (header.indexOf("GET /form/submit?New_HoseGuide_Location") >= 0) {
              parseLoopValues();
              hoseGuidePosition = positionValue;
              HoseGuideG = gainValue;
              HoseGuideD = diffValue;
              HoseGuideClipOut = clip1Value;
              HoseGuideClipIn = clip2Value;
              Serial.println(hoseGuidePosition);
              Serial.println(HoseGuideG);
            }
            else if (header.indexOf("GET /form/submit?New_Hose2drag_ration") >= 0) {
              Serial.println(header);
              parseExtraValues();
              Hose2drag_ratio = extraValue1;
              Boomanglecomp = extraValue2;
              looptimer = Newlooptimer;
              Serial.println(Hose2drag_ratio);
              Serial.println(Boomanglecomp);
            }
            else if (header.indexOf("GET /Refresh") >= 0){
            //Upper_Content = Upper();
            //client.println(Upper_Content);
            //Plant_Run_State = Plant_stopped;
            Serial.println("Saw refresh");
            //goto END;   //Don't need to refresh the entire page
            }

// Display the HTML web page
            //Serial.println("Got to Print Page");
            HTML_Content = Htmlcode();
            client.println(HTML_Content);
            Lower_Content = Lower();
            client.println(Lower_Content);
            Form_Content = BoomForm();
            client.println(Form_Content);
            Form_Content = TrolleyForm();
            client.println(Form_Content);
            Form_Content = DragflowForm();
            client.println(Form_Content);
            Form_Content = HoseReelForm();
            client.println(Form_Content);
            Form_Content = HoseGuideForm();
            client.println(Form_Content);
            
            Form_Content = ExtraVariablesForm();
            client.println(Form_Content);

            
            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            break; // Break out of the while loop
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
/*String Form() {
  String Value ="";
  Value += ("<form action='/get'>");
  Value += ("New_Location: <br><input type='text' name='New_Location'>");
  Value += ("<input type='submit' value='Submit'>");
  Value += ("</form>");
  Value += ("<form action='/get'>");
  Value += ("input2: <br><input type='text' name='input2'>");
  Value += ("<input type='submit' value='Submit'>");
  Value += ("</form>");
  Value += ("<form action='/get'>");
  Value += ("input3: <br><input type='text' name='input3'>");
  Value += ("<input type='submit' value='Submit'>");
  Value += ("</form>");
  return Value;
}*/

 String BoomForm() {
  //Serial.println("Form Boom ");
  char buffer1[10];
  sprintf(buffer1,"%.3f",BoomG);
  char buffer2[10];
  sprintf(buffer2,"%.3f",BoomD); // do this to get 3 decimal places displayed
  String Value ="";
  Value +=("<div>");
  Value +=("<p>Boom Location: "+String(boomCurrentPosition)+" Boom Gain: ");
  Value +=(""+String(buffer1)+" Boom Diff: "+String(buffer2)+" Clip Right: ");
  Value +=(""+String(BoomClipRight)+" Clip Left: "+String(BoomClipRight)+"</p>");
  Value += ("<form action='/form/submit' method='get' <Label> </Label>");
   //Value += ("<Label> </Label>");
      Value += ("New_Boom_Location:<input type='text' value="+String(boomPosition)+" name='New_Boom_Location'/>");
      //Value += ("Gain Value:<input type='text' value="+String(BoomG)+" name='New_Gain'/>");
      Value += ("Gain Value:<input type='text' value="+String(buffer1)+" name='New_Gain'/>");
      Value += ("Diff Value:<input type='text' value="+String(buffer2)+" name='New_Diff'/>");
      Value += ("Clip Right Value:<input type='text' value="+String(BoomClipRight)+" name='New_ClipRight'/>");
      Value += ("Clip Left Value:<input type='text' value="+String(BoomClipLeft)+" name='New_ClipLeft'/>");
      Value += ("<input type='submit' value='Submit' />");
      Value += ("</form></div>");
  return Value;
 }
 String TrolleyForm() {
  //Serial.println("Form Trolley ");
  char buffer1[10];
  sprintf(buffer1,"%.3f",TrolleyG);
  char buffer2[10];
  sprintf(buffer2,"%.3f",TrolleyD); // do this to get 3 decimal places displayed
  String Value ="";
  Value +=("<div>");
  Value +=("<p>Trolley Location: "+String(trolleyCurrentPosition)+" Trolley Gain: ");
  Value +=(""+String(buffer1)+" Trolley Diff: "+String(buffer2)+" Clip Out: ");
  Value +=(""+String(TrolleyClipOut)+" Clip In: "+String(TrolleyClipIn)+"</p>");
  Value += ("<form action='/form/submit' method='get' <Label> </Label>");
  //Value += ("<Label> </Label>");
      Value += ("New_Trolley_Location:<input type='text' value="+String(trolleyPosition)+" name='New_Trolley_Location'/>");
      Value += ("Gain Value:<input type='text' value="+String(buffer1)+" name='New_Gain'/>");
      Value += ("Diff Value:<input type='text' value="+String(buffer2)+" name='New_Diff'/>");
      Value += ("Clip Out Value:<input type='text' value="+String(TrolleyClipOut)+" name='New_ClipRight'/>");
      Value += ("Clip In Value:<input type='text' value="+String(TrolleyClipIn)+" name='New_ClipLeft'/>");
      Value += ("<input type='submit' value='Submit' />");
      Value += ("</form></div>");
  return Value;
 }
 String DragflowForm() {
  //Serial.println("Form Dragflow ");
  char buffer1[10];
  sprintf(buffer1,"%.3f",DragflowG);
  char buffer2[10];
  sprintf(buffer2,"%.3f",DragflowD); // do this to get 3 decimal places displayed
  String Value ="";
  Value +=("<div>");
  Value +=("<p>Current Dragflow Height: "+String(dragflowheight)+" Current Gain Value: ");
  Value +=(""+String(buffer1)+" Dragflow Diff: "+String(buffer2)+" Clip Up: ");
  Value +=(""+String(DragflowClipUp)+" Clip Down: "+String(DragflowClipDown)+"</p>");
  Value += ("<form action='/form/submit' method='get' <Label> </Label>");
  //Value += ("<Label> </Label>");
      Value += ("New_Dragflow_Location:<input type='text' value="+String(dragflowheight)+" name='New_Dragflow_Location'/>");
      Value += ("Gain Value:<input type='text' value="+String(buffer1)+" name='New_Gain'/>");
      Value += ("Diff Value:<input type='text' value="+String(buffer2)+" name='New_Diff'/>");
      Value += ("Clip Up Value:<input type='text' value="+String(DragflowClipUp)+" name='New_ClipRight'/>");
      Value += ("Clip Down Value:<input type='text' value="+String(DragflowClipDown)+" name='New_ClipLeft'/>");
      Value += ("<input type='submit' value='Submit' />");
      Value += ("</form></div>");
  return Value;
 }
 String HoseReelForm() {
  //Serial.println("Form HoseReel ");
  char buffer1[10];
  sprintf(buffer1,"%.3f",HoseReelG);
  char buffer2[10];
  sprintf(buffer2,"%.3f",HoseReelD); // do this to get 3 decimal places displayed
  String Value ="";
  Value +=("<div>");
  Value +=("<p>Current HoseReel Location: "+String(hoseReelCurrentPosition)+" Current Gain Value: ");
  Value +=(""+String(buffer1)+" HoseReel Diff: "+String(buffer2)+" Clip Out: ");
  Value +=(""+String(HoseReelClipOut)+" Clip In: "+String(HoseReelClipIn)+"</p>");
  Value += ("<form action='/form/submit' method='get' <Label> </Label>");
  //Value += ("<Label> </Label>");
      Value += ("New_HoseReel_Location:<input type='text' value="+String(hoseReelPosition)+" name='New_HoseReel_Location'/>");
      Value += ("Gain Value:<input type='text' value="+String(buffer1)+" name='New_Gain'/>");
      Value += ("Diff Value:<input type='text' value="+String(buffer2)+" name='New_Diff'/>");
      Value += ("Clip Out Value:<input type='text' value="+String(HoseReelClipOut)+" name='New_ClipRight'/>");
      Value += ("Clip In Value:<input type='text' value="+String(HoseReelClipIn)+" name='New_ClipLeft'/>");
      Value += ("<input type='submit' value='Submit' />");
      Value += ("</form></div>");
  return Value;
 }
 String HoseGuideForm() {
  //Serial.println("Form HoseGuide ");
  char buffer1[10];
  sprintf(buffer1,"%.3f",HoseGuideG);
  char buffer2[10];
  sprintf(buffer2,"%.3f",HoseGuideD);
  String Value ="";
  Value +=("<div>");
  Value +=("<p>Current HoseGuide Location: "+String(hoseGuideCurrentPosition)+" Current Gain Value: ");
  Value +=(""+String(buffer1)+" HoseGuide Diff: "+String(buffer2)+" Clip Out: ");
  Value +=(""+String(HoseGuideClipOut)+" Clip In: "+String(HoseGuideClipIn)+"</p>");
  Value += ("<form action='/form/submit' method='get' <Label> </Label>");
  //Value += ("<Label> </Label>");
      Value += ("New_HoseGuide_Location:<input type='text' value="+String(hoseGuidePosition)+" name='New_HoseGuide_Location'/>");
      Value += ("Gain Value:<input type='text' value="+String(buffer1)+" name='New_Gain'/>");
      Value += ("Diff Value:<input type='text' value="+String(buffer2)+" name='New_Diff'/>");
      Value += ("Clip Out Value:<input type='text' value="+String(HoseGuideClipOut)+" name='New_ClipRight'/>");
      Value += ("Clip In Value:<input type='text' value="+String(HoseGuideClipIn)+" name='New_ClipLeft'/>");
      Value += ("<input type='submit' value='Submit' />");
      Value += ("</form></div>");
  return Value;
 }
 String ExtraVariablesForm() {
  //Serial.println("Form ExtraVariablesForm ");
  char buffer1[10];
  sprintf(buffer1,"%.3f",Hose2drag_ratio);
  char buffer2[10];
  sprintf(buffer2,"%.3f",Boomanglecomp);
  Newlooptimer = looptimer;
  String Value ="";
  Value +=("<div>");
  Value += ("<form action='/form/submit' method='get' <Label> </Label>");
  //Value += ("<Label> </Label>");
      
      Value += ("Hose_reel_to_dragflow_ratio:<input type='text' value="+String(buffer1)+" name='New_Hose2drag_ration'/>");
      //Value += ("Hose:<input type='text' value="+String(buffer1)+" name='New_Hose2drag_ration'/>");
      Value += ("Boom angle comp:<input type='text' value="+String(buffer2)+" name='New_Boomanglecomp'/>");
      Value += ("looptimer Value:<input type='text' value="+String(Newlooptimer)+" name='New_looptimer'/>");
      Value += ("<input type='submit' value='Submit' />");
      Value += ("</form></div>");
  return Value;
 } 
String Upper() {
  //Serial.println("Upper DIV ");
  
  String Value =("<h1>Test The Control Loop Function</h1>");
  /*if ( (VFD2_Problem+VFD1_Problem+Valve_Problem)> 0){
    // one or more of the modules isn't communicating
 
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
  }*/
    return Value; 
}
String Lower() {
  //Serial.println("Lower DIV ");
  String Value ="";
  
  
  Value += ("<a href=\"/Update_Status\"><button class=\"button btn_off\">Update Status</button></a>");
  Value += ("<a href=\"/Save_To_EEprom\"><button class=\"button btn_on\">Save to EEprom</button></a>");
  Value += ("<a href=\"/Stop_Loop\"><button class=\"button btn_on\">Stop Loop</button></a>");
  Value += ("<a href=\"/Start_Loop\"><button class=\"button btn_off\">Start Loop</button></a>");
  
  
  return Value; 
} // end of Lower function

String Htmlcode() {
  String Value =("");
    Value += ("<!DOCTYPE html><html>");
    Value += ("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    Value += ("<link rel='icon' href='data:,'>"); // here because we have now favicon icon to display
    // CSS to style the on/off buttons 
             
    Value += ("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
    Value += ("input[type=text] {width: 5%; padding: 10px 5px; margin: 1px 1px; box-sizing: border-box; }");
    Value += (" div{border:2px solid red;}");
    Value += (".button { background-color: #4CAF50; border: 2px solid black; color: white; width: 10vw;");
    Value += ("text-decoration: none; height:4vw; font-size: 1vw; margin: 0px; vertical-align:middle; cursor: pointer;}");
    Value += (".btn_off {background-color: red; text-align: center;}");
    Value += (".btn_slow {background-color: blue; text-align: center;}");
    Value += (".btn_50 {background-color: yellow; text-align: center;}");
    Value += (".btn_dis {border-radius: 12px; cursor: not-allowed; border: 2px solid white; text-align: center;}");
    Value += (".btn_large {width: 30vw; height:14vw; font-size: 5vw;}");
    Value += (".btn_Med {width: 18vw; height:9vw; font-size: 2vw;}");
    Value += ("</style></head>");
    
    Value += ("<body><h1>ESP32 Web Server</h1>");
    Value += ("<h2>Test The Control Loop Function</h2>");
  return Value;
}  
