#include <ESP32Encoder.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>
#include <Fonts/FreeSans9pt7b.h>
#include "SparkFun_TCA9534.h"
#include <ArduinoJson.h>
#include <StreamUtils.h>

//#include <Adafruit_PWMServoDriver.h>
//Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
#include "DFRobot_GP8403.h"
DFRobot_GP8403 dac1(&Wire,0x5D);
DFRobot_GP8403 dac2(&Wire,0x5E);
DFRobot_GP8403 dac3(&Wire,0x5F);

Adafruit_SSD1327 display(128, 128, &Wire, -1, 800000);
//U8G2_SSD1327_EA_W128128_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
TCA9534 myGPIO; // define the Sparkfun Gpio module
//const char *SSID = "Tiberius 2.4";
//const char *PWD = "Flagstaff9830";
const char *SSID = "Maria-2.4";
const char *PWD = "Maria_Gold";
const char *Sweeping = "$Flagstaff9830";
const char* ssid1     = "ESP_now";
String hostname = "maria_control";
//IPAddress Static_IP(10, 0, 0, 35);
//IPAddress gateway(10, 0, 0, 1);
//IPAddress subnet(255, 255, 0, 0);

IPAddress Static_IP(192, 168, 10, 199);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0); 

ESP32Encoder boomencoder; // encoder for the boom
ESP32Encoder trolleyencoder; // encoder for the trolley
ESP32Encoder dragflowencoder;
ESP32Encoder hosereelencoder;
ESP32Encoder hoseguideencoder;

Preferences pref; // create pref for using the EEPROM section to store latest Control Loop values
WebServer server(80);
//StaticJsonDocument<250> arrayDoc;
StaticJsonDocument<500> jsonDocument;
char buffer[500];

int t1 = 0;
int t2 = 0;
bool UpdateDisplay = false; // used to limit the number of times we run through the display method
#define NUM_GPIO 8
bool currentPinMode[NUM_GPIO] = {GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN, GPIO_IN};
bool gpioStatus[NUM_GPIO];
const int FlowPin = 27;
int FlowValue = 0;
const int ADC2Pin = 4;
int ADC2Value = 0;
float FlowScale = 2200/4000; // 0-4096 from ADC, and 0 to 2200 GPM for the sensor.
float ADC2Scale;
int MyCounter = 0;
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;
portMUX_TYPE timer1Mux = portMUX_INITIALIZER_UNLOCKED;// Used to keep multiple cpus from accessing the interupt at same time??  
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;
//volatile bool interruptbool1 = false;
bool interruptbool2 = false;
volatile int interrupt1Counter = 0;
volatile int interrupt2Counter = 0;
// Current time
unsigned long currentTime = millis();
unsigned long currentTime2 = millis();
// Previous time
unsigned long previousTime = 0;
unsigned long previousTime2 = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 200;
enum moveStates {NotMoving,StartMove,MoveDredge, MoveTrolley, MovingBoom, FinalDredge}; // the states a single traffic light can be in
//makes a variable data type "moveStates" and assigns values 0,1,2,3,4,5 to NotMoving,StartMove MoveDredge, MoveTrolley, MovingBoom, FinalDredge
moveStates MoveState = NotMoving;
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
bool RunLoop = true;
bool DisplayReady = true;
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
float Bencoderperdeg = 1777.7; // Overwritten when Saved variables are read
float boomEncoderIncrement = 0;
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
float currentTrolleyLocation = 0; //units feet, holds converted encoder count location
float trolleyLocation = 23.5; //units feet, holds requested new location from main computer 
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
float movingdredgeDepth = 0; // units feet, negative depth when below 0 level
float boomAngle = 10.3; // units degrees
float dredgeDepth = -20.3 ; // units feet, negative depth when below 0 level
float finaldredgeDepth = 0; // units feet, final location after move command
float radialVelocity = 1.6;
float flowRate = 102.5; // units gallons/minute

int radialIncrement = 0;
int zIncrement = 0;
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
String controllerstatus = "Ready";

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
bool Hose_reel_guide_timer_switch = false; //#9
bool Hose_reel_timer_switch = false; //#12
bool Trolley_timer_switch = false; //#18
bool Boom_angle_timer_switch = false; //#15
float sweepfinishangle = 0;
float sweepstartangle = 0;

String Empty = "";
String Message = Empty;
String Message2 = Empty;
String Severity = "Warning";
String Severity2 = "Warning";
bool statusOverride = false;
bool limitSwitchOverride = false;
bool SweepDirectionPositive = true;
bool SweepOn = true;
bool Stop = false;
bool FirstTimeThrough = true;

uint8_t newMACAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

void IRAM_ATTR onTimer1() {
  portENTER_CRITICAL_ISR(&timer1Mux);
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
    //SweepControl(); Not needed. This is built into the BoomControl function
    MoveControl();
    StoppingControl();
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
void SweepControl(){
   if(SweepOn) {
     boomPosition += boomEncoderIncrement;
     if(SweepDirectionPositive) {
        if ( boomPosition > sweepfinishangle*Bencoderperdeg){
          SweepOn = false;
          controllerstatus = "Ready";
        }
      }    
     if(!SweepDirectionPositive){
        if (boomPosition < sweepfinishangle*Bencoderperdeg){
          SweepOn = false;
          controllerstatus = "Ready";
        }  
     }
  }// end of SweepOn
 
}
void MoveControl(){
  switch (MoveState){
    case NotMoving:
      break;
    case StartMove:
      // For the first time through we need the new dredge location
      dragflowheight = movingdredgeDepth; //Set the moving dredge depth
      MoveState = MoveDredge;
    break;
    case MoveDredge:
    if( abs(DragflowError) < 3000 ){
      MoveState = MoveTrolley; // dredge is less than .5 ft away from lift point
      //Set the desired trolley location
      trolleyPosition = trolleyLocation*Trencoderperft;
    }
    break;
    case MoveTrolley:
      if( abs(TrolleyError) < 3000 ){
        MoveState = MovingBoom; // Trolley is less than .5 ft away from desired location
        SweepOn = true; 
      }
    break;
    case MovingBoom:
      if(!SweepOn){
        MoveState = FinalDredge; // The boom has finished moving, now lower the dredge to final height
        dragflowheight = finaldredgeDepth; //Set the moving dredge depth
      }
      break;
    case FinalDredge:
      if( abs(DragflowError) < 3000 ){
        MoveState = NotMoving; // dredge is less than .5 ft away from final point
        controllerstatus = "Ready";
      }
  }
}
void StoppingControl(){
  if (Stop){
    if ( FirstTimeThrough ) FirstTimeThrough =false;
    else if( abs(DragflowError) < 3000 ){
      controllerstatus = "Ready"; // The dredge has moved to the correct height
      Stop = false;
    }
  }
}

void BoomControl(){
  //LoopCounter++;
  // counts per foot at 90 ft radius will be 1131
  // Boom errors could be 20,000 or larger!!
  // If BoomG = 3, BoomControlCommand will be 60,000, and we need values of +/- 50
  // If we want max speed with a 10 foot error, then Gain = 1/(10*1131/2048) = .1152
  // We'll then clip the max control effort to 2048 for moves greater than 10 ft. 
  // when we're dealing with PWM width.
   if(SweepOn) {
     boomPosition += boomEncoderIncrement;
     if(SweepDirectionPositive) {
        if ( boomPosition > sweepfinishangle*Bencoderperdeg){
          SweepOn = false;
          controllerstatus = "Ready";
        }
      }    
     if(!SweepDirectionPositive){
        if (boomPosition < sweepfinishangle*Bencoderperdeg){
          SweepOn = false;
          controllerstatus = "Ready";
        }  
     }
    }// end of SweepOn
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
  
   hoseReelPosition = dragflowCurrentPosition * Hose2drag_ratio + Boomanglecomp * boomCurrentPosition + hoseReeladjustment;
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
    currentTime = millis();
    if (currentTime > ( previousTime + 1000)){ // This will wait 1 second before creating another adjustment
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
      previousTime = currentTime;
    } //end of timer
    Hose_reel_upper_switch = false;
  }
  if (Hose_reel_lower_switch) { // when true we have too much hose let out
    currentTime = millis();
    if (currentTime > ( previousTime + 1000)){ // This will wait 1 second before creating another adjustment
      hoseReeladjustment = hoseReeladjustment - Hrencoderperft; //pull in 1 extra foot of hose
      previousTime = currentTime;
    }
  Hose_reel_lower_switch = false;  
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
  bool All_reporting = true;
  Serial.print("Saw a trigger ");
  Serial.println(Trigger_State.Trigger);
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
    Severity2 = "Warning";
    if ( Hose_reel_guide_timer_switch == false){
      Serial.print("Hose_reel_guide_switch failed to report ");
      Message2 = "Hose_reel_guide_switch failed to report";
      All_reporting = false;
    }

    if ( Hose_reel_timer_switch == false){
      Serial.print("Hose_reel_switch failed to report ");
      Message2 = "Hose_reel_switch failed to report";
      All_reporting = false;
    }

    if ( Trolley_timer_switch == false){
      Serial.print("Trolley_switch failed to report ");
      Message2 = "Trolley_switch failed to report";
      All_reporting = false;
    }

    if ( Boom_angle_timer_switch == false){
      Serial.print("Boom_angle_switch failed to report ");
      Message2 = "Boom_angle_switch failed to report";
      All_reporting = false;
    }

    if ( Dragflow_timer_switch == false){
      Serial.print("Dragflow_switch failed to report ");
      Message2 = "Dragflow_switch failed to report";
      All_reporting = false;
    }

    if(All_reporting) Serial.print("All limit switches reporting ");
    // reset all timer switch states
    Dragflow_timer_switch = false;
    Hose_reel_guide_timer_switch = false;
    Boom_angle_timer_switch = false;
    Trolley_timer_switch = false;
    Hose_reel_timer_switch = false;     
  } //end of interruptbool2 = true;
  Trigger_State.Trigger = 0; //set this so if we see a 0 we know we've been back though here.
}
void Limitswitchstop(){
  
  if( limitSwitchOverride ){
   RunLoop = true; 
  } else
  { 
    RunLoop = false; // This will stop the loop from running until it is reset
    if(Trigger_State.Trigger == 10 ) Limitswitchstatus = "Hose_reel_guide_right_switch Tripped";
    if(Trigger_State.Trigger == 11 ) Limitswitchstatus = "Hose_reel_guide_left_switch Tripped";
    if(Trigger_State.Trigger == 19 ) Limitswitchstatus = "Trolley_out_switch Tripped";
    if(Trigger_State.Trigger == 20 ) Limitswitchstatus = "Trolley_in_switch Tripped";
    if(Trigger_State.Trigger == 16 ) Limitswitchstatus = "Boom_low_angle_switch Tripped";
    if(Trigger_State.Trigger == 17 ) Limitswitchstatus = "Boom_high_angle_switch";
    if(Trigger_State.Trigger == 22 ) Limitswitchstatus = "Dragflow_up_switch Tripped";
    Serial.println(Limitswitchstatus);
    Message2 = Limitswitchstatus;
    controllerstatus = "Hold";
    Severity2 = "High";
  }
}
void ReadFlowSensor(){
  FlowValue = analogRead(FlowPin);
  flowRate = FlowScale*FlowValue;
}
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}     
TaskHandle_t Web;
//TaskHandle_t Oled;

void Task1code(void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    server.handleClient();
    vTaskDelay(20);
  }
}

void setup() {
  Serial.begin(115200); 
  Serial.print("Connecting to Wi-Fi");
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid1);
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  WiFi.config(Static_IP, gateway, subnet, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname.c_str()); //define hostname
  
  WiFi.begin(SSID, PWD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
 int channel = getWiFiChannel(SSID);
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.getHostname());
  Serial.print("Connected on Channel: ");
  Serial.println(channel);

  Serial.print("[NEW] ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
 
  // ESP NOW sets the Mac address to 01 so our remote limit switch can talk to this
  
  
  //Init ESP-NOW
    if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  
  esp_now_register_recv_cb(OnDataRecv);
   
  setup_routing();
  
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
 /* xTaskCreatePinnedToCore(
                    Oledcode,   // Task function. 
                    "Oled1",      // name of task.
                    20000,       // Stack size of task 
                    NULL,        // parameter of the task 
                    2,           // priority of the task 
                    &Oled,        // Task handle to keep track of created task 
                    0);          // pin task to core 0*/
  Wire.begin(); //start the I2C bus
  myGPIO.begin(); //Start the gpio module
  myGPIO.pinMode(currentPinMode); // Defined pinmode map
  /*pwm1.begin();
  pwm1.setPWMFreq(900);  // This is the maximum PWM frequency
  pwm1.setPWM(0,1,2048);
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

  display.setFont(&FreeSans9pt7b);
  display.begin(0x3C, true); // Address 0x3C default
  //display.begin();
  //display.setFont(&FreeMono9pt7b);
  
  
  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(SSD1327_WHITE);
  display.setCursor(0,12);
  display.println("Maria Control");
  //display.setTextColor(SSD1327_BLACK, SSD1327_WHITE); // 'inverted' text
  //display.println(3.141592);
  //display.setTextSize(1);
  //display.setTextColor(SSD1327_WHITE);
  display.println("  Running"); //display.println(looptimer);
  display.display();
    
  timer1 = timerBegin(0, 800, true); //Begin timer with 100,000Hz frequency (80MHz/800)
  // use timer0, divide clock by 80 to get 1 mhz, true means count up.
  timerAttachInterrupt(timer1, &onTimer1, true);   //Attach the interrupt to timer1
                                                   // When interupt goes off we'll call the function controlLoop
  timerAlarmWrite(timer1, looptimer, true);      //Initialize the timer (interupt repeates every 50hz
  timerAlarmEnable(timer1);
   
  timer2 = timerBegin(1, 8000, true); // run at .01 mhz max divider is 65,536
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 1.45e8, true); //This should be a little over 4 hours and is used for checking the limit switches
  //timerAlarmWrite(timer2, 1.2e6, true); //This should be 2 minutes and is used for debug
  timerAlarmEnable(timer2);


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
  
   
  
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   // Task function. 
                    "Web1",      // name of task.
                    10000,       // Stack size of task 
                    NULL,        // parameter of the task 
                    1,           // priority of the task 
                    &Web,        // Task handle to keep track of created task 
                    0);          // pin task to core 0
 

    portENTER_CRITICAL(&timer1Mux);
    interrupt1Counter = 0; // Set this to 0 before we start the main loop where we call the control loop
    portEXIT_CRITICAL(&timer1Mux);                     
} // end of setup

void DisplayOled(){
  if (UpdateDisplay) {
    DisplayReady = false;
    display.clearDisplay();
    // text display tests here but know you'll overrun the control loop when you do
    //display.setTextSize(1);
    //display.setTextColor(SSD1327_WHITE);
    display.setCursor(0,15);
    display.print("TrollyP= ");display.println(TrollyP);
    display.print("BoomP= ");display.println(BoomP);
    display.print("DragP= ");display.println(DragP);
    display.print("HoseP= ");display.println(HoseP);
    display.print("HoseGP= ");display.println(HoseGP);
    display.display();
    UpdateDisplay = false;
    DisplayReady = true;
  }
}

void loop() {
  
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
    interruptbool2 = true; // Our 4 hour timer is up and we need to see if all switches have reported
    HandleLimitSwitches();

  }
  
  //Because we're triggering the control loop by observing the interruptbool1 in the main loop
  //we must be careful not to take too much time to come back through this step in the loop
  //we can tell if we're not getting done in time by seeing if the interrupt counters get equal to or greater than 1
  if (interrupt1Counter >= 1){
    Serial.print("Interupt 1 Counter = ");
    Serial.println(interrupt1Counter); 
  }
  if (interrupt2Counter >= 1){
    Serial.print("Interupt 2 Counter = ");
    Serial.println(interrupt2Counter); 
  }
  //HandleGPIOModule(); // Check the Gpio ports for a change
  /*currentTime2 = millis();  // read flow sensor every second.
  if((currentTime2 - previousTime2) > 1000){
    ReadFlowSensor();
    previousTime2 = currentTime2; 
  }*/
  if (trolleyCurrentPosition != TrollyP){
  Serial.print("Trolley Position =");
  Serial.println(trolleyCurrentPosition);
  TrollyP = trolleyCurrentPosition;
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
  BoomP = boomCurrentPosition;
  //if (DisplayReady) UpdateDisplay = true;
  }

  if (dragflowCurrentPosition != DragP){
  Serial.print("DragFlow Position =");
  Serial.println(dragflowCurrentPosition);
  DragP = dragflowCurrentPosition;
  }
  
  if (hoseReelCurrentPosition != HoseP){
  Serial.print("Hose Reel Position =");
  Serial.println(hoseReelCurrentPosition);
  HoseP = hoseReelCurrentPosition;
  }
  if (hoseGuideCurrentPosition != HoseGP){
  Serial.print("Hose Guide Position =");
  Serial.println(hoseGuideCurrentPosition);
  HoseGP = hoseGuideCurrentPosition;
  }
  if (UpdateDisplay){
    /*display.clearDisplay();
    // text display tests here but know you'll overrun the control loop when you do
    //display.setTextSize(1);
    //display.setTextColor(SSD1327_WHITE);
    display.setCursor(0,15);
    display.print("TrollyP= ");display.println(TrollyP);
    display.print("BoomP= ");display.println(BoomP);
    display.print("DragP= ");display.println(DragP);
    display.print("HoseP= ");display.println(HoseP);
    display.print("HoseGP= ");display.println(HoseGP);*/
    //DisplayOled();
  }
} // end of loop
  

void setup_routing() {     
  server.on("/status", HTTP_GET, getstatus);  //These are assummed to be Get requests
  server.on("/boom_parameters", HTTP_GET, getboomparameters);  //These are assummed to be Get requests
  server.on("/trolley_parameters", HTTP_GET, gettrolleyparameters);
  server.on("/hosereel_parameters", HTTP_GET, gethosereelparameters);
  server.on("/hoseguide_parameters", HTTP_GET, gethoseguideparameters);
  server.on("/dredge_parameters", HTTP_GET, getdredgeparameters);
  server.on("/misc_parameters", HTTP_GET, getmiscparameters); 
  server.on("/sweep", HTTP_POST , handlePostsweep);
  server.on("/move", HTTP_POST , handlePostmove);
  server.on("/stop", HTTP_POST , handlePoststop);
  server.on("/status_override", HTTP_POST , handlePoststatusoverride);
  server.on("/limit_switch_override", HTTP_POST , handlePostlimitswitchoverride);
  server.on("/save", HTTP_POST , handlePostsave);    
  server.on("/Set_Parameter", HTTP_POST, handlePostParams);    
     
  server.begin();    
}
 
void create_json_success_or_fail(bool success) { 
  jsonDocument.clear();  
  jsonDocument["Status"] = success ? "Success" : "Failed";
  serializeJson(jsonDocument, buffer);
}

void add_json_error_object( String Message, String Severity) {
  JsonObject obj = jsonDocument.createNestedObject("error");
  obj["message"] = Message;
  obj["severity"] = Severity;
}

void getstatus() {
  Serial.println("Reporting Status");
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
  currentTrolleyLocation = trolleyCurrentPosition/Trencoderperft;
  jsonDocument.clear();
  jsonDocument["Status"] = controllerstatus;
  jsonDocument["Signal_Strength"] = WiFi.RSSI();
  jsonDocument["Dredge_Depth"] = dragflowheight;
  jsonDocument["Boom_Angle"]= boomAngle;
  jsonDocument["Trolley_Location"] = currentTrolleyLocation;
  jsonDocument["Radial_Velocity"] = radialVelocity;
  jsonDocument["Flow_Rate"] = flowRate;
  jsonDocument["Limit_Switch_Override"] = limitSwitchOverride;
  if (Message != Empty) add_json_error_object(Message,Severity);
  if (Message2 != Empty) add_json_error_object(Message2,Severity2);
  //add_json_error_array(Message,Severity);
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}
void getboomparameters() {
  Serial.println("Reporting Boom Parameters");
  jsonDocument.clear();
  jsonDocument["Status"] = "Boom_Paramters";
  jsonDocument["Boom_encoder_setting"] = Bencoderperdeg;
  jsonDocument["Boom_Min_Angle"]= minBoomangle;
  jsonDocument["Boom_Max_Angle"] = maxBoomangle;
  jsonDocument["Boom_Gain"] = BoomG;
  jsonDocument["Boom_Diff"] = BoomD;
  jsonDocument["Boom_Integrator"] = BoomI;
  jsonDocument["Boom_Clip_Right"] = BoomClipRight;
  jsonDocument["Boom_Clip_Left"] = BoomClipLeft;
  jsonDocument["Boom_Dead_Band"] = Boomdb;
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}
void gettrolleyparameters() {
  Serial.println("Reporting Trolley Parameters");
  jsonDocument.clear();
  jsonDocument["Status"] = "Trolley_Paramters";
  jsonDocument["Trolley_Max_Location"] = maxtrolleylocation;
  jsonDocument["Trolley_encoder_setting"] = Trencoderperft;
  jsonDocument["Trolley_Gain"] = TrolleyG;
  jsonDocument["Trolley_Diff"] = TrolleyD;
  jsonDocument["Trolley_Integrator"] = TrolleyI;
  jsonDocument["Trolley_Clip_Out"] = TrolleyClipOut;
  jsonDocument["Trolley_Clip_In"] = TrolleyClipIn;
  jsonDocument["Trolley_Dead_Band"] = Trolleydb;
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}
void gethosereelparameters() {
  Serial.println("Reporting Hose Reel Parameters");
  jsonDocument.clear();
  jsonDocument["Status"] = "Hose_Reel_Paramters";
  jsonDocument["Hosereel_encoder_setting"] = Hrencoderperft;
  jsonDocument["Hosereel_to_dragflow_Ratio"] = Hose2drag_ratio;
  jsonDocument["Hosereel_Gain"] = HoseReelG;
  jsonDocument["Hosereel_Diff"] = HoseReelD;
  jsonDocument["Hosereel_Integrator"] = HoseReelI;
  jsonDocument["Hosereel_Clip_Out"] = HoseReelClipOut;
  jsonDocument["Hosereel_Clip_In"] = HoseReelClipIn;
  jsonDocument["Hosereel_Dead_Band"] = HoseReeldb;
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}
void gethoseguideparameters() {
  Serial.println("Reporting Hose Reel Parameters");
  jsonDocument.clear();
  jsonDocument["Status"] = "Hose_Guide_Paramters";
  jsonDocument["Hoseguide_Position_Scale"] = hoseGPosScale;
  jsonDocument["Hoseguide_Gain"] = HoseGuideG;
  jsonDocument["Hoseguide_Diff"] = HoseGuideD;
  jsonDocument["Hoseguide_Integrator"] = HoseGuideI;
  jsonDocument["Hoseguide_Clip_Out"] = HoseGuideClipOut;
  jsonDocument["Hoseguide_Clip_In"] = HoseGuideClipIn;
  jsonDocument["Hoseguide_Dead_Band"] = HoseGuidedb;
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}
void getdredgeparameters() {
  Serial.println("Reporting Hose Reel Parameters");
  jsonDocument.clear();
  jsonDocument["Status"] = "Dredge_Paramters";
  jsonDocument["Dredge_encoder_setting"] = Dfencoderperft;
  jsonDocument["Dredge_Gain"] = DragflowG;
  jsonDocument["Dredge_Diff"] = DragflowD;
  jsonDocument["Dredge_Integrator"] = DragflowI;
  jsonDocument["Dredge_Clip_Up"] = DragflowClipUp;
  jsonDocument["Dredge_Clip_Down"] = DragflowClipDown;
  jsonDocument["Dredge_Dead_Band"] = Dragflowdb;
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}
 
void getmiscparameters() {
  Serial.println("Reporting Misc Parameters");
  jsonDocument.clear();
  jsonDocument["Status"] = "Misc_Paramters";
  jsonDocument["Loop_Timer_setting"] = looptimer;
  jsonDocument["Moving_Out_Velocity"] = movingOutVel;
  jsonDocument["Moving_In_Velocity"] = movingInVel;
  jsonDocument["Hose_Reel_Stall_Velocity"] = hoseReelStall;
  jsonDocument["Max_Dredge_Depth_Change"] = MaxDdepthchange;
  jsonDocument["Max_Boom_Angle_Change"] = MaxBAngchange;
  jsonDocument["Max_Trolley_Location_Change"] = MaxTrollchange;
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}

void handlePostsweep() {
  bool AllKeysFound = true;
  if (server.hasArg("plain") == false) {
  //Handle the error here
  }
  //Check the state of the controller
  
    String body = server.arg("plain");
    DeserializationError err = deserializeJson(jsonDocument, body);;
  if (err) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(err.c_str());
  }
  Message = Empty; // clear past messages
  Message2 = Empty;
  if (!jsonDocument.containsKey("Dredge_Depth")) AllKeysFound = false;
  if (!jsonDocument.containsKey("Start_Angle")) AllKeysFound = false;
  if (!jsonDocument.containsKey("Trolley_Location")) AllKeysFound = false;
  if (!jsonDocument.containsKey("Finish_Angle")) AllKeysFound = false;
  if (!jsonDocument.containsKey("Radial_Velocity")) AllKeysFound = false;
  //if (!jsonDocument.containsKey("Status_Override")) AllKeysFound = false;
  //if (!jsonDocument.containsKey("Limit_Switch_Override")) AllKeysFound = false;
  if( AllKeysFound ) {
    dredgeDepth = jsonDocument["Dredge_Depth"];
    trolleyLocation = jsonDocument["Trolley_Location"];
    sweepstartangle = jsonDocument["Start_Angle"];
    sweepfinishangle = jsonDocument["Finish_Angle"];
    radialVelocity = jsonDocument["Radial_Velocity"];
    statusOverride = jsonDocument["Status_Override"];
    //limitSwitchOverride = jsonDocument["Limit_Switch_Override"];
    //if( statusOverride ) controllerstatus = "Ready";
    
    if(controllerstatus == "Ready" || controllerstatus == "Waiting"){
      boomAngle = (float)boomLastPosition/Bencoderperdeg;
      currentTrolleyLocation = trolleyCurrentPosition/Trencoderperft;
      Serial.print("Dredge_Depth = ");
      Serial.println(dredgeDepth);
      Serial.print("Radial Velocity = ");
      Serial.println(radialVelocity);
      Serial.print("Start_Angle = ");
      Serial.println(sweepstartangle);
      Serial.print("Finish_Angle = ");
      Serial.println(sweepfinishangle);
      //Check inputs to make sure this sweep request is valid
      // Check for dredge height to be close to current height
      // Check for start angle to be close to current angle
      // check that finish angle is in bounds of allowed sweeping.
      if( abs(dredgeDepth-dragflowheight) > MaxDdepthchange) {
        Message = "Too large a dredge depth change requested";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }else if( dredgeDepth  > 0.0) {
        Message = "Dredge Depth out of range";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }else if( abs(trolleyLocation - currentTrolleyLocation) > MaxTrollchange){
        Message = "Trolley Location too far away from current position";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }else if(trolleyLocation > maxtrolleylocation){
        Message = "Trolley Location out of range";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }else if( abs(sweepstartangle - boomAngle) > MaxBAngchange){
        Message = "Start Angle too far away from current angle";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }else if( (sweepfinishangle < minBoomangle) || (sweepfinishangle > maxBoomangle)){
        Message = "Finish Angle out of range";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }
      else // Everthing is Ok with htis request
      {
       controllerstatus = "Sweeping";
       Message = Empty;
       getstatus();
       StartNewsweep();  
      }
    }
    else{
    controllerstatus = "Not_Ready";
    Message = Empty;
    getstatus();
    }
  }else{   //Not all the keys were found request to have last message resent
    controllerstatus = "Resend_Requested";
    Message = "Not all Keys were found in last request";
    getstatus();
  }
} // end of handlePostsweep

void handlePoststatusoverride() {
   Message = Empty; // clear past messages
  Message2 = Empty;
  controllerstatus = "Ready";
  getstatus(); 
}
void handlePostlimitswitchoverride() {
  Message = Empty; // clear past messages
  Message2 = Empty;
  if (server.hasArg("plain") == false) {
  //Handle the error here
  }
  //Check the state of the controller
  
    String body = server.arg("plain");
    DeserializationError err = deserializeJson(jsonDocument, body);;
  if (err) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(err.c_str());
  }
  if (jsonDocument.containsKey("Limit_Switch_Override")){
  limitSwitchOverride = jsonDocument["Limit_Switch_Override"];
  RunLoop = true; //Allow the loop to run anyway
  controllerstatus = "Ready";
  getstatus();
  } else {
    controllerstatus = "Resend_Requested";
    Message = "Not all Keys were found in last request";
    getstatus();  
  }
   
}
void handlePostmove() {
  bool AllKeysFound = true;
  if (server.hasArg("plain") == false) {
  //Handle the error here
  }
  //Check the state of the controller
  
    String body = server.arg("plain");
    DeserializationError err = deserializeJson(jsonDocument, body);;
  if (err) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(err.c_str());
  }
  Message = Empty; // clear past messages
  Message2 = Empty;
  if (!jsonDocument.containsKey("Moving_Height")) AllKeysFound = false;
  if (!jsonDocument.containsKey("Final_Height")) AllKeysFound = false;
  if (!jsonDocument.containsKey("Trolley_Location")) AllKeysFound = false;
  if (!jsonDocument.containsKey("Finish_Angle")) AllKeysFound = false;
  if (!jsonDocument.containsKey("Radial_Velocity")) AllKeysFound = false;
  //if (!jsonDocument.containsKey("Status_Override")) AllKeysFound = false;
  //if (!jsonDocument.containsKey("Limit_Switch_Override")) AllKeysFound = false;
  if( AllKeysFound ) {
    movingdredgeDepth = jsonDocument["Moving_Height"];
    finaldredgeDepth = jsonDocument["Final_Height"];
    trolleyLocation = jsonDocument["Trolley_Location"];
    sweepfinishangle = jsonDocument["Finish_Angle"];
    radialVelocity = jsonDocument["Radial_Velocity"];
    //statusOverride = jsonDocument["Status_Override"];
    //limitSwitchOverride = jsonDocument["Limit_Switch_Override"];
    
    //if( statusOverride ) controllerstatus = "Ready";
    //if( limitSwitchOverride ) RunLoop = true; //Allow the loop to run anyway
    
    if(controllerstatus == "Ready" || controllerstatus == "Waiting"){
      boomAngle = (float)boomLastPosition/Bencoderperdeg;
      currentTrolleyLocation = trolleyCurrentPosition/Trencoderperft;
      Serial.print("Dredge_Depth = ");
      Serial.println(dredgeDepth);
      Serial.print("Radial Velocity = ");
      Serial.println(radialVelocity);
      Serial.print("Start_Angle = ");
      Serial.println(sweepstartangle);
      Serial.print("Finish_Angle = ");
      Serial.println(sweepfinishangle);
      //Check inputs to make sure this sweep request is valid
      // Check for dredge height to be close to current height
      // Check for start angle to be close to current angle
      // check that finish angle is in bounds of allowed sweeping.
      if( finaldredgeDepth  > 0.0) {
        Message = "Final Dredge Depth out of range";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }else if( movingdredgeDepth  > 0.0) {
        Message = "Moving Dredge Depth out of range";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }else if( (trolleyLocation > maxtrolleylocation) || (trolleyLocation < 0) ){
        Message = "Trolley Location out of range";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }else if( (sweepfinishangle < minBoomangle) || (sweepfinishangle > maxBoomangle)){
        Message = "Finish Angle out of range";
        Severity = "High";
        controllerstatus = "Waiting";
        getstatus();
      }
      else // Everthing is Ok with this request
      {
       controllerstatus = "Moving";
       Message = Empty;
       getstatus();
       StartNewmove();  
      }
    }
    else{
      controllerstatus = "Not_Ready";
      Message = Empty;
      getstatus();
    }
  }
  else{  //Not all the keys were found request to have last message resent
    controllerstatus = "Resend_Requested";
    Message = "Not all Keys were found in last request";
    getstatus();
  }
} // end of  handlePostmove
void handlePoststop(){
  // This can simply stop all motion of all devices, or also request a change in the Dredge depth
  // while stoping other motion
  Stop = false; // Should be false, but just in case.
  if (jsonDocument.containsKey("Dredge_Height")){
    dragflowheight = jsonDocument["Dredge_Height"];
    Stop = true;
  }

  // Set everythings desired location to where it is now.
  boomPosition = boomCurrentPosition;
  trolleyPosition = trolleyCurrentPosition;
  if (!Stop) {
    dragflowPosition = dragflowCurrentPosition;
    Stop = true;
  }
  hoseReelPosition = hoseReelCurrentPosition;
  hoseGuidePosition = hoseGuideCurrentPosition;
  SweepOn = false; // turn off any existing sweep
  MoveState = NotMoving; // turn off any move that might be happening
  
  if (Stop){
    controllerstatus = "Stopping";
    FirstTimeThrough = true;
  }
  else controllerstatus = "Ready";
  create_json_success_or_fail(true);
  server.send(200, "application/json", buffer);
 }
void handlePostsave(){
  SaveEEpromValues();
  create_json_success_or_fail(true);
  server.send(200, "application/json", buffer);
}
void handlePostParams(){
    String body = server.arg("plain");
    
    bool Foundkey = true;
    Serial.print(body);
    //deserializeJson(jsonDocument, body);
    DeserializationError err = deserializeJson(jsonDocument, body);;
    if (err) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(err.c_str());
    }
    
    if (jsonDocument.containsKey("Boom_encoder_setting")) Bencoderperdeg = jsonDocument["Boom_encoder_setting"];
    else if (jsonDocument.containsKey("Boom_Min_Angle")) minBoomangle = jsonDocument["Boom_Min_Angle"];
    else if (jsonDocument.containsKey("Boom_Max_Angle")) maxBoomangle = jsonDocument["Boom_Max_Angle"];
    else if (jsonDocument.containsKey("Boom_Gain")) BoomG = jsonDocument["Boom_Gain"];
    else if (jsonDocument.containsKey("Boom_Diff")) BoomD = jsonDocument["Boom_Diff"];
    else if (jsonDocument.containsKey("Boom_Clip_Right")) BoomClipRight = jsonDocument["Boom_Clip_Right"];
    else if (jsonDocument.containsKey("Boom_Clip_Left")) BoomClipLeft = jsonDocument["Boom_Clip_Left"];
    else if (jsonDocument.containsKey("Boom_Integrator")) BoomI = jsonDocument["Boom_Integrator"];
    else if (jsonDocument.containsKey("Boom_Dead_Band")) Boomdb = jsonDocument["Boom_Dead_Band"];
    
    else if (jsonDocument.containsKey("Trolley_Max_Location"))  maxtrolleylocation = jsonDocument["Trolley_Max_Location"];
    else if (jsonDocument.containsKey("Trolley_encoder_setting")) Trencoderperft = jsonDocument["Trolley_encoder_setting"];
    else if (jsonDocument.containsKey("Trolley_Gain")) TrolleyG = jsonDocument["Trolley_Gain"];
    else if (jsonDocument.containsKey("Trolley_Diff")) TrolleyD = jsonDocument["Trolley_Diff"];
    else if (jsonDocument.containsKey("Trolley_Integrator")) TrolleyI = jsonDocument["Trolley_Integrator"];
    else if (jsonDocument.containsKey("Trolley_Clip_Out")) TrolleyClipOut = jsonDocument["Trolley_Clip_Out"];
    else if (jsonDocument.containsKey("Trolley_Clip_In")) TrolleyClipIn = jsonDocument["Trolley_Clip_In"];
    else if (jsonDocument.containsKey("Trolley_Dead_Band")) Trolleydb = jsonDocument["Trolley_Dead_Band"];

    else if (jsonDocument.containsKey("Hosereel_to_dragflow_Ratio")) Hose2drag_ratio = jsonDocument["Hosereel_to_dragflow_Ratio"];
    else if (jsonDocument.containsKey("Hosereel_encoder_setting")) Hrencoderperft = jsonDocument["Hosereel_encoder_setting"];
    else if (jsonDocument.containsKey("Hosereel_Gain")) HoseReelG = jsonDocument["Hosereel_Gain"];
    else if (jsonDocument.containsKey("Hosereel_Diff")) HoseReelD = jsonDocument["Hosereel_Diff"];
    else if (jsonDocument.containsKey("Hosereel_Integrator")) HoseReelI = jsonDocument["Hosereel_Integrator"];
    else if (jsonDocument.containsKey("Hosereel_Clip_Out")) HoseReelClipOut = jsonDocument["Hosereel_Clip_Out"];
    else if (jsonDocument.containsKey("Hosereel_Clip_In")) HoseReelClipIn = jsonDocument["Hosereel_Clip_In"];
    else if (jsonDocument.containsKey("Hosereel_Dead_Band")) HoseReeldb = jsonDocument["Hosereel_Dead_Band"];

    else if (jsonDocument.containsKey("Hoseguide_Position_Scale")) hoseGPosScale = jsonDocument["Hoseguide_Position_Scale"];
    else if (jsonDocument.containsKey("Hoseguide_Gain")) HoseGuideG = jsonDocument["Hoseguide_Gain"];
    else if (jsonDocument.containsKey("Hoseguide_Diff")) HoseGuideD = jsonDocument["Hoseguide_Diff"];
    else if (jsonDocument.containsKey("Hoseguide_Integrator")) HoseGuideI = jsonDocument["Hoseguide_Integrator"];
    else if (jsonDocument.containsKey("Hoseguide_Clip_Out")) HoseGuideClipOut = jsonDocument["Hoseguide_Clip_Out"];
    else if (jsonDocument.containsKey("Hoseguide_Clip_In")) HoseGuideClipIn = jsonDocument["Hoseguide_Clip_In"];
    else if (jsonDocument.containsKey("Hoseguide_Dead_Band")) HoseGuidedb = jsonDocument["Hoseguide_Dead_Band"];

    else if (jsonDocument.containsKey("Dredge_encoder_setting")) Dfencoderperft = jsonDocument["Dredge_encoder_setting"];
    else if (jsonDocument.containsKey("Dredge_Gain")) DragflowG = jsonDocument["Dredge_Gain"];
    else if (jsonDocument.containsKey("Dredge_Diff")) DragflowD = jsonDocument["Dredge_Diff"];
    else if (jsonDocument.containsKey("Dredge_Integrator")) DragflowI = jsonDocument["Dredge_Integrator"];
    else if (jsonDocument.containsKey("Dredge_Clip_Up")) DragflowClipUp = jsonDocument["Dredge_Clip_Up"];
    else if (jsonDocument.containsKey("Dredge_Clip_Down")) DragflowClipDown = jsonDocument["Dredge_Clip_Down"];
    else if (jsonDocument.containsKey("Dredge_Dead_Band")) Dragflowdb = jsonDocument["Hoseguide_Dead_Band"];

    else if (jsonDocument.containsKey("Loop_Timer_setting")) looptimer = jsonDocument["Loop_Timer_setting"];
    else if (jsonDocument.containsKey("Moving_Out_Velocity")) movingOutVel = jsonDocument["Moving_Out_Velocity"];
    else if (jsonDocument.containsKey("Moving_In_Velocity")) movingInVel = jsonDocument["Moving_In_Velocity"];
    else if (jsonDocument.containsKey("Hose_Reel_Stall_Velocity")) hoseReelStall = jsonDocument["Hose_Reel_Stall_Velocity"];
    else if (jsonDocument.containsKey("Max_Dredge_Depth_Change")) MaxDdepthchange = jsonDocument["Max_Dredge_Depth_Change"];
    else if (jsonDocument.containsKey("Max_Boom_Angle_Change")) MaxBAngchange = jsonDocument["Max_Boom_Angle_Change"];
    else if (jsonDocument.containsKey("Max_Trolley_Location_Change")) MaxTrollchange = jsonDocument["Max_Trolley_Location_Change"];

    else Foundkey = false;
    //zIncrement = jsonDocument["z_increment"];
    Serial.print("radial_increment = ");
    Serial.println(radialIncrement);
    Serial.print("z_increment = ");
    Serial.println(zIncrement);
 
  jsonDocument.clear(); 
  if (Foundkey)jsonDocument["Status"] = "Success";
  else jsonDocument["Status"] = "failed";
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}
void StartNewsweep(){
  //This function will set up the next sweep by moving the dredge
  // and the trolley first, then calculating the number of encoder counts
  // per loop update to move the boom at the correct radial velocity.
    dragflowheight = dredgeDepth; //Set the asked for dredge depth 
    trolleyPosition = trolleyLocation*Trencoderperft; // Set trolley position in encoder counts.

  // angular velocity in rad/sec is simply radialVelocity (ft/sec)/ trolleyPosition (ft)
  // multiply that by encoder counts per radian.  counts/deg * 57.295 = 101859.1636. Then divide this value
  // by the number of times the loop executes in a second 
  // The looptimer is set for 2000, and that produces a .020 second loop time. 
    boomEncoderIncrement =(radialVelocity/trolleyPosition)*Bencoderperdeg*57.295*.02*(float)(looptimer/2000);
    SweepDirectionPositive = true;
    if(sweepfinishangle < sweepstartangle){
      boomEncoderIncrement = -boomEncoderIncrement;
      SweepDirectionPositive = false;
    }
  SweepOn = true;
}
void StartNewmove(){
  // Calculates the boomencoderincrement needed, and move direction
  // This will use the already in place SweepOn = true feature in the BoomControl function

  boomEncoderIncrement =(radialVelocity/trolleyPosition)*Bencoderperdeg*57.295*.02*(float)(looptimer/2000);
  SweepDirectionPositive = true;
  if(sweepfinishangle < sweepstartangle){
    boomEncoderIncrement = -boomEncoderIncrement;
    SweepDirectionPositive = false;
  }
  MoveState = StartMove; //start the MoveControl function
  // All of the timing for moving is handles in the MoveControl function
 }
