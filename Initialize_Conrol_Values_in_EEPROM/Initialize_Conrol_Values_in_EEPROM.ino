#include <Preferences.h>
Preferences pref;
float BoomG = .1152;
float BoomI = 0;
float BoomD = 0;
float Bencoderperdeg= 1131.7;
float minBoomangle = 0; // Probably home position
float maxBoomangle = 180;
int BoomClipRight = 2047; // We don't know the direction yet, so go with this.
int BoomClipLeft = -2047;
int Boomdb = 0;
float TrolleyG = .0268;
float TrolleyI = 0;
float TrolleyD = 0;
float Trencoderperft = 7639.;
int TrolleyClipOut = 2047; // in the model, out is a negative direction
int TrolleyClipIn = -2047;
int Trolleydb = 0;
float HoseReelG = .0268;
float HoseReelI = 0;
float HoseReelD = 0;
float Hrencoderperft = 7381.;
float Hose2drag_ratio = .966;
float Boomanglecomp = 1.; //Guess at this time
int HoseReelClipOut = 2047;
int HoseReelClipIn = -2047;
int HoseReeldb = 0;
float DragflowG = .0268;
float DragflowI = 0;
float DragflowD = 0;
float Dfencoderperft = 7639.;
int DragflowClipUp = 2047;
int DragflowClipDown = -2047;
int Dragflowdb = 0;
float HoseGuideG = .1;
float HoseGuideI = 0;
float HoseGuideD = 0;
float hoseGPosScale = .0245;
int HoseGuideClipOut = 2047;
int HoseGuideClipIn = -2047;
int HoseGuidedb = 0;
int looptimer = 2000; //This is a divider used to set loop interupt time
int movingOutVel = 75;  // This is about .5 ft/sec in encodercounts/sec
int movingInVel = -10; // Used to chech if the trolley or dredge is moving in
int hoseReelStall = -5; // checks to see if the hosereel has stalled pulling in the pipe
float MaxDdepthchange = 2.0;
float MaxBAngchange = .5;
float MaxTrollchange = 2.0;
float Spare1; //Spare variables we save in Flash just in case
float Spare2;
float Spare3;
float Spare4;
float Spare5;



void setup() {
  Serial.begin(115200);
  pref.begin("ControlValues", false);
  pref.clear(); // clears the old values in ControlValues
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
  pref.putShort("EELooptimer",looptimer);
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
 
  pref.end();
  Serial.println("Done Writing to EEPROM");
}

void loop() {
  // put your main code here, to run repeatedly:

}
