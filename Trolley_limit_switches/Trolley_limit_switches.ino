#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
//#include <Wire.h>
// Trolley limit switch vales are 18 for 2 hour timer. 19 and 20 for eiher limit switch activation.
const int ledPin = 19; // This will be used to light the LED for .1 seconds when the
                       // external check pushbutton is pressed.

uint8_t broadcastAddress1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
uint64_t GPIO_reason;
byte Trigger = 0;
//Structure example to receive data
//Must match the sender structure
typedef struct Test_str {
    uint8_t Trigger;
  } Test_str;
Test_str Trigger_state;
byte New_Message = 0;
//#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex use Pin 33 to wake up from sleep
#define BUTTON_PIN_BITMASK 0x108000000 // 2^32 in hex and 2^27 and 2^25 use Pin 32 for one limit switch, pin 27 for the other
                                  //  for limit switch and Pin 25 to light LED. All 3 wake up from sleep
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 :
    //Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
    case ESP_SLEEP_WAKEUP_EXT1 :
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    GPIO_reason = esp_sleep_get_ext1_wakeup_status();
    if(log(GPIO_reason)/log(2) == 32){
      Trigger_state.Trigger = 19; //We'll send the value 13 if this is caused by the limit switch
    }
    if(log(GPIO_reason)/log(2) == 27){
     Trigger_state.Trigger = 20; //We'll send the value 14 if this is caused by the other limit switch
    }   
    
    Serial.print("GPIO that triggered the wake up: GPIO ");
    Serial.println((log(GPIO_reason))/log(2), 0);
    break;
    case ESP_SLEEP_WAKEUP_TIMER :
    Serial.println("Wakeup caused by timer");
    Trigger_state.Trigger = 18; //We'll send the value 12 if this is caused by the Timer
    break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
    //Serial.println("Wakeup caused by touchpad");
    break;
    case ESP_SLEEP_WAKEUP_ULP :
    //Serial.println("Wakeup caused by ULP program");
    break;
    default :
    Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason);
    break;
  }
}
void setup() {
  
  pinMode(32,INPUT_PULLDOWN);
  pinMode(27,INPUT_PULLDOWN);
  pinMode(25,INPUT_PULLDOWN);
  pinMode(26,INPUT_PULLDOWN);
  pinMode(14,INPUT_PULLDOWN);
  Serial.begin(115200);
  delay(10); // give some time to come up and ready
  print_wakeup_reason();
  /*Serial.print("GPIO 16 State : ");
  Serial.println(digitalRead(16));
  Serial.print("GPIO 2 State : ");
  Serial.println(digitalRead(2));*/
  // We won't bother to broadcast anything if the test pushbutton was presssed.
  // Just go back to sleep.
  if (Trigger_state.Trigger > 8){
   WiFi.mode(WIFI_STA);
   int32_t channel = 11;
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false); 
    
    
    //Init ESP-NOW
    if (esp_now_init() != 0) {
      //Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    peerInfo.channel = 11;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      //Serial.println("Failed to add peer");
      return;
    }
    esp_now_register_send_cb(OnDataSent); // register the callback function
      
    Serial.print("Wi-Fi Channel: ");
    Serial.println(WiFi.channel());
     // return;
    //}
    //Trigger_state.Trigger = 1;
    esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *) &Trigger_state, sizeof(Test_str));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      Serial.println(Trigger_state.Trigger);
    }
    else {
      //Serial.println("Error sending the data");
    }
  } // end of the Trigger_state.Trigger <8 
  else{ 
    pinMode(ledPin,OUTPUT);
    digitalWrite(ledPin, HIGH);
    delay (200);
    // turn LED off
    digitalWrite(ledPin, LOW);
  }
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
  //Go to sleep now
  //Serial.println("Going to sleep now");
  //delay(1000);
  esp_sleep_enable_timer_wakeup(7.2e9);
  esp_deep_sleep_start(); 
  //Serial.println("This will never be printed");

}
 
void loop() {

} // end loop
