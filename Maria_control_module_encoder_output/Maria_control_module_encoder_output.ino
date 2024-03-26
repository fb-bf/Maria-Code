#include <ESP32Encoder.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
// REMOTE UPLOAD HEADER
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
///////////////////////
//#include <U8g2lib.h> //Library for oled display using characters




ESP32Encoder boomencoder; // encoder for the boom
ESP32Encoder trolleyencoder; // encoder for the trolley
ESP32Encoder dragflowencoder;
ESP32Encoder hosereelencoder;
ESP32Encoder hoseguideencoder;

unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 200;

 const char* ssid     = "Gold";
 const char* password = "123456789";
 
// Set web server port number to 80
WiFiServer server(80);
String header;
int32_t boomPosition=0;
int32_t trolleyPosition=0;
int32_t dragflowPosition=0;
int32_t hosereelPosition=0;
int32_t hoseguidePosition=0;
TaskHandle_t Web;     
  
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
  
  Serial.print("Wifi channel: "); Serial.println(WiFi.channel());
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
  
   
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   // Task function. 
                    "Web1",      // name of task.
                    10000,       // Stack size of task 
                    NULL,        // parameter of the task 
                    1,           // priority of the task 
                    &Web,        // Task handle to keep track of created task 
                    0);          // pin task to core 1
                    

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


void loop() {
  //REMOTE UPLOAD
  ArduinoOTA.handle();
  //////
  boomPosition = (int32_t)boomencoder.getCount();
  trolleyPosition =(int32_t)trolleyencoder.getCount();
  dragflowPosition =(int32_t)dragflowencoder.getCount();
  hosereelPosition =(int32_t)hosereelencoder.getCount();
  hoseguidePosition =(int32_t)hoseguideencoder.getCount();
  delay(10);
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
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
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
            
           
            if (header.indexOf("GET /Update_Status") >= 0){
              //Form_Content = Form();
              //This will effectively refresh the page   //Don't need to refresh the entire page
              Serial.println("Saw Udate Status");
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
            Form_Content = EncoderForm();
            client.println(Form_Content);
            /*Form_Content = TrolleyForm();
            client.println(Form_Content);
            Form_Content = DragflowForm();
            client.println(Form_Content);
            Form_Content = HoseReelForm();
            client.println(Form_Content);
            Form_Content = HoseGuideForm();
            client.println(Form_Content);
            Form_Content = ExtraVariablesForm();
            client.println(Form_Content);*/

            
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

 String EncoderForm() {
  Serial.println("EncoderForm( ");
  
  String Value ="";
  Value +=("<div>");
  Value +=("<p>Boom Encoder ="+String(boomPosition)+"</p>");
  Value +=("<p>Trolley Encoder ="+String(trolleyPosition)+"</p>");
  Value +=("<p>dragflow Encoder ="+String(dragflowPosition)+"</p>");
  Value +=("<p>hosereel Encoder ="+String(hosereelPosition)+"</p>");
  Value +=("<p>hoseguide Encoder ="+String(hoseguidePosition)+"</p>");

  Value += ("</form></div>");
  return Value;
 }
  
String Upper() {
  //Serial.println("Upper DIV ");
  
  String Value =("<h1>Look for encoder readings</h1>");
  return Value; 
}
String Lower() {
  //Serial.println("Lower DIV ");
  String Value ="";
  
  
  Value += ("<a href=\"/Update_Status\"><button class=\"button btn_off\">Update Status</button></a>");
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
    Value += ("<h2>Look for encoder values</h2>");
  return Value;
}  
