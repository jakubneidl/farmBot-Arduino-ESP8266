#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

//Defining new web server
ESP8266WebServer server;

//Local network WiFi SSID
char* ssid = "Router Jary Cimrmana";
//Local network WiFI Password
char* password = "vceskychbudejovicichbychtelzitkazdy";

// JSON document creation
DynamicJsonDocument doc(1024);

//Hanldes connection to the wifi
void connectToWifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

}

//Clears the serial buffer
void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}

// Function to handle payload
void handlePost() {
  //clear the serial buffer
  serialFlush();

  if (server.hasArg("plain") == false) {
    return ;
  }
  //defining the body of the message
  String body = server.arg("plain");
  //deseriaze the body into a Json document
  deserializeJson(doc, body);
  //This part is here in case the document needs editing
  //Serialize the document into serial
  serializeJson(doc, Serial);

  //Start counting time
  unsigned long currentTime = millis();

  //Wait for return message from arduino
  while (!Serial.available()) {
    //if there won't be a response in 50 seconds. Close the connection
    if (millis() - currentTime > 50000) {
      server.send(408, "application/json", "");
    }
  }

  String returnMessage = "";
  returnMessage = Serial.readString();

  server.send(200, "application/json", returnMessage);
}

//Defining webserver endpoints and start server
void serverRouting() {
  server.on("/arduino", HTTP_POST, handlePost);
  server.begin();
}

void setup() {
  //Start serial comunication
  Serial.begin(9600);
  //Connect to wifi
  connectToWifi();
  //Define endpoints and start server
  serverRouting();
}

void loop() {
  //Handle web requests
  server.handleClient();
}
