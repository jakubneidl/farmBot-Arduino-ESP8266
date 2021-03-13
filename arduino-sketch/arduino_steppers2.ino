#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <DHT.h>

//defining DS18B20 Sensor pin
#define SOIL_HUMIDITYPIN

//defining DHT sensor (humidity, temp) pins and type
#define DHTPIN 22
#define DHTTYPE DHT11

//defining endstop pins
#define ENDSTOP_X 9
#define ENDSTOP_Y 10
#define ENDSTOP_Z 11

//defining enable pin for motors
#define ENABLE_MOTORS_PIN 8
//defining max lengh of each axis in steps
#define MAX_LENGTH_X 1650
#define MAX_LENGTH_Y 1650
#define MAX_LENGTH_Z 1650

//Defining SLEEP PIN for all the motors
#define SLEEP_PIN 8

//---- Global Variables ----
//Stepper motors
AccelStepper stepperX(1, 2, 5);
AccelStepper stepperY(1, 3, 6);
AccelStepper stepperZ(1, 4, 7);
//Steppers enable bool check
bool steppersEnabled = false;

//Sensors
//-Enviroment
DHT dht(DHTPIN, DHTTYPE);

//Serial message
String message = "";
bool messageReady = false;

//Helper functions

//-function to enable or disable steppers
void enableDisableSteppers(bool enable)
{
  if (enable)
  {
    digitalWrite(ENABLE_MOTORS_PIN, LOW);
    steppersEnabled = true;
  }
  else
  {
    digitalWrite(ENABLE_MOTORS_PIN, HIGH);
    steppersEnabled = false;
  }
}

void homeStepper(AccelStepper stepper, int endStopType, int endStopPin)
{
  //Setup motor parameters
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(500);
  stepper.move(-9999);
  //Check if the motors aren't disabled
  if (!steppersEnabled)
  {
    enableDisableSteppers(true);
  }

  if (endStopType == 0)
  {
    while (!digitalRead(endStopPin))
    {
      stepper.run();
    }
  }
  else if (endStopType == 1)
  {
    while (digitalRead(endStopPin))
    {
      stepper.run();
    }
  }
  stepper.setCurrentPosition(0);
}

void setup()
{

  //Starting serial comunication
  //Serial1 using RX1 TX1 pins for comunication
  Serial2.begin(9600);

  //Serial using RX0 TX0 pins for debuging
  Serial.begin(9600);

  // Setting up endstop pins
  pinMode(ENDSTOP_X, INPUT_PULLUP);
  pinMode(ENDSTOP_Y, INPUT_PULLUP);
  pinMode(ENDSTOP_Z, INPUT_PULLUP);

  //Setup sleep pin for motors
  pinMode(SLEEP_PIN, OUTPUT);

  //Enviroment sensors setup
  dht.begin();
}

void loop()
{
  //Wait for Serial input
  while (Serial2.available())
  {
    //save the input into message
    message = Serial2.readString();
    //message is ready to be parsed
    messageReady = true;
    //print it to serial for debuging purposes
    Serial.println(message);
  }
  if (messageReady)
  {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, message);
    if (error)
    {
      Serial.print(F("deserializeJson() failed:"));
      Serial.println(error.c_str());
      messageReady = false;
      return;
    }
    // switch(doc["type"]) {
    //   case "request":
    //     Serial.println("request");
    //     break;
    //   case "request2":
    //     Serial.println("request2");
    //     // code block
    //     break;
    //   case "request3":
    //     Serial.println("request3");
    //     // code block
    //     break;
    //   default:
    //     Serial.print("default");
    // }

    // {"type": "movement", "x": "500",  "y": "500"}
    if (doc["type"] == "movement")
    {
      stepperZ.moveTo(0);
      while (stepperZ.distanceToGo() != 0)
      {
        stepperZ.run();
      }

      int stepToGoX = doc["positionX"];
      int stepToGoY = doc["positionY"];
      int stepToGoZ = doc["positionZ"];

      stepperX.moveTo(stepToGoX);
      stepperY.moveTo(stepToGoY);
      stepperZ.moveTo(stepToGoZ);
      while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
      {
        stepperX.run();
        stepperY.run();
      }
      while (stepperZ.distanceToGo() != 0)
      {
        stepperZ.run();
      }
      delay(2000);

      doc.clear();
      doc["type"] = "response";
      serializeJson(doc, Serial2);
      serializeJson(doc, Serial);
    }

    //  {"type":"plant","x":"800","y":"350","z":"0","seederX":1600,"seederY":0,"seederZ":0}
    if (doc["type"] == "plant")
    {

      // int seederPositionX = doc["positionSeeder"]["positionX"];
      // int seederPositionY = doc["positionSeeder"]["positionY"];
      // int seederPositionZ = doc["positionSeeder"]["positionZ"];

      // int plantPositionX = doc["positionPlant"]["positionX"];
      // int plantPositionY = doc["positionPlant"]["positionY"];
      // int plantPositionZ = doc["positionPlant"]["positionZ"];

      int seederPositionX = doc["seederPositionX"];
      int seederPositionY = doc["seederPositionY"];
      int seederPositionZ = doc["seederPositionZ"];

      int plantPositionX = doc["positionX"];
      int plantPositionY = doc["positionY"];
      int plantPositionZ = doc["positionZ"];

      Serial.println("Seeder position");
      Serial.println(plantPositionX);
      Serial.println(plantPositionY);
      Serial.println(plantPositionZ);
      Serial.println("Seeder position");
      Serial.println(seederPositionX);
      Serial.println(seederPositionY);
      Serial.println(seederPositionZ);

      stepperX.moveTo(seederPositionX);
      stepperY.moveTo(seederPositionY);
      //wait for XY-seeder position to finish
      while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
      {
        stepperX.run();
        stepperY.run();
      }
      //Start downward movement to pickup the seed
      stepperZ.moveTo(seederPositionZ);
      while (stepperZ.distanceToGo() != 0)
      {
        stepperZ.run();
      }

      //wait 2 seconds
      delay(2000);

      //raise the seed
      stepperZ.moveTo(0);
      while (stepperZ.distanceToGo() != 0)
      {
        stepperZ.run();
      }

      //wait for XY - plantposition to finish
      stepperX.moveTo(plantPositionX);
      stepperY.moveTo(plantPositionY);
      while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
      {
        stepperX.run();
        stepperY.run();
      }

      //move the seed downward
      stepperZ.moveTo(plantPositionZ);
      while (stepperZ.distanceToGo() != 0)
      {
        stepperZ.run();
      }

      //wait for 2 seconds
      delay(2000);

      //move the arm back up
      stepperZ.moveTo(0);
      while (stepperZ.distanceToGo() != 0)
      {
        stepperZ.run();
      }
      doc.clear();
      doc["type"] = "response";
      serializeJson(doc, Serial2);
      serializeJson(doc, Serial);
    }

    //{"type": "home"}
    if (doc["type"] == "home")
    {

      homeStepper(stepperX, 0, ENDSTOP_X);
      stepperX.setCurrentPosition(0);
      stepperX.setAcceleration(1000);
      stepperX.setMaxSpeed(1000);

      homeStepper(stepperY, 1, ENDSTOP_Y);
      stepperY.setCurrentPosition(0);
      stepperY.setAcceleration(1000);
      stepperY.setMaxSpeed(1000);

      // homeStepper(stepperZ, 1, ENDSTOP_Z);
      stepperZ.setCurrentPosition(0);
      stepperZ.setAcceleration(1000);
      stepperZ.setMaxSpeed(1000);

      doc.clear();
      doc["type"] = "response";
      serializeJson(doc, Serial2);
    }
    //working up here

    if (doc["type"] == "environment")
    {
      float dhtHumidity;
      float dhtTemperature;
      // float groundTemp;
      if (doc["request"] == "all")
      {
        dhtHumidity = dht.readHumidity();
        dhtTemperature = dht.readTemperature();
        Serial.println("DHT sensor Temperature and humidity: ");
        Serial.println(dhtTemperature, dhtHumidity);
        doc["type"] = "response";
        doc["dhtHumidity"] = dhtHumidity;
        doc["dhtTemperature"] = dhtTemperature;
        serializeJson(doc, Serial2);
      }
      else if (doc["request"] == "soil")
      {
        doc.clear();
        doc["reading"] = dhtHumidity = dht.readHumidity();
        ;
        doc["value"] = "%";
        serializeJson(doc, Serial2);
        serializeJson(doc, Serial);
      }
      doc.clear();
    }

    if (doc["type"] == "disable")
    {
      // enableDisableSteppers(false);
      enableDisableSteppers(false);
      doc["type"] == "response";
      serializeJson(doc, Serial2);
    }

    if (doc["type"] == "enable")
    {
      // enableDisableSteppers(true);
      enableDisableSteppers(true);
      doc["type"] == "response";
      serializeJson(doc, Serial2);
    }

    messageReady = false;
  }
}
