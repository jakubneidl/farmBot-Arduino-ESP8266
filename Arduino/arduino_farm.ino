#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <DHT.h>
//----------SENSORS-------------
//---Soil-Temperature-Sensor---
//defining NTC Termistor analog pin
#define SOIL_TEMP_PIN A15
//Values based on NTC 10k sensor documentation. Used in getSoilTemp()
#define RT0 10000
#define B 3977
#define VCC 5
#define R 10000

//---Air-Humidity-Temperature-Sensor---
//defining DHT sensor (humidity, temp) pins and type
#define DHTPIN 22
#define DHTTYPE DHT11

//--Soil-Humidity--
//defining crop soil humidity analog pin
#define SOIL_HUMIDITY_PIN A8
// soil humidity Max value 
#define SOIL_HUMIDITY_MAX 700
// soil humidity Low value 
#define SOIL_HUMIDITY_MIN 350

//defining global soil humidity analog pin
#define SOIL_GLOBAL_HUMIDITY_PIN A9
// soil globla humidity Max value 
#define SOIL_GLOBAL_HUMIDITY_MAX 450
// soil global humidity Low value 250
#define SOIL_GLOBAL_HUMIDITY_MIN 250


//----------Water-Pump-------------
//defining water pump pin
#define WATER_PUMP_PIN 48
// definging watering time
#define WATERING_TIME 15000

//----------MOTORS-----------------
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
//-Environment
DHT dht(DHTPIN, DHTTYPE);

//Serial message
String message = "";
bool messageReady = false;

//Helper functions
//-function to read soil temperature
float getSoilTemp()
{
  float RT, VR, ln, TX, T0, VRT;

  T0 = 25 + 273.15;
  VRT = analogRead(A15);        //Acquisition analog value of VRT
  VRT = (5.00 / 1023.00) * VRT; //Conversion to voltage
  VR = VCC - VRT;
  RT = VRT / (VR / R); //Resistance of RT

  ln = log(RT / RT0);
  TX = (1 / ((ln / B) + (1 / T0))); //Temperature from thermistorA

  TX = TX - 273.15; //Conversion to Celsius
  return TX;
}

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
void homeStepper(AccelStepper *stepper, int endStopPin)
{
  //Setup motor parameters
  stepper->setMaxSpeed(500);
  stepper->setAcceleration(500);
  stepper->move(-9999);

  while (!digitalRead(endStopPin))
  {
    stepper->run();
  }

  stepper->setCurrentPosition(0);
}

void homeStepper(AccelStepper *stepper, int endStopType, int endStopPin)
{
  //Setup motor parameters
  stepper->setMaxSpeed(500);
  stepper->setAcceleration(500);
  stepper->move(-9999);
  //Check if the motors aren't disabled
  if (!steppersEnabled)
  {
    enableDisableSteppers(true);
  }

  if (endStopType == 0)
  {
    while (!digitalRead(endStopPin))
    {
      stepper->run();
    }
  }
  else if (endStopType == 1)
  {
    while (digitalRead(endStopPin))
    {
      stepper->run();
    }
  }
  stepper->setCurrentPosition(0);
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

  //Setting up water pump pin
  pinMode(WATER_PUMP_PIN, OUTPUT);
  //turning the pump of
  // digitalWrite(WATER_PUMP_PIN, HIGH);

  //Setup sleep pin for motors
  pinMode(SLEEP_PIN, OUTPUT);

  //Enviroment sensors setup
  dht.begin();
  //


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
      if (!doc["request"] == "water")
      {
        while (stepperZ.distanceToGo() != 0)
        {
          stepperZ.run();
        }
      }

      doc.clear();
      doc["type"] = "response";
      serializeJson(doc, Serial2);
      serializeJson(doc, Serial);
    }

    if (doc["type"] == "plant")
    {

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

    if (doc["type"] == "home")
    {

      homeStepper(&stepperX, 0, ENDSTOP_X);
  

      homeStepper(&stepperY, 1, ENDSTOP_Y);
    

      //Manual setting
      // homeStepper(stepperZ, 1, ENDSTOP_Z);
      stepperZ.setCurrentPosition(0);
      stepperZ.setAcceleration(200);
      stepperZ.setMaxSpeed(300);

      doc.clear();
      doc["type"] = "response";
      serializeJson(doc, Serial2);
    }

    if (doc["type"] == "environment")
    {

      // float groundTemp;
      if (doc["request"] == "all")
      {
        doc.clear();
        float dhtHumidity;
        float dhtTemperature;
        dhtHumidity = dht.readHumidity();
        dhtTemperature = dht.readTemperature();
        Serial.println("DHT sensor Temperature and humidity: ");
        Serial.println(dhtTemperature, dhtHumidity);
        doc["type"] = "response";
        doc["dhtHumidity"] = dhtHumidity;
        doc["dhtTemperature"] = dhtTemperature;
        doc["soilTemp"] = getSoilTemp();
        doc["globalSoilHumidity"] = map(analogRead(SOIL_GLOBAL_HUMIDITY_PIN), SOIL_GLOBAL_HUMIDITY_MIN, SOIL_GLOBAL_HUMIDITY_MAX, 0, 100);
        serializeJson(doc, Serial2);
      }
      else if (doc["request"] == "air_temperature")
      {
        doc.clear();
        doc["reading"] = dht.readTemperature();
        doc["value"] = "°C";
        serializeJson(doc, Serial2);
        serializeJson(doc, Serial);
      }
      else if (doc["request"] == "soil_humidity_global")
      {
        doc.clear();
        doc["reading"] = map(analogRead(SOIL_GLOBAL_HUMIDITY_PIN), SOIL_GLOBAL_HUMIDITY_MIN, SOIL_GLOBAL_HUMIDITY_MAX, 0, 100);
        doc["value"] = "%";
        serializeJson(doc, Serial2);
        serializeJson(doc, Serial);
      }
      else if (doc["request"] == "air_humidity")
      {
        doc.clear();
        doc["reading"] = dht.readHumidity();
        doc["value"] = "%";
        serializeJson(doc, Serial2);
        serializeJson(doc, Serial);
      }
      else if (doc["request"] == "soil_humidity_sensor")
      {
        doc.clear();
        doc["reading"] = map(analogRead(SOIL_HUMIDITY_PIN), SOIL_HUMIDITY_MIN, SOIL_HUMIDITY_MAX, 0, 100);
        doc["value"] = "soil_humidity";
        serializeJson(doc, Serial2);
        serializeJson(doc, Serial);
      }
      else if (doc["request"] == "soil_temp_sensor")
      {
        doc.clear();
        doc["reading"] = getSoilTemp();

        doc["value"] = "°C";

        serializeJson(doc, Serial2);
        serializeJson(doc, Serial);
      }
      else if (doc["request"] == "water")
      {
        digitalWrite(WATER_PUMP_PIN, HIGH);
        delay(WATERING_TIME);
        digitalWrite(WATER_PUMP_PIN, LOW);

        doc["type"] = "response";
        serializeJson(doc, Serial2);
        serializeJson(doc, Serial);
        doc.clear();
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
