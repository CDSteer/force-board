#include <Arduino.h>
#include <StandardCplusplus.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>
using namespace std;
// ForceBoard
//
// Description of the project
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author 		Cameron Steer
// 				Cameron Steer
//
// Date			21/03/2016 17:17
// Version		<#version#>
//
// Copyright	© Cameron Steer, 2016
// Licence		<#licence#>
//
// See         ReadMe.txt for references

// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#   include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
#   include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#   include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#   include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#   include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#   include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#   include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#   include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#   include "Arduino.h"
#elif defined(ESP8266) // ESP8266 specific
#   include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#else // error
#   error Platform not definedx
#endif // end IDE

// Include application, user and local libraries

// Prototypes

// Define variables and constants


#define X 3
#define Y 3

#define TOPLEFT 0
#define TOP 1
#define TOPRIGHT 2
#define MIDLEFT 3
#define CENTER 4
#define MIDRIGHT 5
#define BOTLEFT 6
#define BOT 7
#define BOTRIGHT 8

// std::vector< std::vector< float > > item;

struct directions{
    float dx;
    float dy;
};

struct activeAdjacent {
    int dx;
    int dy;
};


struct directions directions[] = {{-1,-1,},{-1,0,},{-1,1},{0,-1}, {0,0}, {0,1},{1,-1},{1,0},{1,1}};

struct float2 {
	float x, y;
};

const float diag = 1.f / sqrt(2.f);
const float2 directions2[9] = { { -diag, -diag }, { 0.f, -1.f }, { diag, -diag },
							   { -1.f, 0.f }, { 0.f, 0.f }, { 1.f, 0.f },
							   { -diag, diag }, { 0.f, 1.f }, { diag, diag } };

struct activeAdjacent activeAdjacents[9];

int fSensors[X][Y];
int fSensorsNewtons[X][Y];
int hX, hY;
int topLeft, top, topRight, center, botLeft, bot, botRight, midLeft, midRight;
static const uint8_t analog_pins[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8};
String direction = "null";


void readForceSensors();
void getAdjacentValues(int x, int y);
int findHighest(int valueArray[X][Y]);
void printForceSensors();
void printHighestAdjacent();
int newtons();
void getDirection();
void printActiveValue();
int findHighestAjacent();
float totalArray(vector<float> _vector);
float forceMagnitude(float x, float y);
void printFloatV(vector<float> _vector);
int highestPoint(vector<float> _vector);
vector<float> forceNormailise(vector<float> _vector);
struct float2 forceGradient(vector<float> _vector);

void setup() {
    Serial.begin(57600);
    for (int i = 0; i < 8; i++) {
        pinMode(analog_pins[i], INPUT);
    }
}

void loop() {
  readForceSensors();
  findHighest(fSensors);
  getAdjacentValues(hX, hY);

  vector<float> normailiseV;
  normailiseV = forceNormailise(normailiseV);

  struct float2 values = forceGradient(normailiseV);
  float angle = forceAngle(values.x, values.y), magnitude = forceMagnitude(values.x, values.y);

  Serial.print(fSensors[hX][hY]);
  Serial.print(",");
  Serial.print(angle);
  Serial.print(",");
  Serial.println(magnitude);

  // Serial.println("--------------------");
  delay(500);
}

void readForceSensors(){
    int k = 0;
    for (int i = 0; i < X; i++) {
        for (int j = 0; j < Y; j++) {
            fSensors[i][j] = analogRead(analog_pins[k]);
            fSensorsNewtons[i][j] = newtons(fSensors[i][j]);
            // Serial.print(k);
            // Serial.print(": ");
            // Serial.println(analogRead(analog_pins[k]));
            k++;
        }
    }
}

void getAdjacentValues(int x, int y) {
    for (int i = 0; i < sizeof(directions) / sizeof(directions[1]); i++) {
        activeAdjacents[i].dx = x + directions[i].dx;
        activeAdjacents[i].dy = y + directions[i].dy;
    }
}

void getDirection(){
  int highestAjacent = findHighestAjacent();
  switch (highestAjacent) {
    case TOP:
      direction = "north";
      break;
    case BOT:
      direction = "south";
      break;
    case MIDLEFT:
      direction = "west";
      break;
    case MIDRIGHT:
      direction = "east";
      break;
    case CENTER:
      direction = "centure";
      break;
    default:
      direction = "null";
      break;
    }
}

int findHighestAjacent() {
    int highest, value;
    for (int i = 0; i < sizeof(activeAdjacents) / sizeof(activeAdjacents[1]); i++) {
      if (i != CENTER){
        if (fSensors[activeAdjacents[i].dx][activeAdjacents[i].dy] > highest) {
          value = fSensors[activeAdjacents[i].dx][activeAdjacents[i].dy];
          highest = i;
        } else if (highest == 0){
          highest = CENTER;
        }
      }
    }
    return highest;
}

int findHighest(int valueArray[3][3]) {
    int highest = valueArray[0][0];
    for (int i = 0; i < X; i++) {
        for (int j = 0; j < Y; j++) {
            if (valueArray[i][j] > highest) {
                highest = valueArray[i][j];
                hX = i;
                hY = j;
            }
        }
    }
    // Serial.print("highest: "); Serial.println(highest);
    return highest;
}

void printForceSensors(){
  for ( int i = 0; i < X; i++ ) {
    for ( int j = 0; j < Y; j++ ) {
      Serial.print(i); Serial.print(","); Serial.print(j);
      Serial.print(" : "); Serial.println(fSensors[i][j]);
      delay(200);
    }
  }
}

void printHighestAdjacent(){
    for (int i = 0; i < sizeof(directions) / sizeof(directions[1]); i++) {
        Serial.print("X: ");
        Serial.print(activeAdjacents[i].dx);
        Serial.print(", ");
        Serial.print("Y: ");
        Serial.print(activeAdjacents[i].dy);
        Serial.print(", ");
        Serial.print("Value: ");
        Serial.println(fSensors[activeAdjacents[i].dx][activeAdjacents[i].dy]);
    }
}


int newtons(int fsrReading) {
    int fsrVoltage;     // the analog reading converted to voltage
    unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
    unsigned long fsrConductance;
    long fsrForce;       // Finally, the resistance converted to force

    // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
    fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);

    if (fsrVoltage != 0) {
        // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
        // so FSR = ((Vcc - V) * R) / V        yay math!
        fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
        fsrResistance *= 10000;                // 10K resistor
        fsrResistance /= fsrVoltage;

        fsrConductance = 1000000;           // we measure in micromhos so
        fsrConductance /= fsrResistance;

        // Use the two FSR guide graphs to approximate the force
        if (fsrConductance <= 1000) {
            fsrForce = fsrConductance / 80;
        } else {
            fsrForce = fsrConductance - 1000;
            fsrForce /= 30;
        }
    }
    return fsrForce;
}

vector<float> forceNormailise(vector<float> _vector){
  for (int i = 0; i < sizeof(activeAdjacents)/sizeof(activeAdjacents[1]); ++i)  {
    _vector.push_back(((float)(fSensors[activeAdjacents[i].dx][activeAdjacents[i].dy])));
  }
  float sforce = totalArray(_vector);
  for (vector<float>::iterator it = _vector.begin(); it != _vector.end(); ++it) {
    *it = ((*it)/(sforce));
  }
  return _vector;
}

struct float2 forceGradient(vector<float> _vector){
  struct float2 values = {0.f, 0.f};
  for (int i = 0; i < sizeof(directions2)/sizeof(directions2[1]); ++i) {
    values.x += directions2[i].x * _vector[i];
    values.y += directions2[i].y * _vector[i];
  }
  return values;
}

float forceAngle(float x, float y) {
    return atan2f(y, x);
    // return atan2f(y, x) * (180.f / 3.14f);
}

float forceMagnitude(float x, float y) {
    return sqrt((x*x)+(y*y));
}

float totalArray(vector<float> _vector){
  float total;
  // Serial.println("totaling....");
  for(vector<float>::iterator it = _vector.begin(); it != _vector.end(); ++it) {
    // Serial.print("add: "); Serial.println(*it);
    total += (*it);
  }
  // Serial.print("total: "); Serial.println(total);
  return total;
}

int highestPoint(vector<float> _vector){
  int value;
  float comp = 0.0;
  for(int i = 0; i < _vector.size(); ++i) {
    if (_vector.at(i) > comp) {
        comp = _vector.at(i);
        value = i;
    }
  }
  return value;
}

void printFloatV(vector<float> _vector){
  // Serial.println("printing....");
  for(auto it = _vector.begin(); it != _vector.end(); ++it) {
    Serial.println(*it);
  }
}

void printActiveValue(){
  Serial.print("activeValue: ");
  Serial.print(activeAdjacents[CENTER].dx);
  Serial.print(", ");
  Serial.println(activeAdjacents[CENTER].dy);
}
