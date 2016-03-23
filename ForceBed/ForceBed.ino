//
// ForceBed
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
//


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

struct {
    int dx;
    int dy;
} directions[] = {{-1,-1,},{-1,0,},{-1,1},{0,-1}, {0,0}, {0,1},{1,-1},{1,0},{1,1}};

struct {
    int dx;
    int dy;
} activeAdjacent[9];

int fSensors[X][Y];
int hX, hY;
int topLeft, top, topRight, center, botLeft, bot, botRight, midLeft, midRight;
static const uint8_t analog_pins[] = {A0, A1, A2, A3, A4, A5, SS, MOSI, MISO};

int fsrPin = A0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance;
long fsrForce;       // Finally, the resistance converted to force

void readForceSensors();
void getAdjacentValues(int x, int y);
int findHighest();
void printForceSensors();
void printHighestAdjacent();
void newtons();

void setup() {
    Serial.begin(9600);
    for (int i = 0; i < 8; i++) {
        pinMode(analog_pins[i], OUTPUT);
        Serial.println(analog_pins[i]);
    }
}

void loop() {
    readForceSensors();
    // printForceSensors();
    // newtons();
    Serial.println("--------------------");
    delay(1000);
//    delay(200);
//    findHighest();
//    getAdjacentValues(hX, hY);
//    Serial.print("activeValue: ");
//    Serial.print(activeAdjacent[CENTER].dx);
//    Serial.print(", ");
//    Serial.println(activeAdjacent[CENTER].dy);
//    delay(200);
}

void readForceSensors(){
    int k;
    for (int i = 0; i < X; i++) {
        for (int j = 0; j < Y; j++) {
            fSensors[i][j] = analogRead(analog_pins[k]);
            Serial.println(analogRead(analog_pins[k]));
            k++;
        }
    }
}

void getAdjacentValues(int x, int y) {
    for (int i = 0; i < sizeof(directions) / sizeof(directions[1]); i++) {
        activeAdjacent[i].dx = x + directions[i].dx;
        activeAdjacent[i].dy = y + directions[i].dy;
    }
    printHighestAdjacent();
}

int findHighest() {    
    int highest = fSensors[0][0];
    for (int i = 0; i < X; i++) {
        for (int j = 0; j < Y; j++) {
            if (fSensors[i][j] > highest) {
                highest = fSensors[i][j];
                hX = i;
                hY = j;             
            }
        } 
    }
    Serial.print("highest: ");
    Serial.println(highest);
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
        Serial.print(activeAdjacent[i].dx);
        Serial.print(", ");
        Serial.print("Y: ");
        Serial.print(activeAdjacent[i].dy);
        Serial.print(", ");
        Serial.print("Value: ");
        Serial.println(fSensors[activeAdjacent[i].dx][activeAdjacent[i].dy]);
    }
}


void newtons() {
    fsrReading = analogRead(fsrPin);
    Serial.print("Analog reading = ");
    Serial.println(fsrReading);
    
    // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
    fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
    Serial.print("Voltage reading in mV = ");
    Serial.println(fsrVoltage);
    
    if (fsrVoltage == 0) {
        Serial.println("No pressure");
    } else {
        // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
        // so FSR = ((Vcc - V) * R) / V        yay math!
        fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
        fsrResistance *= 10000;                // 10K resistor
        fsrResistance /= fsrVoltage;
        Serial.print("FSR resistance in ohms = ");
        Serial.println(fsrResistance);
        
        fsrConductance = 1000000;           // we measure in micromhos so
        fsrConductance /= fsrResistance;
        Serial.print("Conductance in microMhos: ");
        Serial.println(fsrConductance);
        
        // Use the two FSR guide graphs to approximate the force
        if (fsrConductance <= 1000) {
            fsrForce = fsrConductance / 80;
            Serial.print("Force in Newtons: ");
            Serial.println(fsrForce);
        } else {
            fsrForce = fsrConductance - 1000;
            fsrForce /= 30;
            Serial.print("Force in Newtons: ");
            Serial.println(fsrForce);            
        }
    }
    Serial.println("--------------------");
    delay(1000);
}


