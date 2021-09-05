/*
  Matrix Keypad Demo
  keypad-demo.ino
  Demonstrates use of 4x4 matrix membrane keypad with Arduino
  Results on serial monitor

  DroneBot Workshop 2020
  https://dronebotworkshop.com
*/

// Include the Keypad library
#include <Keypad.h>

// Constants for row and column sizes
const byte ROWS = 4;
const byte COLS = 4;

// Array to represent keys on keypad
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// Connections to Arduino
byte rowPins[ROWS] = {15, 2, 17, 22};     // links
byte colPins[COLS] = {33, 32, 38, 37};    // rechts

// Create keypad object
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

void setup() {
  // Setup serial monitor
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("---------------------------");
  Serial.println("--Starting Keypad Sketch!--");
  Serial.println("---------------------------");
  Serial.println(" ");

}

void loop() {
  // Get key value if pressed
  char customKey = customKeypad.getKey();

  if (customKey) {
    // Print key value to serial monitor
    Serial.println(customKey);
    Serial.println(" ");
    delay(5);
  }
}