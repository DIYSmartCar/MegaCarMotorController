#include <Arduino.h>

#include "ctype.h"

// Motor 1
int dir1PinA = 4;
int dir2PinA = 5;
int speedPinA = 9; // Must be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 6;
int dir2PinB = 7;
int speedPinB = 10; // Must be a PWM pin to be able to control motor speed

int wheelCount = 4;

int motorSpeed = 200;

char command;

int posX = 0;
int posY = 0;

#define TRIGGER_PIN  39  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     37  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

int minimumDistance = 20;

bool reverseVehicle = true;
bool invertLeftAndRight = true;

#define OUTPUT_MODE_SERIAL 0
#define OUTPUT_MODE_SERIAL1 1

int outputMode = OUTPUT_MODE_SERIAL;

bool autoStopWhenNoCommands = false;

bool foundCommand = false;

bool isStopped = true;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600); // Bluetooth

  pinMode(dir1PinA, OUTPUT);
  pinMode(dir2PinA, OUTPUT);
  pinMode(speedPinA, OUTPUT);
  pinMode(dir1PinB, OUTPUT);
  pinMode(dir2PinB, OUTPUT);
  pinMode(speedPinB, OUTPUT);

  //rangeRunningAverage.clear(); // explicitly start clean

  // If it's 4wd increase the speed to ensure it has enough power
  //if (wheelCount == 4)
  //  motorSpeed = 255;

  delay(1000);
  // forward();

  println("Starting car");

}

void loop() {
  //forward();
  //Serial.println(millis() - (lastRangeRead + rangeReadInterval));

  /*if (lastRangeOutput + rangeOutputInterval < millis())
    {
    Serial.print(averageDistance());
    Serial.println("cm");
    lastRangeOutput = millis();
    }*/

  checkForCommand();

  /*if (isTooClose()
      && command == 'F')
    {
    Serial.println("Too close. Stopping");

    Stop();
    }*/
  delay(1);
}

void checkForCommand()
{
  foundCommand = false;

  checkForSerialCommand();

  checkForSerial1Command();

  if (!foundCommand
    && autoStopWhenNoCommands
    && !isStopped)
    {
      // Comment disabled to avoid flooding serial output
      // println("Automatically stopping. No commands received.");
      stop();
    }

}

void handleCommand(char command)
{
  if (!isspace(command))
  {
    print("Command: \"");
    print((String)command);
    println("\"");

    switch (command) {
      case 'F':
        forward();
        break;
      case 'B':
        back();
        break;
      case 'L':
        left();
        break;
      case 'R':
        right();
        break;
      case 'S':
        stop();
        break;
      case 'P':
        ping();
        break;
      case 'A':
        toggleAutoStop();
        break;
      default:
        if (isdigit(command))
          setMotorSpeed(map((int)command, 47, 57, 5, 255));
        else
          stop();
        break;
    }
  }
}

void checkForSerialCommand()
{
  while (Serial.available() > 0) {
    // Switch output mode to serial
    if (outputMode != OUTPUT_MODE_SERIAL)
      outputMode = OUTPUT_MODE_SERIAL;

    byte command = Serial.read();

    if (command != "")
      handleCommand(command);
  }
}

void checkForSerial1Command()
{
  while (Serial1.available() > 0) {
    // Switch output mode to bluetooth
    if (outputMode != OUTPUT_MODE_SERIAL1)
      outputMode = OUTPUT_MODE_SERIAL1;

    byte command = Serial1.read();

    if (command != "")
      handleCommand(command);
  }
}

void forward()
{
  println("forward");

  analogWrite(speedPinA, motorSpeed);//Sets speed variable via PWM
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  analogWrite(speedPinB, motorSpeed);//Sets speed variable via PWM
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);

  isStopped = false;
}

void back()
{
  println("back");

  analogWrite(speedPinA, motorSpeed);
  digitalWrite(dir1PinA, HIGH);
  digitalWrite(dir2PinA, LOW);
  analogWrite(speedPinB, motorSpeed);
  digitalWrite(dir1PinB, HIGH);
  digitalWrite(dir2PinB, LOW);

  isStopped = false;
}

void left()
{
  println("left");

  analogWrite(speedPinA, motorSpeed);
  digitalWrite(dir1PinA, HIGH);
  digitalWrite(dir2PinA, LOW);
  analogWrite(speedPinB, motorSpeed);
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);

  isStopped = false;
}

void right()
{
  println("right");

  analogWrite(speedPinA, motorSpeed);//Sets speed variable via PWM
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  analogWrite(speedPinB, motorSpeed);//Sets speed variable via PWM
  digitalWrite(dir1PinB, HIGH);
  digitalWrite(dir2PinB, LOW);

  isStopped = false;
}

void stop()
{
  println("stop");

  analogWrite(speedPinA, 0);
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, LOW);
  analogWrite(speedPinB, 0);
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, LOW);

  isStopped = true;
}

void setMotorSpeed(int speed)
{
  motorSpeed = speed;
  print("Motor speed:");
  println((String)motorSpeed);
}

void print(String text)
{
  switch (outputMode)
  {
    case OUTPUT_MODE_SERIAL:
      Serial.print(text);
      break;
    case OUTPUT_MODE_SERIAL1:
      Serial1.print(text);
      break;
  }
}

void println(String text)
{
  switch (outputMode)
  {
    case OUTPUT_MODE_SERIAL:
      Serial.println(text);
      break;
    case OUTPUT_MODE_SERIAL1:
      Serial1.println(text);
      break;
  }
}

void ping()
{
  println("pong");
}

void toggleAutoStop()
{
  println("Toggling auto stop");

  autoStopWhenNoCommands = !autoStopWhenNoCommands;

  println(autoStopWhenNoCommands ? "On" : "Off");
}
