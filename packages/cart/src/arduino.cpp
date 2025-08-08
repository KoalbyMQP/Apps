
#include <AccelStepper.h>

#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

bool newPositionAvailable = false;
int targetPosition = 0;  // Changed to int for whole-step values

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(100);

  // Homing sequence: Mov4e to position 0 at startup
  Serial.println("Homing to position 0...");
  stepper.moveTo(0);  // Move stepper to zero position
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();  // Ensure smooth movement to zero
  }

  stepper.setCurrentPosition(0);  // Set position reference after homing
  Serial.println("Homing complete.");
}

void loop() {
  readSerialInput();

  if (newPositionAvailable) {
    // Move to the target position
    stepper.moveTo(targetPosition);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    Serial.println("Reached target position.");
    delay(4000);
    // Move back to zero after reaching the target
    Serial.println("Returning to position 0...");
    stepper.moveTo(0);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    Serial.println("Returned to position 0.");

    newPositionAvailable = false;
  }
}

void readSerialInput() {
  static char inputBuffer[4];
  static byte index = 0;

  while (Serial.available() > 0) {
    char receivedChar = Serial.read();

    if (receivedChar == '\n' || receivedChar == '\r') {
      inputBuffer[index] = '\0';
      int input = atoi(inputBuffer);

      if (input >= 1 && input <= 11) {
        int mappedInput;
        if (input >= 1 && input <= 6) {
          mappedInput = input;
        } else {  // input between 7–11
          mappedInput = (input - 12);
        }

        targetPosition = static_cast<int>(round(mappedInput * 51 * (200.0 / 360.0))); 
        newPositionAvailable = true;

        Serial.print("Moving to position ");
        Serial.println(input);
      } else {
        Serial.println("Invalid input. Enter a number between 1 and 11.");
      }

      index = 0; // Reset buffer index
    } else if (index < 3) {
      inputBuffer[index++] = receivedChar;
    }
  }
}
