#include <Arduino.h>

#include "FastAccelStepper.h"
#include "StepperModule.h"
#include "configuration.h"

// Global instance of FastAccelStepperEngine
FastAccelStepperEngine engine = FastAccelStepperEngine();

// Use an array to store a fixed number of stepper module pointers.
StepperModule * motors[MOTOR_COUNT];

// For clarity, Map each motor to its role by index:
enum MotorRoles
{
  BASE = 0,
  SHOULDER,
  ELBOW,
  WRIST1,
  WRIST2,
  WRIST3
};

void setup()
{
  Serial.begin(115200);

  engine.init();

  // Create and initialize each stepper module
  // .Base motor
  motors[BASE] = new StepperModule(baseMotorConfig.stepPin,
                                   baseMotorConfig.dirPin,
                                   baseMotorConfig.stepsPerRev,
                                   baseMotorConfig.gearRatio,
                                   baseMotorConfig.minAngle,
                                   baseMotorConfig.maxAngle,
                                   baseMotorConfig.velocityMax,
                                   baseMotorConfig.accelerationMax);
  motors[BASE]->init(&engine);

  // .Shoulder motor
  motors[SHOULDER] = new StepperModule(shoulderMotorConfig.stepPin,
                                       shoulderMotorConfig.dirPin,
                                       shoulderMotorConfig.stepsPerRev,
                                       shoulderMotorConfig.gearRatio,
                                       shoulderMotorConfig.minAngle,
                                       shoulderMotorConfig.maxAngle,
                                       shoulderMotorConfig.velocityMax,
                                       shoulderMotorConfig.accelerationMax);
  motors[SHOULDER]->init(&engine);

  // .Elbow motor
  motors[ELBOW] = new StepperModule(elbowMotorConfig.stepPin,
                                    elbowMotorConfig.dirPin,
                                    elbowMotorConfig.stepsPerRev,
                                    elbowMotorConfig.gearRatio,
                                    elbowMotorConfig.minAngle,
                                    elbowMotorConfig.maxAngle,
                                    elbowMotorConfig.velocityMax,
                                    elbowMotorConfig.accelerationMax);
  motors[ELBOW]->init(&engine);

  // .Wrist1 motor
  motors[WRIST1] = new StepperModule(wrist1MotorConfig.stepPin,
                                     wrist1MotorConfig.dirPin,
                                     wrist1MotorConfig.stepsPerRev,
                                     wrist1MotorConfig.gearRatio,
                                     wrist1MotorConfig.minAngle,
                                     wrist1MotorConfig.maxAngle,
                                     wrist1MotorConfig.velocityMax,
                                     wrist1MotorConfig.accelerationMax);
  motors[WRIST1]->init(&engine);

  // // .Wrist2 motor
  // motors[WRIST2] = new StepperModule(wrist2MotorConfig.stepPin,
  //                                    wrist2MotorConfig.dirPin,
  //                                    wrist2MotorConfig.stepsPerRev,
  //                                    wrist2MotorConfig.gearRatio,
  //                                    wrist2MotorConfig.minAngle,
  //                                    wrist2MotorConfig.maxAngle,
  //                                    wrist2MotorConfig.velocityMax,
  //                                    wrist2MotorConfig.accelerationMax);
  // motors[WRIST2]->init(&engine);

  // // .Wrist3 motor
  // motors[WRIST3] = new StepperModule(wrist3MotorConfig.stepPin,
  //                                    wrist3MotorConfig.dirPin,
  //                                    wrist3MotorConfig.stepsPerRev,
  //                                    wrist3MotorConfig.gearRatio,
  //                                    wrist3MotorConfig.minAngle,
  //                                    wrist3MotorConfig.maxAngle,
  //                                    wrist3MotorConfig.velocityMax,
  //                                    wrist3MotorConfig.accelerationMax);
  // motors[WRIST3]->init(&engine);
}

// . Command to send angles to the motors: 45.0,90.0,135.0,180.0\n
void loop()
{
  if (Serial.available() > 0) {
    // Read the entire line until newline and trim whitespace
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();

    // Expecting a comma-separated list of 4 angles, e.g. "45.0,90.0,135.0,180.0"
    int firstComma  = inputString.indexOf(',');
    int secondComma = inputString.indexOf(',', firstComma + 1);
    int thirdComma  = inputString.indexOf(',', secondComma + 1);

    if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
      Serial.println("Error: Please send 4 comma-separated float values.");
    } else {
      float angleBase     = inputString.substring(0, firstComma).toFloat();
      float angleShoulder = inputString.substring(firstComma + 1, secondComma).toFloat();
      float angleElbow    = inputString.substring(secondComma + 1, thirdComma).toFloat();
      float angleWrist1   = inputString.substring(thirdComma + 1).toFloat();

      // Send the respective angles to each motor
      motors[BASE]->move_to_angle(angleBase);
      motors[SHOULDER]->move_to_angle(angleShoulder);
      motors[ELBOW]->move_to_angle(angleElbow);
      motors[WRIST1]->move_to_angle(angleWrist1);

      // Acknowledge the received angles
      Serial.print("Received angles: ");
      Serial.print(angleBase);
      Serial.print(", ");
      Serial.print(angleShoulder);
      Serial.print(", ");
      Serial.print(angleElbow);
      Serial.print(", ");
      Serial.println(angleWrist1);
    }
  }

  // Display the current angles of the motors
  Serial.println("Current Motor angles:");
  Serial.print("Base angle: ");
  Serial.println(motors[BASE]->get_current_angle());
  Serial.print("Shoulder angle: ");
  Serial.println(motors[SHOULDER]->get_current_angle());
  Serial.print("Elbow angle: ");
  Serial.println(motors[ELBOW]->get_current_angle());
  Serial.print("Wrist1 angle: ");
  Serial.println(motors[WRIST1]->get_current_angle());

  delay(200);
}