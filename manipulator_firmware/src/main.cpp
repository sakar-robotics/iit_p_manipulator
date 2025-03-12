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

  // .Wrist2 motor
  motors[WRIST2] = new StepperModule(wrist2MotorConfig.stepPin,
                                     wrist2MotorConfig.dirPin,
                                     wrist2MotorConfig.stepsPerRev,
                                     wrist2MotorConfig.gearRatio,
                                     wrist2MotorConfig.minAngle,
                                     wrist2MotorConfig.maxAngle,
                                     wrist2MotorConfig.velocityMax,
                                     wrist2MotorConfig.accelerationMax);
  motors[WRIST2]->init(&engine);

  // .Wrist3 motor
  motors[WRIST3] = new StepperModule(wrist3MotorConfig.stepPin,
                                     wrist3MotorConfig.dirPin,
                                     wrist3MotorConfig.stepsPerRev,
                                     wrist3MotorConfig.gearRatio,
                                     wrist3MotorConfig.minAngle,
                                     wrist3MotorConfig.maxAngle,
                                     wrist3MotorConfig.velocityMax,
                                     wrist3MotorConfig.accelerationMax);
  motors[WRIST3]->init(&engine);
}

void loop()
{
  // Sample Target angles for each motor
  float targetAngles[MOTOR_COUNT] = {45.0, -30.0, 90.0, -90.0, 0.0, 30.0};

  // Command each motor to move its target angle.
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    motors[i]->move_to_angle(targetAngles[i]);
  }

  delay(5000);

  // Log the current angles
  Serial.println("Current Motor angles:");
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" angle: ");
    Serial.println(motors[i]->get_current_angle());
  }

  // Return all motors to 0
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->move_to_angle(0.0);
    }

    delay(5000);
}