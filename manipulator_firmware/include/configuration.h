#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

constexpr uint8_t MOTOR_COUNT = 6U;

// Stepper configuration structure remains the same
struct StepperConfig
{
  uint8_t stepPin;
  uint8_t dirPin;
  int stepsPerRev;
  float gearRatio;
  float minAngle;
  float maxAngle;
  float velocityMax;
  float accelerationMax;
};

// Define each module configuration with descriptive names
const StepperConfig baseMotorConfig     = {41U, 42U, 1600, 1.0F, -180.0F, 180.0F, 800.0F, 800.0F};
const StepperConfig shoulderMotorConfig = {40U, 39U, 1600, 1.0F, -180.0F, 180.0F, 800.0F, 800.0F};
const StepperConfig elbowMotorConfig    = {17U, 18U, 1600, 1.0F, -180.0F, 180.0F, 800.0F, 800.0F};
const StepperConfig wrist1MotorConfig   = {3U, 8U, 1600, 1.0F, -180.0F, 180.0F, 800.0F, 800.0F};
const StepperConfig wrist2MotorConfig   = {12U, 13U, 1600, 1.0F, -180.0F, 180.0F, 800.0F, 800.0F};
const StepperConfig wrist3MotorConfig   = {10U, 11U, 1600, 1.0F, -180.0F, 180.0F, 800.0F, 800.0F};

#endif  // CONFIGURATION_H