#ifndef STEPPER_MODULE_H
#define STEPPER_MODULE_H

#include <Arduino.h>

#include "FastAccelStepper.h"

class StepperModule
{
public:
  // Constructor: set step/dir pins, steps per revolution, gear ratio, limits, and motion
  // parameters.
  StepperModule(uint8_t stepPin,
                uint8_t dirPin,
                int stepsPerRev,
                float gearRatio,
                float minAngle,
                float maxAngle,
                int velocityMax,
                int accelerationMax);

  // Initialize the stepper motor by connecting it to the FastAccelStepper engine
  bool init(FastAccelStepperEngine * engine);

  // Convert an absolute angle to motor steps
  int32_t angle_to_steps(float angle);

  // Command the motor to move to a specific absolute angle
  void move_to_angle(float angle);

  // Get the current motor angle
  float get_current_angle();

  // Retrieve the current position in steps
  int32_t get_current_steps();

private:
  uint8_t _step_pin;
  uint8_t _dir_pin;
  int _steps_per_rev;           // Steps per revolution
  float _gear_ratio;            // Gearbox Ratio
  float _min_angle;             // Minimum allowed angle
  float _max_angle;             // Maximum allowed angle
  int _velocity_max;            // Maximum velocity in Hz (steps per second)
  int _acceleration_max;        // Maximum acceleration in Hz/s (steps per second squared)
  float _steps_per_degree;      // Conversion factor: steps per degree
  FastAccelStepper * _stepper;  // Pointer to the FastAccelStepper object
};

#endif  // STEPPER_MODULE_H
