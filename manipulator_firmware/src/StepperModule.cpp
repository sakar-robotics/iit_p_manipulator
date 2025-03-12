#include "StepperModule.h"

StepperModule::StepperModule(uint8_t stepPin,
                             uint8_t dirPin,
                             int stepsPerRev,
                             float gearRatio,
                             float minAngle,
                             float maxAngle,
                             int velocityMax,
                             int accelerationMax)
  : _step_pin(stepPin)
  , _dir_pin(dirPin)
  , _steps_per_rev(stepsPerRev)
  , _gear_ratio(gearRatio)
  , _min_angle(minAngle)
  , _max_angle(maxAngle)
  , _velocity_max(velocityMax)
  , _acceleration_max(accelerationMax)
  , _stepper(nullptr)
{
  // Calculate conversion factor: (steps per revolution x gear ratio) / 360 allows conversion from
  // degrees to steps.
  _steps_per_degree = (_steps_per_rev * _gear_ratio) / 360.0;
}

bool StepperModule::init(FastAccelStepperEngine * engine)
{
  _stepper = engine->stepperConnectToPin(_step_pin, DRIVER_RMT);
  if (_stepper) {
    _stepper->setDirectionPin(_dir_pin);
    _stepper->setSpeedInHz(_velocity_max);
    _stepper->setAcceleration(_acceleration_max);
    return true;
  }
  return false;
}

int32_t StepperModule::angle_to_steps(float angle)
{
  // Clamp the angle within required limits
  angle = constrain(angle, _min_angle, _max_angle);
  return static_cast<int32_t>(angle * _steps_per_degree);
}

void StepperModule::move_to_angle(float angle)
{
  if (_stepper) {
    _stepper->moveTo(angle_to_steps(angle), false);
  }
}

float StepperModule::get_current_angle()
{
  if (_stepper) {
    return _stepper->getCurrentPosition() / _steps_per_degree;
  }
  return 0.0;
}

int32_t StepperModule::get_current_steps()
{
  if (_stepper) {
    return _stepper->getCurrentPosition();
  }
  return 0;
}