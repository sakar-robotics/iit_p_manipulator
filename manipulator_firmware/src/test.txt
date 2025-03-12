#include <Arduino.h>
#include "FastAccelStepper.h"

#define dirPinStepper1 42
#define stepPinStepper1 41

#define dirPinStepper2 39
#define stepPinStepper2 40

#define dirPinStepper3 18
#define stepPinStepper3 17

#define dirPinStepper4 8
#define stepPinStepper4 3

#define microsteps 1600

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;
FastAccelStepper *stepper4 = NULL;

void setup()
{
  Serial.begin(115200);
  engine.init();

  // Attach stepper 1
  stepper1 = engine.stepperConnectToPin(stepPinStepper1, DRIVER_RMT);
  if (stepper1)
  {
    stepper1->setDirectionPin(dirPinStepper1);
    stepper1->setSpeedInHz(800);
    stepper1->setAcceleration(800);
  }

  // Attach stepper 2
  stepper2 = engine.stepperConnectToPin(stepPinStepper2, DRIVER_RMT);
  if (stepper2)
  {
    stepper2->setDirectionPin(dirPinStepper2);
    stepper2->setSpeedInHz(800);
    stepper2->setAcceleration(800);
  }

  // Attach stepper 3
  stepper3 = engine.stepperConnectToPin(stepPinStepper3, DRIVER_RMT);
  if (stepper3)
  {
    stepper3->setDirectionPin(dirPinStepper3);
    stepper3->setSpeedInHz(800);
    stepper3->setAcceleration(800);
  }

  // Attach stepper 4
  stepper4 = engine.stepperConnectToPin(stepPinStepper4, DRIVER_RMT);
  if (stepper4)
  {
    stepper4->setDirectionPin(dirPinStepper4);
    stepper4->setSpeedInHz(800);
    stepper4->setAcceleration(800);
  }
}

void loop()
{
  // Move all steppers to target position
  if (stepper1)
    stepper1->moveTo(microsteps * 2, false);
  if (stepper2)
    stepper2->moveTo(microsteps * 2, false);
  if (stepper3)
    stepper3->moveTo(microsteps * 2, false);
  if (stepper4)
    stepper4->moveTo(microsteps * 2, false);

  delay(6000);

  Serial.print("Stepper 1 position: ");
  if (stepper1)
    Serial.println(stepper1->getCurrentPosition());
  Serial.print("Stepper 2 position: ");
  if (stepper2)
    Serial.println(stepper2->getCurrentPosition());
  Serial.print("Stepper 3 position: ");
  if (stepper3)
    Serial.println(stepper3->getCurrentPosition());
  Serial.print("Stepper 4 position: ");
  if (stepper4)
    Serial.println(stepper4->getCurrentPosition());

  // Return all steppers to position 0
  if (stepper1)
    stepper1->moveTo(0, false);
  if (stepper2)
    stepper2->moveTo(0, false);
  if (stepper3)
    stepper3->moveTo(0, false);
  if (stepper4)
    stepper4->moveTo(0, false);

  delay(6000);

  Serial.print("Stepper 1 position: ");
  if (stepper1)
    Serial.println(stepper1->getCurrentPosition());
  Serial.print("Stepper 2 position: ");
  if (stepper2)
    Serial.println(stepper2->getCurrentPosition());
  Serial.print("Stepper 3 position: ");
  if (stepper3)
    Serial.println(stepper3->getCurrentPosition());
  Serial.print("Stepper 4 position: ");
  if (stepper4)
    Serial.println(stepper4->getCurrentPosition());
}
