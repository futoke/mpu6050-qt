#include <AccelStepper.h>
#include <MultiStepper.h>
#include <DynamicCommandParser.h>

#define SPEED 1000
#define MOVE_DONE	77
#define ZERO_ALL	88

DynamicCommandParser dcp('^', '$', ',');


AccelStepper stepper_1(AccelStepper::DRIVER, 2, 3); // STEP, DIR
AccelStepper stepper_2(AccelStepper::DRIVER, 4, 5);
AccelStepper stepper_3(AccelStepper::DRIVER, 8, 9);
AccelStepper stepper_4(AccelStepper::DRIVER, 10, 16);

MultiStepper steppers;

void moveto(char **values, int valueCount)
{
  digitalWrite(14, HIGH);
  digitalWrite(15, LOW);
  long positions[] = {0, 0, 0, 0};
  
  for (int i = 1; i < valueCount; i++) {
    positions[i-1] = (long) String(values[i]).toInt();
  }

  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  delay(1000);
  Serial.write(MOVE_DONE);
  digitalWrite(14, LOW);
  digitalWrite(15, HIGH);
}


void zero(char **values, int valueCount)
{
  stepper_1.setCurrentPosition(0);
  stepper_2.setCurrentPosition(0);
  stepper_3.setCurrentPosition(0);
  stepper_4.setCurrentPosition(0);

  //stepper_1.setMaxSpeed(SPEED);
  //stepper_2.setMaxSpeed(SPEED);
  //stepper_3.setMaxSpeed(SPEED);
  //stepper_4.setMaxSpeed(SPEED);
  Serial.write(ZERO_ALL);
}


void setup() {
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  Serial.begin(9600);

  stepper_1.setMaxSpeed(SPEED);
  stepper_2.setMaxSpeed(SPEED);
  stepper_3.setMaxSpeed(SPEED);
  stepper_4.setMaxSpeed(SPEED);

  steppers.addStepper(stepper_1);
  steppers.addStepper(stepper_2);
  steppers.addStepper(stepper_3);
  steppers.addStepper(stepper_4);

  dcp.addParser("ZERO", zero);
  dcp.addParser("MOVE", moveto);
}


void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    dcp.appendChar(c);
    // Serial.write(c);
  }
}

