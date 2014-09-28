#include <Servo.h>

Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

void initMotors()
{
  motor0.attach(MOTOR0);
  motor1.attach(MOTOR1);
  motor2.attach(MOTOR2);
  motor3.attach(MOTOR3);
  
  Serial.println("Writing MOTOR_ARM_START");
  motor0.write(MOTOR_ARM_START);
  Serial.println("Starting 10 seconds delay");
  delay(10000);
  Serial.println("Done");
  motor0.write(MOTOR_ZERO_LEVEL);
}

void updateMotors()
{
  motor0.write(MOTOR_ZERO_LEVEL);
  /*
  int input0Value = analogRead(A3);
  input0Value = min(input0Value,240);
  
  Serial.println(input0Value);
  
  int motor0Value = map(min(input0Value,240), 0, 128, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
  motor0.write(motor0Value);
  */
}
