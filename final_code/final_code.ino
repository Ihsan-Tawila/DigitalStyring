#include <Balboa32U4.h>
#include <LSM6.h>

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
const int16_t GEAR_RATIO = 80;
const int16_t MOTOR_SPEED_LIMIT = 300;
const int16_t ANGLE_RATE_RATIO = 140;
const int16_t ANGLE_RESPONSE = 11;
const int16_t DISTANCE_RESPONSE = 50;
const int16_t DISTANCE_DIFF_RESPONSE = -65; 
const int16_t SPEED_RESPONSE = 2150;
const uint8_t UPDATE_TIME_MS = 10;
const uint8_t CALIBRATION_ITERATIONS = 100;
const int32_t START_BALANCING_ANGLE = 45000;
const int32_t STOP_BALANCING_ANGLE = 70000;
int32_t gYZero;
int32_t angle; // millidegrees
int32_t angleRate; // degrees/s
int32_t distanceLeft;
int32_t speedLeft;
int32_t distanceRight;
int32_t speedRight;
int16_t motorSpeed;
bool isBalancingStatus = false;
bool balanceUpdateDelayedStatus;

void balance()
{
  angle = angle * 999 / 1000;
  int32_t risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;
  motorSpeed += (
    + ANGLE_RESPONSE * risingAngleOffset
    + DISTANCE_RESPONSE * (distanceLeft + distanceRight)
    + SPEED_RESPONSE * (speedLeft + speedRight)
    ) / 100 / GEAR_RATIO;
  if (motorSpeed > MOTOR_SPEED_LIMIT)
  {
    motorSpeed = MOTOR_SPEED_LIMIT;
  }
  if (motorSpeed < -MOTOR_SPEED_LIMIT)
  {
    motorSpeed = -MOTOR_SPEED_LIMIT;
  }
  int16_t distanceDiff = distanceLeft - distanceRight;
  motors.setSpeeds(
    motorSpeed + distanceDiff * DISTANCE_DIFF_RESPONSE / 100,
    motorSpeed - distanceDiff * DISTANCE_DIFF_RESPONSE / 100);
}

void lyingDown()   
{
  motorSpeed = 0;
  distanceLeft = 0;
  distanceRight = 0;
  motors.setSpeeds(0, 0);
  if (angleRate > -2 && angleRate < 2)
  {
    angle = atan2(imu.a.z, imu.a.x) * 57296;
    distanceLeft = 0;
    distanceRight = 0;
  }
}

void balanceUpdateSensors()
{
  imu.read();
//  integrateGyro();
  angleRate = (imu.g.y - gYZero) / 29;

  angle += angleRate * UPDATE_TIME_MS;

//  integrateEncoders();
  static int16_t lastCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;

}

void setup()
{
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); 
  delay(1000);
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }
  gYZero = total / CALIBRATION_ITERATIONS;
}

void loop()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();
  static uint8_t count = 0;
  if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;
  balanceUpdateSensors();  
  
  if (isBalancingStatus)
    {balance(); 
    if (abs(angle) > STOP_BALANCING_ANGLE) 
      {if (++count > 5){isBalancingStatus = false; count = 0;}} 
    else{count = 0;}}

  else{lyingDown();
    if (abs(angle) < START_BALANCING_ANGLE)
      {if (++count > 5){isBalancingStatus = true; count = 0;}}
    else{count = 0;}}}
