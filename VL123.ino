#include <Wire.h>
#include <VL53L0X.h>
// #include <SimpleKalmanFilter.h>
// SimpleKalmanFilter bo_loc(2, 2, 0.01);// Các tham số cho bộ lọc kalman
float u_km;// khai báo biến khoảng cách đã qua lọc kalman

VL53L0X sensor;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  
  sensor.startContinuous();// Chế độ đo liên tục
}

void loop()
{
  float distance =sensor.readRangeContinuousMillimeters();// đọc giá trị nhận được từ cảm biến
 distance = distance - 40;// bù khoảng cách.
  Serial.println(distance);
  Serial.print(",");
  if (sensor.timeoutOccurred())
  { Serial.print(" TIMEOUT"); }
  // u_km = bo_loc.updateEstimate(distance);// lọc kalman

  // Serial.println(u_km);
  Serial.println();
  delay(1000);
}//
