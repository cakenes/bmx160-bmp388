#include <Wire.h>
#include <math.h>
#include <DFRobot_BMX160.h>
#include <DFRobot_BMP3XX.h>

DFRobot_BMX160 bmx160;
DFRobot_BMP388_I2C bmp388(&Wire, bmp388.eSDOGND);

int running = 1;
int frequency = 80;
int cumulative = 1;
float sensitivity[2] = {16384.0, 10};
sBmx160SensorData_t offset[2];
sBmx160SensorData_t imu[3];
float baro[3];

void setup() {
  Serial.begin(115200);
  Wire.begin(2, 0);

  delay(100);

  if (bmp388.begin() != 0) {
    Serial.println("bmp388 init failed");
    while (1);
  }

  if (!bmx160.begin()) {
    Serial.println("bmx160 init failed");
    while (1);
  }

  delay(100);
  calibrate(100);
}

void split(String data, String* result) {
  int index = data.indexOf(":");
  if (index != -1) {
    result[0] = data.substring(0, index);
    result[1] = data.substring(index + 1);
  }
}

void read(String* data) {
  if (data[0] == "running") { running = data[1].toInt(); }
  else if (data[0] == "frequency") { frequency = data[1].toInt(); }
  else if (data[0] == "cumulative") { cumulative = data[1].toInt(); }
  else if (data[0] == "accelSens") { sensitivity[0] = data[1].toFloat(); }
  else if (data[0] == "gyroSens") { sensitivity[1] = data[1].toFloat(); }
  else if (data[0] == "calibrate") { calibrate(data[1].toInt()); }
  else if (data[0] == "softReset") { bmx160.softReset(); }
  else if (data[0] == "setLowPower") { bmx160.setLowPower(); }
  else if (data[0] == "wakeUp") { bmx160.wakeUp(); }
}

void calibrate(int count) {
  sBmx160SensorData_t temp[2];

  float x, y, z = 0;

  for (int i = 0; i < count; i++) {
    unsigned long start = micros();
    bmx160.getAllData(&offset[0], &offset[1], NULL);
    for (int i = 0; i < 2; i++) {
      temp[i].x += offset[i].x;
      temp[i].y += offset[i].y;
      temp[i].z += offset[i].z;
    }

    unsigned long end = micros();
    unsigned long elapsed = end - start;
    unsigned long delay = (1000000 / 100);
    if (elapsed < delay) delayMicroseconds(delay - elapsed);
  }

  for (int i = 0; i < 2; i++) {
    offset[i].x = temp[i].x / count;
    offset[i].y = temp[i].y / count;
    offset[i].z = temp[i].z / count;
  }

  Serial.println("Calibration done");
}

void sensors(sBmx160SensorData_t* imu, float* baro) {
  bmx160.getAllData(&imu[0], &imu[1], &imu[2]);
  baro[0] = bmp388.readTempC();
  baro[1] = bmp388.readPressPa();
  baro[2] = bmp388.readAltitudeM();
}

void print(unsigned long time, sBmx160SensorData_t imu[], float baro[]) {
  Serial.print(time); Serial.print(":");                                            // [0] time
  Serial.print((imu[0].x - offset[0].x) / sensitivity[0], 7); Serial.print(":");    // [1] accel x
  Serial.print((imu[0].y - offset[0].y) / sensitivity[0], 7); Serial.print(":");    // [2] accel y
  Serial.print((imu[0].z - offset[0].z) / sensitivity[0], 7); Serial.print(":");    // [3] accel z
  Serial.print((imu[1].x - offset[1].x) / sensitivity[1], 7); Serial.print(":");    // [4] gyro x
  Serial.print((imu[1].y - offset[1].y) / sensitivity[1], 7); Serial.print(":");    // [5] gyro y
  Serial.print((imu[1].z - offset[1].z) / sensitivity[1], 7); Serial.print(":");    // [6] gyro z
  Serial.print(imu[2].x, 7); Serial.print(":");                                     // [7] mag x
  Serial.print(imu[2].y, 7); Serial.print(":");                                     // [8] mag y
  Serial.print(imu[2].z, 7); Serial.print(":");                                     // [9] mag z
  Serial.print(baro[0], 7); Serial.print(":");                                      // [10] temp
  Serial.print(baro[1], 7); Serial.print(":");                                      // [11] pressure
  Serial.println(baro[2], 7);                                                       // [12] altitude
}

void loop(){
  unsigned long start = micros();

  if (Serial.available() > 0) {
    String data = Serial.readString();
    String splitData[2] = {"", ""};
    split(data, splitData);
    read(splitData);
  }

  if (running == 0) return delay(1000);

  sensors(imu, baro);
  print(start, imu, baro);

  unsigned long end = micros();
  unsigned long elapsed = end - start;
  unsigned long delay = (1000000 / frequency);
  if (elapsed < delay) delayMicroseconds(delay - elapsed);
}