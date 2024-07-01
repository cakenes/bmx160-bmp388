#include <Wire.h>
#include <DFRobot_BMX160.h>
#include <DFRobot_BMP3XX.h>

DFRobot_BMX160 bmx160;
DFRobot_BMP388_I2C bmp388(&Wire, bmp388.eSDOGND);

int running = 1;
unsigned long interval = 1000000 / 200;
unsigned long previousTime = 0;

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
  else if (data[0] == "refreshRate") { interval = 1000000 / data[1].toInt(); }
  else if (data[0] == "softReset") { bmx160.softReset(); }
  else if (data[0] == "setLowPower") { bmx160.setLowPower(); }
  else if (data[0] == "wakeUp") { bmx160.wakeUp(); }
}

void loop(){
  unsigned long now = micros();
  sBmx160SensorData_t imu[3];
  float baro[3];

  if (Serial.available() > 0) {
    String data = Serial.readString();
    String splitData[2] = {"", ""};
    split(data, splitData);
    read(splitData);
  }

  if (running == 0) {
    delay(1000);
    return;
  }

  bmx160.getAllData(&imu[0], &imu[1], &imu[2]); // Magn, Gyro, Accel
  baro[0] = bmp388.readTempC();
  baro[1] = bmp388.readPressPa();
  baro[2] = bmp388.readAltitudeM();

  Serial.print(now); Serial.print(":");
  Serial.print(imu[0].x, 1); Serial.print(":"); Serial.print(imu[0].y, 1); Serial.print(":"); Serial.print(imu[0].z, 1); Serial.print(":");
  Serial.print(imu[1].x, 7); Serial.print(":"); Serial.print(imu[1].y, 7); Serial.print(":"); Serial.print(imu[1].z, 7); Serial.print(":");
  Serial.print(imu[2].x, 7); Serial.print(":"); Serial.print(imu[2].y, 7); Serial.print(":"); Serial.print(imu[2].z, 7); Serial.print(":");
  Serial.print(baro[0], 7); Serial.print(":"); Serial.print(baro[1], 7); Serial.print(":"); Serial.println(baro[2], 7);

  unsigned long elapsedTime = now - previousTime;
  if (elapsedTime >= interval) {
    delayMicroseconds(interval - (elapsedTime % interval));
    previousTime = now;
  }
}