#include <Wire.h>
#include <DFRobot_BMX160.h>
#include <DFRobot_BMP3XX.h>

DFRobot_BMX160 bmx160;
DFRobot_BMP388_I2C bmp388(&Wire, bmp388.eSDOGND);

float calibratedAlt = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(2, 0);

  if (bmp388.begin() != 0) {
    Serial.println("bmp388 init failed");
    while (1);
  }

  if (!bmx160.begin()) {
    Serial.println("bmx160 init failed");
    while (1);
  }

  calibratedAlt = bmp388.readAltitudeM();

  delay(100);
}

void loop(){
  float temp, press, alt;
  sBmx160SensorData_t magn, gyro, accel;
  bmx160.getAllData(&magn, &gyro, &accel);
  temp = bmp388.readTempC();
  press = bmp388.readPressPa();
  alt = bmp388.readAltitudeM();

  /* Display the Barometer results */
  Serial.print("Barometer - ");
  Serial.print("Temp: "); Serial.print(temp); Serial.print(" C"); Serial.print("  ");
  Serial.print("Press: "); Serial.print(press); Serial.print(" Pa"); Serial.print("  ");
  Serial.print("Alt: "); Serial.print(alt - calibratedAlt); Serial.print(" m"); Serial.print("  "); Serial.print("calibration: "); Serial.print(calibratedAlt); Serial.print(" m"); Serial.println("  ");

  /* Display the magnetometer results */
  Serial.print("Magnetometer - ");
  Serial.print("X: "); Serial.print(magn.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(magn.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(magn.z); Serial.print("  ");
  Serial.println("uT");

  /* Display the gyroscope results */
  Serial.print("Gyroscope - ");
  Serial.print("X: "); Serial.print(gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyro.z); Serial.print("  ");
  Serial.println("g");

  /* Display the accelerometer results */
  Serial.print("Accelerometer - ");
  Serial.print("X: "); Serial.print(accel.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accel.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accel.z); Serial.print("  ");
  Serial.println("m/s^2"); Serial.println("");

  delay(1000);
}
