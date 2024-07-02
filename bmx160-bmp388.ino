#include <Wire.h>
#include <math.h>
#include <DFRobot_BMX160.h>
#include <DFRobot_BMP3XX.h>

DFRobot_BMX160 bmx160;
DFRobot_BMP388_I2C bmp388(&Wire, bmp388.eSDOGND);

typedef struct {
    sBmx160SensorData_t imu[3];
    float baro[3];
} Sensor;

bool run = true;
int frequency = 80;
float sensitivity[3] = {16384.0, 16.4, 1};
Sensor offset;

void setup() {
  Serial.begin(115200);
  Wire.begin(2, 0);

  delay(100);

  if (bmp388.begin() != 0) {
    Serial.println("bmp388 Initialization failed");
    while (1);
  }

  if (!bmx160.begin()) {
    Serial.println("bmx160 Initialization failed");
    while (1);
  }

  delay(100);
  calibrate(100);

  bmx160.setLowPower();
}

void split(const String& data, std::vector<String>& result) {
  int start = 0;
  int end = data.indexOf(" ");
  while (end != -1) {
    result.push_back(data.substring(start, end));
    start = end + 1;
    end = data.indexOf(" ", start);
  }
  result.push_back(data.substring(start));
}

void read(std::vector<String>& data) {
  if (data.at(0) == "run") { run = !run; }
  else if (data.at(0) == "reset") { bmx160.softReset(); }
  else if (data.at(0) == "powersave") { bmx160.setLowPower(); }
  else if (data.at(0) == "wake") { bmx160.wakeUp(); }
  else if (data.size() <= 1) { Serial.println("Invalid number parameters, 1 required"); }
  else if (data.at(0) == "frequency") { frequency = data.at(1).toInt(); }
  else if (data.at(0) == "calibrate") { calibrate(data.at(1).toInt()); }
  else if (data.size() <= 3) { Serial.println("Invalid number of parameters, 3 required"); }
  else if (data.at(0) == "sensitivity") { sensitivity[0] = data.at(1).toFloat(); sensitivity[1] = data.at(2).toFloat(); sensitivity[2] = data.at(3).toFloat();  }
}

void calibrate(int count) { // Counts over 100 can cause overflow
  Sensor temp;

  for (int i = 0; i < count; i++) {
    unsigned long start = micros();
    bmx160.getAllData(&offset.imu[0], &offset.imu[1], &offset.imu[2]);
    for (int i = 0; i < 3; i++) {
      temp.imu[i].x += offset.imu[i].x;
      temp.imu[i].y += offset.imu[i].y;
      temp.imu[i].z += offset.imu[i].z;
    }

    unsigned long end = micros();
    unsigned long elapsed = end - start;
    unsigned long delay = (1000000 / 80);
    if (elapsed < delay) delayMicroseconds(delay - elapsed);
  }

  Serial.println("Calibration done: ");

  for (int i = 0; i < 3; i++) {
    offset.imu[i].x = temp.imu[i].x / count;
    offset.imu[i].y = temp.imu[i].y / count;
    offset.imu[i].z = temp.imu[i].z / count;
    if (i == 2) offset.imu[i].z += 10; // Offset mag Z by 10 to calibrate right side up

    Serial.print("X: "); Serial.print(offset.imu[i].x, 7); Serial.print("Y: "); Serial.print(offset.imu[i].y, 7); Serial.print("Z: "); Serial.println(offset.imu[i].z, 7); 
  }
}

void sensors(Sensor* sensor) {
  bmx160.getAllData(&sensor->imu[0], &sensor->imu[1], &sensor->imu[2]);
  sensor->baro[0] = bmp388.readTempC();
  sensor->baro[1] = bmp388.readPressPa();
  sensor->baro[2] = bmp388.readAltitudeM();
}

void print(unsigned long time, Sensor* sensor) {
  Serial.print(time); Serial.print(":");                                                        // [0] time
  Serial.print((sensor->imu[0].x - offset.imu[0].x) / sensitivity[0], 7); Serial.print(":");    // [1] accel x
  Serial.print((sensor->imu[0].y - offset.imu[0].y) / sensitivity[0], 7); Serial.print(":");    // [2] accel y
  Serial.print((sensor->imu[0].z - offset.imu[0].z) / sensitivity[0], 7); Serial.print(":");    // [3] accel z
  Serial.print((sensor->imu[1].x - offset.imu[1].x) / sensitivity[1], 7); Serial.print(":");    // [4] gyro x
  Serial.print((sensor->imu[1].y - offset.imu[1].y) / sensitivity[1], 7); Serial.print(":");    // [5] gyro y
  Serial.print((sensor->imu[1].z - offset.imu[1].z) / sensitivity[1], 7); Serial.print(":");    // [6] gyro z
  Serial.print((sensor->imu[2].x - offset.imu[2].x) / sensitivity[2], 7); Serial.print(":");    // [7] mag x
  Serial.print((sensor->imu[2].y - offset.imu[2].y) / sensitivity[2], 7); Serial.print(":");    // [8] mag y
  Serial.print((sensor->imu[2].z - offset.imu[2].z) / sensitivity[2], 7); Serial.print(":");    // [9] mag z
  Serial.print(sensor->baro[0], 7); Serial.print(":");                                          // [10] temp
  Serial.print(sensor->baro[1], 7); Serial.print(":");                                          // [11] pressure
  Serial.println(sensor->baro[2], 7);                                                           // [12] altitude
}

void loop(){
  Sensor temp;
  unsigned long start = micros();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    std::vector<String> parts;
    split(input, parts);
    read(parts);
  }

  if (run) {
    sensors(&temp);
    print(start, &temp);
  }

  unsigned long end = micros();
  unsigned long elapsed = end - start;
  unsigned long delay = (1000000 / frequency);
  if (elapsed < delay) delayMicroseconds(delay - elapsed);
}