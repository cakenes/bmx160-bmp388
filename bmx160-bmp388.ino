#include <Update.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include <DFRobot_BMX160.h>
#include <DFRobot_BMP3XX.h>

DFRobot_BMX160 bmx160;
DFRobot_BMP388_I2C bmp388(&Wire, bmp388.eSDOGND);

float sensor[12];
float offset[9];

typedef struct {
  unsigned long time;
  float sensor[12];
} Record;

enum class Mode {
  off,
  record,
  debug
};

const int STOP_THRESHOLD_LIMIT = 10;
const int RECORD_BUFFER_SIZE = 3600;
const int MIN_RECORD_SIZE = 50;

WebServer webserver(80);
Mode mode = Mode::record;
Record record[RECORD_BUFFER_SIZE];
int recordIndex = 0;
int stopTreshold = 0;
bool powermode = false;
int frequency[3] = { 100, 100, 10 };                 // [0] = current, [1] = target, [2] = divider
float sensitivity[4] = { 16384.0, 16.4, 1, 0.005 };  // [0] = accel, [1] = gyro, [2] = mag, [3] = record

const char* serverIndex = R"rawliteral(
  <style>
    body { font-family: Arial, sans-serif; display: flex; justify-content: center; align-items: center; height: 100vh; flex-direction: column; margin: 0px; }
    .container { display: flex; flex-direction: column; align-items: center; width: 50%; }
    form { display: flex; background-color: #f2f2f2; padding: 20px; border-radius: 5px; border: 1px solid rgba(0,0,0,0.1); width: 100%; }
    input[type="file"], input[type="submit"] { padding: 10px 20px; border: none; width: calc(100% - 40px); }
    input[type="submit"] { background-color: #4CAF50; color: white; border-radius: 4px; cursor: pointer; }
    input[type="submit"]:hover { background-color: #45a049; }
    #progress-container { width: calc(100% + 40px); background-color: #ddd; border-radius: 5px; border: 1px solid rgba(0,0,0,0.1); text-align: center; }
    #progress-bar { width: 0%; height: 20px; background-color: #4CAF50; margin-top: -18px; border-radius: 5px; }
  </style>
  <script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>
  <div class="container"> <!-- Wrapper to control width -->
    <form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>
      <input type='file' name='update'>
      <input type='submit' value='Update'>
    </form>
    <div id='progress-container'>
      <span id="progress-text">0%</span>
      <div id='progress-bar'></div>
    </div>
  </div>
  <script>
    $('#upload_form').submit(function(e){
      e.preventDefault();
      $.ajax({
        url: '/update',
        type: 'POST',
        data: new FormData(this),
        contentType: false,
        processData: false,
        xhr: () => {
          var xhr = new window.XMLHttpRequest();
          xhr.upload.addEventListener('progress', evt => {
            if (evt.lengthComputable) {
              var percentComplete = Math.round((evt.loaded / evt.total) * 100);
              $('#progress-bar').width(percentComplete + '%');
              $('#progress-text').text(percentComplete + '%');
            }
          });
          return xhr;
        },
        success: () => console.log('success!'),
        error: () => console.log('error!')
      });
    });
  </script>
)rawliteral";

void setup() {
  delay(1000);
  Serial.begin(921600);
  Wire.begin(5, 4);

  delay(100);
  if (bmp388.begin() != 0) Serial.println("bmp388 Initialization failed");
  if (!bmx160.begin()) Serial.println("bmx160 Initialization failed");

  delay(100);
  calibrate(100);
  calibrate(100);
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

void powersave() {
  if (powermode) bmx160.wakeUp();
  else bmx160.setLowPower();
  powermode = !powermode;
}

void command(std::vector<String>& data) {
  switch (data.size()) {
    case 1:
      if (data.at(0) == "restart") {
        ESP.restart();
        return;
      } else if (data.at(0) == "reset") {
        bmx160.softReset();
        return;
      } else if (data.at(0) == "powersave") {
        powersave();
        return;
      }
      break;
      
    case 2:
      if (data.at(0) == "mode") {
        if (data.at(1) == "off") {
          mode = Mode::off;
          return;
        } else if (data.at(1) == "record") mode = Mode::record;
        else if (data.at(1) == "debug") mode = Mode::debug;
        webserver.stop();
        WiFi.disconnect();
        return;
      } else if (data.at(0) == "frequency") {
        frequency[1] = data.at(1).toInt();
        return;
      } else if (data.at(0) == "calibrate") {
        calibrate(data.at(1).toInt());
        return;
      }
      break;

    case 4:
      if (data.at(0) == "wifi") {
        wifi(data.at(1), data.at(2), data.at(3).toInt());
        return;
      }
      break;

    case 5:
      if (data.at(0) == "sensitivity") {
        sensitivity[0] = data.at(1).toFloat();
        sensitivity[1] = data.at(2).toFloat();
        sensitivity[2] = data.at(3).toFloat();
        return;
      }
      break;
  }

  Serial.println();
  Serial.println("Command not found or invalid arguments, try:");
  Serial.println("restart  -  Restarts device");
  Serial.println("reset  -  Resets sensors");
  Serial.println("powersave  -  Toggles low power mode");
  Serial.println("mode <mode as string>  -  Toggles sensor mode (off, record, debug) - default: record");
  Serial.println("frequency <hz as int>  -  Sets sensor reading frequency - default: 100hz");
  Serial.println("calibrate <count as int>  -  Calibrates sensors");
  Serial.println("wifi <ssid as string> <password as string> <retry as int>  -  Connects to wifi and starts web server");
  Serial.println("sensitivity <accel as float> <gyro as float> <mag as float> <record as float>  -  Sets sensor sensitivity - default: 16384.0 16.4 1 0.005");
}

void calibrate(int count) {
  sBmx160SensorData_t imu[3][count];
  sBmx160SensorData_t imuOffset[3];

  for (size_t i = 0; i < 9; i++) {  // Reset to avoid overflow
    offset[i] = 0.0;
  }

  for (size_t i = 0; i < count; i++) {
    unsigned long start = micros();
    bmx160.getAllData(&imu[0][i], &imu[1][i], &imu[2][i]);

    for (size_t a = 0; a < 3; a++) {
      offset[a * 3] += imu[a][i].x;
      offset[a * 3 + 1] += imu[a][i].y;
      offset[a * 3 + 2] += imu[a][i].z;
    }

    unsigned long end = micros();
    unsigned long elapsed = end - start;
    unsigned long delay = (1000000 / 100);
    if (elapsed < delay) delayMicroseconds(delay - elapsed);
  }

  for (size_t b = 0; b < 3; b++) {
    offset[b * 3] /= count;
    offset[b * 3 + 1] /= count;
    offset[b * 3 + 2] /= count;
    if (b == 2) offset[b * 3 + 2] += 10;  // Offset mag Z by 10 to calibrate right side up
  }

  if (mode != Mode::off) {
    Serial.println();
    Serial.println("Calibration done:");
    for (size_t i = 0; i < 9; i++) {
      Serial.print(offset[i], 7);
      if (i == 9 - 1) Serial.println();
      else if ((i + 1) % 3 == 0) Serial.print(" : ");
      else Serial.print(",");
    }
  }
}

void getSensors(float* sensor, const float* offset, const float* sensitivity) {
  sBmx160SensorData_t imu[3];
  bmx160.getAllData(&imu[0], &imu[1], &imu[2]);

  for (size_t i = 0; i < 3; ++i) {
    sensor[i * 3] = (imu[i].x - offset[i * 3]) / sensitivity[i];
    sensor[i * 3 + 1] = (imu[i].y - offset[i * 3 + 1]) / sensitivity[i];
    sensor[i * 3 + 2] = (imu[i].z - offset[i * 3 + 2]) / sensitivity[i];
  }

  sensor[9] = bmp388.readTempC();
  sensor[10] = bmp388.readPressPa();
  sensor[11] = bmp388.readAltitudeM();
}

void print(unsigned long time, float* sensor, size_t sensorSize) {
  Serial.print(time);
  char buffer[15];
  for (size_t i = 0; i < sensorSize; i++) {
    Serial.print(":");
    sprintf(buffer, "%12.7f", sensor[i]);
    Serial.print(buffer);
  }
  Serial.println();
}

void wifi(String ssid, String password, int retry) {
  WiFi.begin(ssid, password);
  Serial.println();

  for (size_t i = 0; i < retry; i++) {
    if (WiFi.status() == WL_CONNECTED) break;
    Serial.println();
    Serial.print("Connecting to: ");
    Serial.println(ssid);
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.print("Failed to connect to: ");
    Serial.println(ssid);
    return;
  }

  Serial.println();
  Serial.print("Connected to: ");
  Serial.println(ssid);

  webserver.on("/", HTTP_GET, []() {
    webserver.sendHeader("Connection", "close");
    webserver.send(200, "text/html", serverIndex);
  });

  webserver.on(
    "/update", HTTP_POST, []() {
      webserver.sendHeader("Connection", "close");
      webserver.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    },
    []() {
      HTTPUpload& upload = webserver.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.print("Updating with: ");
        Serial.println(upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
          Serial.println();
          Serial.print("Update success with: ");
          Serial.print(upload.filename.c_str());
          Serial.print(", size: ");
          Serial.print(upload.totalSize);
          Serial.println(" bytes");
          Serial.println("Rebooting...");
        } else {
          Update.printError(Serial);
        }
      }
    });

  webserver.begin();
  Serial.println();
  Serial.print("Serving at: ");
  Serial.println(WiFi.localIP());
}

bool shouldRecord(const float* sensor, const float* sensitivity) {
  return (fabs(sensor[0]) > sensitivity[3] || fabs(sensor[1]) > sensitivity[3] || fabs(sensor[2]) > sensitivity[3]);
}

void loop() {
  float imu[12];
  unsigned long start = micros();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    std::vector<String> parts;
    split(input, parts);
    command(parts);
  }

  switch (mode) {
    case Mode::off:
      if (WiFi.status() == WL_CONNECTED) webserver.handleClient();
      break;

    case Mode::record:
      getSensors(imu, offset, sensitivity);
      if (recordIndex == 0) frequency[0] = frequency[1];
      if (shouldRecord(imu, sensitivity)) {
        stopTreshold = 0;
        if (recordIndex == RECORD_BUFFER_SIZE) recordIndex = 0;
        record[recordIndex].time = start;
        memcpy(record[recordIndex].sensor, imu, sizeof(imu));
        recordIndex++;
      } else {
        stopTreshold++;
        if (stopTreshold >= STOP_THRESHOLD_LIMIT) {
          if (recordIndex >= MIN_RECORD_SIZE) {
            Serial.println();
            Serial.println("Recorded data: ");
            for (size_t i = 0; i < recordIndex; ++i) {
              print(record[i].time, &record[i].sensor[0], 12);
              record[i] = Record();
            }
          }
          recordIndex = 0;
          frequency[0] = frequency[1] / frequency[2];
        }
      }
      break;

    case Mode::debug:
    default:
      getSensors(sensor, offset, sensitivity);
      print(start, sensor, 12);
      break;
  }

  if (mode != Mode::off) {
    unsigned long end = micros();
    unsigned long elapsed = end - start;
    unsigned long delay = (1000000 / frequency[1]);
    if (elapsed < delay) delayMicroseconds(delay - elapsed);
  }
}
