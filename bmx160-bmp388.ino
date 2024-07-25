#include <Update.h>
#include <WiFi.h>
#include <WebServer.h>
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

typedef struct {
  Sensor sensor;
  unsigned long time;
} Record;

enum class Mode {
  off,
  record,
  realtime
};

Mode mode = Mode::record;
bool run = true;
bool serve = false;
bool powermode = false;
int frequency = 100;
int count = 0;
float sensitivity[4] = { 16384.0, 16.4, 1, 0.005 };
Sensor offset;
Record record[3600];
WebServer webserver(80);

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
      if (data.at(0) == "reboot") {
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
        if (data.at(1) == "off") mode = Mode::off;
        else if (data.at(1) == "record") mode = Mode::record;
        else if (data.at(1) == "realtime") mode = Mode::realtime;
        return;
      } else if (data.at(0) == "frequency") {
        frequency = data.at(1).toInt();
        return;
      } else if (data.at(0) == "calibrate") {
        calibrate(data.at(1).toInt());
        return;
      }
      break;
    case 4:
      if (data.at(0) == "wifi") {
        server(data.at(1), data.at(2), data.at(3).toInt());
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
  Serial.println("reboot  -  Restarts device");
  Serial.println("reset  -  Resets sensors");
  Serial.println("powersave  -  Toggles low power mode");
  Serial.println("mode <mode as string>  -  Toggles sensor mode <off, record(default), realtime> ");
  Serial.println("frequency <hz as int>  -  Sets sensor reading frequency");
  Serial.println("calibrate <count as int>  -  Calibrates sensors");
  Serial.println("wifi <ssid as string> <password as string> <retry as int>  -  Connects to wifi and starts web server");
  Serial.println("sensitivity <accel as float> <gyro as float> <mag as float> <record as float>  -  Sets sensor sensitivity");
}

void calibrate(int count) {
  Sensor temp;

  for (int c = 0; c < 3; c++) {  // Itniialize to avoid overflow
    temp.imu[c].x = 0.0;
    temp.imu[c].y = 0.0;
    temp.imu[c].z = 0.0;
  }

  for (int i = 0; i < count; i++) {
    unsigned long start = micros();
    bmx160.getAllData(&offset.imu[0], &offset.imu[1], &offset.imu[2]);
    for (int a = 0; a < 3; a++) {
      temp.imu[a].x += offset.imu[a].x;
      temp.imu[a].y += offset.imu[a].y;
      temp.imu[a].z += offset.imu[a].z;
    }

    unsigned long end = micros();
    unsigned long elapsed = end - start;
    unsigned long delay = (1000000 / 100);
    if (elapsed < delay) delayMicroseconds(delay - elapsed);
  }

  Serial.println();
  Serial.println("Calibration done: ");

  for (int b = 0; b < 3; b++) {
    offset.imu[b].x = temp.imu[b].x / count;
    offset.imu[b].y = temp.imu[b].y / count;
    offset.imu[b].z = temp.imu[b].z / count;
    if (b == 2) offset.imu[b].z += 10;  // Offset mag Z by 10 to calibrate right side up

    Serial.print("X: ");
    Serial.print(offset.imu[b].x, 7);
    Serial.print(" Y: ");
    Serial.print(offset.imu[b].y, 7);
    Serial.print(" Z: ");
    Serial.println(offset.imu[b].z, 7);
  }
}

void sensors(Sensor* sensor) {
  bmx160.getAllData(&sensor->imu[0], &sensor->imu[1], &sensor->imu[2]);
  sensor->baro[0] = bmp388.readTempC();
  sensor->baro[1] = bmp388.readPressPa();
  sensor->baro[2] = bmp388.readAltitudeM();
}

void print(unsigned long time, Sensor* sensor) {
  Serial.print(time); // [0] time
  Serial.print(":");
  Serial.print((sensor->imu[0].x - offset.imu[0].x) / sensitivity[0], 7); // [1] accel x
  Serial.print(":");
  Serial.print((sensor->imu[0].y - offset.imu[0].y) / sensitivity[0], 7); // [2] accel y
  Serial.print(":");
  Serial.print((sensor->imu[0].z - offset.imu[0].z) / sensitivity[0], 7); // [3] accel z
  Serial.print(":");
  Serial.print((sensor->imu[1].x - offset.imu[1].x) / sensitivity[1], 7); // [4] gyro x
  Serial.print(":");
  Serial.print((sensor->imu[1].y - offset.imu[1].y) / sensitivity[1], 7); // [5] gyro y
  Serial.print(":");
  Serial.print((sensor->imu[1].z - offset.imu[1].z) / sensitivity[1], 7); // [6] gyro z
  Serial.print(":");
  Serial.print((sensor->imu[2].x - offset.imu[2].x) / sensitivity[2], 7); // [7] mag x
  Serial.print(":");
  Serial.print((sensor->imu[2].y - offset.imu[2].y) / sensitivity[2], 7); // [8] mag y
  Serial.print(":");
  Serial.print((sensor->imu[2].z - offset.imu[2].z) / sensitivity[2], 7); // [9] mag z
  Serial.print(":");
  Serial.print(sensor->baro[0], 7); // [10] temp
  Serial.print(":");
  Serial.print(sensor->baro[1], 7); // [11] pressure
  Serial.print(":");
  Serial.println(sensor->baro[2], 7); // [12] altitude
}

void wifi(String ssid, String password, int retry) {
  WiFi.begin(ssid, password);
  Serial.println();

  for (int i = 0; i < retry; i++) {
    if (WiFi.status() == WL_CONNECTED) break;
    Serial.println();
    Serial.print("Connecting to: ");
    Serial.println(ssid);
    delay(1000);
  }

  Serial.println();
  Serial.print("Connected to: ");
  Serial.println(ssid);
}

void server(String ssid, String password, int retry) {
  if (WiFi.status() != WL_CONNECTED) wifi(ssid, password, retry);

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
  serve = true;

  Serial.println();
  Serial.print("Serving at: ");
  Serial.println(WiFi.localIP());
}



void loop() {
  Sensor temp;
  unsigned long start = micros();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    std::vector<String> parts;
    split(input, parts);
    command(parts);
  }

  switch (mode) {
    case Mode::off:
      if (serve) webserver.handleClient();
      break;
    case Mode::record:
      {
        sensors(&temp);
        if (((fabs(temp.imu[0].x - offset.imu[0].x) / sensitivity[0]) > sensitivity[3]) || 
            ((fabs(temp.imu[0].y - offset.imu[0].y) / sensitivity[0]) > sensitivity[3]) || 
            ((fabs(temp.imu[0].z - offset.imu[0].z) / sensitivity[0]) > sensitivity[3])) {
          if (count == 3600) count = 0;
          record[count].sensor = temp;
          record[count].time = start;
          count++;
        } else {
          if (count >= 50) {
            Serial.println();
            Serial.println("Recorded data: ");
            for (int i = 0; i < count; i++) {
              print(record[i].time, &record[i].sensor);
              record[i] = Record();
            }
          }
          count = 0;
        }
        break;
      }
    case Mode::realtime:
    default:
      sensors(&temp);
      print(start, &temp);
      break;
  }

  if (mode != Mode::off) {
    unsigned long end = micros();
    unsigned long elapsed = end - start;
    unsigned long delay = (1000000 / frequency);
    if (elapsed < delay) delayMicroseconds(delay - elapsed);
  }
}