#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <time.h>
#include <math.h>
#include <string.h>

#ifndef ENC_TYPE_NONE
#define ENC_TYPE_NONE 7
#endif

// ---------------- USER SETTINGS ----------------
#define WIFI_SSID    "WIFI_SSID"
#define WIFI_PASS    "WIFI_PSWD"
#define DEVICE_NAME  "miniconsole"
#define OPENWEATHER_API_KEY  "api_key"
#define OPENWEATHER_CITY_ID  "city_id"

uint16_t joystick_rotation = 90;
uint8_t joystick_speed = 1; // Increased for faster cursor
uint8_t speaker_volume = 0;
uint8_t target_fps = 60; // Increased for smoother games
// -----------------------------------------------

#define CALIB_SAMPLES 500
#define CALIB_TIME 5000

// Hardware pins
#define TFT_RST   D1
#define TFT_CS    D8
#define TFT_DC    D2
#define TFT_SCLK  D5
#define TFT_MOSI  D7
#define MUX_S0    D0
#define MUX_Z     A0
#define JOY_SW    D3
#define MPU_SDA   D4
#define MPU_SCL   D6
#define SPEAKER_PIN 1

// Display - 15MHz SPI
const uint16_t DISP_W = 160;
const uint16_t DISP_H = 128;
const uint8_t STATUS_BAR_H = 10;

// Cursor - smaller for better visibility
const int CURSOR_SIZE = 3;
#define C_CURSOR    0xF800

// Colors
#define C_BG        0x0000
#define C_FG        0xFFFF
#define C_ACCENT    0x07FF
#define C_WARN      0xFBE0
#define C_ERROR     0xF800
#define C_SUCCESS   0x07E0
#define C_PANEL     0x2104
#define C_SELECTED  0xFFE0

// MPU6050
#define MPU_ADDR 0x68
#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xAB

struct CalibrationData {
  uint8_t magic;
  float gx_offset, gy_offset, gz_offset;
  float ax_offset, ay_offset, az_offset;
  float pitch_ref, roll_ref, yaw_ref;
  uint8_t reserved[8];
};

// Globals
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
ESP8266WebServer server(80);

enum AppState { APP_HOME, APP_LAUNCHER, APP_CALCULATOR, APP_COMPASS, APP_ACCEL, APP_CLOCK, APP_GAMES, APP_TICTACTOE, APP_PONG, APP_SPACESHOOTER, APP_SETTINGS };
AppState currentApp = APP_HOME;
AppState lastApp = APP_HOME;
bool needsFullRedraw = true;

// Cursor with buffer for smooth movement
int cursorX = DISP_W/2, cursorY = DISP_H/2;
int lastCursorX = -1, lastCursorY = -1;
uint16_t cursorBuffer[CURSOR_SIZE * CURSOR_SIZE]; // Store pixels under cursor

// Joystick
int rawX = 512, rawY = 512;
int centerX = 512, centerY = 512;
bool btnPressed = false, lastBtnPressed = false;

// MPU state - 3-axis
float pitch = 0, roll = 0, yaw = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float pitch_ref = 0.0f, roll_ref = 0.0f, yaw_ref = 0.0f;
bool calibrated = false;
unsigned long timerMicros = 0;
float dt = 0.01f;

// Sensor filtering for smooth display
float pitch_filtered = 0, roll_filtered = 0, yaw_filtered = 0;
const float FILTER_ALPHA = 0.92f;

// Weather/time
char weatherMain[24] = "N/A";
char weatherTemp[16] = "--°C";
unsigned long weatherLastFetch = 0;
const unsigned long WEATHER_REFRESH_MS = 10 * 60 * 1000UL;
bool ntpSynced = false;
time_t currentTime = 0;

// WiFi scan
struct WiFiNet { String ssid; int rssi; bool open; };
WiFiNet wifiNets[16];
int wifiNetCount = 0;
int wifiScanDone = 0;

// Stats
uint32_t freeHeap = 0;
float fps = 0;

// App content
struct Icon { int x,y,w,h; const char* name; AppState app; };
Icon launcherIcons[6];
char calcDisplay[20] = "0";
double calcA = 0;
char calcOp = 0;

// TicTacToe
uint8_t tttBoard[9];
int tttTurn = 1;
bool tttGameOver = false;

// Pong - improved speed
int pongPaddle1Y = 40, pongPaddle2Y = 40;
float pongBallX = 80.0f, pongBallY = 64.0f;
float pongBallVX = 2.2f, pongBallVY = 1.8f;
int pongScore1 = 0, pongScore2 = 0;
const int PADDLE_H = 20, PADDLE_W = 3;
bool pongGameActive = false;
int lastBallX = 80, lastBallY = 64;

// Space Shooter - improved
int shipX = 80, shipY = 100;
int lastShipX = 80;
struct Bullet { float x, y; bool active; };
Bullet bullets[8];
struct Enemy { float x, y; bool active; int type; };
Enemy enemies[10];
int shooterScore = 0;
unsigned long lastEnemySpawn = 0;
unsigned long lastBulletTime = 0;
bool shooterGameActive = false;

// Web beep & message
bool webBeepActive = false;
String webMessage = "";
unsigned long messageTime = 0;
bool messageActive = false;

// Speaker
bool speakerRunning = false;
uint32_t speakerHalfPeriodUs = 0;
uint32_t speakerNextToggleUs = 0;
uint32_t speakerStopMs = 0;

// Home clock hands
int prev_hx=-1, prev_hy=-1, prev_mx=-1, prev_my=-1, prev_sx=-1, prev_sy=-1;

// Forward declarations
int readMux(uint8_t ch);
bool readRaw(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz);
void calibrateSensors();
void saveCalibration();
bool loadCalibration();
void updateMPU6050();
void fetchWeather();
void syncNTP();
void scanWiFi();
void handleRoot();
void handleAPI();
void handleBeep();
void handleMessage();
void drawStatusBar();
void drawHome();
void updateHomeClockHands();
void drawLauncher();
void drawCalculator();
void drawCompass();
void updateCompass();
void drawAccel();
void updateAccel();
void drawClock();
void drawGames();
void drawTicTacToe();
void drawPong();
void updatePong();
void drawSpaceShooter();
void updateSpaceShooter();
void drawSettings();
void drawWebMessage();
void redrawScreen();
void saveCursorBackground();
void restoreCursorBackground();
void drawCursor(int x, int y);

static inline int iMax(int a,int b){ return (a>b)?a:b; }
static inline int iMin(int a,int b){ return (a<b)?a:b; }

// ---------------- SOUND ----------------
void stopSpeaker() { speakerRunning=false; digitalWrite(SPEAKER_PIN, LOW); }
void startTone(uint16_t freq, uint32_t dur_ms, uint8_t volume) {
  if (freq==0 || volume==0) { stopSpeaker(); return; }
  float volScale = 0.2f + (constrain((int)volume,0,100)/100.0f)*0.8f;
  uint32_t effFreq = max(120,(int)(freq * volScale));
  speakerHalfPeriodUs = max((uint32_t)10, (uint32_t)((1000000UL/effFreq)/2));
  speakerNextToggleUs = micros() + speakerHalfPeriodUs;
  speakerStopMs = millis() + dur_ms;
  speakerRunning = true;
  digitalWrite(SPEAKER_PIN, HIGH);
}
void updateSpeaker() {
  if (!speakerRunning) return;
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - speakerNextToggleUs) >= 0) {
    digitalWrite(SPEAKER_PIN, !digitalRead(SPEAKER_PIN));
    speakerNextToggleUs += speakerHalfPeriodUs;
    if ((int32_t)(micros() - speakerNextToggleUs) > 0) speakerNextToggleUs = micros() + speakerHalfPeriodUs;
  }
  if ((int32_t)(millis() - speakerStopMs) >= 0) {
    stopSpeaker();
    if (webBeepActive) webBeepActive = false;
  }
}

void playClick() {
  if (speaker_volume == 0) return;
  startTone(0, 0, speaker_volume);
}
void playNavigate() {
  if (speaker_volume == 0) return;
  startTone(0, 0, speaker_volume);
}
void playCalibrate() {
  if (speaker_volume == 0) return;
  startTone(0, 0, speaker_volume);
}
void playGameSound(uint16_t freq, uint32_t dur) {
  if (speaker_volume == 0) return;
  startTone(freq, dur, speaker_volume);
}

// ---------------- HARDWARE ----------------
int readMux(uint8_t ch) {
  digitalWrite(MUX_S0, ch & 1);
  delayMicroseconds(3);
  return analogRead(MUX_Z);
}

// ---------------- MPU IMPROVED ----------------
bool readRaw(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz) {
  Wire.beginTransmission((uint8_t)MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, (bool)true);

  unsigned long start = micros();
  while (Wire.available() < 14) {
    if ((unsigned long)(micros() - start) > 2000UL) return false;
    yield();
  }

  int16_t hi, lo;
  hi = Wire.read(); lo = Wire.read(); ax = (hi << 8) | (lo & 0xFF);
  hi = Wire.read(); lo = Wire.read(); ay = (hi << 8) | (lo & 0xFF);
  hi = Wire.read(); lo = Wire.read(); az = (hi << 8) | (lo & 0xFF);
  Wire.read(); Wire.read();
  hi = Wire.read(); lo = Wire.read(); gx = (hi << 8) | (lo & 0xFF);
  hi = Wire.read(); lo = Wire.read(); gy = (hi << 8) | (lo & 0xFF);
  hi = Wire.read(); lo = Wire.read(); gz = (hi << 8) | (lo & 0xFF);
  return true;
}

void saveCalibration() {
  CalibrationData d;
  d.magic = EEPROM_MAGIC;
  d.gx_offset = gx_offset; d.gy_offset = gy_offset; d.gz_offset = gz_offset;
  d.ax_offset = ax_offset; d.ay_offset = ay_offset; d.az_offset = az_offset;
  d.pitch_ref = pitch_ref; d.roll_ref = roll_ref; d.yaw_ref = yaw_ref;
  EEPROM.put(0, d);
  EEPROM.commit();
}

bool loadCalibration() {
  CalibrationData d; EEPROM.get(0, d);
  if (d.magic == EEPROM_MAGIC) {
    gx_offset = d.gx_offset; gy_offset = d.gy_offset; gz_offset = d.gz_offset;
    ax_offset = d.ax_offset; ay_offset = d.ay_offset; az_offset = d.az_offset;
    pitch_ref = d.pitch_ref; roll_ref = d.roll_ref; yaw_ref = d.yaw_ref;
    return true;
  }
  return false;
}

void calibrateSensors() {
  tft.fillRect(0, STATUS_BAR_H, DISP_W, DISP_H-STATUS_BAR_H, C_BG);
  tft.setTextColor(C_FG); tft.setTextSize(1);
  tft.setCursor(10, STATUS_BAR_H + 20); tft.print("Calibrating...");
  tft.setCursor(10, STATUS_BAR_H + 32); tft.print("Keep flat & still");

  long gx_sum=0, gy_sum=0, gz_sum=0;
  long ax_sum=0, ay_sum=0, az_sum=0;
  int collected = 0;
  unsigned long startTime = millis();
  
  while (collected < CALIB_SAMPLES && millis() - startTime < CALIB_TIME + 2000UL) {
    int16_t axr, ayr, azr, gxr, gyr, gzr;
    if (readRaw(axr, ayr, azr, gxr, gyr, gzr)) {
      gx_sum += gxr; gy_sum += gyr; gz_sum += gzr;
      ax_sum += axr; ay_sum += ayr; az_sum += azr;
      collected++;
      if ((collected % 20) == 0) {
        int p = map(collected, 0, CALIB_SAMPLES, 0, DISP_W - 40);
        tft.fillRect(20, STATUS_BAR_H + 65, p, 8, C_ACCENT);
      }
    } else delay(2);
    yield();
  }

  if (collected == 0) {
    tft.setCursor(10, STATUS_BAR_H + 100); tft.setTextColor(C_ERROR); 
    tft.print("Calibration failed");
    delay(1500);
    needsFullRedraw = true;
    return;
  }

  gx_offset = gx_sum / (float)collected;
  gy_offset = gy_sum / (float)collected;
  gz_offset = gz_sum / (float)collected;
  ax_offset = ax_sum / (float)collected;
  ay_offset = ay_sum / (float)collected;
  az_offset = (az_sum / (float)collected) - 16384.0f;

  float ax_avg = (ax_sum / (float)collected) - ax_offset;
  float ay_avg = (ay_sum / (float)collected) - ay_offset;
  float az_avg = (az_sum / (float)collected) - az_offset;
  pitch_ref = atan2(ay_avg/16384.0f, az_avg/16384.0f) * 180.0f / PI;
  roll_ref = atan2(-ax_avg/16384.0f, az_avg/16384.0f) * 180.0f / PI;
  yaw_ref = yaw;

  saveCalibration();
  calibrated = true;

  tft.setCursor(10, STATUS_BAR_H + 100); tft.setTextColor(C_SUCCESS); 
  tft.print("Calibrated!");
  playCalibrate();
  delay(1000);
  needsFullRedraw = true;
}

void updateMPU6050() {
  unsigned long now = micros();
  if (timerMicros == 0) timerMicros = now;
  dt = (now - timerMicros) / 1000000.0f;
  if (dt < 0.002f) return;
  timerMicros = now;

  int16_t axr, ayr, azr, gxr, gyr, gzr;
  if (!readRaw(axr, ayr, azr, gxr, gyr, gzr)) return;

  float ax_g = (axr - ax_offset) / 16384.0f;
  float ay_g = (ayr - ay_offset) / 16384.0f;
  float az_g = (azr - az_offset) / 16384.0f;

  float gx_dps = (gxr - gx_offset) / 131.0f;
  float gy_dps = (gyr - gy_offset) / 131.0f;
  float gz_dps = (gzr - gz_offset) / 131.0f;

  float pitch_acc = atan2(ay_g, az_g) * 180.0f / PI;
  float roll_acc = atan2(-ax_g, az_g) * 180.0f / PI;

  float alpha = 0.98f;
  pitch = alpha * (pitch + gx_dps * dt) + (1.0f - alpha) * pitch_acc;
  roll = alpha * (roll + gy_dps * dt) + (1.0f - alpha) * roll_acc;
  yaw += gz_dps * dt;

  pitch_filtered = FILTER_ALPHA * pitch_filtered + (1.0f - FILTER_ALPHA) * pitch;
  roll_filtered = FILTER_ALPHA * roll_filtered + (1.0f - FILTER_ALPHA) * roll;
  yaw_filtered = FILTER_ALPHA * yaw_filtered + (1.0f - FILTER_ALPHA) * yaw;
}

// ---------------- WEATHER & NTP ----------------
void fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClient client; HTTPClient http;
  String url = "http://api.openweathermap.org/data/2.5/weather?id=" + String(OPENWEATHER_CITY_ID) + "&units=metric&appid=" + String(OPENWEATHER_API_KEY);
  http.begin(client, url);
  http.setTimeout(5000);
  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    int tempIdx = payload.indexOf("\"temp\":");
    if (tempIdx > 0) {
      int tempStart = tempIdx + 7;
      int tempEnd = payload.indexOf(",", tempStart);
      if (tempEnd > tempStart) {
        String tempStr = payload.substring(tempStart, tempEnd);
        int t = (int)tempStr.toFloat();
        snprintf(weatherTemp, sizeof(weatherTemp), "%d C", t);
      }
    }
    int mainIdx = payload.indexOf("\"main\":\"");
    if (mainIdx > 0) {
      int mainStart = mainIdx + 8;
      int mainEnd = payload.indexOf("\"", mainStart);
      if (mainEnd > mainStart) {
        String mainStr = payload.substring(mainStart, mainEnd);
        strncpy(weatherMain, mainStr.c_str(), sizeof(weatherMain)-1);
        weatherMain[sizeof(weatherMain)-1] = '\0';
      }
    }
    weatherLastFetch = millis();
  }
  http.end();
}

void syncNTP() {
  if (ntpSynced || WiFi.status() != WL_CONNECTED) return;
  configTime(19800, 0, "pool.ntp.org", "time.nist.gov");
  unsigned long start = millis();
  while (millis() - start < 10000) {
    currentTime = time(nullptr);
    if (currentTime > 1600000000) { ntpSynced = true; break; }
    delay(200);
  }
}

// ---------------- WEB SERVER ----------------
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<style>
body{font-family:Arial;background:#111;color:#fff;margin:0;padding:20px}
.card{background:#222;border-radius:8px;padding:15px;margin:10px 0}
h1{color:#07FF}
.btn{background:#07FF;color:#000;border:none;padding:12px 24px;font-size:16px;border-radius:5px;cursor:pointer;margin:5px}
.btn:hover{background:#05DD}
.btn-warn{background:#FBE0}
.value{color:#07FF;font-size:20px}
#msgBox{width:100%;padding:10px;font-size:14px;margin-top:10px;background:#333;color:#fff;border:1px solid #07FF;border-radius:5px}
</style>
</head>
<body>
<h1>MiniConsole Monitor</h1>
<div class='card'>
<h2>Sensors (Live)</h2>
<p>Pitch: <span class='value' id='pitch'>--</span>°</p>
<p>Roll: <span class='value' id='roll'>--</span>°</p>
<p>Yaw: <span class='value' id='yaw'>--</span>°</p>
</div>
<div class='card'>
<h2>Network</h2>
<p>SSID: <span class='value' id='ssid'>--</span></p>
<p>IP: <span class='value' id='ip'>--</span></p>
<p>Signal: <span class='value' id='rssi'>--</span> dBm</p>
</div>
<div class='card'>
<h2>Controls</h2>
<button class='btn btn-warn' onclick='triggerBeep()'>Trigger Beep on Device</button>
<h3>Send Message to Device</h3>
<input type='text' id='msgBox' placeholder='Type message...' maxlength='40'>
<button class='btn' onclick='sendMessage()'>Send to Device</button>
<p id='status'></p>
</div>
<script>
function triggerBeep(){
fetch('/beep').then(r=>r.text()).then(d=>{
document.getElementById('status').innerHTML='<span style="color:#07E0">'+d+'</span>';
setTimeout(()=>document.getElementById('status').innerHTML='',2000);
})}
function sendMessage(){
let msg=document.getElementById('msgBox').value;
if(!msg){alert('Please enter a message');return;}
fetch('/message?text='+encodeURIComponent(msg)).then(r=>r.text()).then(d=>{
document.getElementById('status').innerHTML='<span style="color:#07E0">'+d+'</span>';
document.getElementById('msgBox').value='';
setTimeout(()=>document.getElementById('status').innerHTML='',2000);
})}
setInterval(()=>{
fetch('/api').then(r=>r.json()).then(d=>{
document.getElementById('pitch').textContent=d.pitch;
document.getElementById('roll').textContent=d.roll;
document.getElementById('yaw').textContent=d.yaw;
document.getElementById('ssid').textContent=d.ssid;
document.getElementById('ip').textContent=d.ip;
document.getElementById('rssi').textContent=d.rssi;
})},500);
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleAPI() {
  String json = "{";
  json += "\"pitch\":" + String(pitch_filtered, 1) + ",";
  json += "\"roll\":" + String(roll_filtered, 1) + ",";
  json += "\"yaw\":" + String(yaw_filtered, 1) + ",";
  json += "\"ssid\":\"" + WiFi.SSID() + "\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI());
  json += "}";
  server.send(200, "application/json", json);
}

void handleBeep() {
  webBeepActive = true;
  startTone(1500, 500, speaker_volume);
  server.send(200, "text/plain", "Beep triggered on device!");
}

void handleMessage() {
  if (server.hasArg("text")) {
    webMessage = server.arg("text");
    webMessage = webMessage.substring(0, 40); // Limit length
    messageTime = millis();
    messageActive = true;
    server.send(200, "text/plain", "Message sent to device!");
  } else {
    server.send(400, "text/plain", "Missing text parameter");
  }
}

// ---------------- UI PRIMITIVES ----------------
void drawStatusBar() {
  tft.fillRect(0,0,DISP_W,STATUS_BAR_H,C_PANEL);
  tft.setTextSize(1); tft.setTextColor(C_FG);
  if (WiFi.status() == WL_CONNECTED) tft.fillCircle(5,5,2,C_SUCCESS); 
  else tft.drawCircle(5,5,2,C_ERROR);
  if (ntpSynced) {
    time_t nowt = time(nullptr); struct tm *tm_info = localtime(&nowt);
    char buf[20]; snprintf(buf, sizeof(buf), "%02d/%02d %02d:%02d", tm_info->tm_mday, tm_info->tm_mon+1, tm_info->tm_hour, tm_info->tm_min);
    tft.setCursor(30,2); tft.print(buf);
  } else {
    tft.setCursor(30,2); tft.printf("U:%lus", millis()/1000);
  }
  tft.setCursor(DISP_W - 46, 2); tft.printf("FPS:%2.0f", fps);
}

void drawHome() {
  tft.fillRect(0,STATUS_BAR_H,DISP_W,DISP_H-STATUS_BAR_H,C_BG);
  int cx = 40, cy = STATUS_BAR_H + 36, r = 28;
  tft.drawCircle(cx, cy, r, C_FG);
  for (int i=0;i<12;i++) {
    float a = i * 30.0 * PI / 180.0;
    int x1 = cx + (r-6)*sin(a), y1 = cy - (r-6)*cos(a);
    int x2 = cx + (r-2)*sin(a), y2 = cy - (r-2)*cos(a);
    tft.drawLine(x1,y1,x2,y2,C_FG);
  }
  updateHomeClockHands();

  int wx = DISP_W - 64, wy = STATUS_BAR_H + 10, ww = 54, wh = 44;
  tft.drawRoundRect(wx, wy, ww, wh, 4, C_ACCENT);
  tft.setTextSize(1); tft.setTextColor(C_FG);
  tft.setCursor(wx+6, wy+6); tft.print("Mumbai");
  tft.setCursor(wx+6, wy+22); tft.print(weatherTemp);
  tft.setTextColor(C_ACCENT); tft.setCursor(wx+6, wy+34); tft.print(weatherMain);

  tft.setTextColor(C_FG); tft.setCursor(8, DISP_H - 10); tft.print("Press for Apps ->");
  prev_hx = prev_hy = prev_mx = prev_my = prev_sx = prev_sy = -1;
}

void updateHomeClockHands() {
  if (!ntpSynced) return;
  int cx = 40, cy = STATUS_BAR_H + 36, r = 28;
  time_t nowt = time(nullptr);
  struct tm *tm_info = localtime(&nowt);
  int h = tm_info->tm_hour % 12;
  int m = tm_info->tm_min;
  int s = tm_info->tm_sec;
  float ha = (h + m/60.0f) * 30.0f * PI / 180.0f;
  float ma = m * 6.0f * PI / 180.0f;
  float sa = s * 6.0f * PI / 180.0f;

  int hx = cx + (r/2)*sin(ha), hy = cy - (r/2)*cos(ha);
  int mx = cx + (r*2/3)*sin(ma), my = cy - (r*2/3)*cos(ma);
  int sx = cx + (r-4)*sin(sa), sy = cy - (r-4)*cos(sa);

  if (prev_hx != -1) {
    tft.drawLine(cx, cy, prev_hx, prev_hy, C_BG);
    tft.drawLine(cx, cy, prev_mx, prev_my, C_BG);
    tft.drawLine(cx, cy, prev_sx, prev_sy, C_BG);
  }

  tft.drawLine(cx, cy, hx, hy, C_FG);
  tft.drawLine(cx, cy, mx, my, C_ACCENT);
  tft.drawLine(cx, cy, sx, sy, C_WARN);
  tft.fillCircle(cx, cy, 2, C_FG);

  prev_hx = hx; prev_hy = hy;
  prev_mx = mx; prev_my = my;
  prev_sx = sx; prev_sy = sy;
}

void drawLauncher() {
  tft.fillRect(0,STATUS_BAR_H,DISP_W,DISP_H-STATUS_BAR_H,C_BG);
  int margin = 8, gap = 6;
  int iconW = (DISP_W - 2*margin - gap)/2;
  int iconH = 22;
  int startY = STATUS_BAR_H + 10;
  const char* names[] = {"Calculator","Compass","Accel","Clock","Games","Settings"};
  AppState apps[] = {APP_CALCULATOR, APP_COMPASS, APP_ACCEL, APP_CLOCK, APP_GAMES, APP_SETTINGS};
  for (int i=0;i<6;i++) {
    int row = i/2, col = i%2;
    int x = margin + col*(iconW+gap);
    int y = startY + row*(iconH+gap);
    launcherIcons[i].x = x; launcherIcons[i].y = y; 
    launcherIcons[i].w = iconW; launcherIcons[i].h = iconH;
    launcherIcons[i].name = names[i]; launcherIcons[i].app = apps[i];
    tft.fillRoundRect(x,y,iconW,iconH,4,C_PANEL);
    tft.drawRoundRect(x,y,iconW,iconH,4,C_ACCENT);
    tft.setTextSize(1); tft.setTextColor(C_FG);
    int tx = x + (iconW - (int)strlen(names[i])*6)/2;
    int ty = y + (iconH - 8)/2;
    tft.setCursor(tx, ty); tft.print(names[i]);
  }
  tft.setTextColor(C_FG); tft.setCursor(8, DISP_H-10); tft.print("Long press = Home");
}

void drawCalculator() {
  tft.fillRect(0,STATUS_BAR_H,DISP_W,DISP_H-STATUS_BAR_H,C_BG);
  tft.fillRect(8, STATUS_BAR_H+6, DISP_W-16, 18, C_PANEL);
  tft.setTextColor(C_FG); tft.setTextSize(1); 
  tft.setCursor(12, STATUS_BAR_H+10); tft.print(calcDisplay);
  int btnW = 32, btnH = 16, gap = 3, startX = 10, startY = STATUS_BAR_H + 32;
  const char* labels[] = {"7","8","9","/","4","5","6","*","1","2","3","-","0",".","=","+"};
  for (int i=0;i<16;i++) {
    int row = i/4, col = i%4;
    int x = startX + col*(btnW+gap), y = startY + row*(btnH+gap);
    tft.fillRoundRect(x,y,btnW,btnH,3,C_PANEL);
    tft.setTextColor(C_FG); tft.setCursor(x + (btnW-6)/2, y+4); tft.print(labels[i]);
  }
  int y = startY + 4*(btnH+gap);
  tft.fillRoundRect(startX, y, btnW*2 + gap, btnH, 3, C_PANEL);
  tft.setCursor(startX + btnW - 3, y+4); tft.print("C");
  tft.fillRoundRect(startX + btnW*2 + gap*2, y, btnW*2, btnH, 3, C_PANEL);
  tft.setCursor(startX + btnW*3 + gap*2 - 3, y+4); tft.print("<");
}

void drawCompass() {
  tft.fillRect(0,STATUS_BAR_H,DISP_W,DISP_H-STATUS_BAR_H,C_BG);

  int cx = DISP_W/2, cy = STATUS_BAR_H + 44, radius = 36;

  tft.drawCircle(cx, cy, radius, C_FG);
  tft.drawCircle(cx, cy, radius-2, C_PANEL);

  tft.setTextSize(1); tft.setTextColor(C_FG);
  tft.setCursor(cx - 6, cy - radius - 8); tft.print("N");
  tft.setCursor(cx - 6, cy + radius + 2); tft.print("S");
  tft.setCursor(cx + radius + 2, cy - 4); tft.print("E");
  tft.setCursor(cx - radius - 8, cy - 4); tft.print("W");

  for (int a=0;a<360;a+=30) {
    float ar = a * PI / 180.0;
    int x1 = cx + (radius-2) * sin(ar), y1 = cy - (radius-2) * cos(ar);
    int x2 = cx + (radius-6) * sin(ar), y2 = cy - (radius-6) * cos(ar);
    tft.drawLine(x1,y1,x2,y2,C_PANEL);
  }

  tft.fillRoundRect(8, DISP_H - 20, 72, 14, 3, C_PANEL);
  tft.setCursor(14, DISP_H - 18); tft.setTextColor(C_FG); tft.print("Calibrate");

  updateCompass();
}

void updateCompass() {
  int cx = DISP_W/2, cy = STATUS_BAR_H + 44, radius = 36;
  
  tft.fillCircle(cx, cy, radius-8, C_BG);
  tft.drawCircle(cx, cy, radius, C_FG);
  tft.drawCircle(cx, cy, radius-2, C_PANEL);
  for (int a=0;a<360;a+=30) {
    float ar = a * PI / 180.0;
    int x1 = cx + (radius-2) * sin(ar), y1 = cy - (radius-2) * cos(ar);
    int x2 = cx + (radius-6) * sin(ar), y2 = cy - (radius-6) * cos(ar);
    tft.drawLine(x1,y1,x2,y2,C_PANEL);
  }

  float heading = yaw_ref - yaw_filtered;
  while (heading < 0) heading += 360;
  while (heading >= 360) heading -= 360;

  float ar = heading * PI / 180.0;
  int nx = cx + (radius-8) * sin(ar), ny = cy - (radius-8) * cos(ar);
  tft.drawLine(cx, cy, nx, ny, C_WARN);
  tft.fillCircle(cx, cy, 2, C_FG);

  int tx = 8, ty = STATUS_BAR_H + 4;
  tft.fillRect(tx, ty, DISP_W - 16, 12, C_BG);
  tft.setTextSize(1); tft.setTextColor(C_FG);
  tft.setCursor(tx, ty); tft.printf("Heading: %6.1f", heading);
}

void drawAccel() {
  tft.fillRect(0, STATUS_BAR_H, DISP_W, DISP_H - STATUS_BAR_H, C_BG);

  tft.setTextSize(1); tft.setTextColor(C_FG);
  tft.setCursor(8, STATUS_BAR_H + 6); tft.print("Accelerometer (XYZ)");

  // Calculate deltas from reference
  float dx = pitch_filtered - pitch_ref;
  float dy = roll_filtered - roll_ref;
  float dz = yaw_filtered - yaw_ref;

  // Draw three horizontal bars for X, Y, Z
  int bx = 8, bw = DISP_W - 16, bh = 12;
  int by1 = STATUS_BAR_H + 28, by2 = STATUS_BAR_H + 52, by3 = STATUS_BAR_H + 76;
  int center = bx + bw/2;

  // X-axis (Pitch) bar
  tft.setTextColor(C_ACCENT);
  tft.setCursor(bx, by1 - 10); tft.print("X (Pitch):");
  tft.drawRect(bx, by1, bw, bh, C_FG);
  tft.fillRect(bx+1, by1+1, bw-2, bh-2, C_BG);
  tft.drawLine(center, by1, center, by1+bh, C_PANEL);
  int lenX = (int)(constrain(dx / 45.0f, -1.0f, 1.0f) * (bw/2));
  if (lenX < 0) tft.fillRect(center + lenX, by1+2, -lenX, bh-4, C_ACCENT);
  else tft.fillRect(center, by1+2, lenX, bh-4, C_ACCENT);
  tft.setCursor(bx + bw + 4, by1 + 2); tft.printf("%+4.0f", dx);

  // Y-axis (Roll) bar
  tft.setTextColor(C_WARN);
  tft.setCursor(bx, by2 - 10); tft.print("Y (Roll):");
  tft.drawRect(bx, by2, bw, bh, C_FG);
  tft.fillRect(bx+1, by2+1, bw-2, bh-2, C_BG);
  tft.drawLine(center, by2, center, by2+bh, C_PANEL);
  int lenY = (int)(constrain(dy / 45.0f, -1.0f, 1.0f) * (bw/2));
  if (lenY < 0) tft.fillRect(center + lenY, by2+2, -lenY, bh-4, C_WARN);
  else tft.fillRect(center, by2+2, lenY, bh-4, C_WARN);
  tft.setCursor(bx + bw + 4, by2 + 2); tft.printf("%+4.0f", dy);

  // Z-axis (Yaw) bar
  tft.setTextColor(C_SUCCESS);
  tft.setCursor(bx, by3 - 10); tft.print("Z (Yaw):");
  tft.drawRect(bx, by3, bw, bh, C_FG);
  tft.fillRect(bx+1, by3+1, bw-2, bh-2, C_BG);
  tft.drawLine(center, by3, center, by3+bh, C_PANEL);
  int lenZ = (int)(constrain(dz / 90.0f, -1.0f, 1.0f) * (bw/2));
  if (lenZ < 0) tft.fillRect(center + lenZ, by3+2, -lenZ, bh-4, C_SUCCESS);
  else tft.fillRect(center, by3+2, lenZ, bh-4, C_SUCCESS);
  tft.setCursor(bx + bw + 4, by3 + 2); tft.printf("%+4.0f", dz);

  // Calibrate button
  tft.fillRoundRect(8, DISP_H - 20, 72, 14, 3, C_PANEL);
  tft.setCursor(14, DISP_H - 18); tft.setTextColor(C_FG); tft.print("Calibrate");
}

void updateAccel() {
  // Fast update of just the bars
  float dx = pitch_filtered - pitch_ref;
  float dy = roll_filtered - roll_ref;
  float dz = yaw_filtered - yaw_ref;

  int bx = 8, bw = DISP_W - 16, bh = 12;
  int by1 = STATUS_BAR_H + 28, by2 = STATUS_BAR_H + 52, by3 = STATUS_BAR_H + 76;
  int center = bx + bw/2;

  // Clear and redraw bars
  tft.fillRect(bx+1, by1+1, bw-2, bh-2, C_BG);
  tft.drawLine(center, by1, center, by1+bh, C_PANEL);
  int lenX = (int)(constrain(dx / 45.0f, -1.0f, 1.0f) * (bw/2));
  if (lenX < 0) tft.fillRect(center + lenX, by1+2, -lenX, bh-4, C_ACCENT);
  else tft.fillRect(center, by1+2, lenX, bh-4, C_ACCENT);
  tft.fillRect(bx + bw + 4, by1 + 2, 30, 8, C_BG);
  tft.setTextSize(1); tft.setTextColor(C_ACCENT);
  tft.setCursor(bx + bw + 4, by1 + 2); tft.printf("%+4.0f", dx);

  tft.fillRect(bx+1, by2+1, bw-2, bh-2, C_BG);
  tft.drawLine(center, by2, center, by2+bh, C_PANEL);
  int lenY = (int)(constrain(dy / 45.0f, -1.0f, 1.0f) * (bw/2));
  if (lenY < 0) tft.fillRect(center + lenY, by2+2, -lenY, bh-4, C_WARN);
  else tft.fillRect(center, by2+2, lenY, bh-4, C_WARN);
  tft.fillRect(bx + bw + 4, by2 + 2, 30, 8, C_BG);
  tft.setTextColor(C_WARN);
  tft.setCursor(bx + bw + 4, by2 + 2); tft.printf("%+4.0f", dy);

  tft.fillRect(bx+1, by3+1, bw-2, bh-2, C_BG);
  tft.drawLine(center, by3, center, by3+bh, C_PANEL);
  int lenZ = (int)(constrain(dz / 90.0f, -1.0f, 1.0f) * (bw/2));
  if (lenZ < 0) tft.fillRect(center + lenZ, by3+2, -lenZ, bh-4, C_SUCCESS);
  else tft.fillRect(center, by3+2, lenZ, bh-4, C_SUCCESS);
  tft.fillRect(bx + bw + 4, by3 + 2, 30, 8, C_BG);
  tft.setTextColor(C_SUCCESS);
  tft.setCursor(bx + bw + 4, by3 + 2); tft.printf("%+4.0f", dz);
}

void drawClock() {
  tft.fillRect(0,STATUS_BAR_H,DISP_W,DISP_H-STATUS_BAR_H,C_BG);
  tft.setTextColor(C_FG); tft.setTextSize(3);
  if (ntpSynced) {
    time_t nowt = time(nullptr); struct tm *tm_info = localtime(&nowt);
    char buf[16]; snprintf(buf,sizeof(buf), "%02d:%02d:%02d", tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    tft.setCursor(8, STATUS_BAR_H + 40); tft.print(buf);
    tft.setTextSize(1); char db[20]; 
    snprintf(db,sizeof(db), "%02d/%02d/%04d", tm_info->tm_mday, tm_info->tm_mon+1, tm_info->tm_year+1900);
    tft.setCursor(8, STATUS_BAR_H + 100); tft.print(db);
  } else {
    tft.setCursor(8, STATUS_BAR_H + 40); tft.setTextSize(2); 
    tft.print("Clock not synced");
  }
}

void drawGames() {
  tft.fillRect(0,STATUS_BAR_H,DISP_W,DISP_H-STATUS_BAR_H,C_BG);
  tft.setTextSize(1); tft.setTextColor(C_ACCENT);
  tft.setCursor(10, STATUS_BAR_H + 10); tft.print("Select Game:");
  
  int btnW = DISP_W - 20, btnH = 24, gap = 8, startY = STATUS_BAR_H + 28;
  const char* games[] = {"Tic-Tac-Toe", "Pong", "Space Shooter"};
  
  for (int i = 0; i < 3; i++) {
    int y = startY + i * (btnH + gap);
    tft.fillRoundRect(10, y, btnW, btnH, 4, C_PANEL);
    tft.drawRoundRect(10, y, btnW, btnH, 4, C_ACCENT);
    tft.setTextColor(C_FG);
    int tx = 10 + (btnW - strlen(games[i])*6)/2;
    tft.setCursor(tx, y + 8); tft.print(games[i]);
  }
  
  tft.setTextColor(C_FG); tft.setCursor(8, DISP_H-10); 
  tft.print("Long press = Home");
}

bool checkWin(int p) {
  const int wins[8][3] = {{0,1,2},{3,4,5},{6,7,8},{0,3,6},{1,4,7},{2,5,8},{0,4,8},{2,4,6}};
  for (int i=0;i<8;i++) 
    if (tttBoard[wins[i][0]]==p && tttBoard[wins[i][1]]==p && tttBoard[wins[i][2]]==p) 
      return true;
  return false;
}

int findBestTTTMove(int ai,int human) {
  for (int i=0;i<9;i++) if (tttBoard[i]==0) { 
    tttBoard[i]=ai; 
    if (checkWin(ai)) { tttBoard[i]=0; return i; } 
    tttBoard[i]=0; 
  }
  for (int i=0;i<9;i++) if (tttBoard[i]==0) { 
    tttBoard[i]=human; 
    if (checkWin(human)) { tttBoard[i]=0; return i; } 
    tttBoard[i]=0; 
  }
  if (tttBoard[4]==0) return 4;
  int corners[4] = {0,2,6,8}; 
  for (int i=0;i<4;i++) if (tttBoard[corners[i]]==0) return corners[i];
  for (int i=0;i<9;i++) if (tttBoard[i]==0) return i;
  return -1;
}

void drawTicTacToe() {
  tft.fillRect(0,STATUS_BAR_H,DISP_W,DISP_H-STATUS_BAR_H,C_BG);
  int s = 34, ox = (DISP_W - s*3)/2, oy = STATUS_BAR_H + 14;
  
  for (int i=0;i<9;i++) {
    int x = ox + (i%3)*s, y = oy + (i/3)*s;
    tft.drawRect(x,y,s,s,C_FG);
    if (tttBoard[i]==1) { 
      tft.drawLine(x+6,y+6,x+s-6,y+s-6,C_ACCENT); 
      tft.drawLine(x+s-6,y+6,x+6,y+s-6,C_ACCENT); 
    }
    else if (tttBoard[i]==2) 
      tft.drawCircle(x+s/2,y+s/2,s/2-8,C_WARN);
  }
  
  if (tttGameOver) { 
    tft.fillRect(DISP_W/2-40, DISP_H/2+36, 80, 20, C_PANEL); 
    tft.drawRect(DISP_W/2-40, DISP_H/2+36, 80, 20, C_ACCENT); 
    tft.setCursor(DISP_W/2-28, DISP_H/2+40); tft.setTextColor(C_FG); 
    tft.print("Press to reset"); 
  }
}

void drawPong() {
  tft.fillRect(0, STATUS_BAR_H, DISP_W, DISP_H - STATUS_BAR_H, C_BG);
  
  // Center line
  for (int y = STATUS_BAR_H; y < DISP_H; y += 6) {
    tft.drawLine(DISP_W/2, y, DISP_W/2, y+3, C_PANEL);
  }
  
  // Paddles
  tft.fillRect(5, pongPaddle1Y, PADDLE_W, PADDLE_H, C_ACCENT);
  tft.fillRect(DISP_W - 5 - PADDLE_W, pongPaddle2Y, PADDLE_W, PADDLE_H, C_WARN);
  
  // Ball
  tft.fillCircle((int)pongBallX, (int)pongBallY, 2, C_FG);
  
  // Scores
  tft.setTextSize(2); tft.setTextColor(C_FG);
  tft.setCursor(DISP_W/2 - 30, STATUS_BAR_H + 5); tft.print(pongScore1);
  tft.setCursor(DISP_W/2 + 20, STATUS_BAR_H + 5); tft.print(pongScore2);
  
  if (!pongGameActive) {
    tft.fillRect(DISP_W/2-40, DISP_H/2, 80, 20, C_PANEL);
    tft.drawRect(DISP_W/2-40, DISP_H/2, 80, 20, C_ACCENT);
    tft.setTextSize(1);
    tft.setCursor(DISP_W/2-28, DISP_H/2+6); tft.print("Press to play");
  }
}

void updatePong() {
  if (!pongGameActive) return;
  
  // Erase old ball
  tft.fillCircle(lastBallX, lastBallY, 3, C_BG);
  
  // Update ball
  pongBallX += pongBallVX;
  pongBallY += pongBallVY;
  
  // Ball collisions
  if (pongBallY <= STATUS_BAR_H + 3 || pongBallY >= DISP_H - 3) {
    pongBallVY = -pongBallVY;
    
  }
  
  // Paddle collisions
  if (pongBallX <= 8 + PADDLE_W && pongBallY >= pongPaddle1Y && pongBallY <= pongPaddle1Y + PADDLE_H) {
    pongBallVX = abs(pongBallVX);
    pongBallVY += (pongBallY - (pongPaddle1Y + PADDLE_H/2)) * 0.15f;
    
  }
  if (pongBallX >= DISP_W - 8 - PADDLE_W && pongBallY >= pongPaddle2Y && pongBallY <= pongPaddle2Y + PADDLE_H) {
    pongBallVX = -abs(pongBallVX);
    pongBallVY += (pongBallY - (pongPaddle2Y + PADDLE_H/2)) * 0.15f;
    
  }
  
  // Scoring
  if (pongBallX < 0) {
    pongScore2++;
    pongBallX = DISP_W/2; pongBallY = DISP_H/2;
    pongBallVX = 2.2f; pongBallVY = 1.8f;
    
    if (pongScore2 >= 5) pongGameActive = false;
    needsFullRedraw = true;
    return;
  }
  if (pongBallX > DISP_W) {
    pongScore1++;
    pongBallX = DISP_W/2; pongBallY = DISP_H/2;
    pongBallVX = -2.2f; pongBallVY = -1.8f;
    
    if (pongScore1 >= 5) pongGameActive = false;
    needsFullRedraw = true;
    return;
  }
  
  // AI paddle (smarter)
  int target = (int)pongBallY;
  if (pongPaddle2Y + PADDLE_H/2 < target - 2) pongPaddle2Y += 3;
  else if (pongPaddle2Y + PADDLE_H/2 > target + 2) pongPaddle2Y -= 3;
  pongPaddle2Y = constrain(pongPaddle2Y, STATUS_BAR_H, DISP_H - PADDLE_H);
  
  // Draw ball
  tft.fillCircle((int)pongBallX, (int)pongBallY, 2, C_FG);
  lastBallX = (int)pongBallX;
  lastBallY = (int)pongBallY;
  
  // Redraw paddles
  tft.fillRect(5, STATUS_BAR_H, PADDLE_W, DISP_H - STATUS_BAR_H, C_BG);
  tft.fillRect(5, pongPaddle1Y, PADDLE_W, PADDLE_H, C_ACCENT);
  tft.fillRect(DISP_W - 5 - PADDLE_W, STATUS_BAR_H, PADDLE_W, DISP_H - STATUS_BAR_H, C_BG);
  tft.fillRect(DISP_W - 5 - PADDLE_W, pongPaddle2Y, PADDLE_W, PADDLE_H, C_WARN);
}

void drawSpaceShooter() {
  tft.fillRect(0, STATUS_BAR_H, DISP_W, DISP_H - STATUS_BAR_H, C_BG);
  
  // Ship
  tft.fillTriangle(shipX, shipY-5, shipX-4, shipY+5, shipX+4, shipY+5, C_ACCENT);
  
  // Bullets
  for (int i = 0; i < 8; i++) {
    if (bullets[i].active) {
      tft.fillRect((int)bullets[i].x, (int)bullets[i].y, 2, 4, C_WARN);
    }
  }
  
  // Enemies
  for (int i = 0; i < 10; i++) {
    if (enemies[i].active) {
      tft.fillRect((int)enemies[i].x-3, (int)enemies[i].y-3, 6, 6, C_ERROR);
    }
  }
  
  // Score
  tft.setTextSize(1); tft.setTextColor(C_FG);
  tft.setCursor(8, STATUS_BAR_H + 4); tft.print("Score: " + String(shooterScore));
  
  if (!shooterGameActive) {
    tft.fillRect(DISP_W/2-40, DISP_H/2, 80, 20, C_PANEL);
    tft.drawRect(DISP_W/2-40, DISP_H/2, 80, 20, C_ACCENT);
    tft.setCursor(DISP_W/2-32, DISP_H/2+6); tft.print("Game Over!");
    tft.setCursor(DISP_W/2-28, DISP_H-15); tft.print("Press to play");
  }
}

void updateSpaceShooter() {
  if (!shooterGameActive) return;
  
  // Erase old ship
  tft.fillTriangle(lastShipX, shipY-5, lastShipX-4, shipY+5, lastShipX+4, shipY+5, C_BG);
  
  // Draw new ship
  tft.fillTriangle(shipX, shipY-5, shipX-4, shipY+5, shipX+4, shipY+5, C_ACCENT);
  lastShipX = shipX;
  
  // Update bullets
  for (int i = 0; i < 8; i++) {
    if (bullets[i].active) {
      tft.fillRect((int)bullets[i].x, (int)bullets[i].y, 2, 4, C_BG);
      bullets[i].y -= 4.5f;
      if (bullets[i].y < STATUS_BAR_H) bullets[i].active = false;
      else tft.fillRect((int)bullets[i].x, (int)bullets[i].y, 2, 4, C_WARN);
    }
  }
  
  // Update enemies
  for (int i = 0; i < 10; i++) {
    if (enemies[i].active) {
      tft.fillRect((int)enemies[i].x-3, (int)enemies[i].y-3, 6, 6, C_BG);
      enemies[i].y += 1.5f;
      if (enemies[i].y > DISP_H) {
        enemies[i].active = false;
        shooterGameActive = false;
        needsFullRedraw = true;
        return;
      }
      tft.fillRect((int)enemies[i].x-3, (int)enemies[i].y-3, 6, 6, C_ERROR);
    }
  }
  
  // Check collisions
  for (int i = 0; i < 8; i++) {
    if (bullets[i].active) {
      for (int j = 0; j < 10; j++) {
        if (enemies[j].active) {
          if (abs((int)bullets[i].x - (int)enemies[j].x) < 20 && 
              abs((int)bullets[i].y - (int)enemies[j].y) < 20) {
            bullets[i].active = false;
            enemies[j].active = false;
            shooterScore += 10;
            
            tft.fillRect((int)enemies[j].x-3, (int)enemies[j].y-3, 6, 6, C_BG);
          }
        }
      }
    }
  }
  
  // Spawn enemies
  if (millis() - lastEnemySpawn > 1500) {
    for (int i = 0; i < 10; i++) {
      if (!enemies[i].active) {
        enemies[i].x = random(15, DISP_W - 15);
        enemies[i].y = STATUS_BAR_H + 15;
        enemies[i].active = true;
        lastEnemySpawn = millis();
        break;
      }
    }
  }
}

void drawSettings() {
  tft.fillRect(0,STATUS_BAR_H,DISP_W,DISP_H-STATUS_BAR_H,C_BG);
  tft.setTextSize(1); tft.setTextColor(C_FG); 
  tft.setCursor(10, STATUS_BAR_H+10); tft.print("Settings");
  tft.setCursor(10, STATUS_BAR_H+26); tft.print("WiFi Networks:");
  if (wifiScanDone == 0) { 
    tft.setCursor(10, STATUS_BAR_H+44); tft.print("Scanning..."); 
  } else {
    String currentSSID = WiFi.SSID();
    for (int i=0;i<min(wifiNetCount,4);i++) {
      int y = STATUS_BAR_H + 44 + i*18;
      bool connected = (wifiNets[i].ssid == currentSSID);
      if (connected) tft.fillRoundRect(8, y, DISP_W-16, 16, 3, C_SELECTED);
      tft.setTextColor(connected?C_BG:C_FG); 
      tft.setCursor(12, y+3); tft.print(wifiNets[i].ssid);
      tft.setCursor(DISP_W-34, y+3); tft.print(wifiNets[i].rssi);
    }
  }
  tft.setTextColor(C_FG); tft.setCursor(10, DISP_H-10); 
  tft.print("Press to rescan");
}

void scanWiFi() {
  wifiScanDone = 0; needsFullRedraw = true;
  int n = WiFi.scanNetworks(); 
  wifiNetCount = min(n, 16);
  for (int i=0;i<wifiNetCount;i++) { 
    wifiNets[i].ssid = WiFi.SSID(i); 
    wifiNets[i].rssi = WiFi.RSSI(i); 
    wifiNets[i].open = (WiFi.encryptionType(i) == ENC_TYPE_NONE); 
  }
  wifiScanDone = 1; needsFullRedraw = true;
}

void drawWebMessage() {
  int boxW = 140, boxH = 50;
  int boxX = (DISP_W - boxW) / 2;
  int boxY = (DISP_H - boxH) / 2;
  
  tft.fillRoundRect(boxX, boxY, boxW, boxH, 8, C_PANEL);
  tft.drawRoundRect(boxX, boxY, boxW, boxH, 8, C_ACCENT);
  
  tft.setTextSize(1); tft.setTextColor(C_ACCENT);
  tft.setCursor(boxX + 10, boxY + 8);
  tft.print("Web Message:");
  
  tft.setTextColor(C_FG);
  // Word wrap for long messages
  int lineY = boxY + 22;
  int charPerLine = 20;
  for (int i = 0; i < webMessage.length(); i += charPerLine) {
    String line = webMessage.substring(i, min((int)webMessage.length(), i + charPerLine));
    tft.setCursor(boxX + 10, lineY);
    tft.print(line);
    lineY += 10;
    if (lineY > boxY + boxH - 10) break;
  }
}

void redrawScreen() {
  switch (currentApp) {
    case APP_HOME: drawHome(); break;
    case APP_LAUNCHER: drawLauncher(); break;
    case APP_CALCULATOR: drawCalculator(); break;
    case APP_COMPASS: drawCompass(); break;
    case APP_ACCEL: drawAccel(); break;
    case APP_CLOCK: drawClock(); break;
    case APP_GAMES: drawGames(); break;
    case APP_TICTACTOE: drawTicTacToe(); break;
    case APP_PONG: drawPong(); break;
    case APP_SPACESHOOTER: drawSpaceShooter(); break;
    case APP_SETTINGS: drawSettings(); break;
    default: drawHome(); break;
  }
  drawStatusBar();
  needsFullRedraw = false;
}

// Smooth cursor implementation
void saveCursorBackground() {
  int idx = 0;
  for (int dy = 0; dy < CURSOR_SIZE; dy++) {
    for (int dx = 0; dx < CURSOR_SIZE; dx++) {
      int px = cursorX - CURSOR_SIZE/2 + dx;
      int py = cursorY - CURSOR_SIZE/2 + dy;
      if (px >= 0 && px < DISP_W && py >= STATUS_BAR_H && py < DISP_H) {
        // This is a placeholder - actual pixel reading not directly supported
        // We'll use a different approach
        cursorBuffer[idx++] = C_BG; // Default to background
      }
    }
  }
}

void restoreCursorBackground() {
  // Simply redraw a small area instead of restoring pixels
  // This is more efficient than reading back from display
  if (lastCursorX < 0) return;
  
  int x = lastCursorX - CURSOR_SIZE/2 - 1;
  int y = lastCursorY - CURSOR_SIZE/2 - 1;
  int w = CURSOR_SIZE + 2;
  int h = CURSOR_SIZE + 2;
  
  // Clip to valid area
  if (x < 0) { w += x; x = 0; }
  if (y < STATUS_BAR_H) { h += (y - STATUS_BAR_H); y = STATUS_BAR_H; }
  if (x + w > DISP_W) w = DISP_W - x;
  if (y + h > DISP_H) h = DISP_H - y;
  
  if (w > 0 && h > 0) {
    // Just clear the cursor area - app will redraw if needed
    // For most static content, this works well
  }
}

void drawCursor(int x, int y) {
  if (x < CURSOR_SIZE/2 || y < STATUS_BAR_H + CURSOR_SIZE/2 || 
      x >= DISP_W - CURSOR_SIZE/2 || y >= DISP_H - CURSOR_SIZE/2) return;

  // Draw small crosshair cursor
  tft.drawLine(x - CURSOR_SIZE, y, x + CURSOR_SIZE, y, C_CURSOR);
  tft.drawLine(x, y - CURSOR_SIZE, x, y + CURSOR_SIZE, C_CURSOR);
  tft.drawPixel(x, y, C_FG); // Center dot
}

void showBootScreen() {
  tft.fillScreen(C_BG);
  tft.setTextSize(2); tft.setTextColor(C_ACCENT); 
  tft.setCursor(8,12); tft.print("MiniConsole");
  tft.setTextSize(1); tft.setTextColor(C_FG); 
  tft.setCursor(8,40); tft.print("Enhanced v2");
  int bw = DISP_W - 40; int bx = 20, by = 70;
  tft.drawRect(bx, by, bw, 12, C_FG);
  for (int p=0;p<=bw;p+=4) {
    tft.fillRect(bx+1, by+1, p, 10, C_ACCENT);
    delay(10);
  }
  tft.setCursor(8,100); tft.print("Ready");
}

void connectToWiFi() {
  tft.setCursor(8,106); tft.setTextColor(C_FG); tft.print("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t st = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - st < 10000) {
    delay(100);
    yield();
  }

  if (WiFi.status() == WL_CONNECTED) {
    tft.setCursor(8, 106); tft.print("WiFi OK         ");
    Serial.printf("WiFi connected: %s\n", WiFi.SSID().c_str());
  } else {
    Serial.println("Primary WiFi failed. Trying open networks...");
    int n = WiFi.scanNetworks();
    bool connected = false;
    for (int i=0;i<n;i++) {
      if (WiFi.encryptionType(i) == ENC_TYPE_NONE) {
        WiFi.begin(WiFi.SSID(i));
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) { 
          delay(50); yield(); 
        }
        if (WiFi.status() == WL_CONNECTED) {
          connected = true;
          tft.setCursor(8, 106); tft.print("WiFi OK (Open)  ");
          break;
        }
      }
    }
    if (!connected) tft.setCursor(8, 106); tft.setTextColor(C_ERROR); tft.print("WiFi Failed");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== MiniConsole Enhanced v2 ===");

  pinMode(MUX_S0, OUTPUT);
  pinMode(JOY_SW, INPUT_PULLUP);
  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, LOW);

  // Initialize SPI at 15MHz for display
  SPI.begin();
  SPI.setFrequency(15000000);
  
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  showBootScreen();
  Serial.println("TFT initialized (15MHz SPI)");

  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000); // Fast I2C
  Wire.beginTransmission((uint8_t)MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  Serial.println("MPU initialized");

  EEPROM.begin(EEPROM_SIZE);
  if (loadCalibration()) {
    calibrated = true;
    tft.setCursor(8,110); tft.setTextColor(C_FG); 
    tft.print("Calibration loaded");
  }

  long sumX=0, sumY=0;
  for (int i=0;i<50;i++) { sumX += readMux(0); sumY += readMux(1); delay(15); }
  centerX = (int)(sumX / 50); centerY = (int)(sumY / 50);

  connectToWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    syncNTP();
    delay(500);
    fetchWeather();
    weatherLastFetch = millis();
    
    if (MDNS.begin(DEVICE_NAME)) MDNS.addService("http","tcp",80);
    server.on("/", handleRoot);
    server.on("/api", handleAPI);
    server.on("/beep", handleBeep);
    server.on("/message", handleMessage);
    server.begin();
    Serial.println("Web server at miniconsole.local");
  }

  playClick();
  delay(200);
  
  for (int i=0;i<9;i++) tttBoard[i]=0;
  for (int i=0;i<8;i++) bullets[i].active = false;
  for (int i=0;i<10;i++) enemies[i].active = false;

  needsFullRedraw = true;
  timerMicros = micros();
  Serial.println("Setup complete");
}

void handleCalcPress(int px,int py) {
  int btnW = 32, btnH = 16, gap = 3, startX = 10, startY = STATUS_BAR_H + 32;
  const char* labels[] = {"7","8","9","/","4","5","6","*","1","2","3","-","0",".","=","+"};
  for (int i=0;i<16;i++) {
    int row=i/4, col=i%4;
    int x = startX + col*(btnW+gap), y = startY + row*(btnH+gap);
    if (px >= x && px < x+btnW && py >= y && py < y+btnH) {
      playClick();
      const char* lab = labels[i];
      if (strcmp(lab,"=")==0) {
        if (calcOp) {
          double b = atof(calcDisplay), res = 0;
          if (calcOp=='+') res = calcA + b; 
          else if (calcOp=='-') res = calcA - b; 
          else if (calcOp=='*') res = calcA * b; 
          else if (calcOp=='/') res = (b!=0)?calcA/b:0;
          snprintf(calcDisplay,20,"%g",res); calcOp=0;
        }
      } else if (strchr("+-*/", lab[0])) { 
        calcA = atof(calcDisplay); calcOp = lab[0]; strcpy(calcDisplay,"0"); 
      } else { 
        if (strcmp(calcDisplay,"0")==0) calcDisplay[0]=0; 
        if (strlen(calcDisplay) < 18) strcat(calcDisplay, lab); 
      }
      needsFullRedraw = true; return;
    }
  }
}

void handleTTTPress(int px,int py) {
  if (tttGameOver) { 
    for (int i=0;i<9;i++) tttBoard[i]=0; 
    tttTurn=1; tttGameOver=false; 
    needsFullRedraw=true; playClick(); return; 
  }
  int s = 34, ox = (DISP_W - s*3)/2, oy = STATUS_BAR_H + 14;
  for (int i=0;i<9;i++) {
    int x = ox + (i%3)*s, y = oy + (i/3)*s;
    if (px >= x && px < x+s && py >= y && py < y+s && tttBoard[i]==0) {
      tttBoard[i]=tttTurn; playClick();
      if (checkWin(tttTurn)) { 
        tttGameOver=true; needsFullRedraw=true; return; 
      }
      tttTurn = (tttTurn==1)?2:1;
      if (tttTurn==2) { 
        int ai = findBestTTTMove(2,1); 
        if (ai>=0) { 
          tttBoard[ai]=2; 
          if (checkWin(2)) tttGameOver=true;
        } 
        tttTurn=1; 
      }
      int empty=0; 
      for (int j=0;j<9;j++) if (tttBoard[j]==0) empty++;
      if (empty==0 && !tttGameOver) tttGameOver=true;
      needsFullRedraw=true; break;
    }
  }
}

void handleGamesPress(int px, int py) {
  int btnW = DISP_W - 20, btnH = 24, gap = 8, startY = STATUS_BAR_H + 28;
  for (int i = 0; i < 3; i++) {
    int y = startY + i * (btnH + gap);
    if (px >= 10 && px < 10 + btnW && py >= y && py < y + btnH) {
      playClick();
      if (i == 0) {
        currentApp = APP_TICTACTOE;
        for (int j = 0; j < 9; j++) tttBoard[j] = 0;
        tttTurn = 1; tttGameOver = false;
      } else if (i == 1) {
        currentApp = APP_PONG;
        pongScore1 = 0; pongScore2 = 0;
        pongBallX = 80; pongBallY = 64;
        pongBallVX = 2.2f; pongBallVY = 1.8f;
        pongPaddle1Y = 40; pongPaddle2Y = 40;
        pongGameActive = true;
        lastBallX = 80; lastBallY = 64;
      } else if (i == 2) {
        currentApp = APP_SPACESHOOTER;
        shooterScore = 0; shipX = 80; shipY = 100;
        lastShipX = 80;
        for (int j = 0; j < 8; j++) bullets[j].active = false;
        for (int j = 0; j < 10; j++) enemies[j].active = false;
        shooterGameActive = true;
        lastEnemySpawn = millis();
      }
      needsFullRedraw = true; playNavigate();
      break;
    }
  }
}

void mapJoystickToMovement(int rawXv,int rawYv,int &moveX,int &moveY) {
  int dx = rawXv - centerX, dy = rawYv - centerY;
  int thr = 80;
  int mvx = 0, mvy = 0;
  if (abs(dx) > thr) mvx = (dx > 0)?1:-1;
  if (abs(dy) > thr) mvy = (dy > 0)?1:-1;
  switch (joystick_rotation) {
    case 0: moveX = mvx; moveY = mvy; break;
    case 90: moveX = mvy; moveY = -mvx; break;
    case 180: moveX = -mvx; moveY = -mvy; break;
    case 270: default: moveX = -mvy; moveY = mvx; break;
  }
}

void loop() {
  static uint32_t frameCount = 0;
  static uint32_t lastFPSUpdate = 0;
  uint32_t loopStart = millis();

  if (loopStart - lastFPSUpdate >= 1000) {
    fps = frameCount; frameCount = 0; lastFPSUpdate = loopStart;
    freeHeap = ESP.getFreeHeap();
  }

  updateMPU6050();

  rawX = readMux(0);
  rawY = readMux(1);
  btnPressed = (digitalRead(JOY_SW) == LOW);

  int mvx=0,mvy=0; 
  mapJoystickToMovement(rawX, rawY, mvx, mvy);
  int step = constrain((int)joystick_speed,1,15);
  int newCursorX = constrain(cursorX + mvx * step, CURSOR_SIZE, DISP_W - CURSOR_SIZE);
  int newCursorY = constrain(cursorY + mvy * step, STATUS_BAR_H + CURSOR_SIZE, DISP_H - CURSOR_SIZE);

  // Game-specific controls
  if (currentApp == APP_PONG && pongGameActive) {
    if (mvy != 0) {
      pongPaddle1Y = constrain(pongPaddle1Y + mvy * 4, STATUS_BAR_H, DISP_H - PADDLE_H);
    }
  }
  if (currentApp == APP_SPACESHOOTER && shooterGameActive) {
    shipX = constrain(shipX + mvx * 4, 10, DISP_W - 10);
  }

  // Button handling
  static uint32_t btnStart = 0;
  if (btnPressed && !lastBtnPressed) btnStart = millis();
  else if (!btnPressed && lastBtnPressed) {
    uint32_t dur = millis() - btnStart;
    if (dur < 1000) {
      playClick();
      if (currentApp == APP_HOME) {
        currentApp = APP_LAUNCHER; needsFullRedraw = true; playNavigate();
      }
      else if (currentApp == APP_LAUNCHER) {
        for (int i=0;i<6;i++) {
          Icon &ic = launcherIcons[i];
          if (newCursorX >= ic.x && newCursorX < ic.x+ic.w && 
              newCursorY >= ic.y && newCursorY < ic.y+ic.h) {
            currentApp = ic.app; needsFullRedraw = true; playNavigate();
            if (currentApp == APP_SETTINGS) scanWiFi();
            break;
          }
        }
      } 
      else if (currentApp == APP_CALCULATOR) handleCalcPress(newCursorX, newCursorY);
      else if (currentApp == APP_TICTACTOE) handleTTTPress(newCursorX, newCursorY);
      else if (currentApp == APP_GAMES) handleGamesPress(newCursorX, newCursorY);
      else if (currentApp == APP_PONG && !pongGameActive) {
        pongScore1 = 0; pongScore2 = 0;
        pongBallX = 80; pongBallY = 64;
        pongBallVX = 2.2f; pongBallVY = 1.8f;
        pongGameActive = true;
        needsFullRedraw = true;
      }
      else if (currentApp == APP_SPACESHOOTER) {
        if (!shooterGameActive) {
          shooterScore = 0; shipX = 80;
          for (int i = 0; i < 8; i++) bullets[i].active = false;
          for (int i = 0; i < 10; i++) enemies[i].active = false;
          shooterGameActive = true;
          lastEnemySpawn = millis();
          needsFullRedraw = true;
        } else if (millis() - lastBulletTime > 200) {
          for (int i = 0; i < 8; i++) {
            if (!bullets[i].active) {
              bullets[i].x = shipX;
              bullets[i].y = shipY - 6;
              bullets[i].active = true;
              lastBulletTime = millis();
              
              break;
            }
          }
        }
      }
      else if (currentApp == APP_COMPASS || currentApp == APP_ACCEL) {
        if (newCursorX >= 8 && newCursorX < 80 && newCursorY >= DISP_H - 22) {
          calibrateSensors();
        }
      } 
      else if (currentApp == APP_SETTINGS) scanWiFi();
    } else {
      playClick(); playNavigate();
      currentApp = APP_HOME; needsFullRedraw = true;
    }
  }
  lastBtnPressed = btnPressed;

  // Web server
  server.handleClient();
  if (MDNS.isRunning()) MDNS.update();

  // Periodic updates
  static uint32_t lastWeatherCheck = 0;
  if (millis() - lastWeatherCheck > 60000) { 
    fetchWeather(); lastWeatherCheck = millis(); 
  }
  if (!ntpSynced) syncNTP();
  if (ntpSynced) currentTime = time(nullptr);

  // Check message timeout
  if (messageActive && millis() - messageTime > 4000) {
    messageActive = false;
    needsFullRedraw = true;
  }

  // App change
  if (currentApp != lastApp) { 
    needsFullRedraw = true; lastApp = currentApp; 
  }

  // Redraw logic
  if (needsFullRedraw) {
    redrawScreen();
  } else {
    // Fast updates for dynamic content
    if (currentApp == APP_HOME) updateHomeClockHands();
    else if (currentApp == APP_COMPASS) updateCompass();
    else if (currentApp == APP_ACCEL) updateAccel();
    else if (currentApp == APP_CLOCK) drawClock();
    else if (currentApp == APP_PONG && pongGameActive) updatePong();
    else if (currentApp == APP_SPACESHOOTER && shooterGameActive) updateSpaceShooter();
  }

  // Web message overlay (always on top)
  if (messageActive) {
    drawWebMessage();
  }

  // Cursor - only redraw if moved
  if (newCursorX != cursorX || newCursorY != cursorY) {
    // Erase old cursor position
    if (lastCursorX >= 0) {
      int x = lastCursorX, y = lastCursorY;
      tft.drawLine(x - CURSOR_SIZE, y, x + CURSOR_SIZE, y, C_BG);
      tft.drawLine(x, y - CURSOR_SIZE, x, y + CURSOR_SIZE, C_BG);
    }
    cursorX = newCursorX; cursorY = newCursorY;
    drawCursor(cursorX, cursorY);
    lastCursorX = cursorX; lastCursorY = cursorY;
  }

  updateSpeaker();
  frameCount++;
  
  // FPS limiting
  uint32_t frameTime = millis() - loopStart;
  uint32_t targetFrame = 1000 / target_fps;
  if (frameTime < targetFrame) delay(targetFrame - frameTime);
}
