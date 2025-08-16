/*
  Zippy: HTTP control + WebSocket telemetry + basic obstacle avoidance
  Board: Arduino Uno R4 WiFi
  Motors: Adafruit Motor Shield (M1..M4)
  UltraSonic: HC-SR04 (TRIG=D7, ECHO=D6)
  WiFi: WiFiS3
  WebSocket: arduinoWebSockets (Server on :81)
*/

#include <WiFiS3.h>
#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include <ArduinoWebsockets.h>

#include "secrets.h" // provides ssid, password

/* ------------------------ Network ------------------------ */
int status = WL_IDLE_STATUS;
WiFiServer httpServer(80);
WebSocketsServer wsServer = WebSocketsServer(81);

/* ------------------------ Motors ------------------------- */
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorFL = AFMS.getMotor(1);
Adafruit_DCMotor *motorFR = AFMS.getMotor(2);
Adafruit_DCMotor *motorBL = AFMS.getMotor(3);
Adafruit_DCMotor *motorBR = AFMS.getMotor(4);

/* ---------------------- Ultrasonic ----------------------- */
const uint8_t PIN_TRIG = 7;
const uint8_t PIN_ECHO = 6;

/* --------------------- Control State --------------------- */
// Desired inputs from app (range roughly -255..255)
volatile int16_t cmdForward = 0;
volatile int16_t cmdStrafe  = 0;
volatile int16_t cmdTurn    = 0;

// Tank mode inputs
volatile int16_t cmdLeft  = 0;
volatile int16_t cmdRight = 0;

bool useTankMode = false;
bool autonomousMode = false; // can toggle with /mode?autonomous=1

// Telemetry
float lastDistanceCm = NAN;

/* -------------------- Timing (non-block) ----------------- */
uint32_t tLastSonar     = 0; // every ~50ms
uint32_t tLastTelemetry = 0; // every ~100ms
uint32_t tLastControl   = 0; // every ~20ms

/* -------------------- Forward Declares ------------------- */
void printWifiStatus();
int parseParam(String req, String key);
void moveMecanum(int forward, int strafe, int turn);
void moveTank(int leftSpeed, int rightSpeed);
void stopMotors();
float readDistanceCm();
float median3(float a, float b, float c);
void applyAutonomyAndDrive();

/* -------------------- WebSocket Events ------------------- */
void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      // Send hello
      wsServer.sendTXT(num, "{\"hello\":\"zippy\",\"type\":\"telemetry\"}");
      break;
    case WStype_TEXT:
      // (Optional) accept simple commands via WS in future
      break;
    default:
      break;
  }
}

/* ========================== Setup ======================== */
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println("Zippy: HTTP + WebSocket + Autonomy starting...");

  // WiFi
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("No WiFi module.");
    while (true) {}
  }
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, password);
    delay(5000);
  }
  httpServer.begin();
  printWifiStatus();

  // WebSocket
  wsServer.begin();
  wsServer.onEvent(onWsEvent);

  // Motors
  AFMS.begin();

  // Sonar
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Small delay for stability
  delay(200);
}

/* =========================== Loop ======================== */
void loop() {
  // 1) Handle HTTP commands
  WiFiClient client = httpServer.available();
  if (client) {
    String req = client.readStringUntil('\r');
    client.flush();

    String message = "OK";
    bool respondJSON = false;

    if (req.indexOf("GET /move") >= 0) {
      useTankMode = false;
      cmdForward = parseParam(req, "forward");
      cmdStrafe  = parseParam(req, "strafe");
      cmdTurn    = parseParam(req, "turn");
    } else if (req.indexOf("GET /tank") >= 0) {
      useTankMode = true;
      cmdLeft  = parseParam(req, "left");
      cmdRight = parseParam(req, "right");
    } else if (req.indexOf("GET /stop") >= 0) {
      cmdForward = cmdStrafe = cmdTurn = 0;
      cmdLeft = cmdRight = 0;
      stopMotors();
    } else if (req.indexOf("GET /ping") >= 0) {
      message = "{\"device\":\"Zippy\"}";
      respondJSON = true;
    } else if (req.indexOf("GET /mode") >= 0) {
      int a = parseParam(req, "autonomous");
      autonomousMode = (a == 1);
      message = String("{\"autonomous\":") + (autonomousMode ? "true" : "false") + "}";
      respondJSON = true;
    }

    client.println("HTTP/1.1 200 OK");
    client.println(respondJSON ? "Content-Type: application/json" : "Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println(message);
    delay(1);
    client.stop();
  }

  // 2) Update WebSocket server
  wsServer.loop();

  // 3) Periodic sonar read (~20Hz)
  uint32_t now = millis();
  if (now - tLastSonar >= 50) {
    tLastSonar = now;
    float a = readDistanceCm(); delay(2);
    float b = readDistanceCm(); delay(2);
    float c = readDistanceCm();
    lastDistanceCm = median3(a, b, c);
  }

  // 4) Periodic telemetry (~10Hz)
  if (now - tLastTelemetry >= 100) {
    tLastTelemetry = now;
    // Broadcast JSON: {"ts":123456,"cm":42.7}
    String payload = "{\"ts\":";
    payload += String(now);
    payload += ",\"cm\":";
    if (isnan(lastDistanceCm)) {
      payload += "null";
    } else {
      payload += String(lastDistanceCm, 1);
    }
    payload += "}";
    wsServer.broadcastTXT(payload);
  }

  // 5) Control tick (~50Hz -> every 20ms)
  if (now - tLastControl >= 20) {
    tLastControl = now;
    applyAutonomyAndDrive();
  }
}

/* ====================== Helpers/Logic ==================== */

void printWifiStatus() {
  Serial.print("SSID: "); Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: "); Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("RSSI: "); Serial.print(rssi); Serial.println(" dBm");
}

int parseParam(String req, String key) {
  int index = req.indexOf(key + "=");
  if (index == -1) return 0;
  int start = index + key.length() + 1;
  int end = req.indexOf('&', start);
  if (end == -1) end = req.indexOf(' ', start);
  return req.substring(start, end).toInt();
}

float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long d = pulseIn(PIN_ECHO, HIGH, 18000); // ~3m cap
  if (d == 0) return NAN;
  return d / 58.0;
}

float median3(float a, float b, float c) {
  // handle NANs crudely: if any are NAN, return the non-NAN if only one, else NAN
  int nanCount = (int)isnan(a) + (int)isnan(b) + (int)isnan(c);
  if (nanCount >= 2) return NAN;
  if (nanCount == 1) {
    if (isnan(a)) return (b + c) * 0.5f;
    if (isnan(b)) return (a + c) * 0.5f;
    return (a + b) * 0.5f;
  }
  // sort three
  if (a > b) { float t=a; a=b; b=t; }
  if (b > c) { float t=b; b=c; c=t; }
  if (a > b) { float t=a; a=b; b=t; }
  return b;
}

/* ----- Autonomy overlay: speed limit + stop/back/turn ----- */
void applyAutonomyAndDrive() {
  int f = cmdForward, s = cmdStrafe, t = cmdTurn;

  if (useTankMode) {
    // Apply autonomy to tank as well (symmetric)
    int L = cmdLeft, R = cmdRight;

    if (autonomousMode) {
      if (!isnan(lastDistanceCm)) {
        if (lastDistanceCm < 25.0f) {
          // Emergency: stop, short reverse, turn, then zero commands
          moveTank(0, 0);
          delay(50);
          moveTank(-120, -120);
          delay(120);
          // random-ish small turn (deterministic here; replace with simple L/R toggle if desired)
          moveTank(140, -140);
          delay(160);
          L = R = 0;
        } else if (lastDistanceCm < 50.0f) {
          // Throttle forward components
          L = (int)(L * 0.4f);
          R = (int)(R * 0.4f);
        }
      }
    }
    moveTank(L, R);
    return;
  }

  // Mecanum mode
  if (autonomousMode) {
    if (!isnan(lastDistanceCm)) {
      if (lastDistanceCm < 25.0f) {
        // Emergency evasive
        moveMecanum(0, 0, 0);
        delay(50);
        moveMecanum(-140, 0, 0);  // short reverse
        delay(120);
        moveMecanum(0, 0, 160);   // rotate
        delay(160);
        f = s = t = 0;            // clear desired motion
      } else if (lastDistanceCm < 50.0f) {
        // Soft limit
        f = (int)(f * 0.4f);
        s = (int)(s * 0.6f);
        t = (int)(t * 0.6f);
      }
    }
  }

  moveMecanum(f, s, t);
}

void moveMecanum(int forward, int strafe, int turn) {
  int fl = forward + strafe + turn;
  int fr = forward - strafe - turn;
  int bl = forward - strafe + turn;
  int br = forward + strafe - turn;

  int maxVal = max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
  if (maxVal > 255) {
    fl = map(fl, -maxVal, maxVal, -255, 255);
    fr = map(fr, -maxVal, maxVal, -255, 255);
    bl = map(bl, -maxVal, maxVal, -255, 255);
    br = map(br, -maxVal, maxVal, -255, 255);
  }

  motorFL->setSpeed(abs(fl));
  motorFR->setSpeed(abs(fr));
  motorBL->setSpeed(abs(bl));
  motorBR->setSpeed(abs(br));

  motorFL->run(fl >= 0 ? FORWARD : BACKWARD);
  motorFR->run(fr >= 0 ? FORWARD : BACKWARD);
  motorBL->run(bl >= 0 ? FORWARD : BACKWARD);
  motorBR->run(br >= 0 ? FORWARD : BACKWARD);
}

void moveTank(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motorFL->setSpeed(abs(leftSpeed));
  motorBL->setSpeed(abs(leftSpeed));
  motorFR->setSpeed(abs(rightSpeed));
  motorBR->setSpeed(abs(rightSpeed));

  motorFL->run(leftSpeed  >= 0 ? FORWARD : BACKWARD);
  motorBL->run(leftSpeed  >= 0 ? FORWARD : BACKWARD);
  motorFR->run(rightSpeed >= 0 ? FORWARD : BACKWARD);
  motorBR->run(rightSpeed >= 0 ? FORWARD : BACKWARD);
}

void stopMotors() {
  motorFL->run(RELEASE);
  motorFR->run(RELEASE);
  motorBL->run(RELEASE);
  motorBR->run(RELEASE);
}