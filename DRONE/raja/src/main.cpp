// ================= ESP32 DRONE CONTROL - UDP ONLY VERSION =================
// Lightweight UDP-only control system

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>

// ==================== CONFIGURATION ====================
// Network Configuration
const char* ssid = "ESP32_Drone";
const char* password = "12345678";
#define UDP_PORT 8888

// Motor Configuration
#define MOTOR_FL 27  // Front Left
#define MOTOR_FR 25  // Front Right
#define MOTOR_RL 26  // Rear Left
#define MOTOR_RR 14  // Rear Right
#define MOTOR_IDLE 1100
#define MOTOR_MIN 1000
#define MOTOR_MAX 1900

// Sensor Pins
#define ULTRA_TRIG 12
#define ULTRA_ECHO 13
#define GPS_RX 16
#define GPS_TX 17
#define BATTERY_PIN 34

// LED Configuration
#define STRIP_PIN 5
#define STRIP_COUNT 306
#define STRIP_BRIGHTNESS 200

// Safety Limits
#define MAX_TILT_ANGLE 45.0
#define LOW_BATTERY_THRESHOLD 3.5
#define BATTERY_CELLS 3
#define FAILSAFE_TIMEOUT 1000

// ==================== HARDWARE OBJECTS ====================
WiFiUDP udp;
Servo escFL, escFR, escRL, escRR;
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_NeoPixel strip(STRIP_COUNT, STRIP_PIN, NEO_GRB + NEO_KHZ800);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ==================== FLIGHT STATE ====================
bool isArmed = false;
String flightMode = "DISARMED";
bool emergencyStop = false;

// Control Inputs
int throttleIn = 1150;
int rollIn = 1500;
int pitchIn = 1500;
int yawIn = 1500;

// ==================== SENSOR DATA ====================
// IMU
float rollAngle = 0, pitchAngle = 0, yawAngle = 0;
float gyroRateX = 0, gyroRateY = 0, gyroRateZ = 0;
float accelX = 0, accelY = 0, accelZ = 0;

// GPS
double gpsLat = 0, gpsLon = 0;
float gpsAlt = 0;
int gpsSats = 0;
bool hasGPSFix = false;
float droneSpeed = 0;

// Sensors
float sonarAltitude = 0;
float batteryVoltage = 11.1;
bool lowBattery = false;

// Motor Outputs
int m1, m2, m3, m4;

// ==================== ALTITUDE HOLD ====================
bool altHoldEnabled = false;
float altHoldTarget = 0;

// ==================== PID CONTROL ====================
// Roll PID
float rollKp = 1.8, rollKi = 0.02, rollKd = 0.05;
float rollPID = 0, rollError = 0, rollErrorSum = 0, prevRollError = 0;

// Pitch PID
float pitchKp = 1.8, pitchKi = 0.02, pitchKd = 0.05;
float pitchPID = 0, pitchError = 0, pitchErrorSum = 0, prevPitchError = 0;

// Yaw PID
float yawKp = 2.0, yawKi = 0.01, yawKd = 0.0;
float yawPID = 0, yawError = 0, yawErrorSum = 0, prevYawError = 0;

// Altitude PID
float altKp = 1.0, altKi = 0.05, altKd = 0.3;
float altPID = 0, altError = 0, altErrorSum = 0, prevAltError = 0;

#define MAX_I_TERM 100

// ==================== TIMING ====================
unsigned long lastPacketTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastPIDTime = 0;
unsigned long lastIMUTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastGPSRead = 0;
unsigned long lastSonarRead = 0;
unsigned long lastStripUpdate = 0;
unsigned long lastBatteryRead = 0;
unsigned long lastSerialDebug = 0;
unsigned long lastWiFiCheck = 0;

#define TELEMETRY_INTERVAL 100
#define DISPLAY_RATE 200
#define GPS_RATE 100
#define SONAR_RATE 50
#define SERIAL_DEBUG_RATE 500

// ==================== CONNECTION STATE ====================
bool udpConnected = false;
uint32_t packetCount = 0;

// ==================== LED STRIP ====================
enum StripMode { STRIP_OFF, STRIP_RAINBOW, STRIP_PULSE, STRIP_FLASH, STRIP_STATUS, STRIP_REDBLUE };
StripMode stripMode = STRIP_REDBLUE;
int stripOffset = 0;

// ==================== RADAR ANIMATION ====================
int radarPulseRadius = 2;
int radarScanAngle = 0;
int radarSignalBars = 0;

// ==================== FUNCTION DECLARATIONS ====================
void setupHardware();
void setupNetwork();
void receiveUDP();
void sendTelemetry();
void readIMU();
void readGPS();
void readSonar();
void readBattery();
void computePID();
void mixMotors();
void setMotors(int fl, int fr, int rl, int rr);
void emergencyLanding();
void resetPIDIntegrals();
void drawRadarUI();
void handleStrip();
void redBluePattern();
void printSerialDebug();

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  ESP32 Quadcopter Flight Controller  ║");
  Serial.println("║       UDP Control System Only         ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  setupHardware();
  setupNetwork();
  
  Serial.println("\n✓ SYSTEM READY FOR FLIGHT\n");
  Serial.println("════════════════════════════════════════\n");
}

// =============================================================
// MAIN LOOP
// =============================================================
void loop() {
  unsigned long currentMillis = millis();
  
  // Handle incoming communications
  receiveUDP();
  
  // Read sensors
  if (currentMillis - lastIMUTime >= 4) {  // 250Hz
    lastIMUTime = currentMillis;
    readIMU();
  }
  
  if (currentMillis - lastGPSRead >= GPS_RATE) {
    lastGPSRead = currentMillis;
    readGPS();
  }
  
  if (currentMillis - lastSonarRead >= SONAR_RATE) {
    lastSonarRead = currentMillis;
    readSonar();
  }
  
  if (currentMillis - lastBatteryRead >= 1000) {
    lastBatteryRead = currentMillis;
    readBattery();
  }
  
  // Failsafe check
  if (udpConnected && currentMillis - lastPacketTime > FAILSAFE_TIMEOUT) {
    Serial.println("⚠ FAILSAFE: Connection lost");
    udpConnected = false;
    isArmed = false;
    emergencyStop = true;
  }
  
  // Emergency conditions
  if (lowBattery || emergencyStop || abs(rollAngle) > MAX_TILT_ANGLE || abs(pitchAngle) > MAX_TILT_ANGLE) {
    if (!emergencyStop) {
      Serial.println("⚠ EMERGENCY: Excessive tilt or low battery!");
      emergencyStop = true;
    }
    emergencyLanding();
  }
  // Normal flight operation
  else if (isArmed) {
    mixMotors();
  } 
  else {
    setMotors(MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN);
    resetPIDIntegrals();
  }
  
  // Display updates
  if (currentMillis - lastDisplayUpdate >= DISPLAY_RATE) {
    lastDisplayUpdate = currentMillis;
    drawRadarUI();
  }
  
  // LED updates
  if (currentMillis - lastStripUpdate >= 100) {
    lastStripUpdate = currentMillis;
    handleStrip();
  }
  
  // Send telemetry for UDP clients
  if (udpConnected && currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = currentMillis;
    sendTelemetry();
  }
  
  // Serial debug output
  if (currentMillis - lastSerialDebug >= SERIAL_DEBUG_RATE) {
    lastSerialDebug = currentMillis;
    printSerialDebug();
  }
  
  // WiFi status check every 5 seconds
  if (currentMillis - lastWiFiCheck >= 5000) {
    lastWiFiCheck = currentMillis;
    int numStations = WiFi.softAPgetStationNum();
    if (numStations > 0) {
      Serial.print("📱 Connected Devices: ");
      Serial.println(numStations);
    }
  }
  
  delay(2);
}

// =============================================================
// HARDWARE SETUP
// =============================================================
void setupHardware() {
  Serial.println("→ Initializing Hardware...\n");
  
  Wire.begin(21, 22);
  
  Serial.print("  ESCs... ");
  escFL.attach(MOTOR_FL, MOTOR_MIN, MOTOR_MAX);
  escFR.attach(MOTOR_FR, MOTOR_MIN, MOTOR_MAX);
  escRL.attach(MOTOR_RL, MOTOR_MIN, MOTOR_MAX);
  escRR.attach(MOTOR_RR, MOTOR_MIN, MOTOR_MAX);
  
  setMotors(MOTOR_MAX, MOTOR_MAX, MOTOR_MAX, MOTOR_MAX);
  delay(2000);
  setMotors(MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN);
  delay(1000);
  Serial.println("✓");
  
  Serial.print("  MPU6050... ");
  if (!mpu.begin()) {
    Serial.println("✗ FAILED!");
    while(1) { delay(1000); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("✓");
  
  Serial.print("  GPS... ");
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("✓");
  
  Serial.print("  Sonar... ");
  pinMode(ULTRA_TRIG, OUTPUT);
  pinMode(ULTRA_ECHO, INPUT);
  Serial.println("✓");
  
  Serial.print("  OLED... ");
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("QUADCOPTER");
    display.println("UDP CONTROL");
    display.println("Initializing...");
    display.display();
    Serial.println("✓");
  } else {
    Serial.println("✗ Not Found");
  }
  
  Serial.print("  LED Strip... ");
  strip.begin();
  strip.setBrightness(STRIP_BRIGHTNESS);
  strip.clear();
  strip.show();
  Serial.println("✓");
  
  Serial.print("  Battery Monitor... ");
  pinMode(BATTERY_PIN, INPUT);
  Serial.println("✓");
  
  Serial.println();
}

// =============================================================
// NETWORK SETUP
// =============================================================
void setupNetwork() {
  Serial.println("→ Starting Network Services...\n");
  
  // Disconnect any previous WiFi connections
  WiFi.disconnect(true);
  delay(100);
  
  // Set WiFi mode to AP only
  WiFi.mode(WIFI_AP);
  delay(100);
  
  // Configure AP with specific settings
  WiFi.softAPConfig(
    IPAddress(192, 168, 4, 1),    // AP IP
    IPAddress(192, 168, 4, 1),    // Gateway
    IPAddress(255, 255, 255, 0)   // Subnet
  );
  
  // Start Access Point with explicit channel and visibility
  // Channel 1, hidden=false, max connections=4
  bool apStarted = WiFi.softAP(ssid, password, 1, false, 4);
  
  if (!apStarted) {
    Serial.println("  ✗ FAILED to start AP!");
    Serial.println("  Retrying with no password...");
    apStarted = WiFi.softAP(ssid, "", 1, false, 4);
  }
  
  if (apStarted) {
    Serial.println("  ✓ Access Point Started Successfully!");
  } else {
    Serial.println("  ✗ CRITICAL: AP Failed to Start!");
  }
  
  delay(1000);
  
  IPAddress IP = WiFi.softAPIP();
  
  Serial.println("  ┌─────────────────────────────────┐");
  Serial.println("  │   WiFi Access Point Active     │");
  Serial.println("  └─────────────────────────────────┘");
  Serial.print("    SSID:       ");
  Serial.println(ssid);
  Serial.print("    Password:   ");
  Serial.println(password);
  Serial.print("    IP Address: ");
  Serial.println(IP);
  Serial.print("    Channel:    1");
  Serial.println();
  Serial.print("    Max Clients: 4");
  Serial.println();
  Serial.print("    UDP Port:   ");
  Serial.println(UDP_PORT);
  Serial.print("    Stations:   ");
  Serial.println(WiFi.softAPgetStationNum());
  Serial.println();
  
  udp.begin(UDP_PORT);
  Serial.println("  ✓ UDP Server Started on port " + String(UDP_PORT));
  Serial.println();
  
  // Print MAC address for debugging
  Serial.print("  AP MAC Address: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.println();
}

// =============================================================
// UDP COMMUNICATION
// =============================================================
void receiveUDP() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  char buf[128];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len] = '\0';

  lastPacketTime = millis();
  udpConnected = true;
  packetCount++;

  Serial.print("📥 UDP RX [");
  Serial.print(packetCount);
  Serial.print("]: ");
  Serial.println(buf);

  String packet = String(buf);
  packet.trim();

  // Strip commands
  if (packet.startsWith("STRIP:")) {
    if (packet == "STRIP:OFF") {
      stripMode = STRIP_OFF;
      Serial.println("   → Strip: OFF");
    }
    else if (packet == "STRIP:RAINBOW") {
      stripMode = STRIP_RAINBOW;
      Serial.println("   → Strip: RAINBOW");
    }
    else if (packet == "STRIP:PULSE") {
      stripMode = STRIP_PULSE;
      Serial.println("   → Strip: PULSE");
    }
    else if (packet == "STRIP:FLASH") {
      stripMode = STRIP_FLASH;
      Serial.println("   → Strip: FLASH");
    }
    else if (packet == "STRIP:STATUS") {
      stripMode = STRIP_STATUS;
      Serial.println("   → Strip: STATUS");
    }
    else if (packet == "STRIP:REDBLUE") {
      stripMode = STRIP_REDBLUE;
      Serial.println("   → Strip: RED/BLUE");
    }
    return;
  }

  // Altitude hold commands
  if (packet == "ALTHOLD:ON") {
    altHoldEnabled = true;
    Serial.println("   → Altitude Hold: ENABLED");
    return;
  }
  if (packet == "ALTHOLD:OFF") {
    altHoldEnabled = false;
    altHoldTarget = 0;
    Serial.println("   → Altitude Hold: DISABLED");
    return;
  }

  // Emergency commands
  if (packet == "EMERGENCY") {
    emergencyStop = true;
    Serial.println("   → 🚨 EMERGENCY STOP ACTIVATED");
    return;
  }

  if (packet == "RESET") {
    emergencyStop = false;
    isArmed = false;
    Serial.println("   → ✓ System Reset");
    return;
  }

  // Control inputs (PWM format: throttle,roll,pitch,yaw,arm)
  int temp_throttle, temp_roll, temp_pitch, temp_yaw, temp_arm;
  if (sscanf(buf, "%d,%d,%d,%d,%d", &temp_throttle, &temp_roll, &temp_pitch, &temp_yaw, &temp_arm) == 5) {
    throttleIn = constrain(temp_throttle, MOTOR_MIN, MOTOR_MAX);
    rollIn = constrain(temp_roll, 1000, 2000);
    pitchIn = constrain(temp_pitch, 1000, 2000);
    yawIn = constrain(temp_yaw, 1000, 2000);
    isArmed = (temp_arm == 1);
    
    if (isArmed) {
      flightMode = "ARMED";
    } else {
      flightMode = "DISARMED";
    }
    
    Serial.print("   → Controls: T=");
    Serial.print(throttleIn);
    Serial.print(" R=");
    Serial.print(rollIn);
    Serial.print(" P=");
    Serial.print(pitchIn);
    Serial.print(" Y=");
    Serial.print(yawIn);
    Serial.print(" Armed=");
    Serial.println(isArmed);
  }
}

void sendTelemetry() {
  if (!udpConnected) return;
  
  char telemetry[256];
  snprintf(telemetry, sizeof(telemetry),
           "T:%d,R:%.1f,P:%.1f,Y:%.1f,M1:%d,M2:%d,M3:%d,M4:%d,B:%.2f,A:%.2f,GPS:%d,S:%.1f,LAT:%.6f,LON:%.6f",
           throttleIn, rollAngle, pitchAngle, yawAngle, 
           m1, m2, m3, m4, batteryVoltage, sonarAltitude, gpsSats, droneSpeed, gpsLat, gpsLon);
  
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print(telemetry);
  udp.endPacket();
}

// =============================================================
// SENSOR READING
// =============================================================
void readIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;

  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  gyroRateX = g.gyro.x * 57.3;
  gyroRateY = g.gyro.y * 57.3;
  gyroRateZ = g.gyro.z * 57.3;

  float accelRoll = atan2(accelY, accelZ) * 57.3;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.3;

  float alpha = 0.98;
  rollAngle = alpha * (rollAngle + gyroRateX * dt) + (1.0 - alpha) * accelRoll;
  pitchAngle = alpha * (pitchAngle + gyroRateY * dt) + (1.0 - alpha) * accelPitch;
}

void readGPS() {
  static bool gpsDataReceived = false;
  
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (!gpsDataReceived) {
        Serial.println("📡 GPS: Data stream detected");
        gpsDataReceived = true;
      }
      
      if (gps.location.isValid()) {
        gpsLat = gps.location.lat();
        gpsLon = gps.location.lng();
        if (!hasGPSFix) {
          Serial.println("✓ GPS: Lock acquired!");
        }
        hasGPSFix = true;
      }
      if (gps.altitude.isValid()) {
        gpsAlt = gps.altitude.meters();
      }
      if (gps.satellites.isValid()) {
        gpsSats = gps.satellites.value();
      }
      if (gps.speed.isValid()) {
        droneSpeed = gps.speed.mps();
      }
    }
  }
  
  if (millis() - gps.location.age() > 2000) {
    if (hasGPSFix) {
      Serial.println("⚠ GPS: Lock lost");
    }
    hasGPSFix = false;
  }
}

void readSonar() {
  digitalWrite(ULTRA_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG, LOW);
  
  long duration = pulseIn(ULTRA_ECHO, HIGH, 30000);
  
  if (duration > 0) {
    float dist = duration * 0.034 / 2.0 / 100.0;
    if (dist >= 0.2 && dist <= 4.0) {
      sonarAltitude = sonarAltitude * 0.7 + dist * 0.3;
    }
  }
}

void readBattery() {
  static bool lowBatteryWarned = false;
  
  int rawValue = analogRead(BATTERY_PIN);
  float voltage = (rawValue / 4095.0) * 3.3 * (BATTERY_CELLS + 1);
  batteryVoltage = batteryVoltage * 0.9 + voltage * 0.1;
  
  bool wasLowBattery = lowBattery;
  lowBattery = (batteryVoltage / BATTERY_CELLS) < LOW_BATTERY_THRESHOLD;
  
  if (lowBattery && !lowBatteryWarned) {
    Serial.println("🔋 WARNING: Low Battery!");
    lowBatteryWarned = true;
  }
  
  if (!lowBattery && lowBatteryWarned) {
    lowBatteryWarned = false;
  }
}

// =============================================================
// PID CONTROL
// =============================================================
void computePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  if (dt <= 0 || dt > 0.1) {
    lastPIDTime = now;
    return;
  }
  lastPIDTime = now;

  int baseThrottle = throttleIn;
  if (altHoldEnabled && sonarAltitude > 0.2) {
    if (altHoldTarget == 0) {
      altHoldTarget = sonarAltitude;
    }
    
    altError = altHoldTarget - sonarAltitude;
    altErrorSum += altError * dt;
    altErrorSum = constrain(altErrorSum, -MAX_I_TERM, MAX_I_TERM);
    float altD = (altError - prevAltError) / dt;
    altPID = altKp * altError + altKi * altErrorSum + altKd * altD;
    prevAltError = altError;
    
    baseThrottle = throttleIn + (int)altPID;
    baseThrottle = constrain(baseThrottle, MOTOR_IDLE, MOTOR_MAX);
  }

  float rollSetpoint = (rollIn - 1500) / 15.0;
  float pitchSetpoint = (pitchIn - 1500) / 15.0;

  rollError = rollSetpoint - rollAngle;
  rollErrorSum += rollError * dt;
  rollErrorSum = constrain(rollErrorSum, -MAX_I_TERM, MAX_I_TERM);
  float rollD = (rollError - prevRollError) / dt;
  rollPID = rollKp * rollError + rollKi * rollErrorSum + rollKd * rollD;
  prevRollError = rollError;

  pitchError = pitchSetpoint - pitchAngle;
  pitchErrorSum += pitchError * dt;
  pitchErrorSum = constrain(pitchErrorSum, -MAX_I_TERM, MAX_I_TERM);
  float pitchD = (pitchError - prevPitchError) / dt;
  pitchPID = pitchKp * pitchError + pitchKi * pitchErrorSum + pitchKd * pitchD;
  prevPitchError = pitchError;

  float yawRate = (yawIn - 1500) / 5.0;
  yawError = yawRate - gyroRateZ;
  yawErrorSum += yawError * dt;
  yawErrorSum = constrain(yawErrorSum, -MAX_I_TERM / 2, MAX_I_TERM / 2);
  float yawD = (yawError - prevYawError) / dt;
  yawPID = yawKp * yawError + yawKi * yawErrorSum + yawKd * yawD;
  prevYawError = yawError;

  rollPID = constrain(rollPID, -400, 400);
  pitchPID = constrain(pitchPID, -400, 400);
  yawPID = constrain(yawPID, -200, 200);
}

// =============================================================
// MOTOR MIXING
// =============================================================
void mixMotors() {
  computePID();

  m1 = throttleIn + pitchPID + rollPID - yawPID;
  m2 = throttleIn + pitchPID - rollPID + yawPID;
  m3 = throttleIn - pitchPID - rollPID - yawPID;
  m4 = throttleIn - pitchPID + rollPID + yawPID;

  m1 = constrain(m1, MOTOR_IDLE, MOTOR_MAX);
  m2 = constrain(m2, MOTOR_IDLE, MOTOR_MAX);
  m3 = constrain(m3, MOTOR_IDLE, MOTOR_MAX);
  m4 = constrain(m4, MOTOR_IDLE, MOTOR_MAX);

  setMotors(m1, m2, m3, m4);
}

void setMotors(int fl, int fr, int rl, int rr) {
  escFL.writeMicroseconds(fl);
  escFR.writeMicroseconds(fr);
  escRL.writeMicroseconds(rl);
  escRR.writeMicroseconds(rr);
}

void emergencyLanding() {
  static unsigned long emergencyStartTime = millis();
  static int descendThrottle = throttleIn;
  
  if (millis() - emergencyStartTime > 20) {
    descendThrottle -= 2;
    if (descendThrottle < MOTOR_IDLE) {
      descendThrottle = MOTOR_MIN;
      isArmed = false;
    }
    emergencyStartTime = millis();
  }
  
  computePID();
  
  m1 = descendThrottle + pitchPID + rollPID;
  m2 = descendThrottle + pitchPID - rollPID;
  m3 = descendThrottle - pitchPID - rollPID;
  m4 = descendThrottle - pitchPID + rollPID;
  
  m1 = constrain(m1, MOTOR_MIN, MOTOR_MAX);
  m2 = constrain(m2, MOTOR_MIN, MOTOR_MAX);
  m3 = constrain(m3, MOTOR_MIN, MOTOR_MAX);
  m4 = constrain(m4, MOTOR_MIN, MOTOR_MAX);
  
  setMotors(m1, m2, m3, m4);
}

void resetPIDIntegrals() {
  rollErrorSum = 0;
  pitchErrorSum = 0;
  yawErrorSum = 0;
  altErrorSum = 0;
  altHoldTarget = 0;
}

// =============================================================
// DISPLAY & LED
// =============================================================
void drawRadarUI() {
  display.clearDisplay();
  int cx = 40;
  int cy = 32;
  
  display.drawCircle(cx, cy, 10, SSD1306_WHITE);
  display.drawCircle(cx, cy, 20, SSD1306_WHITE);
  display.drawCircle(cx, cy, 30, SSD1306_WHITE);
  
  display.drawCircle(cx, cy, radarPulseRadius, SSD1306_WHITE);
  radarPulseRadius++;
  if (radarPulseRadius > 30) radarPulseRadius = 2;
  
  float rad = radarScanAngle * 0.01745;
  int x = cx + 30 * cos(rad);
  int y = cy + 30 * sin(rad);
  display.drawLine(cx, cy, x, y, SSD1306_WHITE);
  radarScanAngle += 7;
  if (radarScanAngle >= 360) radarScanAngle = 0;
  
  for (int i = 0; i < radarSignalBars; i++) {
    display.fillRect(90 + i * 6, 52 - i * 6, 4, i * 6, SSD1306_WHITE);
  }
  radarSignalBars++;
  if (radarSignalBars > 4) radarSignalBars = 0;
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(80, 5);
  display.print("UDP");
  
  display.setCursor(80, 15);
  if (isArmed) {
    display.print("ARMED");
  } else {
    display.print("SAFE");
  }
  
  display.setCursor(80, 25);
  if (udpConnected) {
    display.print("LINK");
  } else {
    display.print("----");
  }
  
  display.setCursor(80, 35);
  display.print(batteryVoltage, 1);
  display.print("V");
  
  display.setCursor(80, 45);
  display.print(sonarAltitude, 1);
  display.print("m");
  
  display.display();
}

void handleStrip() {
  switch (stripMode) {
    case STRIP_REDBLUE:
      redBluePattern();
      break;
      
    case STRIP_RAINBOW:
      for (int i = 0; i < STRIP_COUNT; i++) {
        int hue = (stripOffset + i * 256 / STRIP_COUNT) % 256;
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(hue * 256)));
      }
      strip.show();
      stripOffset = (stripOffset + 1) % 256;
      break;
      
    case STRIP_PULSE:
      {
        uint8_t brightness = (sin(stripOffset * 0.05) + 1) * 127;
        for (int i = 0; i < STRIP_COUNT; i++) {
          strip.setPixelColor(i, strip.Color(brightness, brightness, brightness));
        }
        strip.show();
        stripOffset++;
      }
      break;
      
    case STRIP_FLASH:
      {
        uint32_t color = (stripOffset % 2) ? strip.Color(255, 0, 0) : strip.Color(0, 0, 255);
        strip.fill(color);
        strip.show();
        stripOffset++;
      }
      break;
      
    case STRIP_STATUS:
      {
        uint32_t color;
        if (flightMode == "DISARMED") {
          color = strip.Color(0, 0, 255);
        } else if (flightMode == "ARMED") {
          static bool blinkState = false;
          blinkState = !blinkState;
          color = blinkState ? strip.Color(255, 255, 0) : strip.Color(50, 50, 0);
        } else if (flightMode == "FLYING") {
          color = strip.Color(0, 255, 0);
        } else if (flightMode == "FAILSAFE" || flightMode == "EMERGENCY") {
          static bool blinkState = false;
          blinkState = !blinkState;
          color = blinkState ? strip.Color(255, 0, 0) : strip.Color(0, 0, 0);
        } else {
          color = strip.Color(255, 255, 0);
        }
        strip.fill(color);
        strip.show();
      }
      break;
      
    case STRIP_OFF:
      strip.clear();
      strip.show();
      break;
  }
}

void redBluePattern() {
  for (int i = 0; i < STRIP_COUNT; i++) {
    int pos = (i + stripOffset) % 6;
    if (pos < 3) {
      strip.setPixelColor(i, strip.Color(255, 0, 0));
    } else {
      strip.setPixelColor(i, strip.Color(0, 0, 255));
    }
  }
  strip.show();
  stripOffset++;
  if (stripOffset >= 6) stripOffset = 0;
}

// =============================================================
// SERIAL DEBUG OUTPUT
// =============================================================
void printSerialDebug() {
  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.println("║              DRONE STATUS REPORT (UDP ONLY)               ║");
  Serial.println("╠════════════════════════════════════════════════════════════╣");
  
  Serial.print("║ Mode: ");
  Serial.print(flightMode);
  for (int i = flightMode.length(); i < 12; i++) Serial.print(" ");
  Serial.print("| Armed: ");
  Serial.print(isArmed ? "YES" : "NO ");
  Serial.print(" | Emergency: ");
  Serial.print(emergencyStop ? "YES" : "NO ");
  Serial.println(" ║");
  
  Serial.print("║ Roll: ");
  Serial.print(rollAngle, 1);
  Serial.print("° ");
  Serial.print(" | Pitch: ");
  Serial.print(pitchAngle, 1);
  Serial.print("° ");
  Serial.print(" | Yaw: ");
  Serial.print(yawAngle, 1);
  Serial.println("°        ║");
  
  Serial.print("║ Throttle: ");
  Serial.print(map(throttleIn, MOTOR_MIN, MOTOR_MAX, 0, 100));
  Serial.print("% | Roll: ");
  Serial.print(rollIn);
  Serial.print(" | Pitch: ");
  Serial.print(pitchIn);
  Serial.print(" | Yaw: ");
  Serial.print(yawIn);
  Serial.println("  ║");
  
  Serial.print("║ Motors: M1=");
  Serial.print(map(m1, MOTOR_MIN, MOTOR_MAX, 0, 100));
  Serial.print("% M2=");
  Serial.print(map(m2, MOTOR_MIN, MOTOR_MAX, 0, 100));
  Serial.print("% M3=");
  Serial.print(map(m3, MOTOR_MIN, MOTOR_MAX, 0, 100));
  Serial.print("% M4=");
  Serial.print(map(m4, MOTOR_MIN, MOTOR_MAX, 0, 100));
  Serial.println("%      ║");
  
  Serial.print("║ Altitude: ");
  Serial.print(sonarAltitude, 2);
  Serial.print("m | Battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V | Speed: ");
  Serial.print(droneSpeed, 1);
  Serial.println("m/s    ║");
  
  Serial.print("║ GPS: ");
  if (hasGPSFix) {
    Serial.print("LOCK (");
    Serial.print(gpsSats);
    Serial.print(" sats) | Lat: ");
    Serial.print(gpsLat, 6);
    Serial.print(" Lon: ");
    Serial.print(gpsLon, 6);
  } else {
    Serial.print("NO LOCK | Sats: ");
    Serial.print(gpsSats);
    for (int i = 0; i < 30; i++) Serial.print(" ");
  }
  Serial.println(" ║");
  
  Serial.print("║ PID: Roll=");
  Serial.print(rollPID, 1);
  Serial.print(" Pitch=");
  Serial.print(pitchPID, 1);
  Serial.print(" Yaw=");
  Serial.print(yawPID, 1);
  if (altHoldEnabled) {
    Serial.print(" Alt=");
    Serial.print(altPID, 1);
  }
  Serial.println("           ║");
  
  Serial.print("║ UDP Connection: ");
  Serial.print(udpConnected ? "✓ ACTIVE" : "✗ WAITING");
  Serial.print(" | Packets: ");
  Serial.print(packetCount);
  Serial.println("              ║");
  
  Serial.print("║ Strip Mode: ");
  switch(stripMode) {
    case STRIP_OFF: Serial.print("OFF"); break;
    case STRIP_RAINBOW: Serial.print("RAINBOW"); break;
    case STRIP_PULSE: Serial.print("PULSE"); break;
    case STRIP_FLASH: Serial.print("FLASH"); break;
    case STRIP_STATUS: Serial.print("STATUS"); break;
    case STRIP_REDBLUE: Serial.print("RED/BLUE SCROLL"); break;
  }
  Serial.println("                                  ║");
  
  if (altHoldEnabled) {
    Serial.print("║ Altitude Hold: ACTIVE | Target: ");
    Serial.print(altHoldTarget, 2);
    Serial.println("m                    ║");
  }
  
  Serial.println("╚════════════════════════════════════════════════════════════╝");
}