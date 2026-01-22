#include <Servo.h>

// ================= ANALOG SENSOR PINS =================
#define WATER_LOW_PIN   A0
#define TURB_PIN        A1
#define RAIN_PIN        A2
#define WATER_HIGH_PIN  A3

// ================= ULTRASONIC SENSORS =================
// Main (conveyor trigger)
#define ULTRA1_TRIG     34
#define ULTRA1_ECHO     35
// Bin-1
#define ULTRA2_TRIG     37
#define ULTRA2_ECHO     38
// Bin-2
#define ULTRA3_TRIG     39
#define ULTRA3_ECHO     40

// ================= METAL + SERVO =================
#define METAL_PIN       36     // unchanged
#define SERVO_PIN       9
Servo metalServo;

// ================= WIPER MOTOR (OUT1/OUT2) =================
#define WIPER_EN   5     // ENA (PWM)
#define WIPER_IN1  30
#define WIPER_IN2  31

// ================= CONVEYOR MOTOR (OUT3/OUT4) =================
#define CONV_EN    6     // ENB (PWM)
#define CONV_IN1   32    // IN3
#define CONV_IN2   33    // IN4

// ================= THRESHOLDS =================
#define WATER_LOW_TH        500
#define WATER_HIGH_TH       650
#define TURB_DIRTY_TH       500
#define RAIN_HEAVY_TH       500

#define ULTRA1_DISTANCE_TH  15   // cm (tune)
#define BIN_FULL_CM         8    // cm (tune)

// ================= WIPER SETTINGS =================
const unsigned long WIPER_INTERVAL_MS = 5000;
const unsigned long WIPER_ONE_ROT_MS  = 325;   // your tuned value
const int WIPER_SPEED = 90;

// ================= CONVEYOR SETTINGS =================
const int CONV_SPEED_SLOW = 110;        // slow running PWM (tune)
const int CONV_SPEED_KICK = 200;        // start kick PWM (must be higher)
const unsigned long CONV_KICK_MS = 250; // kick duration

const unsigned long CONV_MIN_RUN_MS = 10000;      // run minimum 10 sec
const unsigned long CONV_NO_DETECT_STOP_MS = 1200; // after min run, stop if no detect for 1.2s

// Ultrasonic noise filter (very important)
const int DETECT_HITS_TO_START = 3;   // need 3 hits to start
const int DETECT_HITS_MAX = 8;
static int detectHits = 0;

// ================= METAL SERVO SETTINGS =================
const int SERVO_THROW_DEG = 35;
const unsigned long SERVO_HOLD_MS = 450;
const unsigned long METAL_LOCKOUT = 3000;

// ================= PRINT SETTINGS =================
const unsigned long PRINT_INTERVAL = 300;

// ================= STATE VARIABLES =================
bool wiperRunning = false;
unsigned long wiperNextStart = 0;
unsigned long wiperStopAt = 0;

bool conveyorRunning = false;
unsigned long conveyorStartAt = 0;
unsigned long lastConveyorDetectAt = 0;
bool conveyorKicking = false;
unsigned long conveyorKickUntil = 0;

// servo state (non-blocking)
bool servoMoving = false;
unsigned long servoMoveAt = 0;
unsigned long lastMetalTrigger = 0;
bool lastMetalDetected = false;

unsigned long printTimer = 0;

// ---------- Ultrasonic read ----------
long readUltrasonicCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return -1;
  return (long)(duration * 0.034 / 2.0);
}

long readUltrasonicSafe(uint8_t trigPin, uint8_t echoPin) {
  long d = readUltrasonicCM(trigPin, echoPin);
  delayMicroseconds(3000); // reduce cross-talk
  return d;
}

bool isObjectDetected(long dist, int thresholdCm) {
  return (dist > 0 && dist <= thresholdCm);
}

// ---------- Conveyor helpers ----------
void conveyorForwardKick(unsigned long now) {
  // Direction forward
  digitalWrite(CONV_IN1, HIGH);
  digitalWrite(CONV_IN2, LOW);

  // Strong kick so motor ALWAYS starts
  analogWrite(CONV_EN, CONV_SPEED_KICK);
  conveyorKicking = true;
  conveyorKickUntil = now + CONV_KICK_MS;
}

void conveyorForwardSlow() {
  digitalWrite(CONV_IN1, HIGH);
  digitalWrite(CONV_IN2, LOW);
  analogWrite(CONV_EN, CONV_SPEED_SLOW);
}

void conveyorCoastStop() {
  // Coast stop (more reliable than brake on some setups)
  digitalWrite(CONV_IN1, LOW);
  digitalWrite(CONV_IN2, LOW);
  analogWrite(CONV_EN, 0);
}

void setup() {
  Serial.begin(9600);
  Serial.println("FINAL (HARD FIX): Reliable OUT3/OUT4 Conveyor + Wiper + 3x Ultrasonic + Metal Servo");

  // Motor pins
  pinMode(WIPER_EN, OUTPUT);
  pinMode(WIPER_IN1, OUTPUT);
  pinMode(WIPER_IN2, OUTPUT);

  pinMode(CONV_EN, OUTPUT);
  pinMode(CONV_IN1, OUTPUT);
  pinMode(CONV_IN2, OUTPUT);

  // Ensure stopped
  digitalWrite(WIPER_IN1, LOW);
  digitalWrite(WIPER_IN2, LOW);
  analogWrite(WIPER_EN, 0);

  conveyorCoastStop();

  // Ultrasonic pins
  pinMode(ULTRA1_TRIG, OUTPUT); pinMode(ULTRA1_ECHO, INPUT); digitalWrite(ULTRA1_TRIG, LOW);
  pinMode(ULTRA2_TRIG, OUTPUT); pinMode(ULTRA2_ECHO, INPUT); digitalWrite(ULTRA2_TRIG, LOW);
  pinMode(ULTRA3_TRIG, OUTPUT); pinMode(ULTRA3_ECHO, INPUT); digitalWrite(ULTRA3_TRIG, LOW);

  // Metal detector input stable
  pinMode(METAL_PIN, INPUT_PULLUP);

  // Servo
  metalServo.attach(SERVO_PIN);
  metalServo.write(0);

  unsigned long now = millis();
  wiperNextStart = now + WIPER_INTERVAL_MS;
  printTimer = now;
}

void loop() {
  unsigned long now = millis();

  // ================= READ ANALOG SENSORS =================
  int waterLow  = analogRead(WATER_LOW_PIN);
  int waterHigh = analogRead(WATER_HIGH_PIN);
  int turbValue = analogRead(TURB_PIN);
  int rainValue = analogRead(RAIN_PIN);

  bool blockageDetected = (waterLow > WATER_LOW_TH) && (turbValue < TURB_DIRTY_TH);
  bool overflowRisk     = (waterHigh > WATER_HIGH_TH) && (rainValue < RAIN_HEAVY_TH);

  // ================= READ ULTRASONICS =================
  long distMain = readUltrasonicSafe(ULTRA1_TRIG, ULTRA1_ECHO);
  long distBin1 = readUltrasonicSafe(ULTRA2_TRIG, ULTRA2_ECHO);
  long distBin2 = readUltrasonicSafe(ULTRA3_TRIG, ULTRA3_ECHO);

  bool mainHit = isObjectDetected(distMain, ULTRA1_DISTANCE_TH);

  // Debounce hits (prevents “sometimes detects, sometimes not”)
  if (mainHit) {
    if (detectHits < DETECT_HITS_MAX) detectHits++;
    lastConveyorDetectAt = now;
  } else {
    if (detectHits > 0) detectHits--;
  }
  bool mainDetected = (detectHits >= DETECT_HITS_TO_START);

  bool bin1Full = (distBin1 > 0 && distBin1 <= BIN_FULL_CM);
  bool bin2Full = (distBin2 > 0 && distBin2 <= BIN_FULL_CM);

  // ================= WIPER =================
  if (!wiperRunning && now >= wiperNextStart) {
    wiperRunning = true;
    wiperStopAt = now + WIPER_ONE_ROT_MS;

    digitalWrite(WIPER_IN1, HIGH);
    digitalWrite(WIPER_IN2, LOW);
    analogWrite(WIPER_EN, WIPER_SPEED);
  }

  if (wiperRunning && now >= wiperStopAt) {
    wiperRunning = false;

    digitalWrite(WIPER_IN1, LOW);
    digitalWrite(WIPER_IN2, LOW);
    analogWrite(WIPER_EN, 0);

    wiperNextStart = now + WIPER_INTERVAL_MS;
  }

  // ================= CONVEYOR (OUT3/OUT4) HARD RELIABLE =================
  // Start only when confirmed detected
  if (!conveyorRunning && mainDetected) {
    conveyorRunning = true;
    conveyorStartAt = now;
    lastConveyorDetectAt = now;
    conveyorForwardKick(now);
  }

  // End of kick -> go slow
  if (conveyorRunning && conveyorKicking && (long)(now - conveyorKickUntil) >= 0) {
    conveyorKicking = false;
    conveyorForwardSlow();
  }

  // Stop: min 10s run, then stop if no detect for 1.2s
  if (conveyorRunning) {
    bool minRunDone = (now - conveyorStartAt >= CONV_MIN_RUN_MS);
    bool noDetectRecently = (now - lastConveyorDetectAt >= CONV_NO_DETECT_STOP_MS);

    if (minRunDone && noDetectRecently) {
      conveyorRunning = false;
      conveyorKicking = false;
      detectHits = 0;
      conveyorCoastStop();
    }
  }

  // ================= METAL + SERVO (ONCE) =================
  // If your sensor is opposite, change == LOW to == HIGH.
  bool metalDetected = (digitalRead(METAL_PIN) == LOW);

  if (metalDetected && !lastMetalDetected && (now - lastMetalTrigger > METAL_LOCKOUT)) {
    lastMetalTrigger = now;
    servoMoving = true;
    servoMoveAt = now;
    metalServo.write(SERVO_THROW_DEG);
  }
  lastMetalDetected = metalDetected;

  if (servoMoving && (now - servoMoveAt >= SERVO_HOLD_MS)) {
    metalServo.write(0);
    servoMoving = false;
  }

  // ================= SERIAL PRINT =================
  if (now - printTimer >= PRINT_INTERVAL) {
    printTimer = now;

    Serial.print("Main(cm): "); Serial.print(distMain);
    Serial.print(" Hits: "); Serial.print(detectHits);
    Serial.print(" Det: "); Serial.print(mainDetected);

    Serial.print(" | Conv: "); Serial.print(conveyorRunning);
    Serial.print(" Kick: "); Serial.print(conveyorKicking);

    Serial.print(" | Bin1: "); Serial.print(distBin1);
    Serial.print(" Bin2: "); Serial.print(distBin2);
    Serial.print(" Bin1Full: "); Serial.print(bin1Full);
    Serial.print(" Bin2Full: "); Serial.print(bin2Full);

    Serial.print(" | Wiper: "); Serial.print(wiperRunning);
    Serial.print(" | Blockage: "); Serial.print(blockageDetected);
    Serial.print(" Overflow: "); Serial.print(overflowRisk);

    Serial.print(" | Metal: "); Serial.print((int)metalDetected);
    Serial.println();
  }
}
