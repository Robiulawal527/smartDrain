#include <Servo.h>

// ================= SENSOR PINS =================
#define WATER_LOW_PIN   A0
#define TURB_PIN        A1
#define RAIN_PIN        A2
#define WATER_HIGH_PIN  A3

#define ULTRA_TRIG      34
#define ULTRA_ECHO      35

#define METAL_PIN       36   // metal detector digital output

// ================= WIPER MOTOR (OUT1/OUT2) =================
#define WIPER_EN   5     // ENA (PWM)
#define WIPER_IN1  30    // IN1
#define WIPER_IN2  31    // IN2

// ================= CONVEYOR MOTOR (OUT3/OUT4) =================
#define CONV_EN    6     // ENB (PWM)
#define CONV_IN1   32    // IN3
#define CONV_IN2   33    // IN4

// ================= SERVO =================
#define SERVO_PIN  9
Servo metalServo;

// ================= THRESHOLDS =================
#define WATER_LOW_TH        500
#define WATER_HIGH_TH       650
#define TURB_DIRTY_TH       500
#define RAIN_HEAVY_TH       500
#define ULTRA_DISTANCE_TH   15   // cm (adjust)

// ================= WIPER SETTINGS =================
const unsigned long WIPER_OFF_TIME = 20000; // 20 sec stop
const unsigned long WIPER_ON_TIME  = 100;   // ms ON pulse
const int WIPER_SPEED = 100;                // 0-255 PWM

// ================= CONVEYOR SETTINGS =================
const unsigned long CONVEYOR_RUN_TIME = 60000; // 1 minute
const int CONV_SPEED = 200;                    // 0-255 PWM

// ================= VARIABLES =================
bool wiperRunning = false;
unsigned long wiperTimer = 0;

bool conveyorRunning = false;
unsigned long conveyorTimer = 0;

// Print timing
unsigned long printTimer = 0;
const unsigned long PRINT_INTERVAL = 300; // ms

// Metal trigger timing
unsigned long lastMetalTrigger = 0;
const unsigned long METAL_LOCKOUT = 3000; // ms (prevents repeated triggers)

// ---------- Ultrasonic helper ----------
long readUltrasonicCM() {
  digitalWrite(ULTRA_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG, LOW);

  // timeout prevents blocking forever
  unsigned long duration = pulseIn(ULTRA_ECHO, HIGH, 25000UL);
  if (duration == 0) return -1; // no echo / out of range

  return (long)(duration * 0.034 / 2.0); // cm
}

void setup() {
  Serial.begin(9600);
  Serial.println("Smart Drain + Wiper + Conveyor + Metal Servo Started");

  // Wiper motor pins
  pinMode(WIPER_EN, OUTPUT);
  pinMode(WIPER_IN1, OUTPUT);
  pinMode(WIPER_IN2, OUTPUT);
  digitalWrite(WIPER_IN1, LOW);
  digitalWrite(WIPER_IN2, LOW);
  analogWrite(WIPER_EN, 0);

  // Conveyor motor pins
  pinMode(CONV_EN, OUTPUT);
  pinMode(CONV_IN1, OUTPUT);
  pinMode(CONV_IN2, OUTPUT);
  digitalWrite(CONV_IN1, LOW);
  digitalWrite(CONV_IN2, LOW);
  analogWrite(CONV_EN, 0);

  // Ultrasonic pins
  pinMode(ULTRA_TRIG, OUTPUT);
  pinMode(ULTRA_ECHO, INPUT);
  digitalWrite(ULTRA_TRIG, LOW);

  // Metal detector pin (stable input)
  pinMode(METAL_PIN, INPUT_PULLUP);

  // Servo
  metalServo.attach(SERVO_PIN);
  metalServo.write(0);

  wiperTimer = millis();
  printTimer = millis();
}

void loop() {
  unsigned long now = millis();

  // ---------- Read analog sensors ----------
  int waterLow  = analogRead(WATER_LOW_PIN);
  int waterHigh = analogRead(WATER_HIGH_PIN);
  int turbValue = analogRead(TURB_PIN);
  int rainValue = analogRead(RAIN_PIN);

  bool waterRising   = (waterLow > WATER_LOW_TH);
  bool waterCritical = (waterHigh > WATER_HIGH_TH);
  bool waterDirty    = (turbValue < TURB_DIRTY_TH);
  bool heavyRain     = (rainValue < RAIN_HEAVY_TH);

  bool blockageDetected = (waterRising && waterDirty);
  bool overflowRisk     = (waterCritical && heavyRain);

  // ---------- Ultrasonic ----------
  long distance = readUltrasonicCM();

  // ---------- WIPER timing ----------
  if (!wiperRunning && now - wiperTimer >= WIPER_OFF_TIME) {
    wiperRunning = true;
    wiperTimer = now;

    digitalWrite(WIPER_IN1, HIGH);
    digitalWrite(WIPER_IN2, LOW);
    analogWrite(WIPER_EN, WIPER_SPEED);
  }

  if (wiperRunning && now - wiperTimer >= WIPER_ON_TIME) {
    wiperRunning = false;
    wiperTimer = now;

    digitalWrite(WIPER_IN1, LOW);
    digitalWrite(WIPER_IN2, LOW);
    analogWrite(WIPER_EN, 0);
  }

  // ---------- CONVEYOR ----------
  if (!conveyorRunning && distance > 0 && distance <= ULTRA_DISTANCE_TH) {
    conveyorRunning = true;
    conveyorTimer = now;

    digitalWrite(CONV_IN1, HIGH);     // IN3
    digitalWrite(CONV_IN2, LOW);      // IN4
    analogWrite(CONV_EN, CONV_SPEED); // ENB PWM
  }

  if (conveyorRunning && now - conveyorTimer >= CONVEYOR_RUN_TIME) {
    conveyorRunning = false;

    digitalWrite(CONV_IN1, LOW);
    digitalWrite(CONV_IN2, LOW);
    analogWrite(CONV_EN, 0);
  }

  // ---------- METAL DETECTION & SERVO (ROTATE ONCE PER DETECTION) ----------
  // With INPUT_PULLUP, most metal sensors go LOW when active.
  // If your sensor goes HIGH when active, change == LOW to == HIGH.
  static bool lastMetalDetected = false;
  bool metalDetected = (digitalRead(METAL_PIN) == LOW);

  // Trigger only when it becomes detected (edge trigger) + lockout
  if (metalDetected && !lastMetalDetected && (now - lastMetalTrigger > METAL_LOCKOUT)) {
    lastMetalTrigger = now;
    Serial.println("Metal Detected -> Servo once");

    metalServo.write(35);
    delay(400);   // short movement time (won't keep spinning)
    metalServo.write(0);
  }
  lastMetalDetected = metalDetected;

  // ---------- PRINT ALL READINGS ----------
  if (now - printTimer >= PRINT_INTERVAL) {
    printTimer = now;

    Serial.print("WLlow: "); Serial.print(waterLow);
    Serial.print(" | WLhigh: "); Serial.print(waterHigh);
    Serial.print(" | Turb: "); Serial.print(turbValue);
    Serial.print(" | Rain: "); Serial.print(rainValue);

    Serial.print(" | Blockage: "); Serial.print(blockageDetected);
    Serial.print(" | Overflow: "); Serial.print(overflowRisk);

    Serial.print(" | Ultra(cm): "); Serial.print(distance);

    Serial.print(" | Wiper: "); Serial.print(wiperRunning);
    Serial.print(" | Conveyor: "); Serial.print(conveyorRunning);

    Serial.print(" | Metal: "); Serial.print((int)metalDetected);
    Serial.println();
  }
}
