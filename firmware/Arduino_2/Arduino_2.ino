/*
 * Arduino 2 — Line Follower + Launcher (DUAL PCA9685)
 * GAME LOGIC VERSION
 *
 * Receives commands from Arduino 1 over SoftwareSerial:
 *   D10 (RX) <- Arduino 1 D11 (TX)
 *   Common GND
 *
 * Commands from Arduino 1:
 *   START        — begin run with 3s ESC arming + home base bypass
 *   STOP         — kill everything immediately, even mid-launch
 *   TEAM:RED     — team is RED  -> flap 165 deg, ESC direction A
 *   TEAM:BLUE    — team is BLUE -> flap 15 deg,  ESC direction B
 *   TARGET:SELF  — IR signal from our team's bucket  -> SKIP (don't score own goal)
 *   TARGET:OPPONENT — IR signal from opponent's bucket -> FIRE (score!)
 *   TARGET:NONE  — no IR signal detected -> skip
 *
 * GAME FLOW:
 *   1. START -> 3s grace (ESC arming, robot stays still)
 *   2. Home base bypass: 0.15s straight forward (junction detection disabled)
 *      — this skips the junction line at the home base
 *   3. Forward line following begins
 *   4. Junction detected -> stop -> wait 5s, collecting TARGET classifications
 *      - Count OPPONENT vs SELF votes (ignoring NONE)
 *      - If OPPONENT votes > SELF votes AND OPPONENT votes >= 3 -> fire
 *      - Otherwise -> skip junction, resume line following
 *   5. After 4 junctions -> STOP PERMANENTLY (run ends)
 *   6. Stays stopped until next START or STOP command
 *
 * Hardware:
 *   PCA9685 #1 @ 0x40 — running at 1000Hz for L298N drive motors
 *   PCA9685 #2 @ 0x41 — running at 50Hz for ESC + flap servo
 *   SparkFun Line Follower Array @ 0x3E (front-facing only)
 *
 * PCA #1 (0x40, drive @ 1000Hz):
 *   ch0=IN1, ch1=IN2, ch2=IN3, ch3=IN4, ch4=ENB, ch5=ENA
 * PCA #2 (0x41, shooter @ 50Hz):
 *   ch0=ESC direction (yellow)
 *   ch6=ESC throttle (white)
 *   ch7=Flap servo
 */

#include <Wire.h>
#include <sensorbar.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

// =============================================
// TOP-LEVEL CONFIGURATION
// =============================================

// -- Serial link --
const byte ARDUINO1_RX_PIN = 10;
const byte ARDUINO1_TX_PIN = 11;

// -- Direction flags --
#define FLIP_MOTORS true
#define FLIP_SENSOR true

// -- Reverse line following strategy --
// true  = Plan A: reverse line following with reversed PID (recommended)
// false = Plan B: just drive straight backwards, no line following
#define USE_PID_REVERSE true

// -- ESC direction servo angles (yellow wire) --
const int MOTOR_DIR_RED  = 180; // direction for RED team
const int MOTOR_DIR_BLUE = 0;   // direction for BLUE team

// Reference angle used during ESC arming.
// Empirically 90 works on this ESC.
const int ESC_ARM_REF_ANGLE = 90;

// -- Flap angles per team --
const int FLAP_CENTER    = 90;  // resting / closed
const int FLAP_OPEN_RED  = 165; // red team release angle
const int FLAP_OPEN_BLUE = 15;  // blue team release angle

// -- Forward PID tuning --
float KP = 0.35;
float KI = 0.00;
float KD = 0.4;

// -- Reverse PID tuning (smaller gains for stability) --
float KP_REV = 0.25;
float KI_REV = 0.00;
float KD_REV = 0.20;

// -- Drive speeds (PCA9685 0-4095) --
int BASE_SPEED    = 1100;
int CURVE_SPEED   = 1500;
int PIVOT_SPEED   = 3000;
int SEARCH_SPEED  = 1200;
int MAX_SPEED     = 3000;
int MIN_MOTOR_SPEED = 800;

// Reverse-mode speeds (slower for stability)
int BASE_SPEED_REV  = 600;
int CURVE_SPEED_REV = 800;
int PIVOT_SPEED_REV = 1500;
int START_BOOST_SPEED = 2300;
unsigned long START_BOOST_MS = 180;

float CENTER_DEADBAND  = 0.05;
float CURVE_THRESHOLD  = 1.0;
float PIVOT_THRESHOLD  = 1.8;

// -- Junction settings --
int           JUNCTION_SENSOR_COUNT  = 5;
unsigned long JUNCTION_DRIVE_MS      = 200;
int           JUNCTION_FORWARD_SPEED = 1200;
unsigned long JUNCTION_COOLDOWN_MS   = 800;
const int     TOTAL_JUNCTIONS        = 4; // stop permanently after this many junctions

// -- Target detection wait at junction --
const unsigned long TARGET_WAIT_MS      = 5000; // 5s window to collect votes
const int           MIN_OPPONENT_VOTES  = 3;    // need at least this many OPPONENT votes
                                                 // AND OPPONENT > SELF to fire

// -- Junction skip (no fire) settings --
const unsigned long SKIP_PAUSE_MS   = 200;  // brief pause before resuming
const unsigned long SKIP_BOOST_MS   = 250;  // boost forward to clear junction
const int           SKIP_BOOST_SPEED = 1500;

// -- Shooter timing --
const float TARGET_THROTTLE = 0.27;

// 3s startup grace period after START (ESC arming window).
const unsigned long STARTUP_GRACE_MS = 3000;
const unsigned long ESC_ARM_MS       = 3000;

// After grace ends, drive forward at HOME_BASE_BYPASS_SPEED for
// HOME_BASE_BYPASS_MS to physically clear the home base line.
// Then keep junction/sensor detection DISABLED for SENSOR_IGNORE_MS
// (measured from the start of bypass).
const unsigned long HOME_BASE_BYPASS_MS    = 150;  // motion duration (boost forward)
const int           HOME_BASE_BYPASS_SPEED = 1800; // motion speed
const unsigned long SENSOR_IGNORE_MS       = 800;  // junction detection stays off this long
                                                    // (must be >= HOME_BASE_BYPASS_MS)

// Per-launch timing
const unsigned long DIR_SETTLE_MS   = 300;
const unsigned long SPINUP_WAIT_MS  = 1500;
const unsigned long FLAP_HOLD_MS    = 600;
const unsigned long FLAP_RETURN_MS  = 600;
const unsigned long POST_FLAP_MS    = 1500;
const unsigned long POST_LAUNCH_MS  = 2000;

// -- PCA9685 #1 channel mapping (drive @ 1000Hz, addr 0x40) --
#define CH_IN1 0
#define CH_IN2 1
#define CH_ENA 5
#define CH_IN3 2
#define CH_IN4 3
#define CH_ENB 4

// -- PCA9685 #2 channel mapping (shooter @ 50Hz, addr 0x41) --
const int ESC_DIR_CHANNEL = 0;
const int ESC_THR_CHANNEL = 6;
const int FLAP_CHANNEL    = 7;

// =============================================
// GLOBALS
// =============================================
SoftwareSerial arduino1Link(ARDUINO1_RX_PIN, ARDUINO1_TX_PIN);
SensorBar bar(0x3E);
Adafruit_PWMServoDriver pwmDrive(0x40);
Adafruit_PWMServoDriver pwmShoot(0x41);

enum TeamColor  { TEAM_RED, TEAM_BLUE };
TeamColor currentTeam = TEAM_RED;

enum TargetState { TARGET_NONE, TARGET_SELF, TARGET_OPPONENT };
TargetState currentTarget = TARGET_NONE;

bool          running        = false;
unsigned long runStartTime   = 0;
bool          inStartupGrace = false;

// True while we're in the home base bypass phase.
bool          inHomeBaseBypass    = false;
unsigned long homeBaseBypassStart = 0;

// Forward (true) or reverse (false) line-following direction.
bool forwardDirection = true;
bool escArmed         = false;

enum GracePhase {
  GRACE_INIT,
  GRACE_ARMING,
  GRACE_IDLE
};
GracePhase    gracePhase      = GRACE_INIT;
unsigned long gracePhaseStart = 0;

// PID state
float integral     = 0.0;
float lastError    = 0.0;
int   lastKnownDir = 0;
int   lastLeftSpeed  = 0;
int   lastRightSpeed = 0;
bool  wasInTurn    = false;

// Junction state
bool          junctionDetected = false;
unsigned long junctionStartTime = 0;
unsigned long lastJunctionTime  = 0;
int           junctionCount     = 0;

// Vote counters for the current junction wait window.
int votesOpponent = 0;
int votesSelf     = 0;
int votesNone     = 0;

// =============================================
// STATE MACHINES
// =============================================

enum JunctionPhase {
  JUNC_IDLE,
  JUNC_APPROACH,          // brief forward drive after detection
  JUNC_WAIT_FOR_TARGET,   // 5s window: collecting TARGET votes
  JUNC_SKIP_PAUSE,        // brief pause when not firing
  JUNC_SKIP_BOOST         // boost through to leave the junction
};
JunctionPhase junctionPhase      = JUNC_IDLE;
unsigned long junctionPhaseStart = 0;

enum LaunchPhase {
  LAUNCH_IDLE,
  LAUNCH_SET_DIRECTION,
  LAUNCH_DIR_SETTLE,
  LAUNCH_SPINUP,
  LAUNCH_FLAP_OPEN,
  LAUNCH_FLAP_RETURN,
  LAUNCH_POST_FLAP,
  LAUNCH_ESC_OFF,
  LAUNCH_POST_LAUNCH,
  LAUNCH_RESUME_BOOST
};
LaunchPhase   launchPhase      = LAUNCH_IDLE;
unsigned long launchPhaseStart = 0;

// =============================================
// PCA9685 HELPERS
// =============================================
int usToCounts50Hz(int us) {
  return (int)((us * 4096L) / 20000L);
}

void setShooterPulseUs(int channel, int us) {
  pwmShoot.setPWM(channel, 0, usToCounts50Hz(us));
}

// Cached last-written values for shooter board.
int lastEscThrottleUs  = -1;
int lastEscDirAngleDeg = -1;
int lastFlapAngleDeg   = -1;

void setEsc(float throttle) {
  if (throttle < 0.0) throttle = 0.0;
  if (throttle > 1.0) throttle = 1.0;
  int us = 1000 + (int)(throttle * 1000.0);
  if (us == lastEscThrottleUs) return;
  setShooterPulseUs(ESC_THR_CHANNEL, us);
  lastEscThrottleUs = us;
}

void setServoAngle(int channel, int degrees) {
  if (degrees < 0)   degrees = 0;
  if (degrees > 180) degrees = 180;
  int us = 1000 + (int)((degrees / 180.0) * 1000.0);
  setShooterPulseUs(channel, us);
}

void setFlap(int degrees) {
  if (degrees < 0)   degrees = 0;
  if (degrees > 180) degrees = 180;
  if (degrees == lastFlapAngleDeg) return;
  setServoAngle(FLAP_CHANNEL, degrees);
  lastFlapAngleDeg = degrees;
}

void setMotorDirectionForTeam() {
  int angle = (currentTeam == TEAM_RED) ? MOTOR_DIR_RED : MOTOR_DIR_BLUE;
  if (angle == lastEscDirAngleDeg) return;
  setServoAngle(ESC_DIR_CHANNEL, angle);
  lastEscDirAngleDeg = angle;
}

void setEscDirectionToArmRef() {
  if (ESC_ARM_REF_ANGLE == lastEscDirAngleDeg) return;
  setServoAngle(ESC_DIR_CHANNEL, ESC_ARM_REF_ANGLE);
  lastEscDirAngleDeg = ESC_ARM_REF_ANGLE;
}

int getFlapOpenAngleForTeam() {
  return (currentTeam == TEAM_RED) ? FLAP_OPEN_RED : FLAP_OPEN_BLUE;
}

void forceShooterChannelsOff() {
  pwmShoot.setPWM(ESC_THR_CHANNEL, 0, 4096);
  pwmShoot.setPWM(FLAP_CHANNEL,    0, 4096);
  lastEscThrottleUs = -1;
  lastFlapAngleDeg  = -1;
}

// =============================================
// DRIVE MOTOR FUNCTIONS (PCA #1 @ 1000Hz)
// =============================================
void pinHigh(uint8_t ch) { pwmDrive.setPWM(ch, 4096, 0); }
void pinLow (uint8_t ch) { pwmDrive.setPWM(ch, 0, 4096); }

void setSpeed(uint8_t ch, int speed) {
  speed = constrain(speed, 0, 4095);
  pwmDrive.setPWM(ch, 0, speed);
}

void stopMotors() {
  setSpeed(CH_ENA, 0);
  setSpeed(CH_ENB, 0);
  pinLow(CH_IN1);
  pinLow(CH_IN2);
  pinLow(CH_IN3);
  pinLow(CH_IN4);
}

void setMotors(int leftSpeed, int rightSpeed) {
  if (FLIP_MOTORS) {
    leftSpeed  = -leftSpeed;
    rightSpeed = -rightSpeed;
  }

  leftSpeed  = constrain(leftSpeed,  -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  if (leftSpeed  > 0 && leftSpeed  < MIN_MOTOR_SPEED)  leftSpeed  = MIN_MOTOR_SPEED;
  if (leftSpeed  < 0 && leftSpeed  > -MIN_MOTOR_SPEED) leftSpeed  = -MIN_MOTOR_SPEED;
  if (rightSpeed > 0 && rightSpeed < MIN_MOTOR_SPEED)  rightSpeed = MIN_MOTOR_SPEED;
  if (rightSpeed < 0 && rightSpeed > -MIN_MOTOR_SPEED) rightSpeed = -MIN_MOTOR_SPEED;

  if (leftSpeed >= 0) { pinHigh(CH_IN1); pinLow(CH_IN2); }
  else                { pinLow(CH_IN1);  pinHigh(CH_IN2); }
  setSpeed(CH_ENA, abs(leftSpeed));

  if (rightSpeed >= 0) { pinLow(CH_IN3);  pinHigh(CH_IN4); }
  else                 { pinHigh(CH_IN3); pinLow(CH_IN4);  }
  setSpeed(CH_ENB, abs(rightSpeed));
}

// =============================================
// SENSOR FUNCTIONS
// =============================================
bool sensorOn(uint8_t raw, int n) { return raw & (1 << n); }

void printRawBits(uint8_t raw) {
  for (int i = 7; i >= 0; i--) Serial.print(sensorOn(raw, i) ? "1" : "0");
}

int countActiveSensors(uint8_t raw) {
  int c = 0;
  for (int i = 0; i < 8; i++) if (sensorOn(raw, i)) c++;
  return c;
}

float getLinePosition() {
  uint8_t raw = bar.getRaw();
  if (raw == 0) return 99.0;

  float weightedSum  = 0.0;
  float totalWeight  = 0.0;
  for (int i = 0; i < 8; i++) {
    if (sensorOn(raw, i)) {
      float pos = FLIP_SENSOR ? ((float)i - 3.5) : (3.5 - (float)i);
      weightedSum += pos;
      totalWeight += 1.0;
    }
  }
  return weightedSum / totalWeight;
}

// =============================================
// STATE RESET
// =============================================
void resetStateForStart() {
  integral      = 0.0;
  lastError     = 0.0;
  lastKnownDir  = 0;
  lastLeftSpeed = 0;
  lastRightSpeed = 0;
  wasInTurn     = false;

  junctionDetected  = false;
  junctionStartTime = 0;
  lastJunctionTime  = millis();
  junctionCount     = 0;

  launchPhase   = LAUNCH_IDLE;
  junctionPhase = JUNC_IDLE;

  forwardDirection = true;
  currentTarget    = TARGET_NONE;

  votesOpponent = 0;
  votesSelf     = 0;
  votesNone     = 0;

  escArmed          = false;
  inHomeBaseBypass  = false;

  lastEscThrottleUs  = -1;
  lastEscDirAngleDeg = -1;
  lastFlapAngleDeg   = -1;
}

void killAll() {
  setEsc(0.0);
  stopMotors();
  setFlap(FLAP_CENTER);
  forceShooterChannelsOff();

  running           = false;
  launchPhase       = LAUNCH_IDLE;
  junctionPhase     = JUNC_IDLE;
  junctionDetected  = false;
  inStartupGrace    = false;
  inHomeBaseBypass  = false;
  gracePhase        = GRACE_INIT;
  escArmed          = false;

  votesOpponent = 0;
  votesSelf     = 0;
  votesNone     = 0;

  lastEscThrottleUs  = -1;
  lastEscDirAngleDeg = -1;
  lastFlapAngleDeg   = -1;

  Serial.println(F("[killAll] all systems stopped"));
}

// =============================================
// COMMAND HANDLERS
// =============================================
void handleStart() {
  if (running) return;
  Serial.println(F("=== START - entering 3s ESC arming grace ==="));
  resetStateForStart();
  running = true;
  stopMotors();
  setEsc(0.0);
  setFlap(FLAP_CENTER);
  runStartTime   = millis();
  inStartupGrace = true;
  gracePhase     = GRACE_INIT;
  gracePhaseStart = millis();
}

void handleStop() {
  Serial.println(F("=== STOP ==="));
  killAll();
}

void handleSerial() {
  while (arduino1Link.available()) {
    String cmd = arduino1Link.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) continue;

    if (cmd == "START") {
      handleStart();
    } else if (cmd == "STOP") {
      handleStop();
    } else if (cmd == "TEAM:RED") {
      bool changed = (currentTeam != TEAM_RED);
      currentTeam = TEAM_RED;
      if (escArmed) {
        setMotorDirectionForTeam();
      }
      if (changed) {
        Serial.print(F("Team: RED | DIR angle = "));
        Serial.print(MOTOR_DIR_RED);
        Serial.println(escArmed ? F(" (applied)") : F(" (deferred until armed)"));
      }
    } else if (cmd == "TEAM:BLUE") {
      bool changed = (currentTeam != TEAM_BLUE);
      currentTeam = TEAM_BLUE;
      if (escArmed) {
        setMotorDirectionForTeam();
      }
      if (changed) {
        Serial.print(F("Team: BLUE | DIR angle = "));
        Serial.print(MOTOR_DIR_BLUE);
        Serial.println(escArmed ? F(" (applied)") : F(" (deferred until armed)"));
      }
    } else if (cmd == "TARGET:SELF") {
      currentTarget = TARGET_SELF;
      if (junctionPhase == JUNC_WAIT_FOR_TARGET) {
        votesSelf++;
      }
    } else if (cmd == "TARGET:OPPONENT") {
      currentTarget = TARGET_OPPONENT;
      if (junctionPhase == JUNC_WAIT_FOR_TARGET) {
        votesOpponent++;
      }
    } else if (cmd == "TARGET:NONE") {
      currentTarget = TARGET_NONE;
      if (junctionPhase == JUNC_WAIT_FOR_TARGET) {
        votesNone++;
      }
    }
  }
}

// =============================================
// GRACE PERIOD STATE MACHINE
// =============================================
void updateGrace(unsigned long now) {
  unsigned long elapsed = now - gracePhaseStart;

  switch (gracePhase) {
    case GRACE_INIT:
      setEscDirectionToArmRef();
      setEsc(0.0);
      gracePhase      = GRACE_ARMING;
      gracePhaseStart = now;
      Serial.print(F("[GRACE] arming ESC at reference angle "));
      Serial.println(ESC_ARM_REF_ANGLE);
      break;

    case GRACE_ARMING:
      setEscDirectionToArmRef();
      setEsc(0.0);
      if (elapsed >= ESC_ARM_MS) {
        escArmed = true;
        setMotorDirectionForTeam();
        gracePhase      = GRACE_IDLE;
        gracePhaseStart = now;
        Serial.print(F("[GRACE] ESC armed - direction set for team "));
        Serial.print(currentTeam == TEAM_RED ? F("RED (") : F("BLUE ("));
        Serial.print(currentTeam == TEAM_RED ? MOTOR_DIR_RED : MOTOR_DIR_BLUE);
        Serial.println(F(")"));
      }
      break;

    case GRACE_IDLE:
      break;
  }
}

// =============================================
// JUNCTION HANDLING STATE MACHINE
// =============================================
void enterJunctionWait() {
  Serial.print(F("Junction #"));
  Serial.print(junctionCount);
  Serial.println(F(" - driving forward briefly to align over junction..."));

  setMotors(JUNCTION_FORWARD_SPEED, JUNCTION_FORWARD_SPEED);

  votesOpponent = 0;
  votesSelf     = 0;
  votesNone     = 0;

  junctionPhase      = JUNC_APPROACH;
  junctionPhaseStart = millis();
}

void updateJunction() {
  handleSerial();
  if (!running) return;

  unsigned long now     = millis();
  unsigned long elapsed = now - junctionPhaseStart;

  switch (junctionPhase) {
    case JUNC_APPROACH:
      setMotors(JUNCTION_FORWARD_SPEED, JUNCTION_FORWARD_SPEED);
      if (elapsed >= JUNCTION_DRIVE_MS) {
        Serial.println(F(" -> aligned, collecting TARGET votes for 5s..."));
        stopMotors();
        votesOpponent = 0;
        votesSelf     = 0;
        votesNone     = 0;
        junctionPhase      = JUNC_WAIT_FOR_TARGET;
        junctionPhaseStart = now;
      }
      break;

    case JUNC_WAIT_FOR_TARGET:
      stopMotors();
      if (elapsed >= TARGET_WAIT_MS) {
        Serial.print(F(" votes: OPP="));
        Serial.print(votesOpponent);
        Serial.print(F(" SELF="));
        Serial.print(votesSelf);
        Serial.print(F(" NONE="));
        Serial.println(votesNone);

        bool shouldFire = (votesOpponent > votesSelf) &&
                          (votesOpponent >= MIN_OPPONENT_VOTES);

        if (shouldFire) {
          Serial.println(F(" -> FIRING (OPPONENT wins majority)"));
          junctionPhase  = JUNC_IDLE;
          launchPhase    = LAUNCH_SET_DIRECTION;
          launchPhaseStart = now;
        } else if (junctionCount >= TOTAL_JUNCTIONS) {
          Serial.println(F(" -> SKIPPING + final junction, stopping in place"));
          stopMotors();
          junctionPhase    = JUNC_IDLE;
          lastJunctionTime = now;
        } else {
          Serial.println(F(" -> SKIPPING (insufficient OPPONENT votes)"));
          junctionPhase      = JUNC_SKIP_PAUSE;
          junctionPhaseStart = now;
        }
      }
      break;

    case JUNC_SKIP_PAUSE:
      stopMotors();
      if (elapsed >= SKIP_PAUSE_MS) {
        junctionPhase      = JUNC_SKIP_BOOST;
        junctionPhaseStart = now;
      }
      break;

    case JUNC_SKIP_BOOST: {
      int s = forwardDirection ? SKIP_BOOST_SPEED : -SKIP_BOOST_SPEED;
      setMotors(s, s);
      if (elapsed >= SKIP_BOOST_MS) {
        junctionPhase    = JUNC_IDLE;
        lastJunctionTime = now;
      }
      break;
    }

    default:
      break;
  }
}

// =============================================
// LAUNCH STATE MACHINE
// =============================================
void updateLaunch() {
  handleSerial();
  if (!running) return;

  unsigned long now     = millis();
  unsigned long elapsed = now - launchPhaseStart;

  switch (launchPhase) {
    case LAUNCH_SET_DIRECTION:
      setEsc(0.0);
      setMotorDirectionForTeam();
      launchPhase      = LAUNCH_DIR_SETTLE;
      launchPhaseStart = now;
      break;

    case LAUNCH_DIR_SETTLE:
      if (elapsed >= DIR_SETTLE_MS) {
        setEsc(TARGET_THROTTLE);
        launchPhase      = LAUNCH_SPINUP;
        launchPhaseStart = now;
      }
      break;

    case LAUNCH_SPINUP:
      if (elapsed >= SPINUP_WAIT_MS) {
        setFlap(getFlapOpenAngleForTeam());
        launchPhase      = LAUNCH_FLAP_OPEN;
        launchPhaseStart = now;
      }
      break;

    case LAUNCH_FLAP_OPEN:
      if (elapsed >= FLAP_HOLD_MS) {
        setFlap(FLAP_CENTER);
        launchPhase      = LAUNCH_FLAP_RETURN;
        launchPhaseStart = now;
      }
      break;

    case LAUNCH_FLAP_RETURN:
      if (elapsed >= FLAP_RETURN_MS) {
        launchPhase      = LAUNCH_POST_FLAP;
        launchPhaseStart = now;
      }
      break;

    case LAUNCH_POST_FLAP:
      if (elapsed >= POST_FLAP_MS) {
        setEsc(0.0);
        launchPhase      = LAUNCH_ESC_OFF;
        launchPhaseStart = now;
      }
      break;

    case LAUNCH_ESC_OFF:
      if (elapsed >= 200) {
        forceShooterChannelsOff();
        launchPhase      = LAUNCH_POST_LAUNCH;
        launchPhaseStart = now;
      }
      break;

    case LAUNCH_POST_LAUNCH:
      if (elapsed >= POST_LAUNCH_MS) {
        if (junctionCount >= TOTAL_JUNCTIONS) {
          stopMotors();
          launchPhase      = LAUNCH_IDLE;
          lastJunctionTime = now;
          Serial.print(F("Final junction #"));
          Serial.print(junctionCount);
          Serial.println(F(" - launch done, stopping in place"));
        } else {
          int s = forwardDirection ? START_BOOST_SPEED : -START_BOOST_SPEED;
          setMotors(s, s);
          launchPhase      = LAUNCH_RESUME_BOOST;
          launchPhaseStart = now;
          lastJunctionTime = now;
        }
      }
      break;

    case LAUNCH_RESUME_BOOST:
      if (elapsed >= START_BOOST_MS) {
        launchPhase = LAUNCH_IDLE;
        Serial.print(F("Resumed after junction #"));
        Serial.println(junctionCount);
      }
      break;

    default:
      break;
  }
}

// =============================================
// LINE FOLLOWING (handles both forward and reverse)
// =============================================
void doLineFollowing() {
  unsigned long now = millis();
  uint8_t raw       = bar.getRaw();
  float position    = getLinePosition();
  int activeSensors = countActiveSensors(raw);

  // Gate junction detection during sensor-ignore window
  bool sensorIgnoreActive = (now - homeBaseBypassStart < SENSOR_IGNORE_MS);

  // Junction detection
  if (!sensorIgnoreActive &&
      activeSensors >= JUNCTION_SENSOR_COUNT &&
      (now - lastJunctionTime > JUNCTION_COOLDOWN_MS)) {

    junctionCount++;
    lastJunctionTime = now;

    Serial.print(F("JUNCTION #"));
    Serial.print(junctionCount);
    Serial.print(F("/"));
    Serial.print(TOTAL_JUNCTIONS);
    Serial.print(F(" sensors="));
    Serial.print(activeSensors);
    Serial.print(F(" raw="));
    printRawBits(raw);
    Serial.print(F(" mode="));
    Serial.println(forwardDirection ? F("FWD") : F("REV"));

    integral   = 0.0;
    lastError  = 0.0;
    wasInTurn  = false;

    enterJunctionWait();
    return;
  }

  // Plan B: simple straight reverse
  if (!forwardDirection && !USE_PID_REVERSE) {
    setMotors(-BASE_SPEED_REV, -BASE_SPEED_REV);
    return;
  }

  // Pick PID gains and speeds based on direction
  float kp      = forwardDirection ? KP      : KP_REV;
  float ki      = forwardDirection ? KI      : KI_REV;
  float kd      = forwardDirection ? KD      : KD_REV;
  int baseSpd   = forwardDirection ? BASE_SPEED  : BASE_SPEED_REV;
  int curveSpd  = forwardDirection ? CURVE_SPEED : CURVE_SPEED_REV;
  int pivotSpd  = forwardDirection ? PIVOT_SPEED : PIVOT_SPEED_REV;

  // CASE 1: line lost
  if (position == 99.0) {
    integral  = 0.0;
    lastError = 0.0;
    if (wasInTurn) {
      setMotors(lastLeftSpeed, lastRightSpeed);
    } else if (lastKnownDir < 0) {
      int l = -SEARCH_SPEED;
      int r = SEARCH_SPEED;
      if (!forwardDirection) { l = -l; r = -r; }
      setMotors(l, r);
      lastLeftSpeed = l; lastRightSpeed = r;
      wasInTurn = true;
    } else if (lastKnownDir > 0) {
      int l = SEARCH_SPEED;
      int r = -SEARCH_SPEED;
      if (!forwardDirection) { l = -l; r = -r; }
      setMotors(l, r);
      lastLeftSpeed = l; lastRightSpeed = r;
      wasInTurn = true;
    } else {
      int s = forwardDirection ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED;
      setMotors(s, s);
    }
    return;
  }

  // CASE 2: sharp pivot turn
  if (abs(position) >= PIVOT_THRESHOLD) {
    integral  = 0.0;
    lastError = 0.0;
    int l, r;
    if (position < 0) {
      lastKnownDir = -1; l = -pivotSpd; r = pivotSpd;
    } else {
      lastKnownDir = 1;  l = pivotSpd;  r = -pivotSpd;
    }
    setMotors(l, r);
    lastLeftSpeed  = l;
    lastRightSpeed = r;
    wasInTurn = true;
    return;
  }

  // CASE 3: normal PID line following
  float error = position;
  if (abs(error) < CENTER_DEADBAND) error = 0.0;
  if (error < -0.15)      lastKnownDir = -1;
  else if (error > 0.15)  lastKnownDir =  1;

  // Flip error sign for reverse so PID corrects in the right sense
  float pidError = forwardDirection ? error : -error;

  float P = kp * pidError;

  if ((pidError > 0 && lastError < 0) || (pidError < 0 && lastError > 0)) {
    integral = 0.0;
  }
  integral += pidError;
  integral  = constrain(integral, -25.0, 25.0);
  float I = ki * integral;
  float D = kd * (pidError - lastError);
  lastError = pidError;

  float correction = P + I + D;

  int currentBaseSpeed;
  if (abs(position) > CURVE_THRESHOLD) {
    currentBaseSpeed = curveSpd;
    wasInTurn = true;
  } else {
    currentBaseSpeed = baseSpd;
    wasInTurn = false;
  }

  int leftSpeed  = currentBaseSpeed + (int)(correction * (float)currentBaseSpeed);
  int rightSpeed = currentBaseSpeed - (int)(correction * (float)currentBaseSpeed);

  if (!forwardDirection) {
    leftSpeed  = -leftSpeed;
    rightSpeed = -rightSpeed;
  }

  leftSpeed  = constrain(leftSpeed,  -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  lastLeftSpeed  = leftSpeed;
  lastRightSpeed = rightSpeed;

  setMotors(leftSpeed, rightSpeed);
}

// =============================================
// SETUP
// =============================================
void setup() {
  Serial.begin(9600);
  arduino1Link.begin(9600);
  arduino1Link.setTimeout(50);

  Wire.begin();

  pwmDrive.begin();
  pwmDrive.setOscillatorFrequency(25000000);
  pwmDrive.setPWMFreq(1000);

  pwmShoot.begin();
  pwmShoot.setOscillatorFrequency(25000000);
  pwmShoot.setPWMFreq(50);

  delay(100);

  stopMotors();
  setEsc(0.0);
  setFlap(FLAP_CENTER);
  setEscDirectionToArmRef();
  escArmed = false;

  if (bar.begin() == false) {
    Serial.println(F("ERROR: line follower array not found."));
    while (1) {
      stopMotors();
      handleSerial();
      delay(200);
    }
  }

  Serial.println(F("=== READY - waiting for START ==="));
}

// =============================================
// MAIN LOOP
// =============================================
void loop() {
  handleSerial();
  if (!running) return;

  unsigned long now = millis();

  // -- Startup grace period (ESC arming) --
  if (inStartupGrace) {
    if (now - runStartTime >= STARTUP_GRACE_MS) {
      Serial.println(F("=== Grace done - entering home base bypass ==="));
      inStartupGrace   = false;
      inHomeBaseBypass = true;
      homeBaseBypassStart = now;
      setMotors(HOME_BASE_BYPASS_SPEED, HOME_BASE_BYPASS_SPEED);
    } else {
      stopMotors();
      updateGrace(now);
      return;
    }
  }

  // -- Home base bypass: drive forward, ignore sensors --
  if (inHomeBaseBypass) {
    if (now - homeBaseBypassStart >= HOME_BASE_BYPASS_MS) {
      Serial.println(F("=== Home base bypass done - line following begins ==="));
      inHomeBaseBypass = false;
      integral  = 0.0;
      lastError = 0.0;
      wasInTurn = false;
      // Fall through to doLineFollowing() immediately
    } else {
      setMotors(HOME_BASE_BYPASS_SPEED, HOME_BASE_BYPASS_SPEED);
      return;
    }
  }

  // -- Launch sequence in progress? --
  if (launchPhase != LAUNCH_IDLE) {
    updateLaunch();
    return;
  }

  // -- Junction handling in progress? --
  if (junctionPhase != JUNC_IDLE) {
    updateJunction();
    return;
  }

  // -- Done with all junctions? Stop permanently. --
  if (junctionCount >= TOTAL_JUNCTIONS) {
    Serial.print(F("=== "));
    Serial.print(TOTAL_JUNCTIONS);
    Serial.println(F(" junctions reached - STOPPING ==="));
    killAll();
    return;
  }

  // -- Normal line following --
  doLineFollowing();
}
