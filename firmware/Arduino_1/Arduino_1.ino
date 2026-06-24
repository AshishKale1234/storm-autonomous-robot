#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// =====================================================
// SERIAL TO ARDUINO 2
// =====================================================
//
// Arduino 1 D10 = RX, connect to Arduino 2 D11
// Arduino 1 D11 = TX, connect to Arduino 2 D10
//
const byte ARDUINO2_RX_PIN = 10;
const byte ARDUINO2_TX_PIN = 11;
SoftwareSerial arduino2Link(ARDUINO2_RX_PIN, ARDUINO2_TX_PIN);

// =====================================================
// LCD
// =====================================================
// Change 0x27 to 0x3F if needed.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
// PINS ON ARDUINO 1
// =====================================================
const byte SENSOR1_PIN = 2;       // Blue team active sensor
const byte SENSOR2_PIN = 3;       // Red team active sensor
const byte START_SWITCH_PIN = 4;  // Start/Stop toggle
const byte TEAM_SWITCH_PIN = 5;   // Team color toggle
const byte LCD_BLUE_CTRL_PIN = 8; // LCD pin 18 blue control

// =====================================================
// SETTINGS
// =====================================================
const unsigned long GAME_TIME_MS = 150000UL; // 2 min 30 sec
const unsigned long MEASURE_WINDOW_MS = 250UL;
const unsigned long LCD_UPDATE_MS = 200UL;
const unsigned long COMMAND_REPEAT_MS = 50UL;
const unsigned long STATUS_SEND_MS = 500UL;

const float FREQ_750_MIN = 600.0;
const float FREQ_750_MAX = 900.0;
const float FREQ_1500_MIN = 1300.0;
const float FREQ_1500_MAX = 1700.0;

// =====================================================
// STATE
// =====================================================
volatile unsigned long pulseCount1 = 0;
volatile unsigned long pulseCount2 = 0;

unsigned long gameStartTime = 0;
unsigned long lastMeasureTime = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastStatusSend = 0;

bool systemIsOn = false;
bool gameOver = false;
bool lastStartSwitchState = false;

enum TeamColor {
  BLUE_TEAM = 0,
  RED_TEAM = 1
};

enum DetectionState {
  NO_SIGNAL = 0,
  DETECT_SELF,
  DETECT_OPPONENT
};

TeamColor currentTeam = BLUE_TEAM;
TeamColor lastSentTeam = BLUE_TEAM;
DetectionState currentDetection = NO_SIGNAL;
DetectionState lastSentDetection = NO_SIGNAL;

// Non-blocking repeated command sender
bool repeatCommandActive = false;
const char* repeatCommandText = "";
unsigned int repeatCommandRemaining = 0;
unsigned long lastRepeatCommandTime = 0;

// =====================================================
// INTERRUPTS
// =====================================================
void countPulse1() {
  pulseCount1++;
}

void countPulse2() {
  pulseCount2++;
}

// =====================================================
// SWITCH HELPERS
// =====================================================
//
// INPUT_PULLUP logic:
// Switch connected to GND = ON / LOW
//
bool startSwitchOn() {
  return digitalRead(START_SWITCH_PIN) == LOW;
}

TeamColor getTeam() {
  if (digitalRead(TEAM_SWITCH_PIN) == LOW) {
    return BLUE_TEAM;
  } else {
    return RED_TEAM;
  }
}

// =====================================================
// LCD COLOR
// =====================================================
//
// LCD color behavior:
//   default              = RED
//   grounding LCD pin 18 = BLUE
//
void setLCDColor(TeamColor team) {
  if (team == BLUE_TEAM) {
    pinMode(LCD_BLUE_CTRL_PIN, OUTPUT);
    digitalWrite(LCD_BLUE_CTRL_PIN, LOW);
  } else {
    pinMode(LCD_BLUE_CTRL_PIN, INPUT);
  }
}

// =====================================================
// LCD HELPERS
// =====================================================
void lcdSystemOff() {
  lcd.noBacklight();
  lcd.clear();
}

void lcdSystemOn() {
  lcd.backlight();
  lcd.clear();
}

void printLine(byte row, String text) {
  const int LCD_WIDTH = 16;
  if (text.length() < LCD_WIDTH) {
    while (text.length() < LCD_WIDTH) {
      text += " ";
    }
  } else if (text.length() > LCD_WIDTH) {
    text = text.substring(0, LCD_WIDTH);
  }
  lcd.setCursor(0, row);
  lcd.print(text);
}

String formatTimeRemaining(unsigned long elapsedMs) {
  unsigned long remainingMs = 0;
  if (elapsedMs < GAME_TIME_MS) {
    remainingMs = GAME_TIME_MS - elapsedMs;
  }
  unsigned long totalSeconds = remainingMs / 1000;
  int minutes = totalSeconds / 60;
  int seconds = totalSeconds % 60;
  char buf[17];
  sprintf(buf, "Time: %02d:%02d", minutes, seconds);
  return String(buf);
}

String detectionText(DetectionState state) {
  if (state == DETECT_SELF)     return "Team: Self";
  if (state == DETECT_OPPONENT) return "Team:Opponent";
  return "Team: ---";
}

// =====================================================
// SENSOR HELPERS
// =====================================================
void resetPulseCounters() {
  noInterrupts();
  pulseCount1 = 0;
  pulseCount2 = 0;
  interrupts();
}

float getActiveSensorFrequency(TeamColor team) {
  unsigned long count1;
  unsigned long count2;
  noInterrupts();
  count1 = pulseCount1;
  count2 = pulseCount2;
  pulseCount1 = 0;
  pulseCount2 = 0;
  interrupts();

  unsigned long activeCount = 0;
  if (team == BLUE_TEAM) {
    activeCount = count1;
  } else {
    activeCount = count2;
  }
  return (activeCount * 1000.0) / MEASURE_WINDOW_MS;
}

DetectionState classifyFrequency(float freq, TeamColor team) {
  bool is750  = (freq > FREQ_750_MIN  && freq < FREQ_750_MAX);
  bool is1500 = (freq > FREQ_1500_MIN && freq < FREQ_1500_MAX);

  if (!is750 && !is1500) {
    return NO_SIGNAL;
  }

  if (team == RED_TEAM) {
    // Bucket loyal to RED blinks at 1500 Hz.
    // So for RED team:
    //   1500 Hz = Self
    //    750 Hz = Opponent
    if (is1500) return DETECT_SELF;
    if (is750)  return DETECT_OPPONENT;
  } else {
    // Bucket loyal to BLUE blinks at 750 Hz.
    // So for BLUE team:
    //    750 Hz = Self
    //   1500 Hz = Opponent
    if (is750)  return DETECT_SELF;
    if (is1500) return DETECT_OPPONENT;
  }
  return NO_SIGNAL;
}

// =====================================================
// SERIAL COMMAND HELPERS
// =====================================================
void sendCommand(const char* command) {
  arduino2Link.println(command);
}

void startRepeatingCommand(const char* command, unsigned int times) {
  repeatCommandText = command;
  repeatCommandRemaining = times;
  repeatCommandActive = true;
  lastRepeatCommandTime = 0;
}

void updateRepeatingCommand() {
  if (!repeatCommandActive) {
    return;
  }
  if (repeatCommandRemaining == 0) {
    repeatCommandActive = false;
    return;
  }
  unsigned long now = millis();
  if (lastRepeatCommandTime == 0 || now - lastRepeatCommandTime >= COMMAND_REPEAT_MS) {
    sendCommand(repeatCommandText);
    lastRepeatCommandTime = now;
    repeatCommandRemaining--;
  }
}

void sendTeamToArduino2(TeamColor team) {
  if (team == BLUE_TEAM) {
    sendCommand("TEAM:BLUE");
  } else {
    sendCommand("TEAM:RED");
  }
  lastSentTeam = team;
}

void sendTargetToArduino2(DetectionState state) {
  if (state == DETECT_SELF)     { sendCommand("TARGET:SELF");     }
  else if (state == DETECT_OPPONENT) { sendCommand("TARGET:OPPONENT"); }
  else                               { sendCommand("TARGET:NONE");     }
  lastSentDetection = state;
}

void sendCurrentStatusToArduino2() {
  sendTeamToArduino2(currentTeam);
  sendTargetToArduino2(currentDetection);
}

void updateStatusMessages() {
  unsigned long now = millis();
  bool teamChanged      = currentTeam      != lastSentTeam;
  bool detectionChanged = currentDetection != lastSentDetection;
  bool periodicSend     = now - lastStatusSend >= STATUS_SEND_MS;

  if (teamChanged)      { sendTeamToArduino2(currentTeam); }
  if (detectionChanged) { sendTargetToArduino2(currentDetection); }
  if (periodicSend) {
    sendCurrentStatusToArduino2();
    lastStatusSend = now;
  }
}

// =====================================================
// READ REPLIES FROM ARDUINO 2
// =====================================================
//
// Replies are ignored. This prevents the SoftwareSerial buffer
// from filling up, but does not print anything.
//
void handleArduino2Reply() {
  while (arduino2Link.available()) {
    arduino2Link.read();
  }
}

// =====================================================
// START / STOP SYSTEM
// =====================================================
void startSystem() {
  if (systemIsOn) {
    return;
  }
  systemIsOn = true;
  gameOver = false;
  gameStartTime = millis();
  lastMeasureTime = millis();
  lastLcdUpdate = 0;
  lastStatusSend = 0;

  currentTeam = getTeam();
  currentDetection = NO_SIGNAL;
  lastSentTeam = currentTeam;
  lastSentDetection = currentDetection;

  resetPulseCounters();
  lcdSystemOn();
  setLCDColor(currentTeam);
  printLine(0, "Team: ---");
  printLine(1, "Time: 02:30");

  sendTeamToArduino2(currentTeam);
  sendTargetToArduino2(currentDetection);
  startRepeatingCommand("START", 3);
}

void stopSystem() {
  systemIsOn = false;
  gameOver = false;
  currentDetection = NO_SIGNAL;
  resetPulseCounters();
  lcdSystemOff();
  pinMode(LCD_BLUE_CTRL_PIN, INPUT);
  startRepeatingCommand("STOP", 10);
}

void endGameByTimer() {
  systemIsOn = false;
  gameOver = true;
  currentDetection = NO_SIGNAL;
  resetPulseCounters();
  startRepeatingCommand("STOP", 10);
  printLine(0, "GAME OVER");
  printLine(1, "Time: 00:00");
}

// =====================================================
// DETECTION UPDATE
// =====================================================
void updateDetection(TeamColor team) {
  unsigned long now = millis();
  if (now - lastMeasureTime >= MEASURE_WINDOW_MS) {
    float freq = getActiveSensorFrequency(team);
    currentDetection = classifyFrequency(freq, team);
    lastMeasureTime = now;
  }
}

// =====================================================
// LCD UPDATE
// =====================================================
void updateRunningLCD(unsigned long elapsed) {
  unsigned long now = millis();
  if (now - lastLcdUpdate >= LCD_UPDATE_MS) {
    printLine(0, detectionText(currentDetection));
    printLine(1, formatTimeRemaining(elapsed));
    lastLcdUpdate = now;
  }
}

void updateGameOverLCD() {
  unsigned long now = millis();
  if (now - lastLcdUpdate >= LCD_UPDATE_MS) {
    setLCDColor(getTeam());
    printLine(0, "GAME OVER");
    printLine(1, "Time: 00:00");
    lastLcdUpdate = now;
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  arduino2Link.begin(9600);

  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(START_SWITCH_PIN, INPUT_PULLUP);
  pinMode(TEAM_SWITCH_PIN,  INPUT_PULLUP);
  pinMode(LCD_BLUE_CTRL_PIN, INPUT);

  lcd.init();
  lcd.noBacklight();
  lcd.clear();

  attachInterrupt(digitalPinToInterrupt(SENSOR1_PIN), countPulse1, RISING);
  attachInterrupt(digitalPinToInterrupt(SENSOR2_PIN), countPulse2, RISING);

  lastStartSwitchState = startSwitchOn();
  currentTeam = getTeam();
  lastSentTeam = currentTeam;
  currentDetection = NO_SIGNAL;
  lastSentDetection = currentDetection;

  if (lastStartSwitchState) {
    startSystem();
  } else {
    lcdSystemOff();
    startRepeatingCommand("STOP", 10);
  }
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  handleArduino2Reply();
  updateRepeatingCommand();

  bool currentStartSwitchState = startSwitchOn();

  // OFF -> ON
  if (currentStartSwitchState && !lastStartSwitchState) {
    startSystem();
  }

  // ON -> OFF
  if (!currentStartSwitchState && lastStartSwitchState) {
    stopSystem();
  }

  lastStartSwitchState = currentStartSwitchState;

  // If game ended by timer, keep LCD showing GAME OVER.
  // User must turn toggle OFF, then ON again to restart.
  if (gameOver) {
    updateGameOverLCD();
    if (!currentStartSwitchState) {
      stopSystem();
    }
    return;
  }

  if (!systemIsOn) {
    return;
  }

  currentTeam = getTeam();
  setLCDColor(currentTeam);
  updateDetection(currentTeam);
  updateStatusMessages();

  unsigned long elapsed = millis() - gameStartTime;
  if (elapsed >= GAME_TIME_MS) {
    endGameByTimer();
    return;
  }

  updateRunningLCD(elapsed);
}
