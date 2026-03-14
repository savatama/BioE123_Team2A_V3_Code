/*
V3 Centrifuge Main Code (Arduino Micro + LM393 Comparator)
Sensor + Actuator + PI Control + Rampdown + I2C LCD (16x2)
Python serial listener can play a song when run finishes.

Features:
- LM393 tachometer input with interrupt counting
- ISR pulse glitch reject
- RPM update every 250 ms
- Spike-clamped measured RPM
- Low-pass filtered controlRPM for PI control
- Rolling 10 value average displayRPM for LCD readability
- PI control
- Integral enable threshold to avoid bad startup windup
- Sensor failure detection while running
- Rampdown state
- RUN_DONE serial message for Python trigger to play song
- SENSOR_FAULT serial message for Python / debugging
- I2C LCD status display

Serial usage:
1) Enter target RPM, press Enter        ex: 1500
2) Enter spin time in seconds, Enter    ex: 30
3) Type s and press Enter to start
4) Type x and press Enter to stop / rampdown
5) Type r and press Enter to reset

Optional live tuning:
  KP 0.10
  KI 0.015

Hardware notes (Arduino Micro):
- LCD I2C: SDA = D2 / SDA, SCL = D3 / SCL
- Sensor interrupt: D7
- Motor PWM: D6
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Try 0x3F if 0x27 does not work
LiquidCrystal_I2C lcd(0x27, 16, 2);

// GLOBALS

volatile unsigned long rotationCounter = 0;

// ISR glitch reject
volatile unsigned long lastPulseMicros = 0;
const unsigned long minPulseIntervalMicros = 2000;

// Pulse scaling
const float pulsesPerRev = 6.0;

// Pins
const int interruptPin = 7;
const int motorPWMPin  = 6;

const bool useInterruptPullup = true;
const int interruptEdgeMode = FALLING;

// User inputs
int targetRPM = 0;
unsigned long spinTimeMs = 0;

// Run state
enum RunState { IDLE, RUNNING, RAMPDOWN };
RunState state = IDLE;

unsigned long startTime = 0;

// RPM timing
const unsigned long rpmWindowMs = 250;
unsigned long lastRPMTime = 0;

float measuredRPM = 0;   // spike-clamped RPM used as the immediate measured value
float controlRPM  = 0;   // low-pass filtered RPM for PI control
float filteredRPM = 0;   // internal spike-clamp reference

// RPM spike clamp
const float maxDeltaRPMPerWindow = 1200.0;

// PI control
float Kp = 0.10;
float Ki = 0.015;
float integralError = 0.0;
const float integralMax = 4000.0;
const float integralEnableFraction = 0.60;

int currentPWM = 0;

// Control RPM low-pass filter
// controlRPM = alpha * old + (1-alpha) * measuredRPM
const float controlAlpha = 0.75;

// Rampdown
const int rampdownStepPWM = 12;
const float stopRPMThreshold = 30.0;
const int stopConfirmWindows = 2;
int lowRPMCount = 0;

// Sensor failure safety
const int minPWMForSensorCheck = 80;
const int sensorFailConfirmWindows = 3;
int zeroCountWindows = 0;
bool sensorFaultLatched = false;

// Done-print guard
bool printedDoneOnce = false;


// LCD DISPLAY SMOOTHING
// Rolling average of last 10 RPM windows = 2.5 sec

const int displayWindowN = 10;
float rpmHistory[displayWindowN] = {0,0,0,0,0,0,0,0,0,0};
int rpmHistIndex = 0;
int rpmHistCount = 0;
float displayRPM = 0;


// HELPERS

void resetDisplayRPM() {
 for (int i = 0; i < displayWindowN; i++) {
   rpmHistory[i] = 0;
 }
 rpmHistIndex = 0;
 rpmHistCount = 0;
 displayRPM = 0;
}

void updateDisplayRPM(float newRPM) {
 rpmHistory[rpmHistIndex] = newRPM;
 rpmHistIndex = (rpmHistIndex + 1) % displayWindowN;

 if (rpmHistCount < displayWindowN) {
   rpmHistCount++;
 }

 float sum = 0;
 for (int i = 0; i < rpmHistCount; i++) {
   sum += rpmHistory[i];
 }

 if (rpmHistCount > 0) {
   displayRPM = sum / rpmHistCount;
 } else {
   displayRPM = newRPM;
 }
}

void lcdPrintPadded(int col, int row, const String &text) {
 lcd.setCursor(col, row);
 lcd.print(text);

 int remaining = 16 - col - text.length();
 for (int i = 0; i < remaining; i++) {
   lcd.print(' ');
 }
}

void resetControlState() {
 measuredRPM = 0;
 filteredRPM = 0;
 controlRPM = 0;
 integralError = 0.0;
}

void resetSensorFailureState() {
 zeroCountWindows = 0;
 sensorFaultLatched = false;
}

void resetPulseState() {
 noInterrupts();
 rotationCounter = 0;
 lastPulseMicros = micros();
 interrupts();
}

// LCD UPDATE

void updateLCD() {
 unsigned long elapsedSec = 0;

 if (state == RUNNING || state == RAMPDOWN) {
   elapsedSec = (millis() - startTime) / 1000UL;
 }

 int minutes = elapsedSec / 60;
 int seconds = elapsedSec % 60;

 String rpmStr = "RPM:" + String((int)displayRPM);

 String timeStr = "";
 if (minutes < 10) timeStr += "0";
 timeStr += String(minutes);
 timeStr += ":";
 if (seconds < 10) timeStr += "0";
 timeStr += String(seconds);

 String line0 = rpmStr;
 while (line0.length() < 16 - timeStr.length()) {
   line0 += " ";
 }
 line0 += timeStr;

 if (line0.length() > 16) {
   line0 = line0.substring(0, 16);
 }

 lcdPrintPadded(0, 0, line0);

 String status = "READY TO START";

 if (sensorFaultLatched) {
   status = "SENSOR FAULT";
 } else if (state == RUNNING) {
   status = "RUNNING";
 } else if (state == RAMPDOWN) {
   status = "RAMPDOWN";
 } else if (state == IDLE && printedDoneOnce) {
   status = "DONE";
 }

 lcdPrintPadded(0, 1, status);
}

// SENSOR ISR

void rotationISR() {
 unsigned long now = micros();
 unsigned long dt = now - lastPulseMicros;

 if (dt >= minPulseIntervalMicros) {
   lastPulseMicros = now;
   rotationCounter++;
 }
}

// SENSOR FAILURE CHECK

void checkSensorFailure(unsigned long countsThisWindow) {
 if (state != RUNNING) {
   zeroCountWindows = 0;
   return;
 }

 if (currentPWM >= minPWMForSensorCheck && countsThisWindow == 0) {
   zeroCountWindows++;
 } else {
   zeroCountWindows = 0;
 }

 if (zeroCountWindows >= sensorFailConfirmWindows) {
   sensorFaultLatched = true;
   state = RAMPDOWN;
   integralError = 0.0;
   lowRPMCount = 0;
   Serial.println("SENSOR_FAULT");
 }
}

// SERIAL INPUT HANDLER

void handleSerial() {
 if (Serial.available() <= 0) return;

 char c = Serial.peek();

 // Start
 if (c == 's' || c == 'S') {
   Serial.read();

   if (targetRPM <= 0 || spinTimeMs == 0) {
     Serial.println("Set RPM and TIME first.");
     return;
   }

   state = RUNNING;
   printedDoneOnce = false;
   lowRPMCount = 0;

   startTime = millis();
   lastRPMTime = millis();

   resetPulseState();
   resetControlState();
   resetDisplayRPM();
   resetSensorFailureState();

   currentPWM = map(targetRPM, 0, 1950, 0, 255);
   currentPWM = constrain(currentPWM, 0, 255);
   analogWrite(motorPWMPin, currentPWM);

   Serial.println("STARTING");
   Serial.println("Type 'x' to stop, 'r' to reset.");
   return;
 }

 // Stop
 if (c == 'x' || c == 'X') {
   Serial.read();

   if (state == RUNNING) {
     state = RAMPDOWN;
     integralError = 0.0;
     lowRPMCount = 0;
     printedDoneOnce = false;
     Serial.println("STOP REQUESTED: ramping down.");
   } else {
     state = IDLE;
     analogWrite(motorPWMPin, 0);
     Serial.println("STOPPED.");
   }
   return;
 }

 // Reset
 if (c == 'r' || c == 'R') {
   Serial.read();

   state = IDLE;
   analogWrite(motorPWMPin, 0);

   targetRPM = 0;
   spinTimeMs = 0;
   currentPWM = 0;

   resetControlState();
   resetDisplayRPM();
   resetSensorFailureState();
   resetPulseState();

   printedDoneOnce = false;
   lowRPMCount = 0;

   Serial.println("Reset done. Enter target RPM again:");
   return;
 }

 // Other line-based input
 String line = Serial.readStringUntil('\n');
 line.trim();

 if (line.length() == 0) return;

 String upperLine = line;
 upperLine.toUpperCase();

 // Kp update
 if (upperLine.startsWith("KP")) {
   int spaceIndex = upperLine.indexOf(' ');
   if (spaceIndex > 0) {
     float newKp = line.substring(spaceIndex + 1).toFloat();
     if (newKp > 0) {
       Kp = newKp;
       Serial.print("Updated Kp to ");
       Serial.println(Kp);
     } else {
       Serial.println("KP value must be > 0");
     }
   } else {
     Serial.println("Format: KP 0.10");
   }
   return;
 }

 // Ki update
 if (upperLine.startsWith("KI")) {
   int spaceIndex = upperLine.indexOf(' ');
   if (spaceIndex > 0) {
     float newKi = line.substring(spaceIndex + 1).toFloat();
     if (newKi >= 0) {
       Ki = newKi;
       Serial.print("Updated Ki to ");
       Serial.println(Ki);
     } else {
       Serial.println("KI value must be >= 0");
     }
   } else {
     Serial.println("Format: KI 0.015");
   }
   return;
 }

 // Numeric input
 int val = line.toInt();
 if (val > 0) {
   if (targetRPM == 0) {
     targetRPM = val;
     Serial.print("targetRPM set to ");
     Serial.println(targetRPM);
     Serial.println("Enter spin time (seconds), press Enter:");
   } else if (spinTimeMs == 0) {
     spinTimeMs = (unsigned long)val * 1000UL;
     Serial.print("spinTime set to ");
     Serial.print(val);
     Serial.println(" seconds");
     Serial.println("Type 's' to start.");
   } else {
     targetRPM = val;
     Serial.print("updated targetRPM to ");
     Serial.println(targetRPM);
   }
 }
}

// RPM UPDATE

bool updateRPM() {
 unsigned long now = millis();
 if (now - lastRPMTime < rpmWindowMs) return false;

 unsigned long dt = now - lastRPMTime;
 lastRPMTime = now;

 noInterrupts();
 unsigned long counts = rotationCounter;
 rotationCounter = 0;
 interrupts();

 float dtSec = dt / 1000.0;
 float rawRPM = 0.0;

 if (dtSec > 0) {
   rawRPM = ((counts / pulsesPerRev) / dtSec) * 60.0;
 }

 if (filteredRPM == 0) {
   filteredRPM = rawRPM;
 }

 float delta = rawRPM - filteredRPM;

 if (delta > maxDeltaRPMPerWindow) {
   rawRPM = filteredRPM + maxDeltaRPMPerWindow;
 }
 if (delta < -maxDeltaRPMPerWindow) {
   rawRPM = filteredRPM - maxDeltaRPMPerWindow;
 }

 filteredRPM = rawRPM;
 measuredRPM = filteredRPM;

 if (controlRPM == 0) {
   controlRPM = measuredRPM;
 } else {
   controlRPM = controlAlpha * controlRPM + (1.0 - controlAlpha) * measuredRPM;
 }

 updateDisplayRPM(measuredRPM);
 checkSensorFailure(counts);

 return true;
}

// PI CONTROL LOOP

void updateControlAndMotor() {
 float error = (float)targetRPM - controlRPM;

 if (targetRPM > 0 && controlRPM >= integralEnableFraction * targetRPM) {
   integralError += error;
 }

 integralError = constrain(integralError, -integralMax, integralMax);

 float controlEffort = Kp * error + Ki * integralError;

 float newPWMFloat = (float)currentPWM + controlEffort;
 int newPWM = (int)newPWMFloat;
 newPWM = constrain(newPWM, 0, 255);

 currentPWM = newPWM;
 analogWrite(motorPWMPin, currentPWM);

 Serial.print("Err: ");
 Serial.print(error);
 Serial.print("  Int: ");
 Serial.print(integralError);
 Serial.print("  RPMraw: ");
 Serial.print(measuredRPM);
 Serial.print("  RPMctrl: ");
 Serial.print(controlRPM);
 Serial.print("  target: ");
 Serial.print(targetRPM);
 Serial.print("  PWM: ");
 Serial.println(currentPWM);
}

// RAMPDOWN
void updateRampdown() {
 if (currentPWM > 0) {
   currentPWM -= rampdownStepPWM;
   if (currentPWM < 0) currentPWM = 0;
   analogWrite(motorPWMPin, currentPWM);
 } else {
   analogWrite(motorPWMPin, 0);
 }

 if (measuredRPM <= stopRPMThreshold) {
   lowRPMCount++;
 } else {
   lowRPMCount = 0;
 }

 if (lowRPMCount >= stopConfirmWindows) {
   state = IDLE;
   analogWrite(motorPWMPin, 0);

   if (!printedDoneOnce) {
     Serial.println("RUN_DONE");
     printedDoneOnce = true;
   }
 } else {
   Serial.print("RAMPDOWN  RPM: ");
   Serial.print(measuredRPM);
   Serial.print("  PWM: ");
   Serial.println(currentPWM);
 }
}


// TIME CHECK

void checkTimeAndTransitionIfDone() {
 if (state == RUNNING) {
   if (millis() - startTime >= spinTimeMs) {
     state = RAMPDOWN;
     integralError = 0.0;
     lowRPMCount = 0;
     printedDoneOnce = false;
     Serial.println("Time complete: ramping down...");
   }
 }
}

// SETUP

void setup() {
 Serial.begin(9600);
 while (!Serial);
 Serial.setTimeout(25);

 lcd.init();
 lcd.backlight();
 lcd.clear();
 lcdPrintPadded(0, 0, "Centrifuge Ready");
 lcdPrintPadded(0, 1, "Enter RPM");

 if (useInterruptPullup) {
   pinMode(interruptPin, INPUT_PULLUP);
 } else {
   pinMode(interruptPin, INPUT);
 }

 pinMode(motorPWMPin, OUTPUT);
 analogWrite(motorPWMPin, 0);

 attachInterrupt(digitalPinToInterrupt(interruptPin), rotationISR, interruptEdgeMode);

 Serial.println("V3 Main Code Ready (PI + filters + LCD + RUN_DONE)");
 Serial.print("Current Kp = ");
 Serial.println(Kp);
 Serial.print("Current Ki = ");
 Serial.println(Ki);
 Serial.println("Enter target RPM (ex: 1500), press Enter:");
}

// MAIN LOOP

void loop() {
 handleSerial();

 bool newRPM = updateRPM();

 if (newRPM) {
   updateLCD();

   if (state == RUNNING) {
     updateControlAndMotor();
   } else if (state == RAMPDOWN) {
     updateRampdown();
   }
 }

 checkTimeAndTransitionIfDone();

 if (state == IDLE) {
   analogWrite(motorPWMPin, 0);
 }
}

