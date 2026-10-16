#include <Arduino.h>

const uint8_t P1_REN = 22, P1_LEN = 21, P1_RPWM = 25, P1_LPWM = 23;
const uint8_t P2_REN = 26, P2_LEN = 13, P2_RPWM = 33, P2_LPWM = 32;
const uint8_t VALVE = 27, FLOAT_PIN = 35, FLOW = 14;
const uint8_t BUTTON = 34;
const int C1_P1_PWM = 127;
const int C1_P2_PWM = 255;
const int C2_P1_PWM = 255;
const int C2_P2_PWM = 127;
const uint32_t KICK_MS = 300;

const int VALVE_OPEN   = 255;
const int VALVE_CLOSED = 0;
const int FLOAT_ACTIVE = LOW;
const int BUTTON_PRESSED = LOW;
const bool FLOAT_STOPS_P2 = false;
const bool FLUSH_FLOAT_STOPS_P1 = true;
const float NOFLOW_LPM = 0.1f;

const uint32_t DEBOUNCE_MS     = 300;
const uint32_t BTN_DEBOUNCE_MS = 50;
const uint32_t BUFFER_MS       = 5000;
const uint32_t NOFLOW_S        = 7;
const float    CAL_K = 108.0f;

enum Mode { STATE1, BUFFER_IN, FLUSH, BUFFER_OUT };
const char *MODE_NAME[] = { "STATE1", "BUFFER_IN", "FLUSH", "BUFFER_OUT" };
Mode mode = STATE1;
uint32_t modeStart = 0;

bool pumpsRequested = false;
bool floatActive = false, floatRaw = false;
uint32_t floatChanged = 0;
bool btnRaw = false, btnState = false; uint32_t btnChanged = 0;
bool valveIsOpen = false;
bool p1NoFlowOff = false;
uint32_t zeroSecs = 0;

struct Pump {
  uint8_t ren, len, rpwm, lpwm;
  int cur;
  bool kicking; uint32_t kickEnd;
};
Pump pump1 = { P1_REN, P1_LEN, P1_RPWM, P1_LPWM, 0, false, 0 };
Pump pump2 = { P2_REN, P2_LEN, P2_RPWM, P2_LPWM, 0, false, 0 };

volatile uint32_t pulses = 0, lastUs = 0;
void IRAM_ATTR onPulse() {
  uint32_t now = micros();
  if (now - lastUs >= 300) { pulses++; lastUs = now; }
}
uint32_t readPulses() { noInterrupts(); uint32_t p = pulses; interrupts(); return p; }

uint32_t runStartMs = 0, runStartP = 0;
uint32_t nextTick = 0, lastP = 0, sec = 0;

void pumpTo(Pump &p, int pwm) {
  if (pwm == p.cur) return;
  if (pwm == 0) {
    analogWrite(p.rpwm, 0);
    digitalWrite(p.ren, LOW); digitalWrite(p.len, LOW);
    p.kicking = false;
  } else {
    digitalWrite(p.lpwm, LOW);
    digitalWrite(p.ren, HIGH); digitalWrite(p.len, HIGH);

    if (p.cur == 0 && pwm < 255) {
      analogWrite(p.rpwm, 255);
      p.kicking = true; p.kickEnd = millis() + KICK_MS;
    } else if (!p.kicking) {
      analogWrite(p.rpwm, pwm);
    }
  }
  p.cur = pwm;
}

void serviceKick(Pump &p, uint32_t now) {
  if (p.kicking && (int32_t)(now - p.kickEnd) >= 0) {
    p.kicking = false;
    if (p.cur > 0) analogWrite(p.rpwm, p.cur);
  }
}

void update() {
  bool openValve = false, want1 = false, want2 = false;
  int pwm1 = 0, pwm2 = 0;

  switch (mode) {
    case STATE1:
      openValve = !floatActive;
      want1 = pumpsRequested && !floatActive;
      want2 = pumpsRequested && !(FLOAT_STOPS_P2 && floatActive);
      pwm1 = C1_P1_PWM; pwm2 = C1_P2_PWM;
      break;
    case BUFFER_IN:
    case BUFFER_OUT:
      openValve = false; want1 = false; want2 = false;
      break;
    case FLUSH:
      openValve = false;
      want1 = pumpsRequested && !p1NoFlowOff && !(FLUSH_FLOAT_STOPS_P1 && floatActive);
      want2 = pumpsRequested;
      pwm1 = C2_P1_PWM; pwm2 = C2_P2_PWM;
      break;
  }

  analogWrite(VALVE, openValve ? VALVE_OPEN : VALVE_CLOSED);
  valveIsOpen = openValve;

  pumpTo(pump1, want1 ? pwm1 : 0);
  pumpTo(pump2, want2 ? pwm2 : 0);
}

void report() {
  Serial.printf("[%s] float_pin=%d  float=%s  valve=%s  P1=%d%%  P2=%d%%\n",
                MODE_NAME[mode], digitalRead(FLOAT_PIN),
                floatActive ? "ACTIVE" : "clear",
                valveIsOpen ? "OPEN" : "CLOSED",
                pump1.cur * 100 / 255, pump2.cur * 100 / 255);
}

void setMode(Mode m) {
  mode = m;
  modeStart = millis();
  if (m == FLUSH) { p1NoFlowOff = false; zeroSecs = 0; }
  Serial.printf(">>> %s\n", MODE_NAME[m]);

  if (m == BUFFER_IN)  Serial.println(">>> OPEN the manual flush valve now (5 s)");
  if (m == BUFFER_OUT) Serial.println(">>> CLOSE the manual flush valve now (5 s)");
  update();
  report();
}

void startRun() {
  runStartMs = millis();
  runStartP = lastP = readPulses();
  nextTick = runStartMs + 1000;
  sec = 0;
}

void endRun() {
  uint32_t total = readPulses() - runStartP;
  float secs   = (millis() - runStartMs) / 1000.0f;
  float avgHz  = secs > 0 ? total / secs : 0;
  float avgLpm = avgHz / CAL_K;
  float volL   = total / (CAL_K * 60.0f);

  Serial.printf("=== RUN END: duration=%.1f s  total_pulses=%lu  avg_hz=%.2f  "
                "avg_flow=%.3f L/min  total_volume=%.3f L ===\n",
                secs, (unsigned long)total, avgHz, avgLpm, volL);
}

bool buttonPressed(uint32_t now) {
  bool r = digitalRead(BUTTON) == BUTTON_PRESSED;
  if (r != btnRaw) { btnRaw = r; btnChanged = now; }
  if (r != btnState && now - btnChanged >= BTN_DEBOUNCE_MS) {
    btnState = r;
    return btnState;
  }
  return false;
}

void setup() {
  for (uint8_t p : {P1_REN, P1_LEN, P1_RPWM, P1_LPWM, P2_REN, P2_LEN, P2_RPWM, P2_LPWM}) {
    pinMode(p, OUTPUT); digitalWrite(p, LOW);
  }
  pinMode(FLOAT_PIN, INPUT);
  pinMode(BUTTON, INPUT);
  pinMode(FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW), onPulse, RISING);

  Serial.begin(115200);
  delay(500);
  floatRaw = floatActive = digitalRead(FLOAT_PIN) == FLOAT_ACTIVE;
  btnRaw = btnState = digitalRead(BUTTON) == BUTTON_PRESSED;
  update();
  Serial.println("\nType r = start, x = stop. Button steps STATE1 -> FLUSH -> STATE1.");
  report();
}

void loop() {
  uint32_t now = millis();

  if (Serial.available()) {
    char c = Serial.read();
    if ((c == 'r' || c == 'R') && !pumpsRequested) {
      pumpsRequested = true; mode = STATE1; update(); report(); startRun();
    }
    if ((c == 'x' || c == 'X') && pumpsRequested) {
      pumpsRequested = false; mode = STATE1; update(); report(); endRun();
    }
  }

  if (buttonPressed(now) && pumpsRequested) {
    if (mode == STATE1)     setMode(BUFFER_IN);
    else if (mode == FLUSH) setMode(BUFFER_OUT);
  }

  if (mode == BUFFER_IN  && now - modeStart >= BUFFER_MS) setMode(FLUSH);
  if (mode == BUFFER_OUT && now - modeStart >= BUFFER_MS) setMode(STATE1);

  serviceKick(pump1, now);
  serviceKick(pump2, now);

  bool r = digitalRead(FLOAT_PIN) == FLOAT_ACTIVE;
  if (r != floatRaw) { floatRaw = r; floatChanged = now; }
  if (r != floatActive && now - floatChanged >= DEBOUNCE_MS) {
    floatActive = r;
    update();
    report();
  }

  if (pumpsRequested && (int32_t)(now - nextTick) >= 0) {
    uint32_t p = readPulses();
    uint32_t dp = p - lastP;
    lastP = p;
    nextTick += 1000;

    Serial.printf("%lu-%lu s  [%s]  pulses=%lu  hz=%lu  flow=%.3f L/min\n",
                  (unsigned long)sec, (unsigned long)(sec + 1), MODE_NAME[mode],
                  (unsigned long)dp, (unsigned long)dp, dp / CAL_K);
    sec++;

    if (mode == FLUSH && !p1NoFlowOff) {
      zeroSecs = (dp / CAL_K <= NOFLOW_LPM) ? zeroSecs + 1 : 0;
      if (zeroSecs > NOFLOW_S && !valveIsOpen) {
        p1NoFlowOff = true;
        Serial.printf("!!! flow <= %.2f L/min for more than %lu s with valve closed -> Pump-1 OFF\n",
                      NOFLOW_LPM, (unsigned long)NOFLOW_S);
        update();
        report();
      }
    }
  }
}