// Bench instrument only. Disconnect ESC battery and all three ESC signal wires.
// Loopback jumpers: 1->14, 2->15, 3->16. Emits real ESC-compatible test pulses.
// Keep ESCs disconnected. Restore the normal flight image after this test.
#include <Arduino.h>
#include <stdint.h>

const int m1Pin=0, m2Pin=1, m3Pin=2, m4Pin=3, m5Pin=4, m6Pin=5;
int m2_command_PWM=120, m3_command_PWM=120, m4_command_PWM=120;

struct Capture {
  volatile uint32_t rise=0, lastRise=0, widthMin=UINT32_MAX, widthMax=0;
  volatile uint32_t periodMin=UINT32_MAX, periodMax=0, count=0;
};
Capture captures[5]; // Physical inputs 13..17; neighbors reveal off-by-one jumper placement.

void captureEdge(unsigned index, bool high) {
  Capture &c=captures[index];
  const uint32_t now=micros();
  if (high) {
    if (c.lastRise) {
      const uint32_t period=now-c.lastRise;
      if (period<c.periodMin) c.periodMin=period;
      if (period>c.periodMax) c.periodMax=period;
    }
    c.rise=c.lastRise=now;
  } else if (c.rise) {
    const uint32_t width=now-c.rise;
    if (width<c.widthMin) c.widthMin=width;
    if (width>c.widthMax) c.widthMax=width;
    ++c.count;
    c.rise=0;
  }
}
void edge13() { captureEdge(0,digitalReadFast(13)); }
void edge14() { captureEdge(1,digitalReadFast(14)); }
void edge15() { captureEdge(2,digitalReadFast(15)); }
void edge16() { captureEdge(3,digitalReadFast(16)); }
void edge17() { captureEdge(4,digitalReadFast(17)); }

void resetCaptures() {
  noInterrupts();
  for (auto &c:captures) {
    c.rise=c.lastRise=c.count=0;
    c.widthMin=c.periodMin=UINT32_MAX;
    c.widthMax=c.periodMax=0;
  }
  interrupts();
}

void setup() {
  for (int pin=0;pin<=5;++pin) { digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
  pinMode(6,INPUT); // Servo PWM remains inactive in this instrument image.
  for (int pin=13;pin<=17;++pin) pinMode(pin,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(13),edge13,CHANGE);
  attachInterrupt(digitalPinToInterrupt(14),edge14,CHANGE);
  attachInterrupt(digitalPinToInterrupt(15),edge15,CHANGE);
  attachInterrupt(digitalPinToInterrupt(16),edge16,CHANGE);
  attachInterrupt(digitalPinToInterrupt(17),edge17,CHANGE);
  Serial.begin(500000);
}

void loop() {
  static unsigned previousPhase=255;
  static uint32_t phaseStarted=0, lastPrint=0;
  const uint32_t frameStarted=micros();
  const unsigned phase=(millis()/3000)%5;
  const int targets[5][3]={{120,120,120},{125,150,175},{150,175,200},
                           {175,200,225},{200,225,250}};
  if (phase!=previousPhase) {
    previousPhase=phase; phaseStarted=millis(); resetCaptures();
  }
  m2_command_PWM=targets[phase][0];
  m3_command_PWM=targets[phase][1];
  m4_command_PWM=targets[phase][2];
  commandMotors();

  if (Serial && millis()-phaseStarted>200 && millis()-lastPrint>=500 && Serial.availableForWrite()>=448) {
    uint32_t widthsMin[5], widthsMax[5], periodsMin[5], periodsMax[5], counts[5];
    noInterrupts();
    for (unsigned i=0;i<5;++i) {
      widthsMin[i]=captures[i].widthMin; widthsMax[i]=captures[i].widthMax;
      periodsMin[i]=captures[i].periodMin; periodsMax[i]=captures[i].periodMax;
      counts[i]=captures[i].count;
    }
    interrupts();
    char line[448];
    const int n=snprintf(line,sizeof(line),
      "TEST:1 PHASE:%u E2:%d E3:%d E4:%d "
      "W2MIN:%lu W2MAX:%lu P2MIN:%lu P2MAX:%lu N2:%lu "
      "W3MIN:%lu W3MAX:%lu P3MIN:%lu P3MAX:%lu N3:%lu "
      "W4MIN:%lu W4MAX:%lu P4MIN:%lu P4MAX:%lu N4:%lu "
      "I13WMIN:%lu I13WMAX:%lu I13N:%lu I17WMIN:%lu I17WMAX:%lu I17N:%lu\n",
      phase,m2_command_PWM,m3_command_PWM,m4_command_PWM,
      widthsMin[1],widthsMax[1],periodsMin[1],periodsMax[1],counts[1],
      widthsMin[2],widthsMax[2],periodsMin[2],periodsMax[2],counts[2],
      widthsMin[3],widthsMax[3],periodsMin[3],periodsMax[3],counts[3],
      widthsMin[0],widthsMax[0],counts[0],widthsMin[4],widthsMax[4],counts[4]);
    if (n>0 && n<(int)sizeof(line)) Serial.write((const uint8_t *)line,n);
    lastPrint=millis();
  }
  while ((uint32_t)(micros()-frameStarted)<500) {}
}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
  uint8_t wentLow = 0;
  bool flagM2 = false, flagM3 = false, flagM4 = false;

  // Only the three connected airframe outputs are driven. Unused outputs stay low.
  digitalWrite(m1Pin, LOW);
  digitalWrite(m5Pin, LOW);
  digitalWrite(m6Pin, LOW);
  digitalWrite(m2Pin, HIGH);
  digitalWrite(m3Pin, HIGH);
  digitalWrite(m4Pin, HIGH);
  const uint32_t pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 3) {
    const uint32_t elapsed = (uint32_t)(micros() - pulseStart);
    if (!flagM2 && elapsed >= (uint32_t)m2_command_PWM) {
      digitalWrite(m2Pin, LOW);
      ++wentLow;
      flagM2 = true;
    }
    if (!flagM3 && elapsed >= (uint32_t)m3_command_PWM) {
      digitalWrite(m3Pin, LOW);
      ++wentLow;
      flagM3 = true;
    }
    if (!flagM4 && elapsed >= (uint32_t)m4_command_PWM) {
      digitalWrite(m4Pin, LOW);
      ++wentLow;
      flagM4 = true;
    }
  }
}
