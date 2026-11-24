// =============================================================
//  Motor 1 test : OpenCR + Cytron MDD10A + JGB37-520 (hall encoder)
// -------------------------------------------------------------
//  Wiring (change the pin numbers below if yours differ)
//    MDD10A PWM1  -> OpenCR D5
//    MDD10A DIR1  -> OpenCR D6
//    MDD10A GND   -> OpenCR GND   (common ground is required!)
//    Encoder A    -> OpenCR D2    (interrupt pin)
//    Encoder B    -> OpenCR D3
//    Encoder VCC  -> OpenCR 3.3V  (keeps the signal at 3.3V logic)
//    Encoder GND  -> OpenCR GND
//
//  Usage (Serial Monitor, 115200 baud, "Newline")
//    Type a number from -255 to 255 and press Enter
//      e.g. 80  -> forward slowly
//          -80  -> reverse slowly
//            0  -> stop
//    Type r  -> reset the encoder count to 0
// =============================================================

const int PIN_PWM = 5;
const int PIN_DIR = 6;
const int PIN_ENC_A = 2;
const int PIN_ENC_B = 3;

// Encoder counts per ONE turn of the output shaft.
// Find it in step 4: reset with 'r', turn the wheel exactly one turn
// by hand, then put the printed count here.
float COUNTS_PER_REV = 0;   // 0 = unknown yet (RPM is not printed)

volatile long encCount = 0;
long lastCount = 0;
unsigned long lastPrint = 0;
int currentCmd = 0;

void encoderISR() {
  // On each rising edge of A, B tells the direction
  if (digitalRead(PIN_ENC_B) == HIGH) encCount++;
  else                                encCount--;
}

void setMotor(int cmd) {
  cmd = constrain(cmd, -255, 255);
  digitalWrite(PIN_DIR, cmd >= 0 ? HIGH : LOW);
  analogWrite(PIN_PWM, abs(cmd));
  currentCmd = cmd;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  // OpenCR: pin D2 is external interrupt 0.
  // If the count never changes, check the OpenCR e-manual interrupt pin table.
  attachInterrupt(0, encoderISR, RISING);

  setMotor(0);
  delay(1000);
  Serial.println("Motor1 test ready. Enter -255~255, or 'r' to reset count.");
}

void loop() {
  // ---- read command ----
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s == "r") {
      noInterrupts(); encCount = 0; interrupts();
      lastCount = 0;
      Serial.println("count reset");
    } else if (s.length() > 0) {
      setMotor(s.toInt());
      Serial.print("cmd = "); Serial.println(currentCmd);
    }
  }

  // ---- print every 200 ms ----
  unsigned long now = millis();
  if (now - lastPrint >= 200) {
    noInterrupts(); long c = encCount; interrupts();
    long delta = c - lastCount;
    float dt = (now - lastPrint) / 1000.0;

    Serial.print("cmd: ");    Serial.print(currentCmd);
    Serial.print("\tcount: "); Serial.print(c);
    Serial.print("\tcps: ");   Serial.print(delta / dt, 1);   // counts per second
    if (COUNTS_PER_REV > 0) {
      Serial.print("\trpm: ");
      Serial.print(delta / dt / COUNTS_PER_REV * 60.0, 1);
    }
    Serial.println();

    lastCount = c;
    lastPrint = now;
  }
}
