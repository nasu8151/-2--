// define led according to pin diagram
#define PWM     4
#define PWM1    5
#define PHASE_A 6
#define PHASE_B 7

#define CLIP(X, M) (X) > 0 ? MIN((X), (M)) : MAX((X), -(M))

volatile long counter;
long target = 0;

long prevMillis = 0;
unsigned long prevMicros = 0;

const float KP = 20.0f;
const float KI = 2.0f;
const float KD = 18.0f;

float I = 0.0f;
long  prevP = 0.0f;

String inputBuffer = "";   // 入力バッファ
bool inputReady = false;   // 1行読み込み完了フラグ
bool isUsingSerial = false;

void IRAM_ATTR HandlePhaseA() {
  if (digitalRead(PHASE_B) == 0) {
    counter++;
  } else {
    counter--;
  }
}
/*
void IRAM_ATTR HandlePhaseB() {
  if (digitalRead(PHASE_A) == 1) {
    counter++;
  } else {
    counter--;
  }
}*/

void setup() {
  // initialize digital pin led as an output
  pinMode(PWM, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PHASE_A, INPUT);
  pinMode(PHASE_B, INPUT);
  
  ledcSetup(0, 25000, 8);
  ledcAttachPin(PWM, 0);
  ledcSetup(1, 25000, 8);
  ledcAttachPin(PWM1, 1);

  attachInterrupt(digitalPinToInterrupt(PHASE_A), HandlePhaseA, RISING);
  // attachInterrupt(digitalPinToInterrupt(PHASE_B), HandlePhaseB, RISING);

  Serial.begin(115200);
  Serial.println("Hello world!!");

  inputBuffer.reserve(32); // メモリを事前確保（長い数にも対応）
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    Serial.write(c);

    if (c == '\n') {
      inputReady = true;
      break;
    } else if (c != '\r') {
      isUsingSerial = true;
      inputBuffer += c;
    }
  }

  if (inputReady) {
    target = inputBuffer.toInt(); // long型に変換（実際には toInt() は long を返す）

    // バッファとフラグをリセット
    inputBuffer = "";
    inputReady = false;
    isUsingSerial = false;
  }

  /* PID制御部分 */

  unsigned long t = micros();
  float dt = (t - prevMicros) / 1000000.0f;
  long  P =  target - counter;
        I += (float)P * dt;
  float D =  (float)(P - prevP) / dt;

  prevMicros = t;
  prevP = P;

  long ledcVal = CLIP(P * KP + I * KI, 220L);
  I = CLIP(I, 2000.0f);

  if (P > 0) {
    ledcWrite(0, 0);
    ledcWrite(1, ledcVal);
  } else {
    ledcWrite(0, -ledcVal);
    ledcWrite(1, 0);
  }

  if (millis() - prevMillis > 20 && !isUsingSerial) {
    Serial.printf("target:%d, counter:%d, speed:%d, I:%.1f, D:%.3f\r\n", target, counter, ledcVal, I, D);
    prevMillis = millis();
  }
  delay(2);
}