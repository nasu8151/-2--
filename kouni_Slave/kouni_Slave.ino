#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define CHANNEL 1
// 第一象限にあるものを0, 第二を1, 第三を2, 第四を3に設定すること
#define SLAVENUM 0

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


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  String Prefix = "Slave:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }

  // STAモードのチャンネルも合わせる（重要）
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP_STA);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

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

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); 
  long recieveddata = data[0 + SLAVENUM * 4] + (data[1 + SLAVENUM * 4] << 8) + (data[2 + SLAVENUM * 4] << 16) + (data[3 + SLAVENUM * 4] << 24);
  for (int i = 0; i < sizeof(data); i++) {
    Serial.print(data[i]); Serial.print(", ");
  }
  Serial.println("");
  Serial.println(recieveddata);
  target = recieveddata;
}

void loop() {
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
