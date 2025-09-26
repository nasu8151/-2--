/**
   ESPNOW - Basic communication - Slave
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and multiple ESP32 Slaves
   Description: This sketch consists of the code for the Slave module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Slave >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave(s)

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/

/* 立方体
        0----------1
       /          /|
      /          / |
     3----------2  |
     |          |  |
     |          | /
     |          |/
     +----------+
         ↓
        前
*/


#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define CHANNEL 1
#define SLAVENUM 3  // Slave番号。0ならMasterのdata0を、3ならMasterのdata3を受信する

//PID
#define PWM 4
#define PWM1 5
#define PHASE_A 6
#define PHASE_B 7

#define CLIP(X, M) (X) > 0 ? MIN((X), (M)) : MAX((X), -(M))

volatile long counter;
long target = 0;

long prevMillis = 0;
unsigned long prevMicros = 0;
unsigned long pretime_receive;  //前回の通信

const float KP = 20.0f;
const float KI = 2.0f;
const float KD = 18.0f;

float I = 0.0f;
long prevP = 0.0f;

String inputBuffer = "";  // 入力バッファ
bool inputReady = false;  // 1行読み込み完了フラグ
bool isUsingSerial = false;

bool data_received_flag = false;

void IRAM_ATTR HandlePhaseA() {
  if (digitalRead(PHASE_B) == 0) {
    counter++;
  } else {
    counter--;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP_STA);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());
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
  inputBuffer.reserve(32);  // メモリを事前確保（長い数にも対応）
  pretime_receive = millis();
}

void loop() {
  PID_move(target);
  if (data_received_flag) {
    pretime_receive = millis();
    data_received_flag = false;
  }
/*
 else if (millis() - pretime_receive > 500) {  //0.5秒通信がなかったら。
    PID_move(0);
  }
*/
}





// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  Serial.print("Last Packet Recv Data: ");
  long recieveddata = (data[0 + SLAVENUM * 4] << 24) + (data[1 + SLAVENUM * 4] << 16) + (data[2 + SLAVENUM * 4] << 8) + (data[3 + SLAVENUM * 4] << 0);
  for (int i = 0; i < data_len; i++) {
    Serial.print(data[i]);
    Serial.print(", ");
  }
  Serial.println("");
  Serial.println(recieveddata);

  target = recieveddata;

  data_received_flag = true;  //メインループ用にフラグを立てる
}

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
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

void PID_move(long target_val) {
  unsigned long t = micros();
  float dt = (t - prevMicros) / 1000000.0f;
  long P = target_val - counter;
  I += (float)P * dt;
  float D = (float)(P - prevP) / dt;

  prevMicros = t;
  prevP = P;

  long ledcVal = CLIP(P * KP + I * KI, 220L);
  I = CLIP(I, 2000.0f);

  if (ledcVal > 0) {
    ledcWrite(0, 0);
    ledcWrite(1, ledcVal);
  } else {
    ledcWrite(0, -ledcVal);
    ledcWrite(1, 0);
  }
  /*
  if (millis() - prevMillis > 20 && !isUsingSerial) {
    Serial.printf("target:%d, counter:%d, speed:%d, I:%.1f, D:%.3f\r\n", target, counter, ledcVal, I, D);
    prevMillis = millis();
  }
  */
  delay(2);
}
