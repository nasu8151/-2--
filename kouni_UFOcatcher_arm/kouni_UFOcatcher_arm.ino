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
#include <ESP32Servo.h>

#define CHANNEL 1


String inputBuffer = "";  // 入力バッファ
bool inputReady = false;  // 1行読み込み完了フラグ
bool isUsingSerial = false;

bool data_received_flag = false;

uint8_t arm_state = 0;
bool open_or_close = false;  //false = 閉じる

#define SERVO_PIN_L D1
#define SERVO_PIN_R D0

const int close_degree = 10;
const int open_degree = 35;

Servo servo_L;
Servo servo_R;


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
  servo_L.attach(SERVO_PIN_L);
  servo_R.attach(SERVO_PIN_R);

  // initialize digital pin led as an output
  inputBuffer.reserve(32);  // メモリを事前確保（長い数にも対応）

  servo_L.write(90 - close_degree);
  servo_R.write(90 + close_degree);
  open_or_close = false;
}

void loop() {
  
  if (data_received_flag) {
    if (arm_state != open_or_close) {  //受け取った信号と状態が違ったら

      if (arm_state == false) {
        for (int i = open_degree; i > close_degree; i--) {
          servo_L.write(90 - i);
          servo_R.write(90 + i);
          delay(20);
        }
        open_or_close = false;

      } else {
         for (int i = close_degree; i < open_degree; i++) {
          servo_L.write(90 - i);
          servo_R.write(90 + i);
          delay(20);
        }
        open_or_close = true;

      }
    }
    Serial.println(arm_state);
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
  uint8_t recieveddata = data[16];
  for (int i = 0; i < data_len; i++) {
    Serial.print(data[i]);
    Serial.print(", ");
  }
  Serial.println("");
  Serial.println(recieveddata);

  arm_state = recieveddata;

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
