/**
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and multiple ESP32 Slaves
   Description: This sketch consists of the code for the Master module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Master >>

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


  // Sample Serial log with 1 master & 2 slaves
      Found 12 devices 
      1: Slave:24:0A:C4:81:CF:A4 [24:0A:C4:81:CF:A5] (-44)
      3: Slave:30:AE:A4:02:6D:CC [30:AE:A4:02:6D:CD] (-55)
      2 Slave(s) found, processing..
      Processing: 24:A:C4:81:CF:A5 Status: Already Paired
      Processing: 30:AE:A4:2:6D:CD Status: Already Paired
      Sending: 9
      Send Status: Success
      Last Packet Sent to: 24:0a:c4:81:cf:a5
      Last Packet Send Status: Delivery Success
      Send Status: Success
      Last Packet Sent to: 30:ae:a4:02:6d:cd
      Last Packet Send Status: Delivery Success

*/

#include <esp_now.h>
#include <WiFi.h>
#include <math.h>  // sqrt()を使うために必要

// Global copy of slave
#define NUMSLAVES 20
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int SlaveCnt = 0;

long data0;  // 送りたいデータの例
long data1;
long data2;
long data3;

int target_x = 0;
int target_y = 0;
int target_z = 180;  //mmだと180

#define CHANNEL 1
#define PRINTSCANRESULTS 0

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

// Scan for slaves in AP mode
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset slaves
  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
          for (int ii = 0; ii < 6; ++ii) {
            slaves[SlaveCnt].peer_addr[ii] = (uint8_t)mac[ii];
          }
        }
        slaves[SlaveCnt].channel = CHANNEL;  // pick a channel
        slaves[SlaveCnt].encrypt = 0;        // no encryption
        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) {
    Serial.print(SlaveCnt);
    Serial.println(" Slave(s) found, processing..");
  } else {
    Serial.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii) {
        Serial.print((uint8_t)slaves[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(slaves[i].peer_addr);
      if (exists) {
        // Slave already paired.
        Serial.println("Already Paired");
      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&slaves[i]);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
}

// send data
void sendData() {
  uint8_t data[16] = { (uint8_t)(data0 >> 24), (uint8_t)(data0 >> 16), (uint8_t)(data0 >> 8), (uint8_t)(data0 >> 0),
                       (uint8_t)(data1 >> 24), (uint8_t)(data1 >> 16), (uint8_t)(data1 >> 8), (uint8_t)(data1 >> 0),
                       (uint8_t)(data2 >> 24), (uint8_t)(data2 >> 16), (uint8_t)(data2 >> 8), (uint8_t)(data2 >> 0),
                       (uint8_t)(data3 >> 24), (uint8_t)(data3 >> 16), (uint8_t)(data3 >> 8), (uint8_t)(data3 >> 0) };
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = slaves[i].peer_addr;
    if (i == 0) {  // print only for first slave
      Serial.print("Sending: ");
      Serial.println(data1);
    }
    esp_err_t result = esp_now_send(peer_addr, data, sizeof(data));
    Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }
    delay(100);
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void calculate_data_to_send() {
  //桁がえぐいことなってまうからmmが最小単位
  Serial.println("start culclating");

  const int frame_x = 1280 / 2;  //mmだと1300
  const int frame_y = 1280 / 2;  //mmだと1300
  const int frame_z = 1400;      //mmだと1300

  const int catcher_center_adjust_x = abs(36);  //mmだと36  //absにする必要はないけどわかりやすいからやってる。
  const int catcher_center_adjust_y = abs(36);  //mmだと36  //アームの中心からワイヤ固定点までの距離

  const int catcher_height_adjust_z = 180;  //mmだと180  //一番おろした時の地面からワイヤ固定点までの高さ
  Serial.println("defalt cuculate");
  const long default_length = calculate_length(frame_x - catcher_center_adjust_x, frame_y - catcher_center_adjust_y, frame_z - catcher_height_adjust_z);

  //data0
  Serial.println("data0 culclating");
  data0 = calculate_length(-frame_x - (target_x - catcher_center_adjust_x), frame_y - (target_y + catcher_center_adjust_x), frame_z - target_z) - default_length;
  //data1
  Serial.println("data1 culclating");
  data1 = calculate_length(frame_x - (target_x + catcher_center_adjust_x), frame_y - (target_y + catcher_center_adjust_x), frame_z - target_z) - default_length;
  //data2
  Serial.println("data2 culclating");
  data2 = calculate_length(frame_x - (target_x + catcher_center_adjust_x), -frame_y - (target_y - catcher_center_adjust_x), frame_z - target_z) - default_length;
  //data3
  Serial.println("data3 culclating");
  data3 = calculate_length(-frame_x - (target_x - catcher_center_adjust_x), -frame_y - (target_y - catcher_center_adjust_x), frame_z - target_z) - default_length;

  /*
  data0 = -1000;
  data1 = -1000;
  data2 = -1000;
  data3 = -1000;
  */
  Serial.print("default_length: ");
  Serial.println(default_length);
  Serial.print("data0: ");
  Serial.println(data0);
  Serial.print("data1: ");
  Serial.println(data1);
  Serial.print("data2: ");
  Serial.println(data2);
  Serial.print("data3: ");
  Serial.println(data3);
}

long calculate_length(long x, long y, long z) {

  Serial.println("culclate length");

  long length = (long)(sqrt((double)x * (double)x + (double)y * (double)y + (double)z * (double)z) + 0.5);  //sqrtはdoubleで計算する必要あり、それに0.5を足して四捨五入、そのうえでlong型に丸め込んでいる。
  // length = length * 181; cmの場合

  length = (long)((float)length * 18.1);

  return length;
}

void get_target() {
  /*
  target_x = 400;
  target_y = 400;
  target_z = 500;
  */
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // 改行まで読み取る
    input.trim();                                 // 前後の空白や改行を除去

    int index1 = input.indexOf(',');
    int index2 = input.indexOf(',', index1 + 1);

    if (index1 > 0 && index2 > 0 ) {
      target_x = input.substring(0, index1).toInt();
      target_y = input.substring(index1 + 1, index2).toInt();
      target_z = input.substring(index2 + 1).toInt();

      // 確認用に出力
      Serial.print("target_x = ");
      Serial.println(target_x);
      Serial.print("target_y = ");
      Serial.println(target_y);
      Serial.print("target_z = ");
      Serial.println(target_z);
    }
  }
}

void setup() {
  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow/Multi-Slave/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (SlaveCnt > 0) {  // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    manageSlave();
    // pair success or already paired
    // Send data to device
    sendData();
  } else {
    // No slave found to process
  }
  get_target();
  calculate_data_to_send();

  // wait for 3seconds to run the logic again
  delay(100);
}
