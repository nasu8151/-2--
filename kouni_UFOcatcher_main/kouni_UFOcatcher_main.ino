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


        +
    0--------1
    |        |
  - |        | +
    |        |
    3--------2
        -
       ↓
       前
*/

// ボビン　一周   100mm
// モーター一周   960カウント
// 糸1mmあたり9.6カウント

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
int target_z = 180;

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
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slaves[SlaveCnt].peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        slaves[SlaveCnt].channel = CHANNEL; // pick a channel
        slaves[SlaveCnt].encrypt = 0; // no encryption
        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) {
    Serial.print(SlaveCnt); Serial.println(" Slave(s) found, processing..");
  } else {
    Serial.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) slaves[i].peer_addr[ii], HEX);
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
void sendData(long data_send_0, long data_send_1, long data_send_2, long data_send_3) {
  uint8_t data[16] = { (uint8_t)(data_send_0 >> 24), (uint8_t)(data_send_0 >> 16), (uint8_t)(data_send_0 >> 8), (uint8_t)(data_send_0 >> 0),
                       (uint8_t)(data_send_1 >> 24), (uint8_t)(data_send_1 >> 16), (uint8_t)(data_send_1 >> 8), (uint8_t)(data_send_1 >> 0),
                       (uint8_t)(data_send_2 >> 24), (uint8_t)(data_send_2 >> 16), (uint8_t)(data_send_2 >> 8), (uint8_t)(data_send_2 >> 0),
                       (uint8_t)(data_send_3 >> 24), (uint8_t)(data_send_3 >> 16), (uint8_t)(data_send_3 >> 8), (uint8_t)(data_send_3 >> 0) };
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = slaves[i].peer_addr;
    if (i == 0) {  // print only for first slave
      Serial.print("Sending: ");
      Serial.printf("%d, %d, %d, %d\r\n", data_send_0, data_send_1, data_send_2, data_send_3);
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
  const int frame_x = 1300 / 2;
  const int frame_y = 1300 / 2;
  const int frame_z = 1300;

  const int catcher_center_adjust_x = abs(36);  //absにする必要はないけどわかりやすいからやってる。
  const int catcher_center_adjust_y = abs(36);  //アームの中心からワイヤ固定点までの距離

  const int catcher_height_adjust_z = 180;  //一番おろした時の地面からワイヤ固定点までの高さ
  const long default_length = calculate_length(frame_x - catcher_center_adjust_x, frame_y - catcher_center_adjust_y, frame_z - catcher_height_adjust_z);

  //data0
  data0 = calculate_length(-frame_x - (target_x - catcher_center_adjust_x), frame_y - (target_y + catcher_center_adjust_x), 1300 - target_z) - default_length;
  //data1
  data1 = calculate_length(frame_x - (target_x + catcher_center_adjust_x), frame_y - (target_y + catcher_center_adjust_x), 1300 - target_z) - default_length;
  //data2
  data2 = calculate_length(frame_x - (target_x + catcher_center_adjust_x), -frame_y - (target_y - catcher_center_adjust_x), 1300 - target_z) - default_length;
  //data3
  data3 = calculate_length(-frame_x - (target_x - catcher_center_adjust_x), -frame_y - (target_y - catcher_center_adjust_x), 1300 - target_z) - default_length;
}

long calculate_length(long x, long y, long z) {

  x = abs(x);  //二乗するから意味ないけど
  y = abs(y);  //やりたくね？
  z = abs(z);

  long length = (long)(sqrt((double)x * (double)x + (double)y * (double)y+(double)z * (double)z) + 0.5); //sqrtはdoubleで計算する必要あり、それに0.5を足して四捨五入、そのうえでlong型に丸め込んでいる。
  length = (long)((float)length * 9.6);
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

void get_target() {
  target_x = 0;
  target_y = 0;
  target_z = 1000;
}

void loop() {
    // In the loop we scan for slave
    ScanForSlave();

    if (SlaveCnt > 0) {  // check if slave channel is defined
      manageSlave();
      get_target();
      calculate_data_to_send();
      sendData(data0, data1, data2, data3);
    } else {
      // No slave found to process
      Serial.println("No slave found to process");
    }

    // wait for 3seconds to run the logic again
    delay(100);
}
