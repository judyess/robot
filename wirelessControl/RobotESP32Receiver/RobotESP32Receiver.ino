// Robot Side (Receiver)
#include <esp_now.h>
#include <WiFi.h>

// *define the same data structure sent by the transmitter*
typedef struct data_struct {
  int a;
} data_struct;

data_struct myData;

void onDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Data received: ");
  Serial.println(len); 
  Serial.print("Data: ");
  Serial.println(myData.a);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
}
 
void loop() {
}