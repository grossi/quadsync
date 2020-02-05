#include <esp_now.h>
#include <WiFi.h>
#include <mavlink.h>

static uint8_t broadcastAddress[] = {0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF};

int msgDelay = 0;
bool waiting = false;
unsigned long timer;

void onReceiveData(const uint8_t *mac, const uint8_t *data, int len) {
  msgDelay = millis() - timer;

  Serial.print("Received data, delay: ");
  Serial.println(msgDelay);

  waiting = false;

  mavlink_message_t msg;
  mavlink_status_t stats; 

  for(int i = 0; i < len; i++) {
    uint8_t c = data[i];

    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &stats)) {
      /**
       * Decodes MavLink message
       */
    }
  }
}

void setup() {
  Serial.begin(115200);
 
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_peer_info_t peerInfo;
   
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(onReceiveData);
}

void loop() {
  if(!waiting) {
    int requestMsg = 19;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &requestMsg, sizeof(int));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      waiting = true;
      /**
       * Starts the timer after sending the request message
       */
      timer = millis();
    }
    else {
      Serial.println("Error sending the data");
      delay(100);
    }
  } else {
    /** 
     * If it doesn't receiver any message in 1 second,
     * reverts back to not waiting to send another
     * request signal
     * 
     */
    if(millis() - timer > 1000) {
      Serial.println("No message for 1000ms, sending another signal");
      waiting = false;
    }
  }
}
