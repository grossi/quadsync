#include <esp_now.h>
#include <WiFi.h>
#include <mavlink.h>

static uint8_t broadcastAddress[] = {0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF};

int msgDelay = 0;
bool waiting = false;
unsigned long timer;

void onReceiveData(const uint8_t *mac, const uint8_t *data, int len) {
  Serial.println("Mac address of sender: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }

  Serial.println();
  msgDelay = millis() - timer;

  Serial.print("Received data, delay: ");
  Serial.println(msgDelay);

  waiting = false;

  mavlink_message_t msg;
  mavlink_status_t stats; 

  for(int i = 0; i < len; i++) {
    uint8_t c = data[i];
    
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &stats)) {
      Serial.print("Message id (msgid): ");
      Serial.println(msg.msgid);

      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);

          Serial.println("HeartBeat");
          Serial.print("system_status: ");
          Serial.println(hb.system_status);

          break;
        }
        case MAVLINK_MSG_ID_GPS_STATUS:
        {
          Serial.println("GPS_STATUS");
          __mavlink_gps_status_t gpsStatus;
          mavlink_msg_gps_status_decode(&msg, &gpsStatus);

          Serial.print("Number of satellites: ");
          Serial.println(gpsStatus.satellites_visible);
          break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
          Serial.println("GPS Position!");
          __mavlink_global_position_int_t gpsPos;
          mavlink_msg_global_position_int_decode(&msg, &gpsPos);

          Serial.print("Relative Altitude: ");
          Serial.println(gpsPos.relative_alt);
          Serial.print("Latitude: ");
          Serial.print(gpsPos.lat);
          Serial.print("  -  Longitude: ");
          Serial.println(gpsPos.lon);
          Serial.print("Altitude above MSL: ");
          Serial.println(gpsPos.alt);
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
          Serial.println("GPS Position!");
          __mavlink_gps_raw_int_t gpsPos;
          mavlink_msg_gps_raw_int_decode(&msg, &gpsPos);

          Serial.print("Number of satellites: ");
          Serial.println(gpsPos.satellites_visible);

          Serial.print("Latitude: ");
          Serial.print(gpsPos.lat);
          Serial.print("  -  Longitude: ");
          Serial.println(gpsPos.lon);
          Serial.print("Altitude above MSL: ");
          Serial.println(gpsPos.alt);
        }
      }
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
