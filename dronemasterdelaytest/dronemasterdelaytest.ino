#include <esp_now.h>
#include <WiFi.h>
#include <mavlink.h>
#include <SoftwareSerial.h>
#define GPS_INFO "GLOBAL_POSITION_INT_COV"
#define ESP32_SYSID 2
#define ESP32_COMPID 1

static uint8_t broadcastAddress[] = {0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF};

SoftwareSerial SerialMav(16, 17);

bool storageMsg = false;
mavlink_message_t globalMsg;

void onReceiveData(const uint8_t *mac, const uint8_t *data, int len) {
  int *dataId = (int*)data;

  /** 
   * It looks for the request signal 
   * which is 19
   */
  if(*dataId != 19){ return; }

  /**
   * Sending message to receiver
   */

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &globalMsg, sizeof(mavlink_message_t));

  if (result == ESP_OK) {
      Serial.println("Sent with success");
  }
  else {
      Serial.println("Error sending the data");
  }
  storageMsg = false;
}

void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  const int  maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_ALL};
  const uint16_t MAVRates[maxStreams] = {0x02};
    
  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(ESP32_SYSID, ESP32_COMPID, &msg, 1, 1, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);    
    SerialMav.write(buf, len);
  }
}

void _MavLink_receive() {
  mavlink_status_t stats;
  while(SerialMav.available()) {
    uint8_t c = SerialMav.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &globalMsg, &stats)) {
      storageMsg = true;
    }
  }
}

void setup() {
  Serial.begin(57600);
  SerialMav.begin(57600);

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
  
  Mav_Request_Data();
}

void loop() {
  if( storageMsg == false ) {
    _MavLink_receive();
  }
  delay(10);
}
