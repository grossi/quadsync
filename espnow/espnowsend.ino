
#include <esp_now.h>

/**
 * This is the broadcastAddress
*/
static uint8_t broadcastAddress[] = {0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF};

void setup() {
  Serial.begin(57600);

  WiFi.mode(WIFI_STA);

/** 
 * Initializes esp-now
 */
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

/**
 * Creates the peer info with the broadcastAddress as the target
 * and no encryption
 */
  esp_now_peer_info_t peerInfo;
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

/**
 * Sends a teste message
 */

  char testMessage[] = "Hello World!";

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &testMessage, sizeof(testMessage));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void loop() { }
