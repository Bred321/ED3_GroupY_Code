#include <Arduino.h>
#include "WIFI.h"
#include <esp_now.h>
#include "esp_now_sender.hpp"


int int_num = 0;
float float_num = 0;
bool bool_num = false;

uint8_t broadcastAddress[] = {0x48, 0xE7, 0x29, 0x95, 0xF5, 0x44}; // to be modified later

// Create a structured object
struct_message message;

// Peer info
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status: ");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Successful delivery" : "Failed delivery");
}


void esp_now_setup() {
  // Set ESP32 as a Wi-Fi station
  WiFi.mode(WIFI_MODE_STA);

  // Fetch the MAC address
  // Serial.print("MAC address: ");
  // Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo)!= ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void esp_now_send_message() {
  // put your main code here, to run repeatedly:

  // Function validation
  int_num = random(1, 100);
  message.v1_bottom_left = int_num * 1.2;
  message.v2_bottom_right = int_num * 1.5;
  message.v3_top_left = int_num * 1.7;
  message.v4_top_right = int_num * 1.9;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&message, sizeof(message));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(1000);
}

