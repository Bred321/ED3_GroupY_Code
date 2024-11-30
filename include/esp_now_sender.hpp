#ifndef ESP_NOW_HPP
#define ESP_NOW_HPP

#include <Arduino.h>
#include "WIFI.h"
#include <esp_now.h>

typedef struct struct_message {
  double v1_bottom_left;
  double v2_bottom_right;
  double v3_top_left;
  double v4_top_right;
} struct_message;


void OnDataSent(const uint8_t*, esp_now_send_status_t);
void esp_now_setup();
void esp_now_send_message();

#endif // HEADER_HPP