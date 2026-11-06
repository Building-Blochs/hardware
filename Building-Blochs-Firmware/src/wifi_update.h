#pragma once
#include <Arduino.h>

struct WifiUpdateChunk { int size; uint8_t bytes[1024]; }; // 0 = EOF, -1 = network error
void wifi_update_setup();
bool wifi_update_busy();
bool wifi_update_configure(const uint8_t *data, size_t size);
bool wifi_update_start(const char *url);
void wifi_update_cancel();
bool wifi_update_read(WifiUpdateChunk &chunk);
uint8_t wifi_update_state(); // 0 unconfigured, 1 connecting, 2 LAN only, 3 internet, 4 disconnected
struct WifiUpdateError { uint8_t stage; int32_t http; int32_t tls; };
WifiUpdateError wifi_update_error();
