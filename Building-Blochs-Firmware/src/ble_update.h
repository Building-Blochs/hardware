#pragma once
class BLEServer;
void ble_update_setup(BLEServer *server);
void ble_update_loop();
void ble_update_disconnect();
bool ble_update_active();
