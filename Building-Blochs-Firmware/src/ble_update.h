#pragma once
class BLEServer;
void ble_update_setup(BLEServer *server);
void ble_update_loop();
void ble_update_disconnect();
bool ble_update_active();
// Read from the main loop, where update packets are processed.
int ble_update_progress(); // -1 while waiting for the signed header, otherwise 0..100
bool ble_update_restarting();
