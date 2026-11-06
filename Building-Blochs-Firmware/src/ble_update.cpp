#include "ble_update.h"
#include "update_public_key.h"
#include "version.h"
#include "bloch.h"
#include "calibration.h"
#include <BLEDevice.h>
#include <esp_ota_ops.h>
#include <mbedtls/pk.h>
#include <mbedtls/sha256.h>
#include <atomic>

extern BlochSphere bloch;
namespace {
constexpr char SERVICE[] = "a0741000-7d16-4a22-b123-97fe36c04200";
struct Packet { uint16_t size; uint8_t bytes[244]; };
QueueHandle_t queue;
BLECharacteristic *status;
std::atomic<bool> active{false}, disconnected{false};
esp_ota_handle_t handle = 0;
const esp_partition_t *partition = nullptr;
uint8_t header[300]; // signed board/protocol marker, size, SHA256, RSA signature
uint32_t offset = 0, imageSize = 0, lastPacket = 0, rebootAt = 0;
mbedtls_sha256_context sha;
bool hashing = false;
uint32_t read32(const uint8_t *p) { return p[0] | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24; }
void report(uint8_t code) {
    uint8_t reply[5] = {code, uint8_t(offset), uint8_t(offset>>8), uint8_t(offset>>16), uint8_t(offset>>24)};
    status->setValue(reply, sizeof(reply));
}
void abortUpdate(uint8_t code) {
    if (handle) esp_ota_abort(handle);
    handle = 0;
    if (hashing) mbedtls_sha256_free(&sha);
    hashing = false;
    active = false;
    xQueueReset(queue);
    report(code);
}
bool verifyHeader() {
    if (memcmp(header, "BBOTA001", 8)) return false;
    imageSize = read32(header+8);
    partition = esp_ota_get_next_update_partition(nullptr);
    if (!partition || !imageSize || imageSize > partition->size) return false;
    uint8_t digest[32];
    mbedtls_sha256(header, 44, digest, 0);
    mbedtls_pk_context key;
    mbedtls_pk_init(&key);
    int rc = mbedtls_pk_parse_public_key(&key, reinterpret_cast<const uint8_t *>(UPDATE_PUBLIC_KEY), sizeof(UPDATE_PUBLIC_KEY));
    if (!rc) rc = mbedtls_pk_verify(&key, MBEDTLS_MD_SHA256, digest, sizeof(digest), header+44, 256);
    mbedtls_pk_free(&key);
    if (rc || esp_ota_begin(partition, imageSize, &handle) != ESP_OK) return false;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);
    hashing = true;
    return true;
}
class Writes : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        auto value = c->getValue();
        Packet packet{};
        if (value.length() < 1 || value.length() > sizeof(packet.bytes)) return;
        packet.size = value.length();
        memcpy(packet.bytes, value.c_str(), packet.size);
        // One outstanding packet is required. No flash/motor/GUI operations on the BLE task.
        xQueueSend(queue, &packet, 0);
    }
};
}
bool ble_update_active() { return active.load(); }
void ble_update_disconnect() { disconnected = true; }
void ble_update_setup(BLEServer *server) {
    queue = xQueueCreate(4, sizeof(Packet));
    auto service = server->createService(SERVICE);
    auto data = service->createCharacteristic("a0741001-7d16-4a22-b123-97fe36c04200", BLECharacteristic::PROPERTY_WRITE);
    data->setCallbacks(new Writes());
    status = service->createCharacteristic("a0741002-7d16-4a22-b123-97fe36c04200", BLECharacteristic::PROPERTY_READ);
    auto info = service->createCharacteristic("a0741003-7d16-4a22-b123-97fe36c04200", BLECharacteristic::PROPERTY_READ);
    info->setValue("CoreS3;" VERSIONSTR);
    report(0);
    service->start();
}
void ble_update_loop() {
    if (rebootAt) { if (int32_t(millis()-rebootAt)>=0) ESP.restart(); return; }
    if (disconnected.exchange(false)) { abortUpdate(0); return; }
    if (active && millis()-lastPacket > 60000) { abortUpdate(7); return; }
    Packet p;
    if (xQueueReceive(queue, &p, 0) != pdTRUE) return;
    lastPacket = millis();
    const uint8_t op = p.bytes[0];
    if (op == 1 && p.size == 1) {
        if (active) { report(1); return; }
        esp_ota_img_states_t state;
        const bool pending = esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY;
        if (pending || bloch.isBusy() || calibration_active()) { report(3); return; }
        offset = 0;
        imageSize = 0;
        active = true;
        report(1);
    } else if (op == 3 && p.size == 1) {
        abortUpdate(0);
    } else if (op == 2 && p.size > 5 && active) {
        const uint32_t position = read32(p.bytes+1);
        const size_t count = p.size-5;
        if (position != offset) { report(1); return; } // retry-safe acknowledgement
        if (offset < sizeof(header)) {
            if (count > sizeof(header)-offset) { abortUpdate(4); return; }
            memcpy(header+offset, p.bytes+5, count);
            offset += count;
            if (offset == sizeof(header) && !verifyHeader()) { abortUpdate(4); return; }
        } else {
            if (!handle || count > imageSize-(offset-sizeof(header))) { abortUpdate(4); return; }
            if (esp_ota_write(handle, p.bytes+5, count) != ESP_OK) { abortUpdate(5); return; }
            mbedtls_sha256_update(&sha, p.bytes+5, count);
            offset += count;
        }
        report(1);
    } else if (op == 4 && p.size == 1 && active) {
        if (!handle || offset != sizeof(header)+imageSize) { abortUpdate(4); return; }
        uint8_t digest[32];
        mbedtls_sha256_finish(&sha, digest);
        if (memcmp(digest, header+12, 32)) { abortUpdate(4); return; }
        esp_err_t result = esp_ota_end(handle);
        handle = 0;
        if (result != ESP_OK || esp_ota_set_boot_partition(partition) != ESP_OK) { abortUpdate(5); return; }
        mbedtls_sha256_free(&sha);
        hashing = false;
        report(2);
        rebootAt = millis()+2000;
    } else { report(6); }
}
