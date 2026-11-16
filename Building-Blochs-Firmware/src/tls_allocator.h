#pragma once
#include <stdint.h>
#include <esp_heap_caps.h>

// The bundled mbedTLS allocator is internal-RAM-only. BLE, Wi-Fi and the
// display leave too little contiguous internal RAM for TLS record buffers.
// Install this once, before any network/crypto tasks exist. Both heaps use
// heap_caps_free, including allocations made before the hook was installed.
inline void *tlsCalloc(size_t count, size_t size) {
    if (size && count > SIZE_MAX / size) return nullptr;
    void *memory = heap_caps_calloc(count, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!memory) memory = heap_caps_calloc(count, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return memory;
}
