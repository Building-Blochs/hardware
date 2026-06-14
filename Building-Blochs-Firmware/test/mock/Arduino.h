#pragma once
// Minimal Arduino.h mock for native (host) unit tests.
// Only provides what the math under test actually needs.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#ifndef PI
#define PI M_PI
#endif

// Eigen pulls in abs() — make sure std versions are available
using std::abs;
using std::sqrt;
using std::cos;
using std::sin;
using std::acos;

// millis() stub — not used in math, but needed if headers pull it in
inline unsigned long millis() { return 0; }

// Stub for Serial
struct _SerialStub {
    template<typename... Args>
    void printf(const char*, Args...) {}
    void begin(int) {}
} Serial;
