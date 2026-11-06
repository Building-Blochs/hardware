#include "ble_update.h"
#include "receiver.h"
#include "gui.h"
#include "bloch.h"
#include "calibration.h"
#include "config.h"
#include "quat_tracker.h"
#include "settings.h"

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

extern BlochSphere bloch; // defined in main.cpp

// ---------------------------------------------------------------------------
// BLE UART service (Nordic UART Service UUIDs)
// ---------------------------------------------------------------------------
namespace
{
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

    BLEServer *pServer = nullptr;
    BLECharacteristic *pTxCharacteristic = nullptr;
    bool deviceConnected = false;
    bool oldDeviceConnected = false;
    String bleBuffer = "";

    class MyServerCallbacks : public BLEServerCallbacks
    {
        void onConnect(BLEServer *) override { deviceConnected = true; }
        void onDisconnect(BLEServer *) override { deviceConnected = false; ble_update_disconnect(); }
    };

    class MyCallbacks : public BLECharacteristicCallbacks
    {
        void onWrite(BLECharacteristic *pCharacteristic) override
        {
            if (ble_update_active()) return;
            String rxValue = pCharacteristic->getValue().c_str();
            if (rxValue.length() > 0)
                bleBuffer += rxValue;
        }
    };
} // namespace

// ---------------------------------------------------------------------------
// Net-rotation tracker — lets receiver_reset_bloch() undo all moves in one
// shortest-path rotation.
// ---------------------------------------------------------------------------
static QuatTracker netRotation;

// Execute a rotation and record it in the tracker.
static void executeRotation(Vector3d axis, double turns)
{
    bloch.rotate(axis, static_cast<int>(STEPS_PER_ROTATION * turns));
    netRotation.applyRotation(axis.data(), turns);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

static String machineName()
{
    return "Bloch-" + String(settings.machine_number);
}

void receiver_setup()
{
    BLEDevice::init(machineName());
    BLEDevice::setMTU(247);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();
    ble_update_setup(pServer);
    pServer->getAdvertising()->start();
}

bool receiver_apply_permute(const BlochSphere::permute_t permute)
{
    switch (permute)
    {
    case BlochSphere::PERMUTE_X:
        executeRotation(Vector3d(1, 0, 0), 0.5);
        return true;
    case BlochSphere::PERMUTE_Y:
        executeRotation(Vector3d(0, 1, 0), 0.5);
        return true;
    case BlochSphere::PERMUTE_Z:
        executeRotation(Vector3d(0, 0, 1), 0.5);
        return true;
    case BlochSphere::PERMUTE_H:
        executeRotation(Vector3d(0.707106, 0, 0.707106), 0.5);
        return true;
    case BlochSphere::PERMUTE_S:
        executeRotation(Vector3d(0, 0, 1), 0.25);
        return true;
    case BlochSphere::PERMUTE_NS:
        executeRotation(Vector3d(0, 0, 1), -0.25);
        return true;
    case BlochSphere::PERMUTE_T:
        executeRotation(Vector3d(0, 0, 1), 0.125);
        return true;
    case BlochSphere::PERMUTE_NT:
        executeRotation(Vector3d(0, 0, 1), -0.125);
        return true;
    default:
        return false;
    }
}

void receiver_reset_bloch()
{
    double resetAxis[3];
    double resetTurns = 0.0;
    netRotation.calculateReset(resetAxis, resetTurns);

    if (resetTurns > 0.001)
        bloch.rotate(Vector3d(resetAxis[0], resetAxis[1], resetAxis[2]),
                     static_cast<int>(STEPS_PER_ROTATION * resetTurns));
}

// Forget all tracked rotations: the sphere is now at |0⟩ by definition.
void receiver_home()
{
    double dummy_axis[3];
    double dummy_turns = 0.0;
    netRotation.calculateReset(dummy_axis, dummy_turns);
    bloch.reset();
}

// Takes effect now when idle; a connected client keeps its connection and the
// new name is advertised after it disconnects.
void receiver_apply_machine_name()
{
    BLEAdvertising *advertising = pServer->getAdvertising();
    advertising->stop();
    advertising->setName(machineName());
    if (!deviceConnected)
    {
        advertising->start();
    }
}

// ---------------------------------------------------------------------------
// BLE receive loop — parses comma-separated command strings.
// ---------------------------------------------------------------------------

static void setScreenColor(int r, int g, int b)
{
    gui_set_screen_color(r, g, b);
}

static bool pulse = false;
static unsigned long pulseEnd = 0;

void receiver_loop()
{
    // --- Connection state changes ---
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500); // give BLE stack time to settle
        pServer->startAdvertising();
        oldDeviceConnected = deviceConnected;
        gui_set_ble_connected(false);
    }
    if (deviceConnected && !oldDeviceConnected)
    {
        oldDeviceConnected = deviceConnected;
        gui_set_ble_connected(true);
    }

    // Moves from a client would invalidate the calibration measurement
    if (calibration_active() || ble_update_active())
    {
        bleBuffer = "";
        return;
    }

    // --- Command parsing ---
    int newlineIndex = bleBuffer.indexOf('\n');
    if (newlineIndex == -1)
    {
        // No complete command yet; handle pulse expiry and return.
        if (pulse && millis() > pulseEnd)
        {
            pulse = false;
            gui_reset_screen_color();
            gui_set_ble_task("");
        }
        return;
    }

    String line = bleBuffer.substring(0, newlineIndex);
    bleBuffer = bleBuffer.substring(newlineIndex + 1);
    line.trim();

    // Split on commas into a small fixed-size token array.
    String tokens[20];
    int tokenCount = 0;
    while (line.length() > 0 && tokenCount < 20)
    {
        int idx = line.indexOf(',');
        if (idx == -1)
        {
            tokens[tokenCount++] = line;
            break;
        }
        tokens[tokenCount++] = line.substring(0, idx);
        line = line.substring(idx + 1);
    }

    for (int i = 0; i < tokenCount; i++)
    {
        const String &cmd = tokens[i];

        // --- Directional moves (legacy BLE command names, Dutch) ---
        if (cmd == "voorwaarts")
            executeRotation(Vector3d(1, 0, 0), 0.5);
        else if (cmd == "achterwaarts")
            executeRotation(Vector3d(1, 0, 0), -0.5);
        else if (cmd == "links")
            executeRotation(Vector3d(0, 1, 0), 0.5);
        else if (cmd == "rechts")
            executeRotation(Vector3d(0, 1, 0), -0.5);
        else if (cmd == "linksboven")
            executeRotation(Vector3d(0.707106, 0, 0.707106), 0.5);
        else if (cmd == "rechtsboven")
            executeRotation(Vector3d(0.707106, 0, 0.707106), -0.5);
        else if (cmd == "linksonder")
            executeRotation(Vector3d(0.707106, 0, -0.707106), 0.5);

        // --- Quantum gate commands ---
        else if (cmd == "Xgate")
            receiver_apply_permute(BlochSphere::PERMUTE_X);
        else if (cmd == "Ygate")
            receiver_apply_permute(BlochSphere::PERMUTE_Y);
        else if (cmd == "Zgate")
            receiver_apply_permute(BlochSphere::PERMUTE_Z);
        else if (cmd == "Hgate")
            receiver_apply_permute(BlochSphere::PERMUTE_H);
        else if (cmd == "Sgate")
            receiver_apply_permute(BlochSphere::PERMUTE_S);
        else if (cmd == "S+gate")
            receiver_apply_permute(BlochSphere::PERMUTE_NS);
        else if (cmd == "Tgate")
            receiver_apply_permute(BlochSphere::PERMUTE_T);
        else if (cmd == "T+gate")
            receiver_apply_permute(BlochSphere::PERMUTE_NT);

        // --- Parametric rotation gates ---
        else if (cmd == "Rx" && i + 1 < tokenCount)
        {
            executeRotation(Vector3d(1, 0, 0), tokens[i + 1].toDouble());
            i++;
        }
        else if (cmd == "Ry" && i + 1 < tokenCount)
        {
            executeRotation(Vector3d(0, 1, 0), tokens[i + 1].toDouble());
            i++;
        }
        else if (cmd == "Rz" && i + 1 < tokenCount)
        {
            executeRotation(Vector3d(0, 0, 1), tokens[i + 1].toDouble());
            i++;
        }
        else if (cmd == "SXgate")
            executeRotation(Vector3d(1, 0, 0), 0.25);

        // --- Control commands ---
        else if (cmd == "reset")
        {
            receiver_reset_bloch();
        }
        // Snap immediately to |0⟩ (hard reset, no animation).
        else if (cmd == "home")
        {
            receiver_home();
        }
        // Animated moves to named Bloch-sphere states.
        else if (cmd == "state0")
        {
            receiver_reset_bloch();
        }
        else if (cmd == "state1")
        {
            receiver_reset_bloch();
            executeRotation(Vector3d(1, 0, 0), 0.5);
        }
        else if (cmd == "statePlus")
        {
            receiver_reset_bloch();
            executeRotation(Vector3d(0, 1, 0), 0.25);
        }
        else if (cmd == "stateMinus")
        {
            receiver_reset_bloch();
            executeRotation(Vector3d(0, 1, 0), -0.25);
        }
        else if (cmd == "stateR")
        {
            receiver_reset_bloch();
            executeRotation(Vector3d(1, 0, 0), -0.25);
        }
        else if (cmd == "stateL")
        {
            receiver_reset_bloch();
            executeRotation(Vector3d(1, 0, 0), 0.25);
        }
        else if (cmd == "led" && i + 3 < tokenCount)
        {
            setScreenColor(tokens[i + 1].toInt(), tokens[i + 2].toInt(), tokens[i + 3].toInt());
            i += 3;
        }
        else if (cmd == "pulse" && i + 4 < tokenCount)
        {
            setScreenColor(tokens[i + 1].toInt(), tokens[i + 2].toInt(), tokens[i + 3].toInt());
            pulse = true;
            pulseEnd = millis() + tokens[i + 4].toInt();
            i += 4;
        }
        else if (cmd == "rotate" && i + 4 < tokenCount)
        {
            double ax = tokens[i + 1].toDouble();
            double ay = tokens[i + 2].toDouble();
            double az = tokens[i + 3].toDouble();
            double turns = tokens[i + 4].toDouble();
            executeRotation(Vector3d(ax, ay, az), turns);
            i += 4;
        }
        // Send the current Bloch vector back to the connected client.
        else if (cmd == "query")
        {
            if (deviceConnected)
            {
                Vector3d s = bloch.getState();
                char buf[64];
                snprintf(buf, sizeof(buf), "state,%0.4f,%0.4f,%0.4f\n",
                         s[0], s[1], s[2]);
                pTxCharacteristic->setValue(buf);
                pTxCharacteristic->notify();
            }
        }
        else if (cmd == "task" && i + 1 < tokenCount)
        {
            gui_set_ble_task(tokens[i + 1].c_str());
            i++;
        }
        else if (cmd == "brightness" && i + 1 < tokenCount)
        {
            gui_set_brightness(static_cast<uint8_t>(tokens[i + 1].toInt()));
            i++;
        }
        else if (cmd == "delay" && i + 1 < tokenCount)
        {
            delay(tokens[i + 1].toInt());
            i++;
        }

        // Echo the command back to the sender.
        if (deviceConnected)
        {
            String ack = cmd + "\n";
            pTxCharacteristic->setValue(ack.c_str());
            pTxCharacteristic->notify();
        }
    }

    // --- Pulse expiry ---
    if (pulse && millis() > pulseEnd)
    {
        pulse = false;
        gui_reset_screen_color();
        gui_set_ble_task("");
    }
}
