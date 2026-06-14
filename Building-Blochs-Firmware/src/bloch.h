#ifndef __BLOCH_H
#define __BLOCH_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <AccelStepper.h>
#include <Module_Stepmotor.h>
#include <queue>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

using namespace Eigen;

class BlochSphere
{
public:
    // Quantum gate identifiers — used by the GUI and receiver to request rotations.
    typedef enum
    {
        PERMUTE_X = 1,
        PERMUTE_Y,
        PERMUTE_Z,
        PERMUTE_H,
        PERMUTE_S,
        PERMUTE_NS, // S†
        PERMUTE_T,
        PERMUTE_NT, // T†
    } permute_t;

    BlochSphere();
    ~BlochSphere();

    bool begin();
    void loop();
    void reset();

    bool isBusy() const;

    // Enqueue a rotation: axis (need not be unit), steps (signed, full rotation = STEPS_PER_ROTATION).
    bool rotate(const Vector3d axis, const int steps);

    // Returns the software-tracked Bloch vector (unit sphere, starts at |0⟩ = (0,0,1)).
    Vector3d getState() const;

private:
    typedef enum
    {
        _NONE,      // sentinel for first-run state-change detection
        RESET,      // initial state, also re-entered on reset()
        IDLE,
        MOVE_START,
        MOVE_BUSY,
        MOVE_END,
    } state_t;

    const char *StateNames[MOVE_END + 1] = {"<none>", "RESET", "IDLE", "START", "BUSY", "END"};

    state_t _state = RESET;

    typedef std::queue<Vector3d> MovementQueue_t;
    MovementQueue_t _queue;
    SemaphoreHandle_t _queue_mutex;

    Module_Stepmotor _driver;
    AccelStepper _motor1;
    AccelStepper _motor2;
    AccelStepper _motor3;

    // Maps a rotation axis in Bloch-sphere coordinates to motor step counts.
    Matrix3d _motormatrix;

    // Software copy of the Bloch vector, updated on every rotate() call.
    Vector3d _bloch_state;
};

#endif // __BLOCH_H