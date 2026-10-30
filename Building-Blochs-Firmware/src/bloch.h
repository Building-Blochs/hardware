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

    // Motor speed and acceleration as a percentage of the maximum, used from the next move on.
    void setSpeedPercent(int percent);

    // Enqueue a rotation: axis (need not be unit), steps (signed, full rotation = STEPS_PER_ROTATION).
    bool rotate(const Vector3d axis, const int steps);

    // Enqueue a rotation that leaves the tracked Bloch vector alone, for calibration.
    void moveUntracked(const Vector3d axis, double degrees);

    // Scales the motor command per axis to compensate for under-rotation.
    void setRotationCorrection(double x, double y, double z);

    // Returns the software-tracked Bloch vector (unit sphere, starts at |0⟩ = (0,0,1)).
    Vector3d getState() const;

private:
    void _queueMotion(const Vector3d motion);
    void _enableMotors(bool enable);
    void _releaseMotorResets();

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

    // Written by the GUI (core 1), read by the stepper task (core 0).
    volatile float _speed_scale = 1.0f;

    Vector3d _correction = Vector3d(1, 1, 1);

    // Software copy of the Bloch vector, updated on every rotate() call.
    Vector3d _bloch_state;
};

#endif // __BLOCH_H