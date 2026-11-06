#include "bloch.h"

#include <stdlib.h>
#include <esp_system.h>
#include <freertos/task.h>

#include "config.h"
#include "i2c_bus.h"
#include "log.h"
#include "m5stack-data.h"
#include "bloch_math.h"

using namespace Eigen;

// Stepper driver register holding one reset line per motor, set = running
static const uint8_t kDriverResetReg = 0x05;
static const uint8_t kDriverResetRelease = 0x07;
static const int kDriverResetAttempts = 3;
static constexpr double kWanderTurnsPerSecond = 12.0 / 360.0;
static constexpr uint32_t kWanderSteeringMs = 40;

static void printMatrix3d(const Eigen::Matrix3d &m, const char *name = "M");

BlochSphere::BlochSphere()
    : _queue_mutex(xSemaphoreCreateMutex()),
      _motor1(AccelStepper::DRIVER, M5_STEPMOTORDRIVER_STP_X, M5_STEPMOTORDRIVER_DIR_X),
      _motor2(AccelStepper::DRIVER, M5_STEPMOTORDRIVER_STP_Y, M5_STEPMOTORDRIVER_DIR_Y),
      _motor3(AccelStepper::DRIVER, M5_STEPMOTORDRIVER_STP_Z, M5_STEPMOTORDRIVER_DIR_Z),
      _bloch_state(0, 0, 1)
{
}

BlochSphere::~BlochSphere() {}

bool BlochSphere::begin()
{
    // Wire1 is used here because LovyanGFX configures the touch controller on Wire1
    // with the same pins. See https://github.com/lovyan03/LovyanGFX/issues/628
    Wire1.begin(M5_STEPMOTORDRIVER_SDA, M5_STEPMOTORDRIVER_SCL, 400000);
    const bool driverReady = _driver.init(Wire1);
    if (!driverReady)
    {
        WARNING("_driver returned false. error?");
    }

    _motor1.enableOutputs();
    _motor2.enableOutputs();
    _motor3.enableOutputs();

    uint8_t status;
    _driver.getFaultStatus(&status);
    DBG("FaultStatus: 0x%0x", status);

    // Build the motor matrix from the physical wheel geometry.
    double m[3][3];
    generate_motor_matrix(MOTOR_ANGLE_DEG, BASE_Z_ROTATION_DEG, m);
    _motormatrix << m[0][0], m[0][1], m[0][2],
                    m[1][0], m[1][1], m[1][2],
                    m[2][0], m[2][1], m[2][2];
    printMatrix3d(_motormatrix, "MotorMatrix");

    _state = RESET;

    return driverReady;
}

void BlochSphere::_enableMotors(bool enable)
{
    i2c_bus_lock();
    _driver.enableMotor(enable ? 1 : 0);
    i2c_bus_unlock();
}

// One write instead of Module_Stepmotor::resetMotor() per motor: that does a
// read-modify-write, so a failed read clears the bits of the other motors and
// leaves them dead until the next reset, with no error.
void BlochSphere::_releaseMotorResets()
{
    uint8_t value = 0;
    for (int attempt = 0; attempt < kDriverResetAttempts; ++attempt)
    {
        i2c_bus_lock();
        Wire1.beginTransmission(_driver._addr);
        Wire1.write(kDriverResetReg);
        Wire1.write(kDriverResetRelease);
        const uint8_t write_error = Wire1.endTransmission();

        Wire1.beginTransmission(_driver._addr);
        Wire1.write(kDriverResetReg);
        const uint8_t select_error = Wire1.endTransmission();
        const bool read_ok = select_error == 0 && Wire1.requestFrom((int)_driver._addr, 1) == 1;
        value = read_ok ? Wire1.read() : 0;
        i2c_bus_unlock();

        if (write_error == 0 && read_ok && (value & kDriverResetRelease) == kDriverResetRelease)
        {
            return;
        }
    }
    WARNING("Motor reset lines not released, register reads 0x%02x", value);
}

void BlochSphere::reset()
{
    stopWander();
    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    _bloch_state = Vector3d(0, 0, 1);
    _net_rotation = {};
    MovementQueue_t empty;
    _queue.swap(empty);
    xSemaphoreGive(_queue_mutex);

    _state = RESET;
}

bool BlochSphere::rotate(const Vector3d axis, const int steps)
{
    stopWander();
    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    _rotate(axis, steps);
    xSemaphoreGive(_queue_mutex);
    return true;
}

void BlochSphere::resetToNorth()
{
    stopWander();
    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    Vector3d axis;
    double turns;
    _net_rotation.calculateReset(axis.data(), turns);
    if (turns > 0.001)
    {
        _rotate(axis, static_cast<int>(STEPS_PER_ROTATION * turns));
        _net_rotation = {};
    }
    xSemaphoreGive(_queue_mutex);
}

// Called with the queue mutex held.
void BlochSphere::_rotate(const Vector3d &axis, int steps)
{
    const Vector3d k = axis.normalized();
    _trackRotation(k, static_cast<double>(steps) / STEPS_PER_ROTATION);
    _queueMotion(k * steps);
}

void BlochSphere::_trackRotation(const Vector3d &axis, double turns)
{
    Vector3d state;
    rodrigues_rotate(_bloch_state.data(), axis.data(), 2.0 * PI * turns, state.data());
    _bloch_state = state.normalized();
    _net_rotation.applyRotation(axis.data(), turns);
}

void BlochSphere::moveUntracked(const Vector3d axis, double degrees)
{
    stopWander();
    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    _queueMotion(axis.normalized() * (degrees / 360.0 * STEPS_PER_ROTATION));
    xSemaphoreGive(_queue_mutex);
}

void BlochSphere::setRotationCorrection(double x, double y, double z)
{
    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    _correction = Vector3d(x, y, z);
    xSemaphoreGive(_queue_mutex);
}

void BlochSphere::_queueMotion(const Vector3d motion)
{
    // Compensate for the measured under-rotation about each axis.
    _queue.push(_motormatrix * motion.cwiseProduct(_correction));
}

Vector3d BlochSphere::getState() const
{
    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    const Vector3d state = _bloch_state;
    xSemaphoreGive(_queue_mutex);
    return state;
}

void BlochSphere::setSpeedPercent(int percent)
{
    _speed_scale = percent / 100.0f;
}

bool BlochSphere::isBusy() const
{
    if (_state >= MOVE_START || _wander_requested) return true;
    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    const bool busy = !_queue.empty() || _state >= MOVE_START || _wander_requested;
    xSemaphoreGive(_queue_mutex);
    return busy;
}

void BlochSphere::startWander()
{
    _wander_requested = true;
}

bool BlochSphere::stopWander()
{
    // Serialize cancellation with the motor task's transition into WANDER.
    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    const bool was_requested = _wander_requested.exchange(false);
    xSemaphoreGive(_queue_mutex);

    // Record the last pulses before queuing a gate or reset.
    while (_state == WANDER) vTaskDelay(1);
    return was_requested;
}

// Called with the queue mutex held.
void BlochSphere::_beginWander()
{
    _wander_to_motors = _motormatrix * _correction.asDiagonal();
    _wander_from_motors = _wander_to_motors.inverse();
    const double heading = 2.0 * PI * esp_random() / 4294967296.0;
    _wander_axis = AngleAxisd(heading, _bloch_state) * _bloch_state.unitOrthogonal();
    _wander_turn = 0;
    _wander_last_steps = Vector3d(_motor3.currentPosition(), _motor2.currentPosition(), _motor1.currentPosition());
    _wander_last_update = millis();
    _motor1.setMaxSpeed(MOTORS_MAX_SPEED);
    _motor2.setMaxSpeed(MOTORS_MAX_SPEED);
    _motor3.setMaxSpeed(MOTORS_MAX_SPEED);
    _steerWander();
}

void BlochSphere::_trackWander()
{
    // Use emitted motor steps so Reset accounts for timing jitter and rounding.
    const Vector3d steps(_motor3.currentPosition(), _motor2.currentPosition(), _motor1.currentPosition());
    const Vector3d motion = _wander_from_motors * (steps - _wander_last_steps);
    _wander_last_steps = steps;
    const double length = motion.norm();
    if (length > 0) _trackRotation(motion / length, length / STEPS_PER_ROTATION);
}

void BlochSphere::_steerWander()
{
    // A tangent rotation axis keeps the arrow's surface speed fixed. Filter
    // the random changes so consecutive directions stay close together.
    const double noise = 2.0 * esp_random() / 4294967296.0 - 1.0;
    _wander_turn = 0.88 * _wander_turn + 0.12 * (0.035 * noise);
    _wander_axis = (_wander_axis - _bloch_state * _bloch_state.dot(_wander_axis)).normalized();
    _wander_axis = AngleAxisd(_wander_turn, _bloch_state) * _wander_axis;
    const Vector3d rates = _wander_to_motors * _wander_axis * (STEPS_PER_ROTATION * kWanderTurnsPerSecond);
    _motor1.setSpeed(rates[2]);
    _motor2.setSpeed(rates[1]);
    _motor3.setSpeed(rates[0]);
}

void BlochSphere::_runWander()
{
    if (!_wander_requested)
    {
        xSemaphoreTake(_queue_mutex, portMAX_DELAY);
        _trackWander();
        xSemaphoreGive(_queue_mutex);
        // Clear AccelStepper's old targets and its continuous speed.
        _motor1.setCurrentPosition(0);
        _motor2.setCurrentPosition(0);
        _motor3.setCurrentPosition(0);
        _enableMotors(false);
        _state = IDLE;
        return;
    }

    _motor1.runSpeed();
    _motor2.runSpeed();
    _motor3.runSpeed();
    const uint32_t now = millis();
    if (now - _wander_last_update >= kWanderSteeringMs)
    {
        xSemaphoreTake(_queue_mutex, portMAX_DELAY);
        _trackWander();
        _steerWander();
        xSemaphoreGive(_queue_mutex);
        _wander_last_update = now;
        vTaskDelay(1);  // Let the idle task run during continuous motion.
    }
}

void BlochSphere::loop()
{
    Vector3d move;
    float max_move, max_speed, max_accel;

    static state_t _prvstate = _NONE;
    if (_state != _prvstate)
    {
        DBG("%lu: State %s -> %s", millis(), StateNames[_prvstate], StateNames[_state]);
        _prvstate = _state;
    }

    switch (_state)
    {
    case _NONE:
    case RESET:
        _enableMotors(false);
        _releaseMotorResets();
        _state = IDLE;
        break;

    case IDLE:
        xSemaphoreTake(_queue_mutex, portMAX_DELAY);
        if (!_queue.empty())
        {
            _state = MOVE_START;
        }
        else if (_wander_requested)
        {
            _beginWander();
            _state = WANDER;
        }
        xSemaphoreGive(_queue_mutex);
        if (_state == WANDER) _enableMotors(true);
        return;

    case MOVE_START:
        xSemaphoreTake(_queue_mutex, portMAX_DELAY);
        if (_queue.empty()) { xSemaphoreGive(_queue_mutex); _state = IDLE; break; }
        move = _queue.front();
        _queue.pop();
        xSemaphoreGive(_queue_mutex);

        _enableMotors(true);

        // Scale each motor's speed and acceleration proportionally to its
        // share of the total move, so all three finish simultaneously.
        max_move = max(max(abs(move[0]), abs(move[1])), abs(move[2]));
        max_speed = MOTORS_MAX_SPEED * _speed_scale;
        max_accel = MOTORS_MAX_ACCEL * _speed_scale;

        _motor1.setAcceleration(abs(move[2]) / max_move * max_accel);
        _motor2.setAcceleration(abs(move[1]) / max_move * max_accel);
        _motor3.setAcceleration(abs(move[0]) / max_move * max_accel);

        _motor1.setMaxSpeed(abs(move[2]) / max_move * max_speed);
        _motor2.setMaxSpeed(abs(move[1]) / max_move * max_speed);
        _motor3.setMaxSpeed(abs(move[0]) / max_move * max_speed);

        DBG("%lu: Move (%.0f, %.0f, %.0f) (max=%.0f)", millis(), move[0], move[1], move[2], max_move);
        _motor1.move(move[2]);
        _motor2.move(move[1]);
        _motor3.move(move[0]);

        _state = MOVE_BUSY;
        break;

    case MOVE_BUSY:
        _motor1.run();
        _motor2.run();
        _motor3.run();

        if (_motor1.distanceToGo()) break;
        if (_motor2.distanceToGo()) break;
        if (_motor3.distanceToGo()) break;

        _state = MOVE_END;
        break;

    case MOVE_END:
        _enableMotors(false);
        _state = IDLE;
        break;

    case WANDER:
        _runWander();
        break;
    }
}

static void printMatrix3d(const Eigen::Matrix3d &m, const char *name)
{
    Serial.printf(" %s = ", name);
    for (int r = 0; r < 3; r++)
        Serial.printf("\t%0.8f %0.8f %0.8f\n", m(r, 0), m(r, 1), m(r, 2));
}
