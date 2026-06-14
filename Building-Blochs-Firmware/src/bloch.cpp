#include "bloch.h"

#include <stdlib.h>

#include "config.h"
#include "log.h"
#include "m5stack-data.h"
#include "bloch_math.h"

using namespace Eigen;

static void printMatrix3d(const Eigen::Matrix3d &m, const char *name = "M");

BlochSphere::BlochSphere()
    : _motor1(AccelStepper::DRIVER, M5_STEPMOTORDRIVER_STP_X, M5_STEPMOTORDRIVER_DIR_X),
      _motor2(AccelStepper::DRIVER, M5_STEPMOTORDRIVER_STP_Y, M5_STEPMOTORDRIVER_DIR_Y),
      _motor3(AccelStepper::DRIVER, M5_STEPMOTORDRIVER_STP_Z, M5_STEPMOTORDRIVER_DIR_Z),
      _bloch_state(0, 0, 1),
      _queue_mutex(nullptr)
{
    _queue_mutex = xSemaphoreCreateMutex();
}

BlochSphere::~BlochSphere() {}

bool BlochSphere::begin()
{
    // Wire1 is used here because LovyanGFX configures the touch controller on Wire1
    // with the same pins. See https://github.com/lovyan03/LovyanGFX/issues/628
    Wire1.begin(M5_STEPMOTORDRIVER_SDA, M5_STEPMOTORDRIVER_SCL, 400000);
    if (!_driver.init(Wire1))
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

    return true;
}

void BlochSphere::reset()
{
    _bloch_state = Vector3d(0, 0, 1);

    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    MovementQueue_t empty;
    _queue.swap(empty);
    xSemaphoreGive(_queue_mutex);

    _state = RESET;
}

bool BlochSphere::rotate(const Vector3d axis, const int steps)
{
    DBG("Queue move");

    // Update the software Bloch vector via Rodrigues' rotation formula.
    const Vector3d k = axis.normalized();
    const double theta = (2.0 * PI * static_cast<double>(steps)) / static_cast<double>(STEPS_PER_ROTATION);
    const double c = cos(theta);
    const double s = sin(theta);
    const Vector3d v = _bloch_state;
    _bloch_state = (v * c) + (k.cross(v) * s) + (k * k.dot(v) * (1.0 - c));
    _bloch_state.normalize();

    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
    _queue.push(_motormatrix * axis.normalized() * steps);
    xSemaphoreGive(_queue_mutex);
    return true;
}

Vector3d BlochSphere::getState() const
{
    return _bloch_state;
}

bool BlochSphere::isBusy() const
{
    return !_queue.empty() || _state == MOVE_START || _state == MOVE_BUSY || _state == MOVE_END;
}

void BlochSphere::loop()
{
    Vector3d move;
    float max_move;

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
        _driver.enableMotor(0);
        _driver.resetMotor(0, 0);
        _driver.resetMotor(1, 0);
        _driver.resetMotor(2, 0);
        _state = IDLE;
        break;

    case IDLE:
        xSemaphoreTake(_queue_mutex, portMAX_DELAY);
        if (!_queue.empty()) _state = MOVE_START;
        xSemaphoreGive(_queue_mutex);
        return;

    case MOVE_START:
        xSemaphoreTake(_queue_mutex, portMAX_DELAY);
        if (_queue.empty()) { xSemaphoreGive(_queue_mutex); _state = IDLE; break; }
        move = _queue.front();
        _queue.pop();
        xSemaphoreGive(_queue_mutex);

        _driver.enableMotor(1);

        // Scale each motor's speed and acceleration proportionally to its
        // share of the total move, so all three finish simultaneously.
        max_move = max(max(abs(move[0]), abs(move[1])), abs(move[2]));

        _motor1.setAcceleration(abs(move[2]) / max_move * MOTORS_MAX_ACCEL);
        _motor2.setAcceleration(abs(move[1]) / max_move * MOTORS_MAX_ACCEL);
        _motor3.setAcceleration(abs(move[0]) / max_move * MOTORS_MAX_ACCEL);

        _motor1.setMaxSpeed(abs(move[2]) / max_move * MOTORS_MAX_SPEED);
        _motor2.setMaxSpeed(abs(move[1]) / max_move * MOTORS_MAX_SPEED);
        _motor3.setMaxSpeed(abs(move[0]) / max_move * MOTORS_MAX_SPEED);

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
        _driver.enableMotor(0);
        _state = _queue.empty() ? IDLE : MOVE_START;
        break;
    }
}

static void printMatrix3d(const Eigen::Matrix3d &m, const char *name)
{
    Serial.printf(" %s = ", name);
    for (int r = 0; r < 3; r++)
        Serial.printf("\t%0.8f %0.8f %0.8f\n", m(r, 0), m(r, 1), m(r, 2));
}
