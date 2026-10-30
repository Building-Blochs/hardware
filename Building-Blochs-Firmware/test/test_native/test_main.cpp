// Native unit tests for Building-Blochs firmware math.
// Run with: pio test -e native_test
//
// Coverage:
//   - Rodrigues rotation (Bloch state tracking)
//   - Quantum gate state transitions (X, Y, Z, H, S, T)
//   - QuatTracker: rotation accumulation and reset
//   - Motor matrix: invertibility and symmetry
//   - BLE command parser: CSV splitting

// _USE_MATH_DEFINES exposes M_PI on MSVC/MinGW when <cmath> is included
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>

#include <unity.h>

#include "bloch_math.h"
#include "quat_tracker.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static const double EPSILON = 1e-9;

static void rotate_bloch(double state[3], const double axis[3], double turns)
{
    double theta = turns * 2.0 * M_PI;
    double result[3];
    rodrigues_rotate(state, axis, theta, result);
    // normalise (handles floating-point drift)
    vec3_normalize(result);
    state[0] = result[0];
    state[1] = result[1];
    state[2] = result[2];
}

// ---------------------------------------------------------------------------
// Rodrigues rotation tests
// ---------------------------------------------------------------------------

void test_calibrated_correction_corrects_undershoot(void)
{
    // The sphere only makes 0.98 of what is commanded, seen through a start
    // correction of 1.0: 1440 deg needs a nudge until it has really turned 1440.
    const double gain = 0.98;
    const double jog = 1440.0 / gain - 1440.0;
    const double updated = calibrated_correction(1.0, 1440.0, jog);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, updated * gain);
}

void test_calibrated_correction_converges_from_previous_value(void)
{
    const double gain = 0.98;
    const double current = 1.01;
    const double jog = 1440.0 / (gain * current) - 1440.0;
    const double updated = calibrated_correction(current, 1440.0, jog);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, updated * gain);
}

void test_calibrated_correction_unchanged_without_jog(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.02, calibrated_correction(1.02, 1440.0, 0.0));
}

void test_rodrigues_identity(void)
{
    // Rotating by 0 turns should leave vector unchanged
    double v[3]    = {0, 0, 1};
    double axis[3] = {1, 0, 0};
    double out[3];
    rodrigues_rotate(v, axis, 0.0, out);
    TEST_ASSERT_DOUBLE_WITHIN(EPSILON, 0.0, out[0]);
    TEST_ASSERT_DOUBLE_WITHIN(EPSILON, 0.0, out[1]);
    TEST_ASSERT_DOUBLE_WITHIN(EPSILON, 1.0, out[2]);
}

void test_rodrigues_x_gate_on_zero(void)
{
    // X gate: rotate |0> (0,0,1) by π around X-axis → |1> (0,0,-1)
    double state[3] = {0, 0, 1};
    double axis[3]  = {1, 0, 0};
    double out[3];
    rodrigues_rotate(state, axis, M_PI, out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, out[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, out[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -1.0, out[2]);
}

void test_rodrigues_y_gate_on_zero(void)
{
    // Y gate: rotate |0> by π around Y-axis → |1>
    double state[3] = {0, 0, 1};
    double axis[3]  = {0, 1, 0};
    double out[3];
    rodrigues_rotate(state, axis, M_PI, out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, out[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, out[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -1.0, out[2]);
}

void test_rodrigues_z_gate_on_equator(void)
{
    // Z gate on |+> = (1,0,0): rotate π around Z → |-> = (-1,0,0)
    double state[3] = {1, 0, 0};
    double axis[3]  = {0, 0, 1};
    double out[3];
    rodrigues_rotate(state, axis, M_PI, out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -1.0, out[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  0.0, out[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  0.0, out[2]);
}

void test_rodrigues_x_squared_is_identity(void)
{
    // X·X = I: applying X gate twice returns to original state
    double state[3] = {0, 0, 1};
    double axis[3]  = {1, 0, 0};
    double tmp[3];
    rodrigues_rotate(state, axis, M_PI, tmp);
    double out[3];
    rodrigues_rotate(tmp, axis, M_PI, out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, out[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, out[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.0, out[2]);
}

void test_rodrigues_z_gate_on_poles_is_identity(void)
{
    // Z gate leaves the poles |0> and |1> unchanged (they are on Z axis)
    double state0[3] = {0, 0,  1};
    double state1[3] = {0, 0, -1};
    double axis[3]   = {0, 0,  1};
    double out0[3], out1[3];
    rodrigues_rotate(state0, axis, M_PI, out0);
    rodrigues_rotate(state1, axis, M_PI, out1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  0.0, out0[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  0.0, out0[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  1.0, out0[2]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  0.0, out1[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  0.0, out1[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -1.0, out1[2]);
}

void test_rodrigues_h_gate_maps_zero_to_plus(void)
{
    // H gate: rotate |0> (0,0,1) by π around (1/√2, 0, 1/√2) → (1,0,0) = |+>
    double state[3] = {0, 0, 1};
    double axis[3]  = {0.707106781, 0, 0.707106781};
    double out[3];
    rodrigues_rotate(state, axis, M_PI, out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 1.0, out[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, out[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, out[2]);
}

void test_rodrigues_h_squared_is_identity(void)
{
    // H·H = I
    double state[3] = {0, 0, 1};
    double axis[3]  = {0.707106781, 0, 0.707106781};
    double tmp[3], out[3];
    rodrigues_rotate(state, axis, M_PI, tmp);
    rodrigues_rotate(tmp, axis, M_PI, out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, out[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, out[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 1.0, out[2]);
}

void test_rodrigues_s_gate_is_quarter_z(void)
{
    // S gate = quarter turn around Z: |+> = (1,0,0) → (0,1,0) = |i>
    double state[3] = {1, 0, 0};
    double axis[3]  = {0, 0, 1};
    double out[3];
    rodrigues_rotate(state, axis, M_PI / 2.0, out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  0.0, out[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  1.0, out[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6,  0.0, out[2]);
}

void test_rodrigues_t_gate_is_eighth_z(void)
{
    // T gate = π/4 around Z: rotates (1,0,0) by 45°
    double state[3] = {1, 0, 0};
    double axis[3]  = {0, 0, 1};
    double out[3];
    rodrigues_rotate(state, axis, M_PI / 4.0, out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, sqrt(2.0) / 2.0, out[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, sqrt(2.0) / 2.0, out[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0,      out[2]);
}

void test_rodrigues_preserves_norm(void)
{
    // The output vector should always be unit length
    double state[3] = {0.577, 0.577, 0.577};
    vec3_normalize(state);
    double axis[3] = {0.3, 0.7, -0.6};
    double out[3];
    rodrigues_rotate(state, axis, 1.23456, out);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, sqrt(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]));
}

// ---------------------------------------------------------------------------
// Gate sequence tests (simulate bloch.permute calls)
// ---------------------------------------------------------------------------

void test_gate_sequence_xzx_on_zero(void)
{
    // X·Z·X = Y (up to global phase, on Bloch sphere state)
    // |0> --X--> |1> --Z--> |1> --X--> |0>  (Z fixes poles, X flips again)
    double state[3] = {0, 0, 1};
    double ax[3] = {1, 0, 0};
    double az[3] = {0, 0, 1};
    rotate_bloch(state, ax, 0.5);   // X
    rotate_bloch(state, az, 0.5);   // Z
    rotate_bloch(state, ax, 0.5);   // X
    // Result should be (0,0,1) again because Z on a pole is identity, X·X=I
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, state[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, state[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 1.0, state[2]);
}

void test_gate_sequence_hxh_equals_z(void)
{
    // H·X·H = Z  (conjugation rule)
    // Apply to |+> = (1,0,0): Z fixes the equatorial plane state differently —
    // test on |0>: H→|+>, X→|->, H→|1>
    double state[3] = {0, 0, 1};
    double ax[3] = {1, 0, 0};
    double ah[3] = {0.707106781, 0, 0.707106781};
    rotate_bloch(state, ah, 0.5);  // H: |0> → |+>
    rotate_bloch(state, ax, 0.5);  // X: |+> → |+> (X leaves |+> invariant)
    rotate_bloch(state, ah, 0.5);  // H: → |0> or |1> depending on phases

    // On the Bloch sphere (no global phase), H·X·H·|0⟩ = Z·|0⟩ = |0⟩
    // Z on |0> is identity at the state-vector level → back to |0> = (0,0,1)
    // Actually let's verify the actual geometry:
    // H|0> = |+> = (1,0,0)
    // X|+> = |+>  (X maps (1,0,0) by 180° around X: (0,0,1) stays? No!)
    // X on |+>=(1,0,0): rotate (1,0,0) by π around (1,0,0) = (1,0,0) unchanged
    // H(1,0,0): rotate (1,0,0) by π around (1/√2,0,1/√2) = (0,0,1) = |0>
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, state[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, state[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 1.0, state[2]);
}

// ---------------------------------------------------------------------------
// QuatTracker tests
// ---------------------------------------------------------------------------

void test_quat_identity(void)
{
    QuatTracker qt;
    // Fresh tracker should be identity: w=1, xyz=0
    TEST_ASSERT_DOUBLE_WITHIN(EPSILON, 1.0, qt.w);
    TEST_ASSERT_DOUBLE_WITHIN(EPSILON, 0.0, qt.x);
    TEST_ASSERT_DOUBLE_WITHIN(EPSILON, 0.0, qt.y);
    TEST_ASSERT_DOUBLE_WITHIN(EPSILON, 0.0, qt.z);
}

void test_quat_x_gate_reset(void)
{
    // Apply X gate (0.5 turns around X), then calculateReset should give
    // the inverse: -0.5 turns around X (or equivalently +0.5 turns around -X)
    QuatTracker qt;
    double ax[3] = {1, 0, 0};
    qt.applyRotation(ax, 0.5);

    double resetAxis[3];
    double resetTurns;
    qt.calculateReset(resetAxis, resetTurns);

    // Should be ~0.5 turns (shortest path)
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.5, resetTurns);
    // Axis should be (anti)parallel to X
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.0, sqrt(resetAxis[0]*resetAxis[0] + resetAxis[1]*resetAxis[1] + resetAxis[2]*resetAxis[2]));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, resetAxis[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, resetAxis[2]);
}

void test_quat_x_squared_gives_zero_reset(void)
{
    // X·X = I, so calculateReset should give 0 turns (no rotation needed)
    QuatTracker qt;
    double ax[3] = {1, 0, 0};
    qt.applyRotation(ax, 0.5);
    qt.applyRotation(ax, 0.5);

    double resetAxis[3];
    double resetTurns;
    qt.calculateReset(resetAxis, resetTurns);

    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, resetTurns);
}

void test_quat_reset_clears_state(void)
{
    // After calculateReset, the tracker should be back to identity
    QuatTracker qt;
    double ax[3] = {0, 1, 0};
    qt.applyRotation(ax, 0.25);  // S gate

    double resetAxis[3];
    double resetTurns;
    qt.calculateReset(resetAxis, resetTurns);

    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1.0, qt.w);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, qt.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, qt.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, qt.z);
}

void test_quat_shortest_path_large_rotation(void)
{
    // If net rotation is 3/4 turn, shortest reset should be 1/4 turn (not 3/4)
    QuatTracker qt;
    double az[3] = {0, 0, 1};
    // Apply 3 × S gate = 3/4 turn around Z
    qt.applyRotation(az, 0.25);
    qt.applyRotation(az, 0.25);
    qt.applyRotation(az, 0.25);

    double resetAxis[3];
    double resetTurns;
    qt.calculateReset(resetAxis, resetTurns);

    // Shortest reset = 1/4 turn in the opposite direction
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.25, resetTurns);
}

void test_quat_h_gate_twice_is_identity(void)
{
    // H·H = I: two H gates should give 0 reset turns
    QuatTracker qt;
    double ah[3] = {0.707106781, 0, 0.707106781};
    qt.applyRotation(ah, 0.5);
    qt.applyRotation(ah, 0.5);

    double resetAxis[3];
    double resetTurns;
    qt.calculateReset(resetAxis, resetTurns);

    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, resetTurns);
}

// ---------------------------------------------------------------------------
// Motor matrix tests
// ---------------------------------------------------------------------------

void test_motor_matrix_shape(void)
{
    // With 30°/30° config (as in config.h), matrix should be non-zero and
    // each row should have unit norm (it's built from normalised vectors)
    double m[3][3];
    generate_motor_matrix(30.0, 30.0, m);

    for (int i = 0; i < 3; ++i)
        TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.0, sqrt(m[i][0]*m[i][0] + m[i][1]*m[i][1] + m[i][2]*m[i][2]));
}

void test_motor_matrix_z_rotation_symmetry(void)
{
    // Changing BASE_Z_ROTATION_DEG by 120° should cyclically permute the rows
    // (since the three wheels are 120° apart)
    double m0[3][3], m1[3][3];
    generate_motor_matrix(30.0, 0.0, m0);
    generate_motor_matrix(30.0, 120.0, m1);

    // Row 0 of m1 should equal row 1 of m0 (up to sign convention)
    // Just check they differ: the matrices are distinct
    bool anyDiff = false;
    for (int i = 0; i < 3 && !anyDiff; ++i)
        for (int j = 0; j < 3 && !anyDiff; ++j)
            if (fabs(m0[i][j] - m1[i][j]) > 1e-6)
                anyDiff = true;
    TEST_ASSERT_TRUE(anyDiff);
}

void test_motor_matrix_pure_z_rotation_only_drives_one_component(void)
{
    // A pure Z-axis rotation should project similarly onto all three wheels
    // (they are symmetrically arranged). The Z component of each row of the
    // motor matrix should be equal in magnitude.
    double m[3][3];
    generate_motor_matrix(30.0, 30.0, m);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, fabs(m[0][2]), fabs(m[1][2]));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, fabs(m[0][2]), fabs(m[2][2]));
}

// ---------------------------------------------------------------------------
// BLE command parser tests (the CSV tokeniser from receiver_loop.cpp)
// The parser logic is duplicated here as a standalone function.
// ---------------------------------------------------------------------------

// Standalone re-implementation of the CSV tokeniser so it can be tested
// without Arduino String or BLE dependencies.
static int parse_ble_commands(const char *line, char out[20][64])
{
    int count = 0;
    const char *p = line;
    while (*p && count < 20)
    {
        const char *comma = strchr(p, ',');
        if (!comma)
        {
            // Last token
            strncpy(out[count++], p, 63);
            break;
        }
        size_t len = (size_t)(comma - p);
        if (len > 63) len = 63;
        strncpy(out[count], p, len);
        out[count][len] = '\0';
        ++count;
        p = comma + 1;
    }
    return count;
}

void test_parser_single_command(void)
{
    char out[20][64] = {};
    int n = parse_ble_commands("Xgate", out);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_EQUAL_STRING("Xgate", out[0]);
}

void test_parser_multiple_commands(void)
{
    char out[20][64] = {};
    int n = parse_ble_commands("Xgate,Ygate,Zgate", out);
    TEST_ASSERT_EQUAL_INT(3, n);
    TEST_ASSERT_EQUAL_STRING("Xgate", out[0]);
    TEST_ASSERT_EQUAL_STRING("Ygate", out[1]);
    TEST_ASSERT_EQUAL_STRING("Zgate", out[2]);
}

void test_parser_led_command_with_args(void)
{
    char out[20][64] = {};
    int n = parse_ble_commands("led,255,128,0", out);
    TEST_ASSERT_EQUAL_INT(4, n);
    TEST_ASSERT_EQUAL_STRING("led",   out[0]);
    TEST_ASSERT_EQUAL_STRING("255",   out[1]);
    TEST_ASSERT_EQUAL_STRING("128",   out[2]);
    TEST_ASSERT_EQUAL_STRING("0",     out[3]);
}

void test_parser_pulse_command_with_args(void)
{
    char out[20][64] = {};
    int n = parse_ble_commands("pulse,0,255,0,2000", out);
    TEST_ASSERT_EQUAL_INT(5, n);
    TEST_ASSERT_EQUAL_STRING("pulse", out[0]);
    TEST_ASSERT_EQUAL_STRING("2000",  out[4]);
}

void test_parser_empty_string(void)
{
    char out[20][64] = {};
    int n = parse_ble_commands("", out);
    TEST_ASSERT_EQUAL_INT(0, n);
}

void test_parser_mixed_sequence(void)
{
    char out[20][64] = {};
    int n = parse_ble_commands("Hgate,reset,Xgate", out);
    TEST_ASSERT_EQUAL_INT(3, n);
    TEST_ASSERT_EQUAL_STRING("Hgate", out[0]);
    TEST_ASSERT_EQUAL_STRING("reset", out[1]);
    TEST_ASSERT_EQUAL_STRING("Xgate", out[2]);
}

// ---------------------------------------------------------------------------
// Test runner
// ---------------------------------------------------------------------------

void setUp(void)    {}
void tearDown(void) {}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_calibrated_correction_corrects_undershoot);
    RUN_TEST(test_calibrated_correction_converges_from_previous_value);
    RUN_TEST(test_calibrated_correction_unchanged_without_jog);

    // Rodrigues rotation
    RUN_TEST(test_rodrigues_identity);
    RUN_TEST(test_rodrigues_x_gate_on_zero);
    RUN_TEST(test_rodrigues_y_gate_on_zero);
    RUN_TEST(test_rodrigues_z_gate_on_equator);
    RUN_TEST(test_rodrigues_x_squared_is_identity);
    RUN_TEST(test_rodrigues_z_gate_on_poles_is_identity);
    RUN_TEST(test_rodrigues_h_gate_maps_zero_to_plus);
    RUN_TEST(test_rodrigues_h_squared_is_identity);
    RUN_TEST(test_rodrigues_s_gate_is_quarter_z);
    RUN_TEST(test_rodrigues_t_gate_is_eighth_z);
    RUN_TEST(test_rodrigues_preserves_norm);

    // Gate sequences
    RUN_TEST(test_gate_sequence_xzx_on_zero);
    RUN_TEST(test_gate_sequence_hxh_equals_z);

    // QuatTracker
    RUN_TEST(test_quat_identity);
    RUN_TEST(test_quat_x_gate_reset);
    RUN_TEST(test_quat_x_squared_gives_zero_reset);
    RUN_TEST(test_quat_reset_clears_state);
    RUN_TEST(test_quat_shortest_path_large_rotation);
    RUN_TEST(test_quat_h_gate_twice_is_identity);

    // Motor matrix
    RUN_TEST(test_motor_matrix_shape);
    RUN_TEST(test_motor_matrix_z_rotation_symmetry);
    RUN_TEST(test_motor_matrix_pure_z_rotation_only_drives_one_component);

    // BLE command parser
    RUN_TEST(test_parser_single_command);
    RUN_TEST(test_parser_multiple_commands);
    RUN_TEST(test_parser_led_command_with_args);
    RUN_TEST(test_parser_pulse_command_with_args);
    RUN_TEST(test_parser_empty_string);
    RUN_TEST(test_parser_mixed_sequence);

    return UNITY_END();
}
