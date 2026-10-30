#include "calibration.h"

#include <lvgl.h>

#include "bloch.h"
#include "bloch_math.h"
#include "config.h"
#include "gui.h"
#include "receiver.h"
#include "settings.h"

extern BlochSphere bloch; // main.cpp

typedef enum
{
    STEP_IDLE,
    STEP_INTRO,
    STEP_READY,    // note the position of the mark
    STEP_MOVING,   // waiting for the motors, then continue at next_step
    STEP_ADJUST,   // nudge the mark back
    STEP_RESULT,
    STEP_REJECTED, // implausible nudge, this axis must be repeated
    STEP_SUMMARY,
} step_t;

// Half turns, not one long move: slip happens mostly while accelerating, so the
// test must have the same motion profile as the gates it calibrates.
static const int kGateCount = 8;
static const double kGateDeg = 180.0;
static const double kCommandedDeg = kGateCount * kGateDeg;

static const int kAxisCount = 3;
static const char *const kAxisNames[kAxisCount] = {"X", "Y", "Z"};
static const Vector3d kAxes[kAxisCount] = {Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 0, 1)};
static const int kAxisZ = 2;

// The mark sits at the top, on the Z axis itself, so turning about Z would not
// move it. This tilt brings it to the equator for the Z test.
static const Vector3d kTiltAxis(0, 1, 0);
static const double kTiltDeg = 90.0;

static const char *kJogMap[] = {"-5", "-1", "-0.2", "+0.2", "+1", "+5", ""};
static const double kJogDeg[] = {-5.0, -1.0, -0.2, 0.2, 1.0, 5.0};

static step_t step = STEP_IDLE;
static step_t next_step = STEP_IDLE;
static int axis = 0;
static double jog_deg = 0.0;
static double corrections[kAxisCount];
static double previous_correction = 1.0;

static lv_obj_t *panel;
static lv_obj_t *title_label;
static lv_obj_t *body_label;
static lv_obj_t *jog_matrix;
static lv_obj_t *primary_button;
static lv_obj_t *primary_label;
static lv_obj_t *secondary_button;
static lv_obj_t *secondary_label;

static void show_step(step_t new_step);
static void apply_corrections()
{
    bloch.setRotationCorrection(corrections[0], corrections[1], corrections[2]);
}

static void restore_settings_corrections()
{
    for (int i = 0; i < kAxisCount; ++i)
    {
        corrections[i] = settings.correction[i];
    }
    apply_corrections();
}

static void close_calibration()
{
    step = STEP_IDLE;
    lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN);
}

static void move_then(step_t after)
{
    next_step = after;
    show_step(STEP_MOVING);
}

static void start_axis()
{
    jog_deg = 0.0;
    if (axis == kAxisZ)
    {
        bloch.moveUntracked(kTiltAxis, kTiltDeg);
        move_then(STEP_READY);
    }
    else
    {
        show_step(STEP_READY);
    }
}

static void finish_adjust()
{
    const double updated = calibrated_correction(corrections[axis], kCommandedDeg, jog_deg);
    if (updated < MIN_ROTATION_CORRECTION || updated > MAX_ROTATION_CORRECTION)
    {
        // Most likely a misread mark, or the sphere slipped badly
        show_step(STEP_REJECTED);
        return;
    }

    previous_correction = corrections[axis];
    corrections[axis] = updated;
    apply_corrections();
    show_step(STEP_RESULT);
}

static void next_axis()
{
    if (++axis < kAxisCount)
    {
        start_axis();
    }
    else
    {
        bloch.moveUntracked(kTiltAxis, -kTiltDeg);
        move_then(STEP_SUMMARY);
    }
}

static void primary_event_handler(lv_event_t *e)
{
    if (bloch.isBusy())
    {
        return; // Also covers a jog still running when Done is tapped
    }

    switch (step)
    {
    case STEP_INTRO:
        receiver_home();
        axis = 0;
        start_axis();
        break;
    case STEP_READY:
        for (int i = 0; i < kGateCount; ++i)
        {
            bloch.moveUntracked(kAxes[axis], kGateDeg);
        }
        jog_deg = 0.0;
        move_then(STEP_ADJUST);
        break;
    case STEP_ADJUST:
        finish_adjust();
        break;
    case STEP_RESULT:
        next_axis();
        break;
    case STEP_REJECTED:
        show_step(STEP_READY);
        break;
    case STEP_SUMMARY:
        for (int i = 0; i < kAxisCount; ++i)
        {
            settings.correction[i] = corrections[i];
        }
        gui_settings_changed();
        close_calibration();
        break;
    default:
        break;
    }
}

static void secondary_event_handler(lv_event_t *e)
{
    if (bloch.isBusy())
    {
        return;
    }

    switch (step)
    {
    case STEP_RESULT:
        show_step(STEP_READY); // Verify: the same test again, with the new correction
        break;
    case STEP_READY:
    case STEP_ADJUST:
    case STEP_REJECTED:
        if (axis == kAxisZ)
        {
            bloch.moveUntracked(kTiltAxis, -kTiltDeg);
        }
        restore_settings_corrections();
        close_calibration();
        break;
    default:
        restore_settings_corrections();
        close_calibration();
        break;
    }
}

static void jog_event_handler(lv_event_t *e)
{
    const uint16_t selected = lv_btnmatrix_get_selected_btn(jog_matrix);
    if (selected == LV_BTNMATRIX_BTN_NONE)
    {
        return;
    }

    bloch.moveUntracked(kAxes[axis], kJogDeg[selected]);
    jog_deg += kJogDeg[selected];
    lv_label_set_text_fmt(body_label, "Nudge the mark exactly back to the noted point.\n+ turns further, - turns back.\n\nNudged: %+.1f deg", jog_deg);
}

static void set_button(lv_obj_t *button, lv_obj_t *label, const char *text)
{
    if (text == NULL)
    {
        lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(button, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(label, text);
}

static void show_step(step_t new_step)
{
    step = new_step;

    const char *primary = NULL;
    const char *secondary = NULL;

    switch (step)
    {
    case STEP_INTRO:
        lv_label_set_text(title_label, "Calibration");
        lv_label_set_text(body_label, "Put a small sticker on the sphere and turn it by hand so the sticker is exactly at the top.\n\nThe saved speed is used, so set your usual speed first.");
        primary = "Start";
        secondary = "Cancel";
        break;
    case STEP_READY:
        lv_label_set_text_fmt(title_label, "%s axis (%d of %d)", kAxisNames[axis], axis + 1, kAxisCount);
        lv_label_set_text_fmt(body_label, "Hold a pen tip right next to the sticker, or note a fixed point it lines up with.\n\nThe sphere then makes %d half turns.", kGateCount);
        primary = "Rotate";
        secondary = "Cancel";
        break;
    case STEP_MOVING:
        lv_label_set_text(body_label, "Rotating, do not touch the sphere.");
        break;
    case STEP_ADJUST:
        lv_label_set_text_fmt(body_label, "Nudge the mark exactly back to the noted point.\n+ turns further, - turns back.\n\nNudged: %+.1f deg", jog_deg);
        primary = "Done";
        secondary = "Cancel";
        break;
    case STEP_RESULT:
        lv_label_set_text_fmt(body_label, "Correction %.4f to %.4f\nError per half turn: %.2f deg\n\nA repeat checks the result: the nudge should now be close to 0.",
                              previous_correction, corrections[axis], jog_deg / kGateCount);
        primary = (axis == kAxisCount - 1) ? "Finish" : "Next axis";
        secondary = "Repeat";
        break;
    case STEP_REJECTED:
        lv_label_set_text_fmt(body_label, "A nudge of %+.1f deg is not plausible.\nPut the mark back at the noted point and repeat this axis.", jog_deg);
        primary = "Repeat";
        secondary = "Cancel";
        break;
    case STEP_SUMMARY:
        lv_label_set_text(title_label, "Calibration done");
        lv_label_set_text_fmt(body_label, "X %.4f\nY %.4f\nZ %.4f\n\nThe sticker should be back at the top.",
                              corrections[0], corrections[1], corrections[2]);
        primary = "Save";
        secondary = "Discard";
        break;
    default:
        break;
    }

    if (step == STEP_ADJUST)
    {
        lv_obj_clear_flag(jog_matrix, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(jog_matrix, LV_OBJ_FLAG_HIDDEN);
    }
    set_button(primary_button, primary_label, primary);
    set_button(secondary_button, secondary_label, secondary);
}

static lv_obj_t *create_button(int x, lv_color_t color, lv_event_cb_t handler, lv_obj_t **label)
{
    lv_obj_t *button = lv_btn_create(panel);
    lv_obj_set_pos(button, x, 194);
    lv_obj_set_size(button, 145, 38);
    lv_obj_set_style_bg_color(button, color, 0);
    lv_obj_add_event_cb(button, handler, LV_EVENT_CLICKED, NULL);

    *label = lv_label_create(button);
    lv_obj_set_style_text_color(*label, lv_color_hex(0xF2F7F7), 0);
    lv_obj_center(*label);

    return button;
}

void calibration_create()
{
    panel = lv_obj_create(lv_layer_top());
    lv_obj_set_size(panel, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_set_style_border_width(panel, 0, 0);
    lv_obj_set_style_radius(panel, 0, 0);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x1A2A2A), 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN);

    title_label = lv_label_create(panel);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_26, 0);
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xF2F7F7), 0);
    lv_obj_set_pos(title_label, 10, 6);

    body_label = lv_label_create(panel);
    lv_label_set_long_mode(body_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(body_label, DISPLAY_WIDTH - 20);
    lv_obj_set_style_text_color(body_label, lv_color_hex(0xD9F0F0), 0);
    lv_obj_set_pos(body_label, 10, 42);

    jog_matrix = lv_btnmatrix_create(panel);
    lv_btnmatrix_set_map(jog_matrix, kJogMap);
    lv_obj_set_pos(jog_matrix, 10, 134);
    lv_obj_set_size(jog_matrix, DISPLAY_WIDTH - 20, 50);
    lv_obj_set_style_bg_opa(jog_matrix, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(jog_matrix, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(jog_matrix, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(jog_matrix, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(jog_matrix, lv_color_hex(0x1F7A8C), LV_PART_ITEMS);
    lv_obj_set_style_text_color(jog_matrix, lv_color_hex(0xF2F7F7), LV_PART_ITEMS);
    lv_obj_add_event_cb(jog_matrix, jog_event_handler, LV_EVENT_VALUE_CHANGED, NULL);

    secondary_button = create_button(10, lv_color_hex(0x3A5F5F), secondary_event_handler, &secondary_label);
    primary_button = create_button(165, lv_color_hex(0x1F7A8C), primary_event_handler, &primary_label);
}

void calibration_start()
{
    restore_settings_corrections();
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_HIDDEN);
    show_step(STEP_INTRO);
}

void calibration_loop()
{
    if (step == STEP_MOVING && !bloch.isBusy())
    {
        show_step(next_step);
    }
}

bool calibration_active()
{
    return step != STEP_IDLE;
}
