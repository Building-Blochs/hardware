#include "gui.h"
#include "bloch.h"
#include "calibration.h"
#include "config.h" // For DISPLAY_WIDTH, DISPLAY_HEIGHT
#include "i2c_bus.h"
#include "lgfx.h"
#include "receiver.h"
#include "settings.h"
#include "version.h"

extern BlochSphere bloch; // main.cpp

// --- LVGL object tree (all module-private) ---
static lv_obj_t *screen;
static lv_obj_t *tileview;
static lv_obj_t *tile1;
static lv_obj_t *tile2;
static lv_obj_t *tile3;
// Non-interactive copies at both ends of the row, so swiping past the last or
// first page slides onto a lookalike before gui_wrap_tiles() jumps to the real one
static lv_obj_t *tile_copy_of_3;
static lv_obj_t *tile_copy_of_1;

static lv_obj_t *gate_buttons[8]; // Expanded to 8 buttons
static lv_obj_t *reset_button;
static lv_obj_t *version_label;
static lv_obj_t *prev_page_bar;
static lv_obj_t *next_page_bar;
static lv_obj_t *hint_label;
static lv_obj_t *state_label;

static lv_obj_t *settings_panel;
static lv_obj_t *speed_slider;
static lv_obj_t *speed_value_label;
static lv_obj_t *brightness_slider;
static lv_obj_t *brightness_value_label;
static lv_obj_t *page_mode_matrix;
static lv_obj_t *dim_matrix;
static lv_obj_t *machine_value_label;
static lv_obj_t *defaults_label;

static lv_style_t style_gate_btn;
static lv_style_t style_gate_label;
static lv_style_t style_reset_btn;
static lv_style_t style_reset_label;
static lv_style_t style_info_label;
static lv_style_t style_state_label;

static lv_obj_t *ble_overlay;
static lv_obj_t *ble_label;

static bool ble_is_connected = false;
static unsigned long ble_disconnect_time = 0;
static char current_ble_task[64] = "";

enum BleGuiState
{
    BLE_GUI_HIDDEN,
    BLE_GUI_CONNECTED,
    BLE_GUI_DISCONNECTED,
    BLE_GUI_BUSY
};
static BleGuiState current_ble_gui_state = BLE_GUI_HIDDEN;

static uint8_t current_brightness = DEFAULT_BRIGHTNESS;
static bool screen_dimmed = false;
static bool settings_dirty = false;
static unsigned long settings_changed_time = 0;
static bool machine_name_dirty = false;
static bool defaults_armed = false;
static unsigned long defaults_armed_time = 0;

typedef struct
{
    const char *label;
    BlochSphere::permute_t permute;
} gate_def_t;

static const int kTotalButtons = 8;
static const gate_def_t kGates[] = {
    {"X", BlochSphere::PERMUTE_X},
    {"Y", BlochSphere::PERMUTE_Y},
    {"Z", BlochSphere::PERMUTE_Z},
    {"H", BlochSphere::PERMUTE_H},
    {"S", BlochSphere::PERMUTE_S},
    {"S*", BlochSphere::PERMUTE_NS},
    {"T", BlochSphere::PERMUTE_T},
    {"T*", BlochSphere::PERMUTE_NT},
};

static const int top_h = 22;
static const int bottom_h = 22;
static const int gap = 6;
static const int grid_w = DISPLAY_WIDTH - 8;
static const int grid_h = DISPLAY_HEIGHT - top_h - bottom_h - 10;
static const int btn_w = (grid_w - gap) / 2;
static const int btn_h = (grid_h - gap) / 2;
static const int left_x = 4;
static const int right_x = left_x + btn_w + gap;
static const int top_y = top_h + 2;
static const int bottom_y = top_y + btn_h + gap;
static const int bar_w = 44;

static const unsigned long kSettingsSaveDelayMs = 2000;
static const int kSettingsAnimMs = 200;
static const unsigned long kDefaultsConfirmMs = 3000;

static const char *kPageModeMap[] = {"Swipe", "Flip", ""};
static const char *kDimMap[] = {"Off", "1m", "5m", "10m", ""};
static const uint8_t kDimMinutes[] = {0, 1, 5, 10};

static void gate_event_handler(lv_event_t *e);
static void reset_event_handler(lv_event_t *e);
static void update_state_label();
static lv_obj_t *create_gate_button(lv_obj_t *parent, int gate);
static lv_obj_t *create_reset_page(lv_obj_t *parent);
static lv_obj_t *create_version_label(lv_obj_t *parent);
static void place_gate_button(lv_obj_t *btn, int gate, int area_x, int area_w);
static void gui_wrap_tiles();
static lv_obj_t *create_page_bar(lv_align_t align, const char *symbol, int step);
static void page_bar_event_handler(lv_event_t *e);
static void screen_gesture_handler(lv_event_t *e);
static void settings_gesture_handler(lv_event_t *e);
static void create_settings_panel();
static lv_obj_t *create_settings_slider(const char *name, int y, int min, int max, lv_obj_t **value_label);
static lv_obj_t *create_settings_matrix(const char *name, int y, const char **map);
static void show_settings(bool open);
static void apply_settings();
static void apply_page_mode();
static void update_settings_widgets();
static void update_value_labels();
static void update_screen_dim();
static void apply_backlight();

void gui_begin()
{
    // init hw drivers & lvgl
    lgfx_init();
    lvgl_init();

    // create (empty) screen
    screen = lv_obj_create(NULL);
    lv_obj_set_style_pad_all(screen, 4, 0);
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_set_style_radius(screen, 0, 0);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x1A2A2A), 0);
    lv_obj_set_style_bg_grad_color(screen, lv_color_hex(0x264D4D), 0);
    lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, 0);
    // The full-size BLE overlay overflows the screen padding; a vertical swipe
    // must stay a gesture instead of scrolling the screen.
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(screen, screen_gesture_handler, LV_EVENT_GESTURE, NULL);

    lv_style_init(&style_gate_btn);
    lv_style_set_radius(&style_gate_btn, 10);
    lv_style_set_bg_color(&style_gate_btn, lv_color_hex(0x1F7A8C));
    lv_style_set_bg_grad_color(&style_gate_btn, lv_color_hex(0x58A4B0));
    lv_style_set_bg_grad_dir(&style_gate_btn, LV_GRAD_DIR_VER);
    lv_style_set_border_width(&style_gate_btn, 2);
    lv_style_set_border_color(&style_gate_btn, lv_color_hex(0xBEE3DB));
    lv_style_set_pad_all(&style_gate_btn, 8);

    lv_style_init(&style_gate_label);
    lv_style_set_text_font(&style_gate_label, &lv_font_montserrat_26);
    lv_style_set_text_color(&style_gate_label, lv_color_hex(0xF2F7F7));

    lv_style_init(&style_reset_btn);
    lv_style_set_radius(&style_reset_btn, 10);
    lv_style_set_bg_color(&style_reset_btn, lv_color_hex(0xB23A48));
    lv_style_set_bg_grad_color(&style_reset_btn, lv_color_hex(0xC95D63));
    lv_style_set_bg_grad_dir(&style_reset_btn, LV_GRAD_DIR_VER);
    lv_style_set_border_width(&style_reset_btn, 2);
    lv_style_set_border_color(&style_reset_btn, lv_color_hex(0xFFD6DA));
    lv_style_set_pad_all(&style_reset_btn, 8);

    lv_style_init(&style_reset_label);
    lv_style_set_text_font(&style_reset_label, &lv_font_montserrat_26);
    lv_style_set_text_color(&style_reset_label, lv_color_hex(0xFFF8F8));

    lv_style_init(&style_info_label);
    lv_style_set_text_font(&style_info_label, &lv_font_montserrat_14);
    lv_style_set_text_color(&style_info_label, lv_color_hex(0xD9F0F0));

    lv_style_init(&style_state_label);
    lv_style_set_text_font(&style_state_label, &lv_font_montserrat_14);
    lv_style_set_text_color(&style_state_label, lv_color_hex(0xFFF3CD));

    hint_label = lv_label_create(screen);
    lv_obj_add_style(hint_label, &style_info_label, 0);
    lv_obj_align(hint_label, LV_ALIGN_TOP_MID, 0, 2);

    state_label = lv_label_create(screen);
    lv_obj_add_style(state_label, &style_state_label, 0);
    lv_obj_align(state_label, LV_ALIGN_BOTTOM_MID, 0, -2);

    // --- NEW: Tileview Setup ---
    tileview = lv_tileview_create(screen);
    // Make tileview background transparent so the screen gradient shows
    lv_obj_set_style_bg_opa(tileview, 0, 0);
    lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF); // Hide scrollbars

    // Add horizontal tiles, with the copies on either side of the real pages
    tile_copy_of_3 = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_HOR);
    tile1 = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_HOR);
    tile2 = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_HOR);
    tile3 = lv_tileview_add_tile(tileview, 3, 0, LV_DIR_HOR);
    tile_copy_of_1 = lv_tileview_add_tile(tileview, 4, 0, LV_DIR_HOR);

    for (int i = 0; i < kTotalButtons; ++i)
    {
        // Decide which tile the button goes on
        lv_obj_t *parent_tile = (i < 4) ? tile1 : tile2;

        gate_buttons[i] = create_gate_button(parent_tile, i);
        lv_obj_add_event_cb(gate_buttons[i], gate_event_handler, LV_EVENT_CLICKED, NULL);
    }

    reset_button = create_reset_page(tile3);
    version_label = create_version_label(tile3);
    lv_obj_add_event_cb(reset_button, reset_event_handler, LV_EVENT_CLICKED, NULL);

    for (int i = 0; i < 4; ++i)
    {
        lv_obj_clear_flag(create_gate_button(tile_copy_of_1, i), LV_OBJ_FLAG_CLICKABLE);
    }
    lv_obj_clear_flag(create_reset_page(tile_copy_of_3), LV_OBJ_FLAG_CLICKABLE);
    create_version_label(tile_copy_of_3);

    lv_obj_set_tile(tileview, tile1, LV_ANIM_OFF);

    prev_page_bar = create_page_bar(LV_ALIGN_LEFT_MID, LV_SYMBOL_LEFT, -1);
    next_page_bar = create_page_bar(LV_ALIGN_RIGHT_MID, LV_SYMBOL_RIGHT, 1);

    update_state_label();

    ble_overlay = lv_obj_create(screen);
    lv_obj_set_size(ble_overlay, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_obj_align(ble_overlay, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(ble_overlay, 255, 0);
    lv_obj_set_style_radius(ble_overlay, 0, 0);
    lv_obj_set_style_border_width(ble_overlay, 0, 0);
    lv_obj_clear_flag(ble_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ble_overlay, LV_OBJ_FLAG_HIDDEN); // Hidden by default

    ble_label = lv_label_create(ble_overlay);
    lv_obj_center(ble_label);
    lv_obj_set_style_text_font(ble_label, &lv_font_montserrat_26, 0);
    lv_obj_set_style_text_color(ble_label, lv_color_hex(0xFFFFFF), 0);

    create_settings_panel();
    calibration_create();
    apply_settings();

    // load screen
    lv_scr_load_anim(screen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false);

    return;
};

void gui_loop()
{
    time_t now = millis();

    static time_t prv_state_tick = 0;
    if (now - prv_state_tick > 120)
    {
        update_state_label();
        prv_state_tick = now;
    }

    static time_t prv_tick = 0;
    if (now - prv_tick > (1000 / DISPLAY_MAXFPS))
    {
        lv_tick_inc(now - prv_tick);
        prv_tick = now;
        lv_timer_handler();
        gui_wrap_tiles();
    }

    update_screen_dim();
    calibration_loop();

    if (defaults_armed && now - defaults_armed_time > kDefaultsConfirmMs)
    {
        defaults_armed = false;
        lv_label_set_text(defaults_label, "Restore defaults");
    }

    if (settings_dirty && !bloch.isBusy() && !calibration_active() && now - settings_changed_time > kSettingsSaveDelayMs)
    {
        settings_save();
        settings_dirty = false;
        if (machine_name_dirty)
        {
            receiver_apply_machine_name();
            machine_name_dirty = false;
        }
    }

    // --- NEW: Handle BLE Overlay State Machine ---
    if (!ble_is_connected)
    {
        if (current_ble_gui_state != BLE_GUI_HIDDEN)
        {
            // Switch to red error screen
            if (current_ble_gui_state != BLE_GUI_DISCONNECTED)
            {
                lv_obj_clear_flag(ble_overlay, LV_OBJ_FLAG_HIDDEN);
                lv_obj_set_style_bg_color(ble_overlay, lv_color_hex(0x420f14), 0); // Red
                lv_label_set_text(ble_label, "DISCONNECTED");
                current_ble_gui_state = BLE_GUI_DISCONNECTED;
            }
            // Auto-hide the red error screen after 2 seconds to reveal manual buttons
            if (millis() - ble_disconnect_time > 2000)
            {
                lv_obj_add_flag(ble_overlay, LV_OBJ_FLAG_HIDDEN);
                current_ble_gui_state = BLE_GUI_HIDDEN;
            }
        }
    }
    else
    {
        // We are connected. Determine if moving or idle.
        if (bloch.isBusy() && current_ble_task[0] != '\0')
        {
            // Only update text/color if state changes or task string changes
            if (current_ble_gui_state != BLE_GUI_BUSY || strcmp(lv_label_get_text(ble_label), current_ble_task) != 0)
            {
                lv_obj_clear_flag(ble_overlay, LV_OBJ_FLAG_HIDDEN);
                lv_obj_set_style_bg_color(ble_overlay, lv_color_hex(0x000000), 0); // Black
                lv_label_set_text(ble_label, current_ble_task);
                current_ble_gui_state = BLE_GUI_BUSY;
            }
        }
        else
        {
            if (current_ble_gui_state != BLE_GUI_CONNECTED)
            {
                lv_obj_clear_flag(ble_overlay, LV_OBJ_FLAG_HIDDEN);
                lv_obj_set_style_bg_color(ble_overlay, lv_color_hex(0x0C3214), 0); // Green
                lv_label_set_text(ble_label, "CONNECTED");
                current_ble_gui_state = BLE_GUI_CONNECTED;
            }
        }
    }
}

// --- ADD NEW BLE HOOKS ---
void gui_set_ble_connected(bool connected)
{
    ble_is_connected = connected;
    if (!connected)
    {
        ble_disconnect_time = millis();
    }
    else
    {
        current_ble_task[0] = '\0';
    }
}

void gui_set_ble_task(const char *task)
{
    strncpy(current_ble_task, task, sizeof(current_ble_task) - 1);
    current_ble_task[sizeof(current_ble_task) - 1] = '\0';
}

static lv_obj_t *create_gate_button(lv_obj_t *parent, int gate)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_add_style(btn, &style_gate_btn, 0);
    place_gate_button(btn, gate, left_x, grid_w);

    lv_obj_t *label = lv_label_create(btn);
    lv_obj_add_style(label, &style_gate_label, 0);
    lv_label_set_text(label, kGates[gate].label);
    lv_obj_center(label);

    return btn;
}

// Lay a gate out in the 2x2 grid spanning area_w from area_x
static void place_gate_button(lv_obj_t *btn, int gate, int area_x, int area_w)
{
    // Local index (0 to 3) for layout math on the current tile
    int local_i = gate % 4;
    const int w = (area_w - gap) / 2;
    const int x = (local_i % 2 == 0) ? area_x : area_x + w + gap;
    const int y = (local_i < 2) ? top_y : bottom_y;

    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, btn_h);
}

static lv_obj_t *create_reset_page(lv_obj_t *parent)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_add_style(btn, &style_reset_btn, 0);
    lv_obj_set_size(btn, grid_w - 8, grid_h / 2);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 6);

    lv_obj_t *label = lv_label_create(btn);
    lv_obj_add_style(label, &style_reset_label, 0);
    lv_label_set_text(label, "RESET");
    lv_obj_center(label);

    return btn;
}

// Sits just above state_label: a git version is too wide to share its row
static lv_obj_t *create_version_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_add_style(label, &style_info_label, 0);
    lv_label_set_text(label, VERSIONSTR);
    lv_obj_align(label, LV_ALIGN_BOTTOM_LEFT, left_x, -22);
    return label;
}

// Once a swipe has settled on a copy, jump to the real page it mirrors
static void gui_wrap_tiles()
{
    if (lv_obj_is_scrolling(tileview))
    {
        return;
    }

    const lv_coord_t scroll_x = lv_obj_get_scroll_x(tileview);
    if (scroll_x == lv_obj_get_x(tile_copy_of_3))
    {
        lv_obj_set_tile(tileview, tile3, LV_ANIM_OFF);
    }
    else if (scroll_x == lv_obj_get_x(tile_copy_of_1))
    {
        lv_obj_set_tile(tileview, tile1, LV_ANIM_OFF);
    }
}

static lv_obj_t *create_page_bar(lv_align_t align, const char *symbol, int step)
{
    lv_obj_t *bar = lv_btn_create(screen);
    lv_obj_add_style(bar, &style_gate_btn, 0);
    lv_obj_set_size(bar, bar_w, grid_h);
    lv_obj_align(bar, align, 0, 0);
    lv_obj_add_event_cb(bar, page_bar_event_handler, LV_EVENT_CLICKED, (void *)(intptr_t)step);

    lv_obj_t *label = lv_label_create(bar);
    lv_obj_add_style(label, &style_gate_label, 0);
    lv_label_set_text(label, symbol);
    lv_obj_center(label);

    return bar;
}

static void page_bar_event_handler(lv_event_t *e)
{
    lv_obj_t *const pages[] = {tile1, tile2, tile3};
    const int page_count = sizeof(pages) / sizeof(pages[0]);
    const int step = (int)(intptr_t)lv_event_get_user_data(e);

    const lv_obj_t *active = lv_tileview_get_tile_act(tileview);
    int page = 0;
    while (page < page_count - 1 && pages[page] != active)
    {
        ++page;
    }
    lv_obj_set_tile(tileview, pages[(page + step + page_count) % page_count], LV_ANIM_OFF);
}

static void screen_gesture_handler(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_get_act();
    if (lv_indev_get_gesture_dir(indev) == LV_DIR_TOP)
    {
        // Cancel the press, or a swipe starting on a button would also click it
        lv_indev_wait_release(indev);
        show_settings(true);
    }
}

static void settings_gesture_handler(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_get_act();
    if (lv_indev_get_gesture_dir(indev) == LV_DIR_BOTTOM)
    {
        lv_indev_wait_release(indev);
        show_settings(false);
    }
}

static void settings_slider_event_handler(lv_event_t *e)
{
    const lv_obj_t *target = lv_event_get_target(e);
    if (target == speed_slider)
    {
        settings.speed_percent = lv_slider_get_value(speed_slider);
        bloch.setSpeedPercent(settings.speed_percent);
    }
    else
    {
        settings.brightness = lv_slider_get_value(brightness_slider);
        gui_set_brightness(settings.brightness);
    }
    update_value_labels();
    gui_settings_changed();
}

static void settings_matrix_event_handler(lv_event_t *e)
{
    lv_obj_t *target = lv_event_get_target(e);
    const uint16_t selected = lv_btnmatrix_get_selected_btn(target);
    if (selected == LV_BTNMATRIX_BTN_NONE)
    {
        return;
    }

    if (target == page_mode_matrix)
    {
        settings.flip_pages = (selected == 1);
        apply_page_mode();
    }
    else
    {
        settings.dim_minutes = kDimMinutes[selected];
    }
    gui_settings_changed();
}

static void machine_number_event_handler(lv_event_t *e)
{
    const int step = (int)(intptr_t)lv_event_get_user_data(e);
    // Wrap around, so 99 is one tap away from 1
    settings.machine_number = (settings.machine_number - 1 + step + MAX_MACHINE_NUMBER) % MAX_MACHINE_NUMBER + 1;
    update_value_labels();
    machine_name_dirty = true;
    gui_settings_changed();
}

static void calibrate_event_handler(lv_event_t *e)
{
    show_settings(false);
    calibration_start();
}

static void settings_close_event_handler(lv_event_t *e)
{
    show_settings(false);
}

// Also discards the calibration, so it takes a second tap
static void settings_defaults_event_handler(lv_event_t *e)
{
    if (!defaults_armed)
    {
        defaults_armed = true;
        defaults_armed_time = millis();
        lv_label_set_text(defaults_label, "Tap again to confirm");
        return;
    }

    defaults_armed = false;
    lv_label_set_text(defaults_label, "Restore defaults");
    settings_restore_defaults();
    apply_settings();
    machine_name_dirty = true;
    gui_settings_changed();
}

static lv_obj_t *create_machine_button(int x, const char *text, int step)
{
    lv_obj_t *button = lv_btn_create(settings_panel);
    lv_obj_add_style(button, &style_gate_btn, 0);
    lv_obj_set_pos(button, x, 158);
    lv_obj_set_size(button, 56, 28);
    lv_obj_set_style_pad_all(button, 0, 0);
    lv_obj_add_event_cb(button, machine_number_event_handler, LV_EVENT_CLICKED, (void *)(intptr_t)step);
    lv_obj_add_event_cb(button, machine_number_event_handler, LV_EVENT_LONG_PRESSED_REPEAT, (void *)(intptr_t)step);

    lv_obj_t *label = lv_label_create(button);
    lv_obj_add_style(label, &style_info_label, 0);
    lv_label_set_text(label, text);
    lv_obj_center(label);

    return button;
}

static lv_obj_t *create_settings_row_label(const char *name, int y)
{
    lv_obj_t *label = lv_label_create(settings_panel);
    lv_obj_add_style(label, &style_info_label, 0);
    lv_label_set_text(label, name);
    lv_obj_set_pos(label, 10, y);
    return label;
}

static lv_obj_t *create_settings_slider(const char *name, int y, int min, int max, lv_obj_t **value_label)
{
    create_settings_row_label(name, y);

    lv_obj_t *slider = lv_slider_create(settings_panel);
    lv_obj_set_pos(slider, 104, y + 4);
    lv_obj_set_size(slider, 156, 10);
    lv_obj_set_ext_click_area(slider, 12);
    lv_slider_set_range(slider, min, max);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x3A5F5F), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x58A4B0), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0xBEE3DB), LV_PART_KNOB);
    lv_obj_add_event_cb(slider, settings_slider_event_handler, LV_EVENT_VALUE_CHANGED, NULL);

    *value_label = lv_label_create(settings_panel);
    lv_obj_add_style(*value_label, &style_state_label, 0);
    lv_obj_set_pos(*value_label, 272, y);

    return slider;
}

static lv_obj_t *create_settings_matrix(const char *name, int y, const char **map)
{
    create_settings_row_label(name, y + 6);

    lv_obj_t *matrix = lv_btnmatrix_create(settings_panel);
    lv_btnmatrix_set_map(matrix, map);
    lv_btnmatrix_set_btn_ctrl_all(matrix, LV_BTNMATRIX_CTRL_CHECKABLE);
    lv_btnmatrix_set_one_checked(matrix, true);
    lv_obj_set_pos(matrix, 104, y);
    lv_obj_set_size(matrix, 206, 28);
    lv_obj_set_style_bg_opa(matrix, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(matrix, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(matrix, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(matrix, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(matrix, lv_color_hex(0x3A5F5F), LV_PART_ITEMS);
    lv_obj_set_style_text_color(matrix, lv_color_hex(0xF2F7F7), LV_PART_ITEMS);
    lv_obj_set_style_bg_color(matrix, lv_color_hex(0x1F7A8C), (lv_style_selector_t)LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_add_event_cb(matrix, settings_matrix_event_handler, LV_EVENT_VALUE_CHANGED, NULL);

    return matrix;
}

// Lives on the top layer so it also opens over the BLE overlay
static void create_settings_panel()
{
    settings_panel = lv_obj_create(lv_layer_top());
    lv_obj_set_size(settings_panel, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_obj_set_pos(settings_panel, 0, DISPLAY_HEIGHT);
    lv_obj_set_style_pad_all(settings_panel, 0, 0);
    lv_obj_set_style_border_width(settings_panel, 0, 0);
    lv_obj_set_style_radius(settings_panel, 0, 0);
    lv_obj_set_style_bg_color(settings_panel, lv_color_hex(0x1A2A2A), 0);
    lv_obj_set_style_bg_grad_color(settings_panel, lv_color_hex(0x264D4D), 0);
    lv_obj_set_style_bg_grad_dir(settings_panel, LV_GRAD_DIR_VER, 0);
    lv_obj_clear_flag(settings_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(settings_panel, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(settings_panel, settings_gesture_handler, LV_EVENT_GESTURE, NULL);

    lv_obj_t *title = lv_label_create(settings_panel);
    lv_obj_add_style(title, &style_gate_label, 0);
    lv_label_set_text(title, "Settings");
    lv_obj_set_pos(title, 10, 6);

    lv_obj_t *close_button = lv_btn_create(settings_panel);
    lv_obj_add_style(close_button, &style_gate_btn, 0);
    lv_obj_set_pos(close_button, DISPLAY_WIDTH - 54, 6);
    lv_obj_set_size(close_button, 44, 32);
    lv_obj_add_event_cb(close_button, settings_close_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *close_label = lv_label_create(close_button);
    lv_obj_add_style(close_label, &style_info_label, 0);
    lv_label_set_text(close_label, LV_SYMBOL_CLOSE);
    lv_obj_center(close_label);

    speed_slider = create_settings_slider("Speed", 44, MIN_SPEED_PERCENT, 100, &speed_value_label);
    brightness_slider = create_settings_slider("Brightness", 68, MIN_BRIGHTNESS, 255, &brightness_value_label);
    page_mode_matrix = create_settings_matrix("Pages", 90, kPageModeMap);
    dim_matrix = create_settings_matrix("Dim after", 124, kDimMap);

    create_settings_row_label("Bluetooth", 164);
    create_machine_button(104, LV_SYMBOL_MINUS, -1);
    create_machine_button(254, LV_SYMBOL_PLUS, 1);
    machine_value_label = lv_label_create(settings_panel);
    lv_obj_add_style(machine_value_label, &style_state_label, 0);
    lv_obj_set_width(machine_value_label, 86);
    lv_obj_set_style_text_align(machine_value_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(machine_value_label, 164, 164);

    lv_obj_t *calibrate_button = lv_btn_create(settings_panel);
    lv_obj_add_style(calibrate_button, &style_gate_btn, 0);
    lv_obj_set_pos(calibrate_button, 10, 194);
    lv_obj_set_size(calibrate_button, 145, 38);
    lv_obj_add_event_cb(calibrate_button, calibrate_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *calibrate_label = lv_label_create(calibrate_button);
    lv_obj_add_style(calibrate_label, &style_info_label, 0);
    lv_label_set_text(calibrate_label, "Calibrate");
    lv_obj_center(calibrate_label);

    lv_obj_t *defaults_button = lv_btn_create(settings_panel);
    lv_obj_add_style(defaults_button, &style_reset_btn, 0);
    lv_obj_set_pos(defaults_button, 165, 194);
    lv_obj_set_size(defaults_button, 145, 38);
    lv_obj_add_event_cb(defaults_button, settings_defaults_event_handler, LV_EVENT_CLICKED, NULL);

    defaults_label = lv_label_create(defaults_button);
    lv_obj_add_style(defaults_label, &style_info_label, 0);
    lv_label_set_text(defaults_label, "Restore defaults");
    lv_obj_center(defaults_label);

    lv_obj_t *settings_version_label = lv_label_create(settings_panel);
    lv_obj_add_style(settings_version_label, &style_info_label, 0);
    lv_label_set_text(settings_version_label, VERSIONSTR);
    lv_obj_align(settings_version_label, LV_ALIGN_TOP_RIGHT, -64, 14);
}

static void settings_set_y_cb(void *obj, int32_t y)
{
    lv_obj_set_y((lv_obj_t *)obj, y);
}

static void settings_hidden_cb(lv_anim_t *a)
{
    lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);
}

static void show_settings(bool open)
{
    lv_obj_clear_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, settings_panel);
    lv_anim_set_exec_cb(&a, settings_set_y_cb);
    lv_anim_set_values(&a, lv_obj_get_y(settings_panel), open ? 0 : DISPLAY_HEIGHT);
    lv_anim_set_time(&a, kSettingsAnimMs);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    if (!open)
    {
        lv_anim_set_ready_cb(&a, settings_hidden_cb);
    }
    lv_anim_start(&a);
}

static void apply_settings()
{
    bloch.setSpeedPercent(settings.speed_percent);
    bloch.setRotationCorrection(settings.correction[0], settings.correction[1], settings.correction[2]);
    gui_set_brightness(settings.brightness);
    apply_page_mode();
    update_settings_widgets();
}

static void apply_page_mode()
{
    const bool flip = settings.flip_pages;

    if (flip)
    {
        lv_obj_clear_flag(tileview, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(prev_page_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(next_page_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(hint_label, "Tap the bars for pages   " LV_SYMBOL_UP " settings");
    }
    else
    {
        lv_obj_add_flag(tileview, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(prev_page_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(next_page_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(hint_label, "Swipe " LV_SYMBOL_LEFT " " LV_SYMBOL_RIGHT " pages   " LV_SYMBOL_UP " settings");
    }

    // Keep the real pages clear of the bars; the copies are only seen when swiping
    const int area_x = flip ? bar_w + gap : left_x;
    const int area_w = flip ? grid_w - 2 * area_x : grid_w;
    for (int i = 0; i < kTotalButtons; ++i)
    {
        place_gate_button(gate_buttons[i], i, area_x, area_w);
    }
    lv_obj_set_width(reset_button, flip ? area_w : grid_w - 8);
    lv_obj_align(version_label, LV_ALIGN_BOTTOM_LEFT, area_x, -22);
}

static void update_settings_widgets()
{
    lv_slider_set_value(speed_slider, settings.speed_percent, LV_ANIM_OFF);
    lv_slider_set_value(brightness_slider, settings.brightness, LV_ANIM_OFF);
    update_value_labels();

    lv_btnmatrix_set_btn_ctrl(page_mode_matrix, settings.flip_pages ? 1 : 0, LV_BTNMATRIX_CTRL_CHECKED);

    int dim_index = 0;
    for (size_t i = 0; i < sizeof(kDimMinutes); ++i)
    {
        if (kDimMinutes[i] == settings.dim_minutes)
        {
            dim_index = i;
        }
    }
    lv_btnmatrix_set_btn_ctrl(dim_matrix, dim_index, LV_BTNMATRIX_CTRL_CHECKED);
}

static void update_value_labels()
{
    lv_label_set_text_fmt(speed_value_label, "%d%%", (int)lv_slider_get_value(speed_slider));
    lv_label_set_text_fmt(brightness_value_label, "%d%%", (int)lv_slider_get_value(brightness_slider) * 100 / 255);
    lv_label_set_text_fmt(machine_value_label, "Bloch-%d", settings.machine_number);
}

// Saved later from gui_loop(), once the motors are idle
void gui_settings_changed()
{
    settings_dirty = true;
    settings_changed_time = millis();
}

static void update_screen_dim()
{
    const uint32_t timeout_ms = settings.dim_minutes * 60000UL;
    const bool dim = timeout_ms > 0 && lv_disp_get_inactive_time(NULL) > timeout_ms && !bloch.isBusy();
    if (dim == screen_dimmed)
    {
        return;
    }

    if (!dim)
    {
        // The touch that wakes the display must not also press a button
        lv_indev_wait_release(lv_indev_get_next(NULL));
    }
    screen_dimmed = dim;
    apply_backlight();
}

static void apply_backlight()
{
    const bool use_dim = screen_dimmed && current_brightness > DIM_BRIGHTNESS;
    i2c_bus_lock();
    _lgfx.setBrightness(use_dim ? DIM_BRIGHTNESS : current_brightness);
    i2c_bus_unlock();
}

static void gate_event_handler(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    {
        return;
    }

    const lv_obj_t *target = lv_event_get_target(e);
    for (int i = 0; i < kTotalButtons; ++i)
    {
        if (target == gate_buttons[i])
        {
            receiver_apply_permute(kGates[i].permute);
            update_state_label();
            return;
        }
    }
}

static void reset_event_handler(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    {
        return;
    }

    receiver_reset_bloch();
    update_state_label();
}

static void update_state_label()
{
    const Vector3d s = bloch.getState();
    char buf[64];
    snprintf(buf, sizeof(buf), "x:%0.3f  y:%0.3f  z:%0.3f", s[0], s[1], s[2]);
    lv_label_set_text(state_label, buf);
}
void gui_set_screen_color(int r, int g, int b)
{
    if (screen != NULL)
    {
        lv_color_t color = lv_color_make(r, g, b);

        // Overwrite both the base color and gradient color to make it solid
        lv_obj_set_style_bg_color(screen, color, 0);
        lv_obj_set_style_bg_grad_color(screen, color, 0);
    }
}
void gui_reset_screen_color()
{
    if (screen != NULL)
    {
        // Restore the original gradient colors
        lv_obj_set_style_bg_color(screen, lv_color_hex(0x1A2A2A), 0);
        lv_obj_set_style_bg_grad_color(screen, lv_color_hex(0x264D4D), 0);
        lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, 0);
    }
}

void gui_set_brightness(uint8_t brightness)
{
    current_brightness = brightness;
    apply_backlight();

    if (brightness_slider != NULL)
    {
        lv_slider_set_value(brightness_slider, brightness, LV_ANIM_OFF);
        update_value_labels();
    }
}