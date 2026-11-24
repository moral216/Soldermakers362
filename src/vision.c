#include "vision.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <math.h>
#include <stddef.h>

#define MLX90640_ADDR 0x33
#define MLX_REFRESH_RATE_8HZ 0x04
#define MLX_EMISSIVITY 0.95f
#define MLX_TA_SHIFT_C 8.0f

#define IRON_DETECT_TEMP_C 150.0f
#define IRON_PIXEL_MIN_C 148.0f
#define IRON_PEAK_WINDOW_C 15.0f

static uint32_t core1_stack[2048];

static uint16_t ee_data[MLX90640_EEPROM_DUMP_NUM];
static uint16_t frame_data[834];
static paramsMLX90640 mlx_params;
static float temp_map[MLX90640_PIXEL_NUM];

// Shared state between Core 1 (writer) and Core 0 (reader)
static critical_section_t target_lock;
static IronTarget latest_target = {0.0f, 0.0f, false};
static bool has_valid_frame = false;
static bool vision_started = false;

static IronTarget compute_centroid(const float *temps) {
    IronTarget result = {0.0f, 0.0f, false};

    float max_temp = -1000.0f;
    for (int i = 0; i < MLX90640_PIXEL_NUM; i++) {
        float t = temps[i];
        if (!isnan(t) && t > max_temp) {
            max_temp = t;
        }
    }

    if (max_temp < IRON_DETECT_TEMP_C) {
        return result; // iron_detected remains false
    }

    float cutoff = max_temp - IRON_PEAK_WINDOW_C;
    if (cutoff < IRON_PIXEL_MIN_C) {
        cutoff = IRON_PIXEL_MIN_C;
    }

    float sum_w = 0.0f;
    float sum_wx = 0.0f;
    float sum_wy = 0.0f;

    for (int row = 0; row < MLX90640_LINE_NUM; row++) {
        int row_offset = row * MLX90640_COLUMN_NUM;
        for (int col = 0; col < MLX90640_COLUMN_NUM; col++) {
            float t = temps[row_offset + col];
            if (!isnan(t) && t > cutoff) {
                float w = t - cutoff;
                sum_w  += w;
                sum_wx += w * (float)col;
                sum_wy += w * (float)row;
            }
        }
    }

    if (sum_w > 0.0f) {
        result.x = sum_wx / sum_w;
        result.y = sum_wy / sum_w;
        result.iron_detected = true;
    }

    return result;
}

static void core1_vision_loop(void) {
    MLX90640_I2CInit();

    while (MLX90640_DumpEE(MLX90640_ADDR, ee_data) != MLX90640_NO_ERROR) {
        sleep_ms(50);
    }

    
    (void)MLX90640_ExtractParameters(ee_data, &mlx_params);

    (void)MLX90640_SetChessMode(MLX90640_ADDR);
    (void)MLX90640_SetRefreshRate(MLX90640_ADDR, MLX_REFRESH_RATE_8HZ);
    (void)MLX90640_SynchFrame(MLX90640_ADDR);

    uint8_t subpages_seen = 0;

    while (1) {
        int subpage = MLX90640_GetFrameData(MLX90640_ADDR, frame_data);
        if (subpage < 0) {
            sleep_ms(10);
            continue;
        }

        float ta = MLX90640_GetTa(frame_data, &mlx_params);
        float tr = ta - MLX_TA_SHIFT_C;
        MLX90640_CalculateTo(frame_data, &mlx_params, MLX_EMISSIVITY, tr, temp_map);

        // Wait until both subpage 0 and subpage 1 have populated temp_map once
        subpages_seen |= (uint8_t)(1u << (subpage & 1));
        if (subpages_seen != 0x3u) {
            continue;
        }

        int mode = (frame_data[832] & MLX90640_CTRL_MEAS_MODE_MASK) >> MLX90640_CTRL_MEAS_MODE_SHIFT;
        MLX90640_BadPixelsCorrection(mlx_params.brokenPixels, temp_map, mode, &mlx_params);
        MLX90640_BadPixelsCorrection(mlx_params.outlierPixels, temp_map, mode, &mlx_params);

        IronTarget next = compute_centroid(temp_map);

        critical_section_enter_blocking(&target_lock);
        latest_target = next;
        has_valid_frame = true;
        critical_section_exit(&target_lock);
    }
}

void vision_start(void) {
    if (vision_started) {
        return;
    }
    critical_section_init(&target_lock);
    vision_started = true;
    multicore_launch_core1_with_stack(core1_vision_loop, core1_stack, sizeof(core1_stack));
}

// Populates *out with the latest target.
// Returns true once Core 1 has produced at least one full thermal frame.
bool vision_get_target(IronTarget *out) {
    if (!out || !vision_started) {
        return false;
    }

    critical_section_enter_blocking(&target_lock);
    bool valid = has_valid_frame;
    if (valid) {
        *out = latest_target;
    }
    critical_section_exit(&target_lock);

    return valid;
}