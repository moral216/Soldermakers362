#include "vision.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

static void core1_vision_loop(void) {
    MLX90640_I2CInit();

    while (1) {
        tight_loop_contents();
    }
}

void vision_start(void) {
    multicore_launch_core1(core1_vision_loop);
}

bool vision_get_target(IronTarget *out) {
    (void)out;
    return false;
}