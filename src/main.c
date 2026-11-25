/**
 * main.c — RP2350 ADC self-test. NOTHING needs to be wired.
 *
 * Reads the RP2350's built-in temperature sensor, which is wired to the ADC
 * inside the chip. If you get sensible numbers, the ADC works.
 *
 * Serial monitor at 115200. Press a fingertip on the big chip in the middle
 * of the board and watch the temperature climb.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "vision.h"

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("Launching Core 1 vision...\n");
    vision_start();

    while (1) {
        IronTarget target;
        if (vision_get_target(&target)) {
            if (target.iron_detected) {
                printf("DETECTED | x=%5.2f, y=%5.2f\n", target.x, target.y);
            } else {
                printf("IDLE     | no hot object above threshold\n");
            }
        } else {
            printf("WAITING  | initializing camera...\n");
        }
        sleep_ms(200);
    }

    return 0;
}

// ALEX CODE
/*
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define TEMP_ADC_INPUT  (NUM_ADC_CHANNELS - 1)  
#define ADC_VREF        3.3f
#define ADC_MAX         4095.0f
#define N_AVG           64

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);                      

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(TEMP_ADC_INPUT);

    printf("\n=== RP2350 ADC self-test (no wiring) ===\n");
    printf("Reading internal temperature sensor on ADC input %d\n\n", TEMP_ADC_INPUT);

    while (true) {
        uint32_t sum = 0;
        for (int i = 0; i < N_AVG; i++) {
            sum += adc_read();
        }
        float raw     = (float)sum / N_AVG;
        float volts   = raw * ADC_VREF / ADC_MAX;
        float temp_c  = 27.0f - (volts - 0.706f) / 0.001721f;

        printf("raw %6.1f   voltage %.4f V   chip temp %5.1f C\n", raw, volts, temp_c);
        sleep_ms(500);
    }
}
*/
