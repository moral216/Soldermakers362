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