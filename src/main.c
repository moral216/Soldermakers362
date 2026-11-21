#include <stdio.h>
#include "pico/stdlib.h"
#include "vision.h"

int main(void) {
    stdio_init_all();
    vision_start();

    while (1) {
        sleep_ms(1000);
    }

    return 0;
}