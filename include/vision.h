#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x;            // 0.0 to 31.0
    float y;            // 0.0 to 23.0
    bool iron_detected;
} IronTarget;

void vision_start(void);
bool vision_get_target(IronTarget *out);