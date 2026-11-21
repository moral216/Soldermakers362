#pragma once

typedef enum {
    STATE_IDLE,          // No soldering iron detected
    STATE_TRACKING,      // Iron detected, arm actively following
    STATE_FAULT_STALL    // Current spike detected, motors halted
} SystemState;