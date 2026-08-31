#ifndef DRIVE_H
#define DRIVE_H

#include <stdbool.h>

struct coordinate {
    float latitude;
    float longitude;
    float altitude;
};

struct rover_state {
    struct coordinate position;
    float heading_rad;
};

struct wheel_velocity {
    float left;
    float right;
};

enum drive_status {
    DRIVE_REACHED_TARGET = 0,
    DRIVE_INVALID_INPUT = -1,
    DRIVE_INVALID_COMMAND = -2,
    DRIVE_MAX_STEPS_EXCEEDED = -3
};

enum drive_status drive_to_target(struct rover_state *rover,
                                  const struct coordinate *target);

#endif
