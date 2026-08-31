/*
 * Differential-drive recruitment task
 *
 * The communication and decoding stages provide a target coordinate. Implement
 * drive_to_target() so the simulated differential-drive rover reaches the
 * target using valid left and right wheel velocities.
 */

#include <math.h>
#include <stdbool.h>

#define PI_F 3.14159265358979323846f

#define WHEEL_RADIUS 0.15f
#define WHEEL_SEPARATION 0.77f
#define MAX_LINEAR_VELOCITY 1.0f
#define MAX_ANGULAR_VELOCITY 2.0f
#define MAX_WHEEL_VELOCITY 10.0f
#define HEADING_GAIN 1.25f

#define TARGET_TOLERANCE 0.10f
#define DRIVE_DT_SECONDS 0.02f
#define MAX_DRIVE_STEPS 6000

/*
 * Latitude and longitude are normalized local simulation coordinates
 * measured in metres.
 *
 * Latitude  = north/south axis
 * Longitude = east/west axis
 *
 * The differential-drive rover is planar, so altitude is received
 * but not changed.
 */
struct coordinate {
    float latitude;
    float longitude;
    float altitude;
};

/*
 * Heading is in radians:
 * 0 radians = east
 * positive rotation = counter-clockwise
 */
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

/* Provided simulator helpers. */
static float normalize_angle(float angle);

static bool apply_wheel_velocities(struct rover_state *rover,
                                   struct wheel_velocity velocity);


/*
 * Drive the rover toward the target coordinate.
 */
enum drive_status drive_to_target(struct rover_state *rover,
                                  const struct coordinate *target)
{
    /* Check for invalid pointers */
    if (rover == NULL || target == NULL) {
        return DRIVE_INVALID_INPUT;
    }

    /* Check rover values */
    if (!isfinite(rover->position.latitude) ||
        !isfinite(rover->position.longitude) ||
        !isfinite(rover->position.altitude) ||
        !isfinite(rover->heading_rad)) {
        return DRIVE_INVALID_INPUT;
    }

    /* Check target values */
    if (!isfinite(target->latitude) ||
        !isfinite(target->longitude) ||
        !isfinite(target->altitude)) {
        return DRIVE_INVALID_INPUT;
    }

    /*
     * Run the controller for a maximum number of steps.
     * This guarantees that the function will terminate.
     */
    for (int step = 0; step < MAX_DRIVE_STEPS; ++step) {

        /*
         * Calculate distance from rover to target.
         *
         * longitude = X/east
         * latitude  = Y/north
         */
        float dx =
            target->longitude - rover->position.longitude;

        float dy =
            target->latitude - rover->position.latitude;

        float distance = hypotf(dx, dy);

        /*
         * Stop if the rover is close enough to the target.
         */
        if (distance <= TARGET_TOLERANCE) {

            struct wheel_velocity stop = {
                0.0f,
                0.0f
            };

            if (!apply_wheel_velocities(rover, stop)) {
                return DRIVE_INVALID_COMMAND;
            }

            return DRIVE_REACHED_TARGET;
        }

        /*
         * Calculate the direction from the rover to the target.
         */
        float desired_heading = atan2f(dy, dx);

        /*
         * Calculate the smallest heading error.
         * normalize_angle() handles the +/- PI wraparound.
         */
        float heading_error =
            normalize_angle(desired_heading -
                            rover->heading_rad);

        /*
         * Forward velocity.
         *
         * If the target is behind the rover, rotate first instead
         * of driving strongly in the wrong direction.
         */
        float linear_velocity = 0.0f;

        if (fabsf(heading_error) < (PI_F / 2.0f)) {
            linear_velocity =
                fminf(distance, MAX_LINEAR_VELOCITY);
        }

        /*
         * Proportional heading controller.
         */
        float angular_velocity =
            HEADING_GAIN * heading_error;

        /*
         * Limit angular velocity.
         */
        if (angular_velocity > MAX_ANGULAR_VELOCITY) {
            angular_velocity = MAX_ANGULAR_VELOCITY;
        }

        if (angular_velocity < -MAX_ANGULAR_VELOCITY) {
            angular_velocity = -MAX_ANGULAR_VELOCITY;
        }

        /*
         * Convert linear/angular velocity into
         * left/right wheel velocities.
         *
         * left  = (v - w*L/2) / r
         * right = (v + w*L/2) / r
         */
        float left_velocity =
            (linear_velocity -
             angular_velocity * WHEEL_SEPARATION / 2.0f)
            / WHEEL_RADIUS;

        float right_velocity =
            (linear_velocity +
             angular_velocity * WHEEL_SEPARATION / 2.0f)
            / WHEEL_RADIUS;

        /*
         * Keep left wheel within allowed range.
         */
        if (left_velocity > MAX_WHEEL_VELOCITY) {
            left_velocity = MAX_WHEEL_VELOCITY;
        }

        if (left_velocity < -MAX_WHEEL_VELOCITY) {
            left_velocity = -MAX_WHEEL_VELOCITY;
        }

        /*
         * Keep right wheel within allowed range.
         */
        if (right_velocity > MAX_WHEEL_VELOCITY) {
            right_velocity = MAX_WHEEL_VELOCITY;
        }

        if (right_velocity < -MAX_WHEEL_VELOCITY) {
            right_velocity = -MAX_WHEEL_VELOCITY;
        }

        struct wheel_velocity velocity = {
            left_velocity,
            right_velocity
        };

        /*
         * Apply the wheel command to the simulator.
         */
        if (!apply_wheel_velocities(rover, velocity)) {
            return DRIVE_INVALID_COMMAND;
        }
    }

    /*
     * Target was not reached within the allowed number
     * of simulation steps.
     */
    return DRIVE_MAX_STEPS_EXCEEDED;
}


/*
 * Keep an angle between -PI and +PI.
 */
static float normalize_angle(float angle)
{
    while (angle > PI_F) {
        angle -= 2.0f * PI_F;
    }

    while (angle < -PI_F) {
        angle += 2.0f * PI_F;
    }

    return angle;
}


/*
 * Simulator helper.
 *
 * Converts left/right wheel velocities into rover movement.
 */
static bool apply_wheel_velocities(struct rover_state *rover,
                                   struct wheel_velocity velocity)
{
    /*
     * Reject invalid wheel commands.
     */
    if (rover == NULL ||
        !isfinite(velocity.left) ||
        !isfinite(velocity.right) ||
        fabsf(velocity.left) > MAX_WHEEL_VELOCITY ||
        fabsf(velocity.right) > MAX_WHEEL_VELOCITY) {
        return false;
    }

    /*
     * Differential-drive equations.
     */
    const float linear_velocity =
        WHEEL_RADIUS *
        (velocity.left + velocity.right) /
        2.0f;

    const float angular_velocity =
        WHEEL_RADIUS *
        (velocity.right - velocity.left) /
        WHEEL_SEPARATION;

    /*
     * Update heading.
     */
    rover->heading_rad =
        normalize_angle(
            rover->heading_rad +
            angular_velocity * DRIVE_DT_SECONDS
        );

    /*
     * Update longitude (east/west).
     */
    rover->position.longitude +=
        linear_velocity *
        cosf(rover->heading_rad) *
        DRIVE_DT_SECONDS;

    /*
     * Update latitude (north/south).
     */
    rover->position.latitude +=
        linear_velocity *
        sinf(rover->heading_rad) *
        DRIVE_DT_SECONDS;

    return true;
}
