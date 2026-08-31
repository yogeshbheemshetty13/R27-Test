/*
 * Differential-drive recruitment task
 *
 * The communication and decoding stages provide a target coordinate.
 * This implementation drives the simulated differential-drive rover
 * toward the target while respecting wheel-speed limits.
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
 */
struct coordinate {
  float latitude;
  float longitude;
  float altitude;
};

/* Heading is in radians: zero points east and positive rotation is CCW. */
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
 * Drive the rover to the requested target.
 */
enum drive_status drive_to_target(struct rover_state *rover,
                                  const struct coordinate *target) {
  /* Validate pointers. */
  if (rover == NULL || target == NULL) {
    return DRIVE_INVALID_INPUT;
  }

  /* Validate all input values. */
  if (!isfinite(rover->position.latitude) ||
      !isfinite(rover->position.longitude) ||
      !isfinite(rover->position.altitude) ||
      !isfinite(rover->heading_rad) ||
      !isfinite(target->latitude) ||
      !isfinite(target->longitude) ||
      !isfinite(target->altitude)) {
    return DRIVE_INVALID_INPUT;
  }

  for (int step = 0; step < MAX_DRIVE_STEPS; step++) {

    /* Calculate distance to target. */
    float dx = target->longitude - rover->position.longitude;
    float dy = target->latitude - rover->position.latitude;

    float distance = hypotf(dx, dy);

    /* Stop when target is reached. */
    if (distance <= TARGET_TOLERANCE) {
      struct wheel_velocity stop = {0.0f, 0.0f};

      if (!apply_wheel_velocities(rover, stop)) {
        return DRIVE_INVALID_COMMAND;
      }

      return DRIVE_REACHED_TARGET;
    }

    /* Calculate desired heading. */
    float desired_heading = atan2f(dy, dx);

    /* Calculate shortest heading error. */
    float heading_error =
        normalize_angle(desired_heading - rover->heading_rad);

    float left_velocity;
    float right_velocity;

    /*
     * If the rover is facing too far away from the target,
     * rotate first.
     */
    if (fabsf(heading_error) > 0.5f) {

      float angular_velocity = HEADING_GAIN * heading_error;

      if (angular_velocity > MAX_ANGULAR_VELOCITY) {
        angular_velocity = MAX_ANGULAR_VELOCITY;
      }

      if (angular_velocity < -MAX_ANGULAR_VELOCITY) {
        angular_velocity = -MAX_ANGULAR_VELOCITY;
      }

      /* Rotate in place. */
      left_velocity =
          -(angular_velocity * WHEEL_SEPARATION) /
          (2.0f * WHEEL_RADIUS);

      right_velocity =
          (angular_velocity * WHEEL_SEPARATION) /
          (2.0f * WHEEL_RADIUS);

    } else {

      /*
       * Move forward while correcting the heading.
       */
      float linear_velocity = distance;

      if (linear_velocity > MAX_LINEAR_VELOCITY) {
        linear_velocity = MAX_LINEAR_VELOCITY;
      }

      float angular_velocity = HEADING_GAIN * heading_error;

      if (angular_velocity > MAX_ANGULAR_VELOCITY) {
        angular_velocity = MAX_ANGULAR_VELOCITY;
      }

      if (angular_velocity < -MAX_ANGULAR_VELOCITY) {
        angular_velocity = -MAX_ANGULAR_VELOCITY;
      }

      left_velocity =
          (linear_velocity -
           angular_velocity * WHEEL_SEPARATION / 2.0f)
          / WHEEL_RADIUS;

      right_velocity =
          (linear_velocity +
           angular_velocity * WHEEL_SEPARATION / 2.0f)
          / WHEEL_RADIUS;
    }

    /*
     * Keep both wheel velocities within the required limit.
     */
    float max_velocity = fmaxf(fabsf(left_velocity),
                               fabsf(right_velocity));

    if (max_velocity > MAX_WHEEL_VELOCITY) {
      float scale = MAX_WHEEL_VELOCITY / max_velocity;

      left_velocity *= scale;
      right_velocity *= scale;
    }

    struct wheel_velocity velocity = {
      left_velocity,
      right_velocity
    };

    /* Apply the calculated wheel velocities. */
    if (!apply_wheel_velocities(rover, velocity)) {
      return DRIVE_INVALID_COMMAND;
    }
  }

  /* Target was not reached within the allowed number of steps. */
  return DRIVE_MAX_STEPS_EXCEEDED;
}

/*
 * Normalize an angle to the range [-PI, PI].
 */
static float normalize_angle(float angle) {
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
 * Candidates should not modify this function.
 */
static bool apply_wheel_velocities(struct rover_state *rover,
                                   struct wheel_velocity velocity) {
  if (!isfinite(velocity.left) || !isfinite(velocity.right) ||
      fabsf(velocity.left) > MAX_WHEEL_VELOCITY ||
      fabsf(velocity.right) > MAX_WHEEL_VELOCITY) {
    return false;
  }

  const float linear_velocity =
      WHEEL_RADIUS * (velocity.left + velocity.right) / 2.0f;

  const float angular_velocity =
      WHEEL_RADIUS * (velocity.right - velocity.left) /
      WHEEL_SEPARATION;

  rover->heading_rad = normalize_angle(
      rover->heading_rad + angular_velocity * DRIVE_DT_SECONDS);

  rover->position.longitude +=
      linear_velocity * cosf(rover->heading_rad) *
      DRIVE_DT_SECONDS;

  rover->position.latitude +=
      linear_velocity * sinf(rover->heading_rad) *
      DRIVE_DT_SECONDS;

  return true;
}
