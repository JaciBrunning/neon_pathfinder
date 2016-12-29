#include "pf_arm/scurve.h"

#include <math.h>

#ifndef MINI
    #define MINI(a,b) (((a)<(b))?(a):(b))
    #define MAXI(a,b) (((a)>(b))?(a):(b))
#endif

float f1_buffer[8192];

int pf_scurve_segment_count(float time_delta, float distance, float jerk, float max_acceleration, float max_velocity) {
    float ma2 = max_acceleration * max_acceleration;
    float mj2 = jerk * jerk;

    float checked_max_velocity = MINI(
        max_velocity,
        (-ma2 + sqrt(ma2 * ma2 + 4 * (mj2 * max_acceleration * distance))) / (2 * jerk)
    );

    int filter_1 = (int)ceil(checked_max_velocity / max_acceleration / time_delta);
    int filter_2 = (int)ceil(max_acceleration / jerk / time_delta);

    float impulse = distance / checked_max_velocity / time_delta;
    return (int)ceil(filter_1 + filter_2 + impulse);
}

// https://github.com/JacisNonsense/Pathfinder/blob/master/Pathfinder-Core/src/trajectory.c#L24
int pf_scurve_generate(Segment1D *segments_out, float time_delta, float distance, float jerk, float max_acceleration, float max_velocity) {
    float ma2 = max_acceleration * max_acceleration;
    float mj2 = jerk * jerk;

    float checked_max_velocity = MINI(
        max_velocity,
        (-ma2 + sqrt(ma2 * ma2 + 4 * (mj2 * max_acceleration * distance))) / (2 * jerk)
    );

    int filter_1 = (int)ceil(checked_max_velocity / max_acceleration / time_delta);
    int filter_2 = (int)ceil(max_acceleration / jerk / time_delta);

    float impulse = distance / checked_max_velocity / time_delta;
    int len = (int)ceil(filter_1 + filter_2 + impulse);

    float f1_last = 0, f2 = 0, last_vel = 0, last_dist = 0, last_acc = 0;
    int i;
    for (i = 0; i < len; i++) {
        float input = MINI(impulse, 1);
        if (input < 1) {
            input -= 1;
            impulse = 0;
        } else {
            impulse -= input;
        }

        f1_buffer[i] = MAXI(0.0, MINI(filter_1, f1_last + input));
        f2 = 0;
        int j;
        for (j = 0; j < filter_2; j++) {
            if (i - j < 0) break;
            f2 += f1_buffer[i - j];
        }
        f2 = f2 / filter_1;

        segments_out[i].velocity = f2 / filter_2 * checked_max_velocity;
        segments_out[i].distance = (last_vel + segments_out[i].velocity) / 2.0 * time_delta + last_dist; // s = (0.5*u*v)t + s0
        segments_out[i].acceleration = (segments_out[i].velocity - last_vel) / time_delta;
        segments_out[i].jerk = (segments_out[i].acceleration - last_acc) / time_delta;

        last_vel = segments_out[i].velocity;
        last_dist = segments_out[i].distance;
        last_acc = segments_out[i].acceleration;

        f1_last = f1_buffer[i];
    }
    return len;
}