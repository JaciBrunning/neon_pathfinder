#include "pf_arm/trapezoid.h"

#include <math.h>

extern void pf_trapezoid_generate_internal(Segment1D *segments, float time_delta, float acceleration, int segments_needed, float initial_distance, float initial_velocity);

int pf_trapezoid_segment_count(float time_delta, float distance, float acceleration, float max_velocity) {
    float accel_dist = 0.5 * max_velocity * max_velocity / acceleration;
    if (accel_dist * 2 > distance) {
        return (int)(sqrt(distance/acceleration) / time_delta) * 2;
    } else {
        return (int)(max_velocity / acceleration / time_delta) * 2 + (distance - accel_dist * 2) / max_velocity / time_delta;
    }
}

int pf_trapezoid_generate_simd(Segment1D *segments_out, float time_delta, float distance, float acceleration, float max_velocity) {
    float accel_dist = 0.5 * max_velocity * max_velocity / acceleration;    // Rearranging v^2 = 2as for s

    if (accel_dist * 2 > distance) {
        // We'll never steady out at max_velocity, use a triangular profile instead
        int accel_segments = (int)(sqrt(distance / acceleration) / time_delta);

        // Generate the speed up
        pf_trapezoid_generate_internal(segments_out, time_delta, acceleration, accel_segments, 0, 0);
        // Generate the speed down
        pf_trapezoid_generate_internal(segments_out + accel_segments, time_delta, -acceleration, accel_segments, segments_out[accel_segments].distance, segments_out[accel_segments].velocity);
        return accel_segments * 2;
    } else {
        float remaining_distance = distance - (accel_dist * 2);
        float hold_time = remaining_distance / max_velocity;
        
        int accel_segments = (int)(max_velocity / acceleration / time_delta);
        int hold_segments = hold_time / time_delta;

        // Generate the Speed Up
        pf_trapezoid_generate_internal(segments_out, time_delta, acceleration, accel_segments, 0, 0);
        // Generate the Speed Hold
        pf_trapezoid_generate_internal(segments_out + accel_segments, time_delta, 0, hold_segments, segments_out[accel_segments].distance, segments_out[accel_segments].velocity);
        // Generate the Speed Down
        pf_trapezoid_generate_internal(segments_out + accel_segments + hold_segments, time_delta, -acceleration, accel_segments, segments_out[accel_segments+hold_segments].distance, segments_out[accel_segments+hold_segments].velocity);
        return accel_segments * 2 + hold_segments;
    }
}

void pf_trapezoid_generate_c_internal(Segment1D *segments, float time_delta, float acceleration, int segments_needed, float initial_distance, float initial_velocity) {
    int i;
    for (i = 0; i < segments_needed; i++) {
        float time = time_delta * i;
        // v = u + at
        float velocity = initial_velocity + acceleration * time;
        // s = s0 + ut + 0.5at^2
        float distance = initial_distance + initial_velocity * time + 0.5 * acceleration * time * time;

        segments[i].distance = distance;
        segments[i].velocity = velocity;
        segments[i].acceleration = acceleration;
        segments[i].jerk = 0.0;
    }
}

int pf_trapezoid_generate_c(Segment1D *segments_out, float time_delta, float distance, float acceleration, float max_velocity) {
    float accel_dist = 0.5 * max_velocity * max_velocity / acceleration;    // Rearranging v^2 = 2as for s

    if (accel_dist * 2 > distance) {
        // We'll never steady out at max_velocity, use a triangular profile instead
        int accel_segments = (int)(sqrt(distance / acceleration) / time_delta);

        // Generate the speed up
        pf_trapezoid_generate_c_internal(segments_out, time_delta, acceleration, accel_segments, 0, 0);
        // Generate the speed down
        pf_trapezoid_generate_c_internal(segments_out + accel_segments, time_delta, -acceleration, accel_segments, segments_out[accel_segments].distance, segments_out[accel_segments].velocity);
        
        return accel_segments * 2;
    } else {
        float remaining_distance = distance - (accel_dist * 2);
        float hold_time = remaining_distance / max_velocity;
        
        int accel_segments = (int)(max_velocity / acceleration / time_delta);
        int hold_segments = hold_time / time_delta;

        // Generate the Speed Up
        pf_trapezoid_generate_c_internal(segments_out, time_delta, acceleration, accel_segments, 0, 0);
        // Generate the Speed Hold
        pf_trapezoid_generate_c_internal(segments_out + accel_segments, time_delta, 0, hold_segments, segments_out[accel_segments].distance, segments_out[accel_segments].velocity);
        // Generate the Speed Down
        pf_trapezoid_generate_c_internal(segments_out + accel_segments + hold_segments, time_delta, -acceleration, accel_segments, segments_out[accel_segments+hold_segments].distance, segments_out[accel_segments+hold_segments].velocity);

        return accel_segments * 2 + hold_segments;
    }
}