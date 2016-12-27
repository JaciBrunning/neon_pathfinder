#ifndef TRAPEZOID_H
#define TRAPEZOID_H

typedef struct {
    float distance;
    float velocity;
    float acceleration;
    float time_delta;
} Trapezoid_Segment;

// Generate a trapezoidal motion profile given the target acceleration, distance to cover and the maximum
// velocity we want to use
// Returns the number of segments provided.
int pf_trapezoid_generate_simd(Trapezoid_Segment *segments_out, float time_delta, float distance, float acceleration, float max_velocity);

int pf_trapezoid_generate_c(Trapezoid_Segment *segments_out, float time_delta, float distance, float acceleration, float max_velocity);

#endif