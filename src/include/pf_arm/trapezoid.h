#ifndef TRAPEZOID_H
#define TRAPEZOID_H

#include "pf_arm/segment.h"

int pf_trapezoid_segment_count(float time_delta, float distance, float acceleration, float max_velocity);

int pf_trapezoid_generate_simd(Segment1D *segments_out, float time_delta, float distance, float acceleration, float max_velocity);

int pf_trapezoid_generate_c(Segment1D *segments_out, float time_delta, float distance, float acceleration, float max_velocity);

#endif