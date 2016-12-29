#ifndef SCURVE_H
#define SCURVE_H

#include "pf_arm/segment.h"

int pf_scurve_segment_count(float time_delta, float distance, float jerk, float max_acceleration, float max_velocity);
int pf_scurve_generate(Segment1D *segments_out, float time_delta, float distance, float jerk, float max_acceleration, float max_velocity);

#endif