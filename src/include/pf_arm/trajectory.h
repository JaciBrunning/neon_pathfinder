#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "pf_arm/segment.h"

typedef struct {
    float x, y, angle;
} __attribute__((packed)) Waypoint2D;

typedef struct {
    float x, y;
} __attribute__((packed)) Coord2D;

typedef struct {
    float a, b, c, d, e;
    float x_offset, y_offset, angle_offset, knot_distance, arc_length;
} __attribute__((packed)) Spline2D;

enum CurveType {
    CurveType_SCurve, CurveType_Trapezoid
};

int pf_generate_trajectory(Segment1D *segments, Segment2D_Ext *segments_out, int curve_type, Waypoint2D *waypoints, 
                            int waypoint_count, int sample_count, float time_delta, float max_velocity, float max_acceleration, 
                            float max_jerk, int simd_optimize);

// Fit is fairly low iterations. Hermite cubic
void pf_fit_spline(Waypoint2D a, Waypoint2D b, Spline2D *s);

// Spline_Distance (arc_length) is high iterations
float pf_spline_distance(Spline2D *s, int samples);

// Spline {coords, deriv, deriv2, angle} are all single iteration
Coord2D pf_spline_coords(Spline2D *s, float percentage);
float pf_spline_deriv(Spline2D *s, float percentage);
float pf_spline_deriv_2(float a, float b, float c, float d, float e, float k, float p);
float pf_spline_angle(Spline2D *s, float percentage);

#endif
