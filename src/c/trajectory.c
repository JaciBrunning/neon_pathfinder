#include "pf_arm/trajectory.h"
#include "pf_arm/scurve.h"
#include "pf_arm/trapezoid.h"

#include <math.h>

#ifndef MINI
    #define MINI(a,b) (((a)<(b))?(a):(b))
    #define MAXI(a,b) (((a)>(b))?(a):(b))
#endif

#define PI 3.14159265358979323846
#define TAU PI*2
Spline2D _splines[16];

float bound_radians(float angle) {
    float newAngle = fmod(angle, TAU);
    if (newAngle < 0) newAngle = TAU + newAngle;
    return newAngle;
}

extern float pf_spline_distance_internal(Spline2D *spline, int sample_count);

int pf_generate_trajectory(Segment1D *segments, Segment2D_Ext *segments_out, int curve_type, Waypoint2D *waypoints, 
                            int waypoint_count, int sample_count, float time_delta, float max_velocity, float max_acceleration, 
                            float max_jerk, int simd_optimize) {
    float total_length = 0;

    int i;
    for (i = 0; i < waypoint_count - 1; i++) {
        Spline2D *s = &_splines[i];
        pf_fit_spline(waypoints[i], waypoints[i+1], s);
        if (simd_optimize)
            total_length += pf_spline_distance_internal(s, sample_count);
        else
            total_length += pf_spline_distance(s, sample_count);
    }

    int segments_len = 0;

    if (curve_type == CurveType_SCurve) {
        segments_len = pf_scurve_generate(segments, time_delta, total_length, max_jerk, max_acceleration, max_velocity);
    } else if (curve_type == CurveType_Trapezoid) {
        if (simd_optimize)
            segments_len = pf_trapezoid_generate_simd(segments, time_delta, total_length, max_acceleration, max_velocity);
        else
            segments_len = pf_trapezoid_generate_c(segments, time_delta, total_length, max_acceleration, max_velocity);
    }

    float delta_angle = waypoints[waypoint_count-1].angle - waypoints[0].angle;
    for (i = 0; i < segments_len; i++) {
        Segment2D_Ext *e = &segments_out[i];
        Segment1D s = segments[i];
        e->x = s.distance;
        e->y = 0;
        e->heading = waypoints[0].angle + delta_angle * (s.distance) / (segments[segments_len-1].distance);
    }

    int spline_i = 0;
    float spline_pos_initial = 0, splines_complete = 0;
    for (i = 0; i < segments_len; i++) {
        float pos = segments[i].distance;
        int found = 0;
        while (!found) {
            float pos_rel = pos - spline_pos_initial;
            Spline2D si = _splines[spline_i];
            if (pos_rel <= si.arc_length) {
                float percentage = pos_rel / si.arc_length;
                Coord2D coords = pf_spline_coords(&si, percentage);
                segments_out[i].x = coords.x;
                segments_out[i].y = coords.y;
                segments_out[i].heading = pf_spline_angle(&si, percentage);
                found = 1;
            } else if (spline_i < waypoint_count - 2) {
                splines_complete += si.arc_length;
                spline_pos_initial = splines_complete;
                spline_i += 1;
            } else {
                Coord2D coords = pf_spline_coords(&si, 1.0);
                segments_out[i].x = coords.x;
                segments_out[i].y = coords.y;
                segments_out[i].heading = pf_spline_angle(&si, 1.0);
                found = 1;
            }
        }
    }

    return segments_len;
}

void pf_fit_spline(Waypoint2D a, Waypoint2D b, Spline2D *s) {
    s->x_offset = a.x;
    s->y_offset = a.y;

    float delta = sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
    s->knot_distance = delta;
    s->angle_offset = atan2(b.y - a.y, b.x - a.x);

    float a0_delta = tan(bound_radians(a.angle - s->angle_offset));
    float a1_delta = tan(bound_radians(b.angle - s->angle_offset));

    s->a = 0;
    s->b = 0;
    s->c = (a0_delta + a1_delta) / (s->knot_distance * s->knot_distance);
    s->d = -(2 * a0_delta + a1_delta) / s->knot_distance;
    s->e = a0_delta;
}

float pf_spline_distance(Spline2D *s, int sample_count) {
    double sample_count_d = (double) sample_count;
    
    double a = s->a; double b = s->b; double c = s->c; 
    double d = s->d; double e = s->e; double knot = s->knot_distance;
    
    double arc_length = 0, t = 0, dydt = 0;
    
    double deriv0 = pf_spline_deriv_2(a, b, c, d, e, knot, 0);
    
    double integrand = 0;
    double last_integrand = sqrt(1 + deriv0*deriv0) / sample_count_d;
    
    int i;
    for (i = 0; i < sample_count; i = i + 1) {
        t = i / sample_count_d;
        dydt = pf_spline_deriv_2(a, b, c, d, e, knot, t);
        integrand = sqrt(1 + dydt*dydt) / sample_count_d;
        arc_length += (integrand + last_integrand) / 2;
        last_integrand = integrand;
    }
    double al = knot * arc_length;
    s->arc_length = al;
    return al;
}

// Progress

Coord2D pf_spline_coords(Spline2D *s, float percentage) {
    percentage = MAXI(MINI(percentage, 1), 0);
    double x = percentage * s->knot_distance;
    double y = (s->a*x + s->b) * (x*x*x*x) + (s->c*x + s->d) * (x*x) + s->e*x;    // Heh, sex
    
    double cos_theta = cos(s->angle_offset);
    double sin_theta = sin(s->angle_offset);
    
    Coord2D c = {
        x * cos_theta - y * sin_theta + s->x_offset,
        x * sin_theta + y * cos_theta + s->y_offset
    };
    return c;
}

float pf_spline_deriv(Spline2D *s, float percentage) {
    double x = percentage * s->knot_distance;
    return (5*s->a*x + 4*s->b) * (x*x*x) + (3*s->c*x + 2*s->d) * x + s->e;
}

float pf_spline_deriv_2(float a, float b, float c, float d, float e, float k, float p) {
    double x = p * k;
    return (5*a*x + 4*b) * (x*x*x) + (3*c*x + 2*d) * x + e;
}

float pf_spline_angle(Spline2D *s, float percentage) {
    return bound_radians(atan(pf_spline_deriv(s, percentage)) + s->angle_offset);
}