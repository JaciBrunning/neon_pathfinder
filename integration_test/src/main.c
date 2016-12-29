#include "pf_arm/trapezoid.h"
#include "pf_arm/scurve.h"
#include "pf_arm/trajectory.h"

#include <sys/time.h>
#include <time.h>
#include <stdio.h>

long long current_time_millis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)((unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000);
}

Segment1D segments[8192];
Segment2D_Ext segments_ext[8192];
Waypoint2D waypoints[16];

void print_header(const char *test_name) {
    fprintf(stderr, "%s\n", test_name);
}

void print_stat(const char *test_method, float dt, int iterations, int samples, long long time) {
    float dtKhz = dt * 1000;
    if (samples == 0) {
        // 1D Test
        fprintf(stderr, "\t%-4s (%.1fkHz x%d): %6lldms AVG: %.2fms\n", test_method, dtKhz, iterations, time, time / (double)iterations);
    } else {
        float sampleKhz = (float)samples / 1000;
        fprintf(stderr, "\t%-4s (%.1fkHz %.0fKSamples x%d): %6lldms AVG: %.2fms\n", test_method, dtKhz, sampleKhz, iterations, time, time / (double)iterations);
    }
}

void trapezoid() {
    int segs, i;
    long long start_simd, end_simd, start_c, end_c;

    float dt = 0.001, dist = 10, acc = 2, vel = 2, seg_time = 0.0;
    int iterations = 100;

    print_header("TRAPEZOID (LINEAR)");

    start_simd = current_time_millis();
    for (i = 0; i < iterations; i++)
        segs = pf_trapezoid_generate_simd(segments, dt, dist, acc, vel);
    end_simd = current_time_millis();

    start_c = current_time_millis();
    for (i = 0; i < iterations; i++)
        segs = pf_trapezoid_generate_c(segments, dt, dist, acc, vel);
    end_c = current_time_millis();

    FILE *file = fopen("test_trapezoid_linear.csv", "w");
    fprintf(file, "distance,velocity,acceleration,jerk,time\n");
    for (i = 0; i < (segs); i++) {
        Segment1D seg = segments[i];
        fprintf(file, "%f,%f,%f,%f,%f\n", seg.distance, seg.velocity, seg.acceleration, seg.jerk, seg_time);
        seg_time += dt;
    }
    fclose(file);
    print_stat("SIMD", dt, iterations, 0, end_simd - start_simd);
    print_stat("C", dt, iterations, 0, end_c - start_c);
}

void scurve() {
    int segs, i;
    long long start_c, end_c;

    float dt = 0.001, dist = 10, jerk = 10, acc = 2, vel = 2, seg_time = 0.0;
    int iterations = 100;

    print_header("SCURVE (LINEAR)");

    start_c = current_time_millis();
    for (i = 0; i < 100; i++)
        segs = pf_scurve_generate(segments, dt, dist, jerk, acc, vel);
    end_c = current_time_millis();

    FILE *file = fopen("test_scurve_linear.csv", "w");
    fprintf(file, "distance,velocity,acceleration,jerk,time\n");
    for (i = 0; i < segs; i++) {
        Segment1D seg = segments[i];
        fprintf(file, "%f,%f,%f,%f,%f\n", seg.distance, seg.velocity, seg.acceleration, seg.jerk, seg_time);
        seg_time += dt;
    }
    fclose(file);
    print_stat("C", dt, iterations, 0, end_c - start_c);
}

void trajectory(int curve_type) {
    int segs, i;
    long long start_c, end_c, start_simd, end_simd;

    Waypoint2D p1 = { -4, -1, 3.1415/4 };     // Waypoint @ x=-4, y=-1, exit angle=45 degrees
    Waypoint2D p2 = { -1, 2, 0 };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
    Waypoint2D p3 = {  2, 4, 0 };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
    waypoints[0] = p1;
    waypoints[1] = p2;
    waypoints[2] = p3;

    int samples = 10000, iterations = 100, len = 0, waypoint_count = 3;
    float dt = 0.001, jerk = 10, acc = 2, vel = 2, seg_time = 0.0;
    
    FILE *file;

    if (curve_type == CurveType_SCurve) {
        print_header("SCURVE (TRAJECTORY)");
        file = fopen("test_scurve_trajectory.csv", "w");
    } else if (curve_type == CurveType_Trapezoid) {
        print_header("TRAPEZOID (TRAJECTORY)");
        file = fopen("test_trapezoid_trajectory.csv", "w");
    }

    start_c = current_time_millis();
    for (i = 0; i < iterations; i++)
        len = pf_generate_trajectory(segments, segments_ext, curve_type, waypoints, waypoint_count, samples, dt, vel, acc, jerk, 0);
    end_c = current_time_millis();

    start_simd = current_time_millis();
    for (i = 0; i < iterations; i++)
        len = pf_generate_trajectory(segments, segments_ext, curve_type, waypoints, waypoint_count, samples, dt, vel, acc, jerk, 1);
    end_simd = current_time_millis();

    fprintf(file, "x,y,heading,distance,velocity,acceleration,jerk,time\n");
    for (i = 0; i < len; i++) {
        Segment1D seg = segments[i];
        Segment2D_Ext ext = segments_ext[i];
        fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f\n", ext.x, ext.y, ext.heading, seg.distance, seg.velocity, seg.acceleration, seg.jerk, seg_time);
        seg_time += dt;
    }
    fclose(file);
    print_stat("SIMD", dt, iterations, samples, end_simd - start_simd);
    print_stat("C", dt, iterations, samples, end_c - start_c);
}


int main() {
    trapezoid();
    fprintf(stderr, "\n");
    scurve();
    fprintf(stderr, "\n");
    trajectory(CurveType_Trapezoid);
    fprintf(stderr, "\n");
    trajectory(CurveType_SCurve);
    return 0;
}