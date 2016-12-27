#include "pf_arm/trapezoid.h"

#include <sys/time.h>
#include <time.h>
#include <stdio.h>

long long current_time_millis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)((unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000);
}

Trapezoid_Segment segments[8092];

int main() {
    int segs, i;
    double time;
    long long start_simd, end_simd, start_c, end_c;

    start_simd = current_time_millis();
    for (i = 0; i < 1000; i++)
        segs = pf_trapezoid_generate_simd(segments, 0.001, 10, 2, 2);
    end_simd = current_time_millis();

    start_c = current_time_millis();
    for (i = 0; i < 1000; i++)
        segs = pf_trapezoid_generate_c(segments, 0.001, 10, 2, 2);
    end_c = current_time_millis();

    printf("distance,velocity,acceleration,time\n");
    for (i = 0; i < (segs); i++) {
        Trapezoid_Segment seg = segments[i];
        printf("%f,%f,%f,%f\n", seg.distance, seg.velocity, seg.acceleration, time);
        time += seg.time_delta;
    }
    fprintf(stderr, "SIMD Time Taken: %lldms\n", end_simd - start_simd);
    fprintf(stderr, "C Time Taken: %lldms\n", end_c - start_c);
    return 0;
}