#ifndef SEGMENT_H
#define SEGMENT_H

typedef struct {
    float distance;
    float velocity;
    float acceleration;
    float jerk;
} __attribute__((packed)) Segment1D;

// Extra attributes on top of Segment1D for 2D trajectories
typedef struct {
    float x;
    float y;
    float heading;
} __attribute__((packed)) Segment2D_Ext;

#endif