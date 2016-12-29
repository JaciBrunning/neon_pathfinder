.section .rodata
    f32_step_1:
        .float 0
        .float 1
    f32_step_2:
        .float 2
        .float 3

// Process 4 at a time

// d0-d1 {Q0} -> Distances
// d2-d3 {Q1} -> Velocities
// d4-d5 {Q2} -> Acceleration
// d6-d7 {Q3} -> Jerk

.global pf_trapezoid_generate_internal
pf_trapezoid_generate_internal:
    push {fp}
    add fp, sp, #0  // Push new Stack Frame

    // Arg 0: r0: Segments
    // Arg 1: r1: Time Delta
    // Arg 2: r2: Acceleration
    // Arg 3: r3: Segments Needed

    // Args 4+ are stored on the stack
    ldr r4, [sp, #(0+4)]    // Load Arg 4 (Initial Distance) into r4
    vdup.32 q11, r4         // Initial Distance in Q11
    ldr r5, [sp, #(4+4)]    // Load Arg 5 (Initial Velocity) into r5
    vdup.32 q12, r5         // Initial Velocity in Q12

    lsr r3, #2      // Logical Shift Right by 2 Bits (2^2 = 4 = 4 segments at a time)
    add r3, r3, #1  // Add one to r3 for cases where the number of segments isn't exactly divisible by 4

    // Duplicate number of segments for each iteration into q7
    vmov.f32 q7, #4.0

    vdup.32 q2, r2  // Write acceleration into q2
    vmov.f32 q3, #0.0   // Write jerk into q3 (this is 0 since acceleration isn't sloped)
    vdup.32 q14, r1     // Write time delta into q14

    vmov.f32 q4, #0.0  // Duplicate the segment index offset into q4. This is used in the calculation of time. Increases after each iteration set
    
    // Write 'segment step' into q5
    ldr r6, =f32_step_1
    vld1.32 d10, [r6]
    ldr r6, =f32_step_2
    vld1.32 d11, [r6]

    // Write '0.5' into q10
    vmov.f32 q10, #0.5

    ._pftgi_loop:
        // Current iterator (segment_index) will be stored in q6
        vadd.f32 q6, q4, q5

        // Store time index in q8 (t = time_delta * segment_index)
        vmul.f32 q8, q14, q6

        // Store velocity in q1 (v = u + a*t)
        vmul.f32 q1, q8, q2
        vadd.f32 q1, q1, q12    // Add initial velocity

        // s = s0 + ut + 0.5at^2
        vmul.f32 q13, q12, q8   // Calculate ut into q13
        vmul.f32 q8, q8, q8     // Square time
        vmul.f32 q9, q2, q10    // Half acceleration
        vmul.f32 q0, q8, q9     // Store distance in q0
        vadd.f32 q0, q0, q13    // Add ut to the distance
        vadd.f32 q0, q0, q11    // Add initial distance to distance

        vst4.32 {d0, d2, d4, d6}, [r0]!
        vst4.32 {d1, d3, d5, d7}, [r0]!

        // Increase segment index offset by 4
        vadd.f32 q4, q4, q7

        sub r3, r3, #1
        cmp r3, #0
        bgt ._pftgi_loop

    sub sp, fp, #0  // Pop our Stack Frame
    pop {fp}
    bx lr