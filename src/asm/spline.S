.section .rodata
    f32_step_1:
        .float 0
        .float 1
    f32_step_2:
        .float 2
        .float 3

.global pf_spline_distance_internal
pf_spline_distance_internal:
    push {fp}
    add fp, sp, #0  // Push new Stack Frame

    // Arg 0: r0: Spline Pointer
    // Arg 1: r1: Sample Count
    // <return>: : Total Distance of all Splines (Arc Length)

    mov r3, r1      // Duplicate Sample Count into r3 for percentage.
    lsr r1, #2      // Logical Shift Right by 2 Bits (2^2 = 4 = 4 samples at a time)

    // Initialize Arc Length into s0 (d0)
    veor d0, d0     // XOR itself to 0

    // Convert Sample Count into a Single Precision float s2
    vmov s2, r3
    vcvt.f32.s32 s2, s2
    vmov r3, s2

    // Deriv of Spline Initial (deriv0) in r2 and s3
    //    x = p * k. p == 0 \therefore x = 0
    //    dy/dt = (5ax + 4b)x^3 + (3cx + 2d)x + e
    //        \therefore dy/dt = e
    ldr r2, [r0, #(4*4)]
    vmov s3, r2

    // Load last integrand into s4
    //      last_integrand = sqrt(1 + deriv0^2) / sample_count
    vmul.f32 s4, s3, s3
    vmov.f32 s5, #1.0       // Temporary Storage
    vadd.f32 s4, s4, s5
    vsqrt.f32 s4, s4
    vdiv.f32 s4, s4, s2

    // Load Spline properties (a, b, c, d, e, knot) into q2-q6 + q12
    ldr r4, [r0, #(0*4)]
    vdup.32 q2, r4
    ldr r4, [r0, #(1*4)]
    vdup.32 q3, r4
    ldr r4, [r0, #(2*4)]
    vdup.32 q4, r4
    ldr r4, [r0, #(3*4)]
    vdup.32 q5, r4
    ldr r4, [r0, #(4*4)]
    vdup.32 q6, r4
    ldr r4, [r0, #(8*4)]
    vdup.32 q12, r4

    // Write 'time base' into q8. Increased by 4 every iteration
    vmov.f32 q8, #0.0

    // Write 'time step' into q9
    ldr r6, =f32_step_1
    vld1.32 d18, [r6]
    ldr r6, =f32_step_2
    vld1.32 d19, [r6]

    // Move 1 / sample_count into q10. Not using reciprocal as this is more accurate
    vmov s28, r3
    vmov s29, #1.0
    vdiv.f32 s28, s29, s28
    vmov r7, s28
    vdup.32 q10, r7

    // Duplicate number of segments for each iteration in q11
    vmov.f32 q11, #4.0

    ._psdi_loop:
        // Store current iterator in q7
        vadd.f32 q7, q8, q9
        // Set q7 to our percentage of the way through sampling
        //      t = i / sample_count
        vmul.f32 q7, q7, q10

        // Store Deriv dy/dt in q13
        //      x = t * k
        vmul.f32 q14, q7, q12   // Q14 Temp for x, We can now reuse q7
        //      dy/dt = (5ax + 4b)x^3 + (3cx + 2d)x + e
        vmov.f32 q13, q6        // e
        // 3cx
        vmov.f32 q7, #3.0
        vmul.f32 q7, q7, q4
        vmul.f32 q7, q7, q14
        // 2d
        vmov.f32 q15, #2.0
        vmul.f32 q15, q5
        // (3cx + 2d) * x
        vadd.f32 q7, q7, q15
        vmul.f32 q7, q7, q14
        // Add to q13 (our dy/dt running total)
        vadd.f32 q13, q13, q7

        // Since we're using Hermite Cubic, we actually don't need this, but we're going to use
        // it anyway in case we decide to move to quartic or quintic in the future.
        // The lines 107 to 121 will actually equate to adding 0 to q13 (our running dy/dt total)

        // 5ax
        vmov.f32 q7, #5.0
        vmul.f32 q7, q7, q2
        vmul.f32 q7, q7, q14
        // 4b
        vmov.f32 q15, #4.0
        vmul.f32 q15, q3
        // 5ax + 4b
        vadd.f32 q7, q7, q15
        // x^3 (x is no longer needed, we can do this in place @ Q14 using Q15 as scratch)
        vmul.f32 q15, q14, q14
        vmul.f32 q14, q14, q15
        // (5ax + 4b) * x^3
        vmul.f32 q7, q7, q14
        // Add to q13 (our dy/dt running total)
        vadd.f32 q13, q13, q7

        // dy/dt is now in q13, (q7 + q14 + q15 are now free)
        // Calculate the integrand into q7
        //      integrand = sqrt(1 + dydt^2) / sample_count
        vmov.f32 q7, #1.0
        vmul.f32 q14, q13, q13
        vadd.f32 q7, q7, q14

        // We're operating on q7[0] to q7[3], which is s28 to s31
        vsqrt.f32 s28, s28
        vsqrt.f32 s29, s29
        vsqrt.f32 s30, s30
        vsqrt.f32 s31, s31

        vmul.f32 q7, q7, q10

        // Increase time set by 4
        vadd.f32 q8, q8, q11

        // Now we can do the averaging of the integrands, adding to the arc length.
        // Store the previous integrand in s4, arc length totalling in s0
        // s1 used as scratch storage
        // s2 used to store 2.0
        vmov.f32 s2, #2.0
        //      arc_length += (integrand + last_integrand) / 2
        // We're going to have to do this for every entry in the registers above.

        // Last Integrand & Index 0 (q7[0] = s28)
        vadd.f32 s1, s4, s28
        vdiv.f32 s1, s1, s2
        vadd.f32 s0, s0, s1

        // Index 0 & Index 1 (q7[1] = s29)
        vadd.f32 s1, s28, s29
        vdiv.f32 s1, s1, s2
        vadd.f32 s0, s0, s1

        // Index 1 & Index 2 (q7[2] = s30)
        vadd.f32 s1, s29, s30
        vdiv.f32 s1, s1, s2
        vadd.f32 s0, s0, s1

        // Index 2 & Index 3 (q7[3] = s31)
        vadd.f32 s1, s30, s31
        vdiv.f32 s1, s1, s2
        vadd.f32 s0, s0, s1

        // Load Index 3 into the Last Integrand s4
        vmov.f32 s4, s31

        sub r1, r1, #1
        cmp r1, #0
        bgt ._psdi_loop

    ._psdi_loop_out:

    // Temporarily move knot into q7 so we can access it on the s registers
    vmov.f32 q7, q12

    // Multiply our arc length by knot (knot = q7[any idx] = s28)
    vmul.f32 s0, s0, s28

    // Move s0 (our total arc length) into r1 temporarily
    vmov r1, s0

    // Move r1 into our Spline Pointer's arc_length field
    str r1, [r0, #(9*4)]

    // Move r1 into r0 for the return value
    mov r0, r1

    sub sp, fp, #0  // Pop our Stack Frame
    pop {fp}
    bx lr