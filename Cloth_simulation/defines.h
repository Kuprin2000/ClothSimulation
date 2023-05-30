#pragma once

// if we use V-sync

#define V_SYNC


// if we use anti-aliasing

#define MSAA4


// if we need nice timings statistic
// useful for debug, but affects performance

// #define MEASURE_TIME


// let you check vertices corrections constraints are performing
// useful for debug, but affects performance

// #define CHECK_DELTAS
#ifdef CHECK_DELTAS
#define MAX_LENGTH 1.0f
#endif


// run test for FRAMES_COUNT frames
// and measure total time

// #define PERFORMANCE_TEST
#ifdef PERFORMANCE_TEST
#define FRAMES_COUNT 100
#endif
