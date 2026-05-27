#pragma once

#include <cstdint>

/**
 * POD struct with every user-tunable scan parameter.
 * Lives on the host. Read by VolumeIntegration once per step
 * (under a mutex) so the GUI can mutate it on another thread.
 */
struct ScanParameters {
    // ---- Volume (require reset to apply) ----
    unsigned int xDim      = 400;
    unsigned int yDim      = 400;
    unsigned int zDim      = 400;
    float        voxelSize = 0.01f;   // meters

    // ---- TSDF ----
    float maxTruncation = 0.03f;      // meters

    // ---- Bilateral filter ----
    float sigma_d = 5.0f;             // spatial sigma (kernel size derived; reset to apply)
    float sigma_r = 5.0f;             // range sigma  (live)

    // ---- Normal estimation ----
    float normalThreshold = 0.03f;

    // ---- Raycast ----
    float raycastNear = 0.3f;
    float raycastFar  = 4.0f;
    float raycastStep = 0.025f;

    // ---- ICP (require reset to apply) ----
    int   icpIterations0 = 10;
    int   icpIterations1 = 5;
    int   icpIterations2 = 4;
    float icpDistThresh  = 0.10f;
    float icpAngleThresh = 0.342f;    // sin(20 deg)

    // ---- Marching cubes ----
    float isoValue = 0.0f;

    // ---- Display gating ----
    bool useDisplay = true;           // false => no cv::imshow/waitKey
};
