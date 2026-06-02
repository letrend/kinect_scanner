#pragma once

#include <cstdint>
#include <string>

enum PoseSource {
    PoseSourceIcp = 0,
    PoseSourceActuatedTcp = 1,
    PoseSourceSimulation = 2
};

/**
 * POD struct with every user-tunable scan parameter.
 * Lives on the host. Read by VolumeIntegration once per step
 * (under a mutex) so the GUI can mutate it on another thread.
 */
struct ScanParameters {
    // ---- Pose source ----
    int poseSource = PoseSourceIcp;

    // ---- Volume (require reset to apply) ----
    unsigned int xDim      = 400;
    unsigned int yDim      = 400;
    unsigned int zDim      = 400;
    // Default to ~2 mm: roughly the depth-noise floor of the Kinect v2 at
    // typical scan distance (0.5-1.5 m). Smaller voxels just add memory
    // pressure without resolving more real geometry.
    float        voxelSize = 0.002f;  // meters

    // ---- Initial volume position (offset of volume center from the
    //      camera's initial pose, in camera coordinates: +X right,
    //      +Y down, +Z forward). Applied at the next grid init. ----
    float gridInitOffsetX = 0.0f;
    float gridInitOffsetY = 0.0f;
    float gridInitOffsetZ = 1.0f;     // 1 m in front of the camera

    // ---- TSDF ----
    float maxTruncation = 0.03f;      // meters
    float depthEdgeThreshold = 0.02f; // meters; 0 disables silhouette rejection
    float tsdfMaxWeight = 64.0f;
    bool  tsdfConflictDecay = true;

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
    float icpMinInlierRatio = 0.03f;
    float icpMaxResidual = 0.035f;    // meters
    float icpMaxTranslationStep = 0.10f; // meters
    float icpMaxRotationStepDeg = 15.0f;
    int   icpLostFrameLimit = 3;
    int   icpRecoveryFrameCount = 2;
    float icpDepthCutoff = 0.0f;      // meters; <=0 means use raycastFar

    // ---- Global ICP recovery ----
    bool  globalRecoveryEnabled = true;
    int   globalRecoveryMinFrames = 20;
    float globalRecoveryMinVoxelWeight = 8.0f;
    float globalRecoveryYawStepDeg = 20.0f;
    float globalRecoveryPitchMinDeg = -35.0f;
    float globalRecoveryPitchMaxDeg = 35.0f;
    float globalRecoveryPitchStepDeg = 15.0f;
    std::string globalRecoveryRadiusOffsetsM = "-0.20,0.0,0.20";
    int   globalRecoveryTopCandidates = 12;
    float globalRecoveryMinInlierRatio = 0.06f;
    float globalRecoveryMaxResidual = 0.03f;
    float globalRecoveryCooldownMs = 500.0f;

    // ---- Marching cubes ----
    float isoValue = 0.0f;

    // ---- Actuated scan path ----
    float angleStartDeg = 0.0f;
    float angleEndDeg = 360.0f;
    float angleStepDeg = 10.0f;
    float stageStartMm = 0.0f;
    float stageEndMm = 0.0f;
    float stageStepMm = 25.0f;
    int   framesPerPose = 1;
    float targetSettleMs = 100.0f;
    float angleToleranceDeg = 0.5f;
    float stageToleranceMm = 1.0f;
    float targetTimeoutMs = 10000.0f;

    // ---- Turntable / sensor geometry ----
    float turntableRadiusMm = 150.0f;
    float turntableHeightMm = 40.0f;
    float kinectOffsetXMm = 0.0f;
    float kinectOffsetYMm = -150.0f;
    float kinectOffsetZMm = -1000.0f;
    float kinectRollDeg = 0.0f;
    float kinectPitchDeg = 0.0f;
    float kinectYawDeg = 0.0f;
    float stageAxisX = 0.0f;
    float stageAxisY = -1.0f;
    float stageAxisZ = 0.0f;

    // ---- Simulation ----
    bool simulationEnabled = false;
    bool simulationAutoStart = false;
    bool simBenchmarkEnabled = false;
    std::string simScenario = "object_turntable";
    std::string simMotionPreset = "room_sweep";
    std::string simMotionPath;
    std::string simReportPath = "simulation_report.json";
    std::string simStlPath;
    float simStlScale = 1.0f;            // STL units -> mm
    bool simAutoCenter = true;
    bool simRenderTurntable = true;
    float simDepthNoiseMm = 0.0f;
    float simDropoutPercent = 0.0f;
    float simRaycastNearMm = 100.0f;
    float simRaycastFarMm = 4000.0f;
    float simRoomWidthM = 4.0f;
    float simRoomHeightM = 2.6f;
    float simRoomDepthM = 5.0f;
    int   simClutterCount = 8;
    bool  simTextureFeatures = true;
    float simPoseJitterMm = 0.0f;
    float simPoseJitterDeg = 0.0f;

    // ---- TCP control ----
    bool controlTcpEnabled = true;
    std::string controlTcpHost = "127.0.0.1";
    int controlTcpPort = 5056;
    bool actuatorTcpEnabled = false;
    std::string actuatorTcpHost = "0.0.0.0";
    int actuatorTcpPort = 5055;

    // ---- Display gating ----
    bool useDisplay = true;           // false => no cv::imshow/waitKey
};
