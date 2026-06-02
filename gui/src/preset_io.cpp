#include "preset_io.hpp"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

namespace preset_io {

static QJsonObject toJson(const ScanParameters &p) {
    QJsonObject o;
    o["poseSource"] = p.poseSource;
    o["xDim"] = (int)p.xDim;
    o["yDim"] = (int)p.yDim;
    o["zDim"] = (int)p.zDim;
    o["voxelSize"]       = p.voxelSize;
    o["maxTruncation"]   = p.maxTruncation;
    o["depthEdgeThreshold"] = p.depthEdgeThreshold;
    o["tsdfMaxWeight"]   = p.tsdfMaxWeight;
    o["tsdfConflictDecay"] = p.tsdfConflictDecay;
    o["sigma_d"]         = p.sigma_d;
    o["sigma_r"]         = p.sigma_r;
    o["normalThreshold"] = p.normalThreshold;
    o["raycastNear"]     = p.raycastNear;
    o["raycastFar"]      = p.raycastFar;
    o["raycastStep"]     = p.raycastStep;
    o["icpIterations0"]  = p.icpIterations0;
    o["icpIterations1"]  = p.icpIterations1;
    o["icpIterations2"]  = p.icpIterations2;
    o["icpDistThresh"]   = p.icpDistThresh;
    o["icpAngleThresh"]  = p.icpAngleThresh;
    o["icpMinInlierRatio"] = p.icpMinInlierRatio;
    o["icpMaxResidual"] = p.icpMaxResidual;
    o["icpMaxTranslationStep"] = p.icpMaxTranslationStep;
    o["icpMaxRotationStepDeg"] = p.icpMaxRotationStepDeg;
    o["icpLostFrameLimit"] = p.icpLostFrameLimit;
    o["icpRecoveryFrameCount"] = p.icpRecoveryFrameCount;
    o["icpDepthCutoff"] = p.icpDepthCutoff;
    o["globalRecoveryEnabled"] = p.globalRecoveryEnabled;
    o["globalRecoveryMinFrames"] = p.globalRecoveryMinFrames;
    o["globalRecoveryMinVoxelWeight"] = p.globalRecoveryMinVoxelWeight;
    o["globalRecoveryYawStepDeg"] = p.globalRecoveryYawStepDeg;
    o["globalRecoveryPitchMinDeg"] = p.globalRecoveryPitchMinDeg;
    o["globalRecoveryPitchMaxDeg"] = p.globalRecoveryPitchMaxDeg;
    o["globalRecoveryPitchStepDeg"] = p.globalRecoveryPitchStepDeg;
    o["globalRecoveryRadiusOffsetsM"] = QString::fromStdString(p.globalRecoveryRadiusOffsetsM);
    o["globalRecoveryTopCandidates"] = p.globalRecoveryTopCandidates;
    o["globalRecoveryMinInlierRatio"] = p.globalRecoveryMinInlierRatio;
    o["globalRecoveryMaxResidual"] = p.globalRecoveryMaxResidual;
    o["globalRecoveryCooldownMs"] = p.globalRecoveryCooldownMs;
    o["isoValue"]        = p.isoValue;
    o["angleStartDeg"]   = p.angleStartDeg;
    o["angleEndDeg"]     = p.angleEndDeg;
    o["angleStepDeg"]    = p.angleStepDeg;
    o["stageStartMm"]    = p.stageStartMm;
    o["stageEndMm"]      = p.stageEndMm;
    o["stageStepMm"]     = p.stageStepMm;
    o["framesPerPose"]   = p.framesPerPose;
    o["targetSettleMs"]  = p.targetSettleMs;
    o["angleToleranceDeg"] = p.angleToleranceDeg;
    o["stageToleranceMm"] = p.stageToleranceMm;
    o["targetTimeoutMs"] = p.targetTimeoutMs;
    o["turntableRadiusMm"] = p.turntableRadiusMm;
    o["turntableHeightMm"] = p.turntableHeightMm;
    o["kinectOffsetXMm"] = p.kinectOffsetXMm;
    o["kinectOffsetYMm"] = p.kinectOffsetYMm;
    o["kinectOffsetZMm"] = p.kinectOffsetZMm;
    o["kinectRollDeg"]   = p.kinectRollDeg;
    o["kinectPitchDeg"]  = p.kinectPitchDeg;
    o["kinectYawDeg"]    = p.kinectYawDeg;
    o["stageAxisX"]      = p.stageAxisX;
    o["stageAxisY"]      = p.stageAxisY;
    o["stageAxisZ"]      = p.stageAxisZ;
    o["actuatorTcpHost"] = QString::fromStdString(p.actuatorTcpHost);
    o["actuatorTcpPort"] = p.actuatorTcpPort;
    o["simStlPath"]      = QString::fromStdString(p.simStlPath);
    o["simBenchmarkEnabled"] = p.simBenchmarkEnabled;
    o["simScenario"] = QString::fromStdString(p.simScenario);
    o["simMotionPreset"] = QString::fromStdString(p.simMotionPreset);
    o["simMotionPath"] = QString::fromStdString(p.simMotionPath);
    o["simReportPath"] = QString::fromStdString(p.simReportPath);
    o["simDepthNoiseMm"] = p.simDepthNoiseMm;
    o["simDropoutPercent"] = p.simDropoutPercent;
    o["simRoomWidthM"] = p.simRoomWidthM;
    o["simRoomHeightM"] = p.simRoomHeightM;
    o["simRoomDepthM"] = p.simRoomDepthM;
    o["simClutterCount"] = p.simClutterCount;
    o["simTextureFeatures"] = p.simTextureFeatures;
    o["simPoseJitterMm"] = p.simPoseJitterMm;
    o["simPoseJitterDeg"] = p.simPoseJitterDeg;
    return o;
}

static void fromJson(const QJsonObject &o, ScanParameters &p) {
    p.poseSource = o.value("poseSource").toInt(p.poseSource);
    p.xDim = (unsigned)o.value("xDim").toInt((int)p.xDim);
    p.yDim = (unsigned)o.value("yDim").toInt((int)p.yDim);
    p.zDim = (unsigned)o.value("zDim").toInt((int)p.zDim);
    p.voxelSize       = (float)o.value("voxelSize").toDouble(p.voxelSize);
    p.maxTruncation   = (float)o.value("maxTruncation").toDouble(p.maxTruncation);
    p.depthEdgeThreshold = (float)o.value("depthEdgeThreshold").toDouble(p.depthEdgeThreshold);
    p.tsdfMaxWeight   = (float)o.value("tsdfMaxWeight").toDouble(p.tsdfMaxWeight);
    p.tsdfConflictDecay = o.value("tsdfConflictDecay").toBool(p.tsdfConflictDecay);
    p.sigma_d         = (float)o.value("sigma_d").toDouble(p.sigma_d);
    p.sigma_r         = (float)o.value("sigma_r").toDouble(p.sigma_r);
    p.normalThreshold = (float)o.value("normalThreshold").toDouble(p.normalThreshold);
    p.raycastNear     = (float)o.value("raycastNear").toDouble(p.raycastNear);
    p.raycastFar      = (float)o.value("raycastFar").toDouble(p.raycastFar);
    p.raycastStep     = (float)o.value("raycastStep").toDouble(p.raycastStep);
    p.icpIterations0  = o.value("icpIterations0").toInt(p.icpIterations0);
    p.icpIterations1  = o.value("icpIterations1").toInt(p.icpIterations1);
    p.icpIterations2  = o.value("icpIterations2").toInt(p.icpIterations2);
    p.icpDistThresh   = (float)o.value("icpDistThresh").toDouble(p.icpDistThresh);
    p.icpAngleThresh  = (float)o.value("icpAngleThresh").toDouble(p.icpAngleThresh);
    p.icpMinInlierRatio = (float)o.value("icpMinInlierRatio").toDouble(p.icpMinInlierRatio);
    p.icpMaxResidual = (float)o.value("icpMaxResidual").toDouble(p.icpMaxResidual);
    p.icpMaxTranslationStep = (float)o.value("icpMaxTranslationStep").toDouble(p.icpMaxTranslationStep);
    p.icpMaxRotationStepDeg = (float)o.value("icpMaxRotationStepDeg").toDouble(p.icpMaxRotationStepDeg);
    p.icpLostFrameLimit = o.value("icpLostFrameLimit").toInt(p.icpLostFrameLimit);
    p.icpRecoveryFrameCount = o.value("icpRecoveryFrameCount").toInt(p.icpRecoveryFrameCount);
    p.icpDepthCutoff = (float)o.value("icpDepthCutoff").toDouble(p.icpDepthCutoff);
    p.globalRecoveryEnabled = o.value("globalRecoveryEnabled").toBool(p.globalRecoveryEnabled);
    p.globalRecoveryMinFrames = o.value("globalRecoveryMinFrames").toInt(p.globalRecoveryMinFrames);
    p.globalRecoveryMinVoxelWeight = (float)o.value("globalRecoveryMinVoxelWeight").toDouble(p.globalRecoveryMinVoxelWeight);
    p.globalRecoveryYawStepDeg = (float)o.value("globalRecoveryYawStepDeg").toDouble(p.globalRecoveryYawStepDeg);
    p.globalRecoveryPitchMinDeg = (float)o.value("globalRecoveryPitchMinDeg").toDouble(p.globalRecoveryPitchMinDeg);
    p.globalRecoveryPitchMaxDeg = (float)o.value("globalRecoveryPitchMaxDeg").toDouble(p.globalRecoveryPitchMaxDeg);
    p.globalRecoveryPitchStepDeg = (float)o.value("globalRecoveryPitchStepDeg").toDouble(p.globalRecoveryPitchStepDeg);
    p.globalRecoveryRadiusOffsetsM = o.value("globalRecoveryRadiusOffsetsM").toString(QString::fromStdString(p.globalRecoveryRadiusOffsetsM)).toStdString();
    p.globalRecoveryTopCandidates = o.value("globalRecoveryTopCandidates").toInt(p.globalRecoveryTopCandidates);
    p.globalRecoveryMinInlierRatio = (float)o.value("globalRecoveryMinInlierRatio").toDouble(p.globalRecoveryMinInlierRatio);
    p.globalRecoveryMaxResidual = (float)o.value("globalRecoveryMaxResidual").toDouble(p.globalRecoveryMaxResidual);
    p.globalRecoveryCooldownMs = (float)o.value("globalRecoveryCooldownMs").toDouble(p.globalRecoveryCooldownMs);
    p.isoValue        = (float)o.value("isoValue").toDouble(p.isoValue);
    p.angleStartDeg   = (float)o.value("angleStartDeg").toDouble(p.angleStartDeg);
    p.angleEndDeg     = (float)o.value("angleEndDeg").toDouble(p.angleEndDeg);
    p.angleStepDeg    = (float)o.value("angleStepDeg").toDouble(p.angleStepDeg);
    p.stageStartMm    = (float)o.value("stageStartMm").toDouble(p.stageStartMm);
    p.stageEndMm      = (float)o.value("stageEndMm").toDouble(p.stageEndMm);
    p.stageStepMm     = (float)o.value("stageStepMm").toDouble(p.stageStepMm);
    p.framesPerPose   = o.value("framesPerPose").toInt(p.framesPerPose);
    p.targetSettleMs  = (float)o.value("targetSettleMs").toDouble(p.targetSettleMs);
    p.angleToleranceDeg = (float)o.value("angleToleranceDeg").toDouble(p.angleToleranceDeg);
    p.stageToleranceMm = (float)o.value("stageToleranceMm").toDouble(p.stageToleranceMm);
    p.targetTimeoutMs = (float)o.value("targetTimeoutMs").toDouble(p.targetTimeoutMs);
    p.turntableRadiusMm = (float)o.value("turntableRadiusMm").toDouble(p.turntableRadiusMm);
    p.turntableHeightMm = (float)o.value("turntableHeightMm").toDouble(p.turntableHeightMm);
    p.kinectOffsetXMm = (float)o.value("kinectOffsetXMm").toDouble(p.kinectOffsetXMm);
    p.kinectOffsetYMm = (float)o.value("kinectOffsetYMm").toDouble(p.kinectOffsetYMm);
    p.kinectOffsetZMm = (float)o.value("kinectOffsetZMm").toDouble(p.kinectOffsetZMm);
    p.kinectRollDeg   = (float)o.value("kinectRollDeg").toDouble(p.kinectRollDeg);
    p.kinectPitchDeg  = (float)o.value("kinectPitchDeg").toDouble(p.kinectPitchDeg);
    p.kinectYawDeg    = (float)o.value("kinectYawDeg").toDouble(p.kinectYawDeg);
    p.stageAxisX      = (float)o.value("stageAxisX").toDouble(p.stageAxisX);
    p.stageAxisY      = (float)o.value("stageAxisY").toDouble(p.stageAxisY);
    p.stageAxisZ      = (float)o.value("stageAxisZ").toDouble(p.stageAxisZ);
    p.actuatorTcpHost = o.value("actuatorTcpHost").toString(QString::fromStdString(p.actuatorTcpHost)).toStdString();
    p.actuatorTcpPort = o.value("actuatorTcpPort").toInt(p.actuatorTcpPort);
    p.simStlPath      = o.value("simStlPath").toString(QString::fromStdString(p.simStlPath)).toStdString();
    p.simBenchmarkEnabled = o.value("simBenchmarkEnabled").toBool(p.simBenchmarkEnabled);
    p.simScenario = o.value("simScenario").toString(QString::fromStdString(p.simScenario)).toStdString();
    p.simMotionPreset = o.value("simMotionPreset").toString(QString::fromStdString(p.simMotionPreset)).toStdString();
    p.simMotionPath = o.value("simMotionPath").toString(QString::fromStdString(p.simMotionPath)).toStdString();
    p.simReportPath = o.value("simReportPath").toString(QString::fromStdString(p.simReportPath)).toStdString();
    p.simDepthNoiseMm = (float)o.value("simDepthNoiseMm").toDouble(p.simDepthNoiseMm);
    p.simDropoutPercent = (float)o.value("simDropoutPercent").toDouble(p.simDropoutPercent);
    p.simRoomWidthM = (float)o.value("simRoomWidthM").toDouble(p.simRoomWidthM);
    p.simRoomHeightM = (float)o.value("simRoomHeightM").toDouble(p.simRoomHeightM);
    p.simRoomDepthM = (float)o.value("simRoomDepthM").toDouble(p.simRoomDepthM);
    p.simClutterCount = o.value("simClutterCount").toInt(p.simClutterCount);
    p.simTextureFeatures = o.value("simTextureFeatures").toBool(p.simTextureFeatures);
    p.simPoseJitterMm = (float)o.value("simPoseJitterMm").toDouble(p.simPoseJitterMm);
    p.simPoseJitterDeg = (float)o.value("simPoseJitterDeg").toDouble(p.simPoseJitterDeg);
    p.simulationEnabled = (p.poseSource == PoseSourceSimulation);
    p.actuatorTcpEnabled = (p.poseSource == PoseSourceActuatedTcp);
}

bool saveJson(const QString &path, const ScanParameters &p, QString *err) {
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (err) *err = f.errorString();
        return false;
    }
    QJsonDocument doc(toJson(p));
    f.write(doc.toJson(QJsonDocument::Indented));
    if (!f.commit()) {
        if (err) *err = f.errorString();
        return false;
    }
    return true;
}

bool loadJson(const QString &path, ScanParameters &p, QString *err) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (err) *err = f.errorString();
        return false;
    }
    QJsonParseError pe;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError) {
        if (err) *err = pe.errorString();
        return false;
    }
    if (!doc.isObject()) {
        if (err) *err = "Preset JSON root is not an object";
        return false;
    }
    fromJson(doc.object(), p);
    return true;
}

} // namespace preset_io
