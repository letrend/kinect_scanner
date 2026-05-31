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
    o["simDepthNoiseMm"] = p.simDepthNoiseMm;
    o["simDropoutPercent"] = p.simDropoutPercent;
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
    p.simDepthNoiseMm = (float)o.value("simDepthNoiseMm").toDouble(p.simDepthNoiseMm);
    p.simDropoutPercent = (float)o.value("simDropoutPercent").toDouble(p.simDropoutPercent);
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
