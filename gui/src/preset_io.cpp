#include "preset_io.hpp"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

namespace preset_io {

static QJsonObject toJson(const ScanParameters &p) {
    QJsonObject o;
    o["xDim"] = (int)p.xDim;
    o["yDim"] = (int)p.yDim;
    o["zDim"] = (int)p.zDim;
    o["voxelSize"]       = p.voxelSize;
    o["maxTruncation"]   = p.maxTruncation;
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
    return o;
}

static void fromJson(const QJsonObject &o, ScanParameters &p) {
    p.xDim = (unsigned)o.value("xDim").toInt((int)p.xDim);
    p.yDim = (unsigned)o.value("yDim").toInt((int)p.yDim);
    p.zDim = (unsigned)o.value("zDim").toInt((int)p.zDim);
    p.voxelSize       = (float)o.value("voxelSize").toDouble(p.voxelSize);
    p.maxTruncation   = (float)o.value("maxTruncation").toDouble(p.maxTruncation);
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
