#include "scanner_worker.hpp"
#include "simulated_frame_source.hpp"

#include <QCoreApplication>
#include <QDebug>
#include <QEventLoop>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <QtMath>

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace {

/// Convert a CV_32FC3 BGR float image in [0,1] to a QImage (RGB888).
QImage matBGRf32_to_rgb888(const cv::Mat &m) {
    if (m.empty()) return QImage();
    QImage out(m.cols, m.rows, QImage::Format_RGB888);
    for (int y = 0; y < m.rows; ++y) {
        const float *src = m.ptr<float>(y);
        uchar *dst = out.scanLine(y);
        for (int x = 0; x < m.cols; ++x) {
            const float b = src[3*x + 0];
            const float g = src[3*x + 1];
            const float r = src[3*x + 2];
            dst[3*x + 0] = (uchar)qBound(0, int(r * 255.0f), 255);
            dst[3*x + 1] = (uchar)qBound(0, int(g * 255.0f), 255);
            dst[3*x + 2] = (uchar)qBound(0, int(b * 255.0f), 255);
        }
    }
    return out;
}

/// Convert a CV_32FC1 depth (mm) image to a QImage (Grayscale8) normalized
/// against the given max depth (default 4 m = 4000 mm).
QImage matDepthMM_to_gray8(const cv::Mat &m, float maxMM = 4000.0f) {
    if (m.empty()) return QImage();
    QImage out(m.cols, m.rows, QImage::Format_Grayscale8);
    const float inv = 255.0f / maxMM;
    for (int y = 0; y < m.rows; ++y) {
        const float *src = m.ptr<float>(y);
        uchar *dst = out.scanLine(y);
        for (int x = 0; x < m.cols; ++x) {
            float v = src[x];
            int g = (v > 0.0f) ? int(v * inv) : 0;
            dst[x] = (uchar)qBound(0, g, 255);
        }
    }
    return out;
}

QMatrix4x4 eigenToQMatrix(const Eigen::Matrix4f &m) {
    QMatrix4x4 q;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            q(r, c) = m(r, c);
    return q;
}

bool hasDepthSamples(const cv::Mat &depth) {
    if (depth.empty()) return false;
    const int stepY = std::max(1, depth.rows / 64);
    const int stepX = std::max(1, depth.cols / 64);
    for (int y = 0; y < depth.rows; y += stepY) {
        const float *row = depth.ptr<float>(y);
        for (int x = 0; x < depth.cols; x += stepX) {
            if (row[x] > 0.0f)
                return true;
        }
    }
    return false;
}

float wrappedAngleErrorDeg(float a, float b) {
    float d = std::fmod(a - b + 180.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return std::fabs(d - 180.0f);
}

bool targetReached(const ScanParameters &p, const ActuatorState &state,
                   float targetAngleDeg, float targetStageMm) {
    if (!state.valid || state.moving)
        return false;
    return wrappedAngleErrorDeg(state.turntableAngleDeg, targetAngleDeg) <= p.angleToleranceDeg &&
           std::fabs(state.linearStageMm - targetStageMm) <= p.stageToleranceMm;
}

Eigen::Matrix3f lookAtOriginRotation(const Eigen::Vector3f &cameraPosition) {
    Eigen::Vector3f forward = -cameraPosition;
    if (forward.norm() < 1e-6f)
        forward = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    forward.normalize();

    Eigen::Vector3f downRef(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f down = downRef - forward * downRef.dot(forward);
    if (down.norm() < 1e-6f) {
        downRef = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        down = downRef - forward * downRef.dot(forward);
    }
    down.normalize();

    Eigen::Vector3f right = down.cross(forward);
    right.normalize();
    down = forward.cross(right);
    down.normalize();

    Eigen::Matrix3f rotation;
    rotation.col(0) = right;
    rotation.col(1) = down;
    rotation.col(2) = forward;
    return rotation;
}

Eigen::Matrix4f lookAtPose(const Eigen::Vector3f &position, const Eigen::Vector3f &target) {
    Eigen::Vector3f forward = target - position;
    if (forward.norm() < 1e-6f)
        forward = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    forward.normalize();
    Eigen::Vector3f downRef(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f down = downRef - forward * downRef.dot(forward);
    if (down.norm() < 1e-6f)
        down = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    down.normalize();
    Eigen::Vector3f right = down.cross(forward).normalized();
    down = forward.cross(right).normalized();
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,1>(0,0) = right;
    pose.block<3,1>(0,1) = down;
    pose.block<3,1>(0,2) = forward;
    pose.block<3,1>(0,3) = position;
    return pose;
}

QString trackingStateName(int state) {
    switch (state) {
    case TrackingStateLost: return "lost";
    case TrackingStateRecovering: return "recovering";
    case TrackingStateRecovered: return "recovered";
    case TrackingStateTracking:
    default: return "tracking";
    }
}

QJsonArray matrixJson(const Eigen::Matrix4f &m) {
    QJsonArray arr;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            arr.append(m(r,c));
    return arr;
}

float rotationErrorDeg(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b) {
    Eigen::Matrix3f d = a.block<3,3>(0,0).transpose() * b.block<3,3>(0,0);
    const float trace = std::max(-1.0f, std::min(3.0f, d.trace()));
    const float angle = std::acos(std::max(-1.0f, std::min(1.0f, (trace - 1.0f) * 0.5f)));
    return angle * 180.0f / 3.14159265358979323846f;
}

QJsonObject benchmarkParametersJson(const ScanParameters &p) {
    QJsonObject o;
    o["simScenario"] = QString::fromStdString(p.simScenario);
    o["simMotionPreset"] = QString::fromStdString(p.simMotionPreset);
    o["simMotionPath"] = QString::fromStdString(p.simMotionPath);
    o["simRoomWidthM"] = p.simRoomWidthM;
    o["simRoomHeightM"] = p.simRoomHeightM;
    o["simRoomDepthM"] = p.simRoomDepthM;
    o["simClutterCount"] = p.simClutterCount;
    o["simTextureFeatures"] = p.simTextureFeatures;
    o["simDepthNoiseMm"] = p.simDepthNoiseMm;
    o["simDropoutPercent"] = p.simDropoutPercent;
    o["simPoseJitterMm"] = p.simPoseJitterMm;
    o["simPoseJitterDeg"] = p.simPoseJitterDeg;
    o["icpMinInlierRatio"] = p.icpMinInlierRatio;
    o["icpMaxResidual"] = p.icpMaxResidual;
    o["icpMaxTranslationStep"] = p.icpMaxTranslationStep;
    o["icpMaxRotationStepDeg"] = p.icpMaxRotationStepDeg;
    o["icpLostFrameLimit"] = p.icpLostFrameLimit;
    o["icpRecoveryFrameCount"] = p.icpRecoveryFrameCount;
    o["globalRecoveryEnabled"] = p.globalRecoveryEnabled;
    o["globalRecoveryMinFrames"] = p.globalRecoveryMinFrames;
    o["globalRecoveryMinVoxelWeight"] = p.globalRecoveryMinVoxelWeight;
    o["tsdfMaxWeight"] = p.tsdfMaxWeight;
    o["tsdfConflictDecay"] = p.tsdfConflictDecay;
    return o;
}

QVector<Eigen::Matrix4f> loadPoseScript(const QString &path) {
    QVector<Eigen::Matrix4f> poses;
    if (path.isEmpty()) return poses;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return poses;
    QJsonParseError pe;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) return poses;
    QJsonArray items = doc.object().value("poses").toArray();
    for (const QJsonValue &v : items) {
        QJsonArray a = v.toObject().value("matrix_row_major").toArray();
        if (a.size() != 16) continue;
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                m(r,c) = float(a.at(r*4+c).toDouble());
        poses.append(m);
    }
    return poses;
}

QVector<Eigen::Matrix4f> makeBenchmarkPath(const ScanParameters &p) {
    QVector<Eigen::Matrix4f> scripted = loadPoseScript(QString::fromStdString(p.simMotionPath));
    if (!scripted.isEmpty()) return scripted;
    QVector<Eigen::Matrix4f> poses;
    const float pi = 3.14159265358979323846f;
    const Eigen::Vector3f roomTarget(0.0f, 0.15f, 3.0f);
    const QString preset = QString::fromStdString(p.simMotionPreset);
    const int n = 120;
    poses.reserve(n);
    for (int i = 0; i < n; ++i) {
        float u = float(i) / float(std::max(1, n - 1));
        Eigen::Vector3f pos(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f target = roomTarget;
        if (preset == "handheld_loop") {
            float a = 2.0f * pi * u;
            pos = Eigen::Vector3f(0.35f * std::sin(a), 0.08f * std::sin(2.0f*a), 0.35f * (1.0f - std::cos(a)));
            target = Eigen::Vector3f(0.0f, 0.1f, 3.0f);
        } else if (preset == "object_orbit") {
            float a = 2.0f * pi * u;
            Eigen::Vector3f center(0.0f, 0.65f, 2.7f);
            pos = center + Eigen::Vector3f(1.0f * std::sin(a), -0.25f, -1.0f * std::cos(a));
            target = center;
        } else if (preset == "tracking_loss_stress") {
            float a = 2.0f * pi * u;
            pos = Eigen::Vector3f(0.45f * std::sin(a), 0.04f * std::sin(3.0f*a), 0.9f * u);
            target = roomTarget;
            if (i > 45 && i < 55)
                pos += Eigen::Vector3f(1.0f, -0.25f, 0.7f);
        } else {
            pos = Eigen::Vector3f(0.45f * std::sin(2.0f * pi * u),
                                  0.04f * std::sin(4.0f * pi * u),
                                  1.2f * u);
            target = roomTarget + Eigen::Vector3f(0.25f * std::sin(pi * u), 0.0f, 0.0f);
        }
        poses.append(lookAtPose(pos, target));
    }
    if (!poses.isEmpty())
        poses[0] = Eigen::Matrix4f::Identity();
    return poses;
}

Eigen::Matrix4f jitteredInitialPose(const Eigen::Matrix4f &base, const ScanParameters &p) {
    if (p.simPoseJitterMm <= 0.0f && p.simPoseJitterDeg <= 0.0f)
        return base;
    const float mm = 0.001f;
    const float deg = 3.14159265358979323846f / 180.0f;
    Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
    delta.block<3,1>(0,3) = Eigen::Vector3f(p.simPoseJitterMm * mm,
                                            -0.5f * p.simPoseJitterMm * mm,
                                            0.25f * p.simPoseJitterMm * mm);
    Eigen::Matrix3f rx = Eigen::AngleAxisf(p.simPoseJitterDeg * deg, Eigen::Vector3f::UnitX()).toRotationMatrix();
    Eigen::Matrix3f ry = Eigen::AngleAxisf(-0.5f * p.simPoseJitterDeg * deg, Eigen::Vector3f::UnitY()).toRotationMatrix();
    Eigen::Matrix3f rz = Eigen::AngleAxisf(0.35f * p.simPoseJitterDeg * deg, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    delta.block<3,3>(0,0) = rz * ry * rx;
    return base * delta;
}

bool usesSimulationSource(const ScanParameters &p) {
    return p.poseSource == PoseSourceSimulation || p.simulationEnabled || p.simBenchmarkEnabled;
}

bool usesActuatorSource(const ScanParameters &p) {
    return p.poseSource == PoseSourceActuatedTcp || p.actuatorTcpEnabled;
}

bool simulationSceneChanged(const ScanParameters &a, const ScanParameters &b) {
    return a.simScenario != b.simScenario ||
           a.simStlPath != b.simStlPath ||
           a.simStlScale != b.simStlScale ||
           a.simAutoCenter != b.simAutoCenter ||
           a.simRenderTurntable != b.simRenderTurntable ||
           a.simRoomWidthM != b.simRoomWidthM ||
           a.simRoomHeightM != b.simRoomHeightM ||
           a.simRoomDepthM != b.simRoomDepthM ||
           a.simClutterCount != b.simClutterCount ||
           a.simTextureFeatures != b.simTextureFeatures;
}

} // namespace

ScannerWorker::ScannerWorker(QObject *parent) : QObject(parent) {
    qRegisterMetaType<ActuatorState>("ActuatorState");
}

ScannerWorker::~ScannerWorker() {
    if (m_scanner) m_scanner->requestStop();
}

ScanParameters ScannerWorker::currentParameters() const {
    QMutexLocker lock(&m_mutex);
    return m_scanner ? m_scanner->getParameters() : ScanParameters{};
}

void ScannerWorker::initialize(ScanParameters params) {
    try {
        m_initialized = false;
        params.useDisplay = false;
        if (params.simBenchmarkEnabled) {
            params.simulationEnabled = true;
            if (params.simScenario.empty() || params.simScenario == "object_turntable")
                params.simScenario = "room_object";
            if (std::fabs(params.voxelSize - 0.002f) < 1e-6f) {
                params.voxelSize = 0.01f;
                emit statusMessage("Benchmark simulation: using 10 mm voxels for room-scale volume.");
            }
            if (std::fabs(params.gridInitOffsetZ - 1.0f) < 1e-6f)
                params.gridInitOffsetZ = 2.5f;
        }

        // Auto-size the voxel grid to fill ~70% of free GPU memory, but
        // only on the very first init. After that we keep whatever dims
        // the user (or the previous auto-size) chose so Reset doesn't
        // surprise the user by silently shrinking the volume because the
        // pipeline now occupies more GPU memory than it did at startup.
        if (!m_scanner) {
            size_t freeBytes = 0, totalBytes = 0;
            if (cudaMemGetInfo(&freeBytes, &totalBytes) == cudaSuccess && freeBytes > 0) {
                const double budget   = double(freeBytes) * 0.70;
                const double perVoxel = 3.0 * sizeof(float) + 3.0 * sizeof(unsigned char);
                double n = std::cbrt(budget / perVoxel);
                unsigned int dim = (unsigned int)std::floor(n / 16.0) * 16;
                if (dim < 64)   dim = 64;
                if (dim > 1024) dim = 1024;
                params.xDim = params.yDim = params.zDim = dim;
                emit statusMessage(QString("Auto-sized volume to %1^3 voxels "
                                           "(%2 MB / %3 MB free GPU)")
                                   .arg(dim)
                                   .arg(qint64(double(dim)*dim*dim*perVoxel/(1024.0*1024.0)))
                                   .arg(qint64(freeBytes/(1024*1024))));
            }
        }

        // Free the old scanner BEFORE constructing the new one so the
        // Kinect device handle is released; otherwise re-opening the
        // device in the new VolumeIntegration races against the old one
        // still holding it (which causes the next grid-init to see no
        // valid depth).
        m_scanner.reset();
        if (m_actuatorServer) {
            delete m_actuatorServer;
            m_actuatorServer = nullptr;
        }

        std::shared_ptr<FrameSource> source;
        QVector<float> simVerts;
        QVector<unsigned char> simColors;
        QVector<unsigned int> simIndices;
        if (params.poseSource == PoseSourceSimulation || params.simulationEnabled || params.simBenchmarkEnabled) {
            std::shared_ptr<SimulatedFrameSource> sim(new SimulatedFrameSource(params));
            emit statusMessage(QString("Simulation source: %1").arg(sim->sourceName()));
            sim->exportPreviewMesh(simVerts, simColors, simIndices);
            source = sim;
            params.simulationEnabled = true;
            if (!params.simBenchmarkEnabled)
                params.poseSource = PoseSourceSimulation;
            else
                params.poseSource = PoseSourceIcp;
            params.useDisplay = false;
        }
        if (params.poseSource == PoseSourceActuatedTcp) {
            params.poseSource = PoseSourceActuatedTcp;
            params.actuatorTcpEnabled = true;
            params.useDisplay = false;
            m_actuatorServer = new ActuatorTcpServer(this);
            connect(m_actuatorServer, &ActuatorTcpServer::statusMessage,
                    this, &ScannerWorker::statusMessage);
            connect(m_actuatorServer, &ActuatorTcpServer::protocolError,
                    this, &ScannerWorker::statusMessage);
            QString actuatorError;
            if (!m_actuatorServer->start(QString::fromStdString(params.actuatorTcpHost),
                                         params.actuatorTcpPort, &actuatorError)) {
                throw std::runtime_error(QString("Actuator TCP failed: %1")
                                         .arg(actuatorError).toStdString());
            }
        }
        m_scanner.reset(new VolumeIntegration(params.xDim, params.yDim,
                                              params.zDim, params.voxelSize,
                                              source));
        m_scanner->setParameters(params);
        m_scanner->setOnFrame([this](const FrameBundle &fb) {
            emitFromBundle(fb);
        });
        m_scanner->setOnStatus([this](const std::string &msg) {
            emit statusMessage(QString::fromStdString(msg));
        });
        m_initialized = true;
        emit initialized(params);
        emit simulationMeshReady(simVerts, simColors, simIndices);
        emit statusMessage("Scanner initialized");
    } catch (const std::exception &e) {
        emit error(QString("Scanner init failed: %1").arg(e.what()));
    } catch (...) {
        emit error("Scanner init failed: unknown error");
    }
}

void ScannerWorker::start() {
    if (!m_scanner) {
        emit error("start() called before initialize()");
        return;
    }
    if (m_running) return;
    m_running = true;
    emit scanStarted();
    emit statusMessage("Starting scan...");

    if (m_scanner->getParameters().simBenchmarkEnabled) {
        runSimulationBenchmarkLoop();
        m_running = false;
        emit scanStopped();
        return;
    } else if (m_scanner->getParameters().poseSource == PoseSourceSimulation) {
        runSimulationLoop();
        m_running = false;
        emit scanStopped();
        return;
    }

    if (m_scanner->getParameters().poseSource == PoseSourceActuatedTcp) {
        runActuatedLoop();
        m_running = false;
        emit scanStopped();
        return;
    }

    // First-time: position the volume centroid based on current depth.
    if (m_scanner->frameCount() == 0) {
        emit statusMessage("Waiting for Kinect frames to settle...");
        if (!m_scanner->intializeGridPosition()) {
            emit error("Could not initialize grid position (no valid depth).");
            m_running = false;
            emit scanStopped();
            return;
        }
        emit statusMessage("Grid position locked");
    }

    runLoop();

    m_running = false;
    emit scanStopped();
}

static QVector<float> makeSweep(float start, float end, float step) {
    QVector<float> values;
    if (std::fabs(step) < 1e-6f || std::fabs(end - start) < 1e-6f) {
        values.append(start);
        return values;
    }
    const bool increasing = end > start;
    if ((increasing && step < 0.0f) || (!increasing && step > 0.0f))
        step = -step;
    int guard = 0;
    for (float v = start; guard++ < 10000; v += step) {
        if (increasing) {
            if (v >= end) break;
        } else {
            if (v <= end) break;
        }
        values.append(v);
    }
    if (values.empty())
        values.append(start);
    return values;
}

void ScannerWorker::runSimulationLoop() {
    ScanParameters p = m_scanner->getParameters();
    QVector<float> stages = makeSweep(p.stageStartMm, p.stageEndMm, p.stageStepMm);
    QVector<float> angles = makeSweep(p.angleStartDeg, p.angleEndDeg, p.angleStepDeg);
    emit statusMessage(QString("Simulation scan: %1 stage(s), %2 angle(s)")
                       .arg(stages.size()).arg(angles.size()));

    Eigen::Matrix4f firstPose = poseForTarget(p, angles.front(), stages.front());
    m_scanner->setExternalPose(firstPose);
    if (m_scanner->frameCount() == 0) {
        emit statusMessage("Initializing simulated volume...");
        if (!m_scanner->intializeGridPosition()) {
            emit error("Could not initialize simulated grid position.");
            return;
        }
    }

    int framesPerPose = std::max(1, p.framesPerPose);
    for (float stage : stages) {
        for (float angle : angles) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
            if (!m_running) return;
            while (m_scanner->isPaused() && m_running) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
                QThread::msleep(20);
            }
            if (!m_running) return;
            Eigen::Matrix4f targetPose = poseForTarget(p, angle, stage);
            m_scanner->setExternalPose(targetPose);
            for (int i = 0; i < framesPerPose; ++i) {
                if (!m_scanner->stepOnce()) {
                    emit error("Simulation frame integration failed.");
                    return;
                }
            }
        }
    }
    emit statusMessage("Simulation scan complete.");
}

void ScannerWorker::runSimulationBenchmarkLoop() {
    ScanParameters p = m_scanner->getParameters();
    QVector<Eigen::Matrix4f> poses = makeBenchmarkPath(p);
    if (poses.isEmpty()) {
        emit error("Simulation benchmark has no poses.");
        return;
    }
    emit statusMessage(QString("Simulation benchmark: %1 poses, scenario=%2, motion=%3")
                       .arg(poses.size())
                       .arg(QString::fromStdString(p.simScenario))
                       .arg(QString::fromStdString(p.simMotionPreset)));

    m_scanner->setBenchmarkGroundTruth(poses.front(), 0);
    m_scanner->setTrackingPose(jitteredInitialPose(poses.front(), p));
    if (m_scanner->frameCount() == 0) {
        emit statusMessage("Initializing benchmark volume...");
        if (!m_scanner->intializeGridPosition()) {
            emit error("Could not initialize benchmark grid position.");
            return;
        }
    }

    QJsonObject report;
    report["scenario"] = QString::fromStdString(p.simScenario);
    report["motion"] = QString::fromStdString(p.simMotionPreset);
    report["parameters"] = benchmarkParametersJson(p);
    QJsonArray frames;
    double ateSum2 = 0.0;
    double rotSum2 = 0.0;
    double rpeSum2 = 0.0;
    double rpeRotSum2 = 0.0;
    double depthMaeSum = 0.0;
    double depthCompletenessSum = 0.0;
    int metricFrames = 0;
    int rpeFrames = 0;
    int lostFrames = 0;
    int recoveries = 0;
    int lastState = TrackingStateTracking;
    Eigen::Matrix4f prevEstimated = Eigen::Matrix4f::Identity();

    for (int i = 0; i < poses.size(); ++i) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        if (!m_running) break;
        while (m_scanner->isPaused() && m_running) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            QThread::msleep(20);
        }
        if (!m_running) break;
        m_scanner->setBenchmarkGroundTruth(poses[i], i);
        if (!m_scanner->stepOnce()) {
            emit error("Simulation benchmark frame integration failed.");
            break;
        }
        TrackingDiagnostics td = m_scanner->trackingDiagnostics();
        Eigen::Matrix4f estimated = m_scanner->currentPose();
        QJsonObject jf;
        jf["index"] = i;
        jf["gt_pose"] = matrixJson(poses[i]);
        jf["estimated_pose"] = matrixJson(estimated);
        jf["ate_m"] = td.poseErrorM;
        jf["ate_rot_deg"] = td.poseErrorRotDeg;
        float rpeM = 0.0f;
        float rpeRotDeg = 0.0f;
        if (i > 0) {
            Eigen::Matrix4f gtRel = poses[i - 1].inverse() * poses[i];
            Eigen::Matrix4f estRel = prevEstimated.inverse() * estimated;
            rpeM = (gtRel.block<3,1>(0,3) - estRel.block<3,1>(0,3)).norm();
            rpeRotDeg = rotationErrorDeg(gtRel, estRel);
            rpeSum2 += double(rpeM) * double(rpeM);
            rpeRotSum2 += double(rpeRotDeg) * double(rpeRotDeg);
            ++rpeFrames;
        }
        prevEstimated = estimated;
        jf["rpe_m"] = rpeM;
        jf["rpe_rot_deg"] = rpeRotDeg;
        jf["icp_residual_m"] = td.icpResidual;
        jf["icp_inlier_ratio"] = td.icpInlierRatio;
        jf["tracking_state"] = trackingStateName(td.state);
        jf["depth_mae_m"] = m_lastDepthMaeM;
        jf["depth_completeness"] = m_lastDepthCompleteness;
        jf["rejection_reason"] = QString::fromStdString(td.rejectionReason);
        frames.append(jf);
        ateSum2 += double(td.poseErrorM) * double(td.poseErrorM);
        rotSum2 += double(td.poseErrorRotDeg) * double(td.poseErrorRotDeg);
        depthMaeSum += m_lastDepthMaeM;
        depthCompletenessSum += m_lastDepthCompleteness;
        ++metricFrames;
        if (td.state == TrackingStateLost || td.state == TrackingStateRecovering)
            ++lostFrames;
        if (td.state == TrackingStateRecovered && lastState != TrackingStateRecovered)
            ++recoveries;
        lastState = td.state;
    }
    m_scanner->clearBenchmarkGroundTruth();
    report["frames"] = frames;
    QJsonObject summary;
    const double denom = std::max(1, metricFrames);
    summary["ate_rmse_m"] = std::sqrt(ateSum2 / denom);
    summary["ate_rot_rmse_deg"] = std::sqrt(rotSum2 / denom);
    const double rpeDenom = std::max(1, rpeFrames);
    summary["rpe_rmse_m"] = std::sqrt(rpeSum2 / rpeDenom);
    summary["rpe_rot_rmse_deg"] = std::sqrt(rpeRotSum2 / rpeDenom);
    summary["tracking_lost_frames"] = lostFrames;
    summary["recovery_count"] = recoveries;
    summary["depth_mae_m"] = depthMaeSum / denom;
    summary["depth_completeness"] = depthCompletenessSum / denom;
    summary["mesh_chamfer_m"] = 0.0;
    report["summary"] = summary;

    QString path = QString::fromStdString(p.simReportPath.empty() ? std::string("simulation_report.json") : p.simReportPath);
    QFile f(path);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        f.write(QJsonDocument(report).toJson(QJsonDocument::Indented));
        emit statusMessage(QString("Simulation benchmark report saved: %1").arg(path));
    } else {
        emit error(QString("Could not write simulation benchmark report: %1").arg(f.errorString()));
    }
    emit statusMessage("Simulation benchmark complete.");
}

bool ScannerWorker::waitForActuatorTarget(const ScanParameters &p, float angleDeg,
                                          float stageMm, ActuatorState *state) {
    if (!m_actuatorServer)
        return false;

    QElapsedTimer totalTimer;
    totalTimer.start();
    QElapsedTimer settleTimer;
    bool settling = false;
    const qint64 timeoutMs = std::max<qint64>(1, (qint64)p.targetTimeoutMs);
    const qint64 settleMs = std::max<qint64>(0, (qint64)p.targetSettleMs);

    while (m_running && totalTimer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        const ActuatorState current = m_actuatorServer->state();
        if (targetReached(p, current, angleDeg, stageMm)) {
            if (!settling) {
                settling = true;
                settleTimer.start();
            }
            if (settleTimer.elapsed() >= settleMs) {
                if (state) *state = current;
                return true;
            }
        } else {
            settling = false;
        }
        QThread::msleep(10);
    }
    return false;
}

void ScannerWorker::runActuatedLoop() {
    if (!m_actuatorServer || !m_actuatorServer->isListening()) {
        emit error("Actuated TCP mode selected, but the actuator TCP server is not listening.");
        return;
    }

    ScanParameters p = m_scanner->getParameters();
    QVector<float> stages = makeSweep(p.stageStartMm, p.stageEndMm, p.stageStepMm);
    QVector<float> angles = makeSweep(p.angleStartDeg, p.angleEndDeg, p.angleStepDeg);
    emit statusMessage(QString("Actuated TCP scan: %1 stage(s), %2 angle(s)")
                       .arg(stages.size()).arg(angles.size()));

    const qint64 timeoutMs = std::max<qint64>(1, (qint64)p.targetTimeoutMs);
    if (!m_actuatorServer->hasClients()) {
        emit statusMessage("Waiting for actuator TCP client...");
        QElapsedTimer clientTimer;
        clientTimer.start();
        while (m_running && !m_actuatorServer->hasClients() &&
               clientTimer.elapsed() < timeoutMs) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            QThread::msleep(20);
        }
        if (!m_actuatorServer->hasClients()) {
            emit error("No actuator TCP client connected before timeout.");
            return;
        }
    }

    int framesPerPose = std::max(1, p.framesPerPose);
    for (float stage : stages) {
        for (float angle : angles) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
            if (!m_running) return;
            while (m_scanner->isPaused() && m_running) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
                QThread::msleep(20);
            }
            if (!m_running) return;

            m_actuatorServer->sendTarget(angle, stage);
            ActuatorState reachedState;
            if (!waitForActuatorTarget(p, angle, stage, &reachedState)) {
                emit error(QString("Actuator target timed out: angle %1 deg, stage %2 mm")
                           .arg(angle, 0, 'f', 2).arg(stage, 0, 'f', 2));
                return;
            }

            m_scanner->setExternalPose(
                poseForTarget(p, reachedState.turntableAngleDeg, reachedState.linearStageMm));

            if (m_scanner->frameCount() == 0) {
                emit statusMessage("Initializing actuated volume...");
                if (!m_scanner->intializeGridPosition()) {
                    emit error("Could not initialize actuated grid position (no valid depth).");
                    return;
                }
                emit statusMessage("Grid position locked");
            }

            for (int i = 0; i < framesPerPose; ++i) {
                if (!m_running) return;
                while (m_scanner->isPaused() && m_running) {
                    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
                    QThread::msleep(20);
                }
                ActuatorState current = m_actuatorServer->state();
                if (current.valid) {
                    m_scanner->setExternalPose(
                        poseForTarget(p, current.turntableAngleDeg, current.linearStageMm));
                }
                if (!m_scanner->stepOnce()) {
                    emit error("Actuated frame integration failed.");
                    return;
                }
            }
        }
    }
    emit statusMessage("Actuated TCP scan complete.");
}

void ScannerWorker::runLoop() {
    // Run until stop() is called. Each stepOnce() updates GUI via callback.
    // We yield via QCoreApplication::processEvents-equivalent by posting back
    // to ourselves so queued slots (stop/pause/applyParameters) are handled.
    while (m_running && m_scanner) {
        // Process any queued slot invocations (stop, pause toggle, etc.).
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        if (!m_running) break;
        if (!m_scanner->stepOnce()) {
            // pause -> sleep briefly; capture failure -> stop
            if (m_scanner->isPaused()) {
                QThread::msleep(20);
                continue;
            }
            emit error("Kinect frame capture failed; stopping scan.");
            break;
        }
    }
}

void ScannerWorker::setPaused(bool paused) {
    if (!m_scanner) return;
    m_scanner->setPaused(paused);
    emit pausedChanged(paused);
    emit statusMessage(paused ? "Paused" : "Resumed");
}

void ScannerWorker::stop() {
    m_running = false;
    if (m_scanner) m_scanner->requestStop();
    if (m_actuatorServer) m_actuatorServer->sendStop();
}

void ScannerWorker::reset() {
    if (!m_scanner) return;
    if (m_running) {
        emit error("Stop the scan before resetting.");
        return;
    }
    // If volume dims / voxel size changed, re-create the scanner instead.
    ScanParameters p = m_scanner->getParameters();
    if (p.xDim != m_scanner->volumeWidth() ||
        p.yDim != m_scanner->volumeHeight() ||
        p.zDim != m_scanner->volumeDepth() ||
        std::abs(p.voxelSize - m_scanner->volumeVoxelSize()) > 1e-6f) {
        emit statusMessage("Rebuilding volume...");
        m_initialized = false;
        initialize(p);
    } else {
        m_scanner->reset();
    }
    emit volumeReset();
    emit statusMessage("Volume reset");
}

void ScannerWorker::applyParameters(ScanParameters params) {
    if (!m_scanner) return;
    ScanParameters cur = m_scanner->getParameters();
    params.useDisplay = false;

    const bool poseSourceChanged =
        params.poseSource != cur.poseSource ||
        params.simulationEnabled != cur.simulationEnabled ||
        params.simBenchmarkEnabled != cur.simBenchmarkEnabled ||
        params.actuatorTcpEnabled != cur.actuatorTcpEnabled;
    const bool simulationActive = usesSimulationSource(params) || usesSimulationSource(cur);
    const bool actuatorActive = usesActuatorSource(params) || usesActuatorSource(cur);
    const bool sourceChanged =
        poseSourceChanged ||
        (simulationActive && simulationSceneChanged(params, cur)) ||
        (actuatorActive &&
            (params.actuatorTcpHost != cur.actuatorTcpHost ||
             params.actuatorTcpPort != cur.actuatorTcpPort));
    if (sourceChanged) {
        if (m_running) {
            emit error("Stop the scan before changing pose source, simulation scene, or actuator TCP endpoint.");
            return;
        }
        initialize(params);
        return;
    }

    m_scanner->setParameters(params);
}

void ScannerWorker::extractMesh() {
    if (!m_scanner) return;
    if (m_running) {
        emit error("Stop the scan before extracting the mesh.");
        return;
    }
    emit statusMessage("Extracting mesh...");
    try {
        m_scanner->extractMesh();
    } catch (const std::exception &e) {
        emit error(QString("Mesh extraction failed: %1").arg(e.what()));
        return;
    }
    const MarchingCubes *mc = m_scanner->mesh();
    if (!mc) {
        emit error("Mesh extraction produced no output.");
        return;
    }
    QVector<float> verts;
    QVector<unsigned char> cols;
    QVector<unsigned int> idx;
    verts.reserve(int(mc->m_vertices.size()) * 3);
    cols.reserve(int(mc->m_colors.size()) * 3);
    idx.reserve(int(mc->m_faces.size()) * 3);
    for (const auto &v : mc->m_vertices) {
        verts.append(float(v.x()));
        verts.append(float(v.y()));
        verts.append(float(v.z()));
    }
    for (const auto &c : mc->m_colors) {
        cols.append(c[0]);
        cols.append(c[1]);
        cols.append(c[2]);
    }
    for (const auto &f : mc->m_faces) {
        idx.append((unsigned int)f.x());
        idx.append((unsigned int)f.y());
        idx.append((unsigned int)f.z());
    }
    emit meshReady(verts, cols, idx);
    emit statusMessage(QString("Mesh extracted: %1 verts, %2 tris")
                        .arg(verts.size()/3).arg(idx.size()/3));
}

void ScannerWorker::saveMesh(QString path) {
    if (!m_scanner || !m_scanner->mesh()) {
        emit error("Extract the mesh first.");
        return;
    }
    // Pass the full path through; VolumeIntegration::saveMesh honors
    // absolute paths and treats relative names as dataFolder-relative.
    bool ok = m_scanner->saveMesh(path.toStdString());
    emit statusMessage(ok ? QString("Saved mesh to %1").arg(path)
                          : QString("Failed to save mesh to %1").arg(path));
}

void ScannerWorker::calibrate() {
    if (!m_scanner) return;
    if (m_running) {
        emit error("Stop the scan before calibrating.");
        return;
    }
    // Legacy calibrate() uses cv::imshow + cv::waitKey: temporarily allow
    // OpenCV windows for the calibration session only.
    ScanParameters p = m_scanner->getParameters();
    bool wasDisplay = p.useDisplay;
    p.useDisplay = true;
    m_scanner->setParameters(p);
    emit statusMessage("Calibration started (use the OpenCV window).");
    bool ok = m_scanner->calibrate();
    p.useDisplay = wasDisplay;
    m_scanner->setParameters(p);
    emit statusMessage(ok ? "Calibration finished" : "Calibration failed");
}

void ScannerWorker::recoverPose() {
    if (!m_scanner) return;
    const bool ok = m_scanner->forceGlobalRecovery();
    TrackingDiagnostics td = m_scanner->trackingDiagnostics();
    emit trackingInfo(trackingStateName(td.state),
                      td.icpResidual,
                      td.icpInlierRatio,
                      QString::fromStdString(td.rejectionReason),
                      td.recoveryAttempts,
                      td.bestRecoveryScore,
                      td.poseErrorM,
                      td.poseErrorRotDeg,
                      td.benchmarkFrame);
    emit statusMessage(ok ? "Manual pose recovery accepted."
                          : "Manual pose recovery did not find a valid pose.");
}

Eigen::Matrix4f ScannerWorker::poseForTarget(const ScanParameters &p, float angleDeg, float stageMm) const {
    const float mm = 0.001f;
    Eigen::Vector3f t(p.kinectOffsetXMm * mm,
                      p.kinectOffsetYMm * mm,
                      p.kinectOffsetZMm * mm);
    Eigen::Vector3f axis(p.stageAxisX, p.stageAxisY, p.stageAxisZ);
    if (axis.norm() < 1e-6f)
        axis = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
    axis.normalize();
    t += axis * (stageMm * mm);

    const float pi = 3.14159265358979323846f;
    const float roll = p.kinectRollDeg * pi / 180.0f;
    const float pitch = p.kinectPitchDeg * pi / 180.0f;
    const float yaw = p.kinectYawDeg * pi / 180.0f;
    Eigen::Matrix3f rx = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()).toRotationMatrix();
    Eigen::Matrix3f ry = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()).toRotationMatrix();
    Eigen::Matrix3f rz = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    Eigen::Matrix4f mount = Eigen::Matrix4f::Identity();
    const Eigen::Matrix3f correction = rz * ry * rx;
    mount.block<3,3>(0,0) = lookAtOriginRotation(t) * correction;
    mount.block<3,1>(0,3) = t;

    const float theta = -angleDeg * pi / 180.0f;
    Eigen::Matrix4f turntableInv = Eigen::Matrix4f::Identity();
    turntableInv.block<3,3>(0,0) =
        Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()).toRotationMatrix();
    return turntableInv * mount;
}

void ScannerWorker::emitFromBundle(const FrameBundle &fb) {
    m_lastDepthMaeM = 0.0f;
    m_lastDepthCompleteness = 0.0f;
    if (!fb.depthMM.empty() && !fb.raycastDepth.empty()) {
        int liveCount = 0;
        int overlap = 0;
        double absSum = 0.0;
        for (int y = 0; y < fb.depthMM.rows; ++y) {
            const float *live = fb.depthMM.ptr<float>(y);
            const float *model = fb.raycastDepth.ptr<float>(y);
            for (int x = 0; x < fb.depthMM.cols; ++x) {
                if (live[x] <= 0.0f) continue;
                ++liveCount;
                if (model[x] <= 0.0f) continue;
                ++overlap;
                absSum += std::fabs(double(model[x] - live[x])) * 0.001;
            }
        }
        if (overlap > 0)
            m_lastDepthMaeM = float(absSum / double(overlap));
        if (liveCount > 0)
            m_lastDepthCompleteness = float(overlap) / float(liveCount);
    }

    // 2D panels
    QImage rgb         = matBGRf32_to_rgb888(fb.rgb);
    QImage depth       = matDepthMM_to_gray8(fb.depthMM);
    QImage raycastRgb  = matBGRf32_to_rgb888(fb.raycastRGB);
    QImage raycastDep  = matDepthMM_to_gray8(fb.raycastDepth);
    QMatrix4x4 pose    = eigenToQMatrix(fb.pose);
    emit frameReady(rgb, depth, raycastRgb, raycastDep, pose, fb.fps,
                    (quint64)fb.frameIndex);

    // 3D point cloud: prefer the TSDF raycast, but fall back to live depth so
    // simulation shows the object immediately even before the model raycast has
    // enough integrated data.
    const cv::Mat *cloudDepth = &fb.raycastDepth;
    const cv::Mat *cloudRgb = &fb.raycastRGB;
    if (!hasDepthSamples(*cloudDepth) || cloudRgb->empty()) {
        cloudDepth = &fb.depthMM;
        cloudRgb = &fb.rgb;
    }
    if (!cloudDepth->empty() && !cloudRgb->empty() && m_scanner) {
        const CameraIntrinsics ci = m_scanner->cameraIntrinsics();
        const float fx = ci.fx;
        const float fy = ci.fy;
        const float cx = ci.cx;
        const float cy = ci.cy;
        const int W = cloudDepth->cols;
        const int H = cloudDepth->rows;
        QVector<float> xyz;
        QVector<unsigned char> rgb8;
        // Downsample for performance: every other pixel.
        const int step = 2;
        xyz.reserve((W/step) * (H/step) * 3);
        rgb8.reserve((W/step) * (H/step) * 3);
        for (int y = 0; y < H; y += step) {
            const float *dRow = cloudDepth->ptr<float>(y);
            const float *cRow = cloudRgb->ptr<float>(y);
            for (int x = 0; x < W; x += step) {
                float zmm = dRow[x];
                if (zmm <= 0.0f) continue;
                float z = zmm * 0.001f;
                float X = (x - cx) * z / fx;
                float Y = (y - cy) * z / fy;
                xyz.append(X); xyz.append(Y); xyz.append(z);
                float bC = cRow[3*x + 0];
                float gC = cRow[3*x + 1];
                float rC = cRow[3*x + 2];
                rgb8.append((uchar)qBound(0, int(rC*255.0f), 255));
                rgb8.append((uchar)qBound(0, int(gC*255.0f), 255));
                rgb8.append((uchar)qBound(0, int(bC*255.0f), 255));
            }
        }
        emit pointCloudReady(xyz, rgb8, pose);
    }
    emit trackingInfo(trackingStateName(fb.tracking.state),
                      fb.tracking.icpResidual,
                      fb.tracking.icpInlierRatio,
                      QString::fromStdString(fb.tracking.rejectionReason),
                      fb.tracking.recoveryAttempts,
                      fb.tracking.bestRecoveryScore,
                      fb.tracking.poseErrorM,
                      fb.tracking.poseErrorRotDeg,
                      fb.tracking.benchmarkFrame);
}
