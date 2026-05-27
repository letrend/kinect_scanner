#include "scanner_worker.hpp"

#include <QCoreApplication>
#include <QDebug>
#include <QEventLoop>
#include <QFileInfo>
#include <QThread>
#include <QTimer>
#include <QtMath>

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

} // namespace

ScannerWorker::ScannerWorker(QObject *parent) : QObject(parent) {}

ScannerWorker::~ScannerWorker() {
    if (m_scanner) m_scanner->requestStop();
}

ScanParameters ScannerWorker::currentParameters() const {
    QMutexLocker lock(&m_mutex);
    return m_scanner ? m_scanner->getParameters() : ScanParameters{};
}

void ScannerWorker::initialize(ScanParameters params) {
    fprintf(stderr, "[worker] initialize() entered on thread %p\n", (void*)QThread::currentThreadId());
    fflush(stderr);
    try {
        params.useDisplay = false;
        fprintf(stderr, "[worker] creating VolumeIntegration(%u,%u,%u,%.4f)\n",
                params.xDim, params.yDim, params.zDim, params.voxelSize);
        fflush(stderr);
        m_scanner.reset(new VolumeIntegration(params.xDim, params.yDim,
                                              params.zDim, params.voxelSize));
        fprintf(stderr, "[worker] VolumeIntegration constructed\n"); fflush(stderr);
        m_scanner->setParameters(params);
        m_scanner->setOnFrame([this](const FrameBundle &fb) {
            emitFromBundle(fb);
        });
        m_scanner->setOnStatus([this](const std::string &msg) {
            emit statusMessage(QString::fromStdString(msg));
        });
        m_initialized = true;
        emit initialized();
        emit statusMessage("Scanner initialized");
        fprintf(stderr, "[worker] initialize() done\n"); fflush(stderr);
    } catch (const std::exception &e) {
        fprintf(stderr, "[worker] initialize() exception: %s\n", e.what()); fflush(stderr);
        emit error(QString("Scanner init failed: %1").arg(e.what()));
    } catch (...) {
        fprintf(stderr, "[worker] initialize() unknown exception\n"); fflush(stderr);
        emit error("Scanner init failed: unknown error");
    }
}

void ScannerWorker::start() {
    fprintf(stderr, "[worker] start() entered on thread %p\n", (void*)QThread::currentThreadId());
    fflush(stderr);
    if (!m_scanner) {
        emit error("start() called before initialize()");
        return;
    }
    if (m_running) return;
    m_running = true;
    emit scanStarted();
    emit statusMessage("Starting scan...");

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
    // Volume dim & voxel size only take effect on reset; keep current values
    // so the live integration isn't broken.
    ScanParameters cur = m_scanner->getParameters();
    params.xDim      = cur.xDim;
    params.yDim      = cur.yDim;
    params.zDim      = cur.zDim;
    params.voxelSize = cur.voxelSize;
    params.useDisplay = false;
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
    QFileInfo fi(path);
    std::string name = fi.fileName().toStdString();
    bool ok = m_scanner->saveMesh(name);
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

void ScannerWorker::emitFromBundle(const FrameBundle &fb) {
    // 2D panels
    QImage rgb         = matBGRf32_to_rgb888(fb.rgb);
    QImage depth       = matDepthMM_to_gray8(fb.depthMM);
    QImage raycastRgb  = matBGRf32_to_rgb888(fb.raycastRGB);
    QImage raycastDep  = matDepthMM_to_gray8(fb.raycastDepth);
    QMatrix4x4 pose    = eigenToQMatrix(fb.pose);
    emit frameReady(rgb, depth, raycastRgb, raycastDep, pose, fb.fps,
                    (quint64)fb.frameIndex);

    // 3D point cloud: back-project raycast depth+RGB through Kinect intrinsics.
    if (!fb.raycastDepth.empty() && !fb.raycastRGB.empty() && m_scanner) {
        const float fx = m_scanner->kinect()->irCameraParams.fx;
        const float fy = m_scanner->kinect()->irCameraParams.fy;
        const float cx = m_scanner->kinect()->irCameraParams.cx;
        const float cy = m_scanner->kinect()->irCameraParams.cy;
        const int W = fb.raycastDepth.cols;
        const int H = fb.raycastDepth.rows;
        QVector<float> xyz;
        QVector<unsigned char> rgb8;
        // Downsample for performance: every other pixel.
        const int step = 2;
        xyz.reserve((W/step) * (H/step) * 3);
        rgb8.reserve((W/step) * (H/step) * 3);
        for (int y = 0; y < H; y += step) {
            const float *dRow = fb.raycastDepth.ptr<float>(y);
            const float *cRow = fb.raycastRGB.ptr<float>(y);
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
}
