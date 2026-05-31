#pragma once

#include <QObject>
#include <QImage>
#include <QMatrix4x4>
#include <QString>
#include <QMutex>
#include <memory>

#include "VolumeIntegration.cuh"
#include "actuator_tcp_server.hpp"
#include "scan_parameters.hpp"

Q_DECLARE_METATYPE(ScanParameters)

/**
 * ScannerWorker owns a VolumeIntegration and drives it on a background QThread.
 *
 * Threading model:
 *   - The worker object lives on a dedicated QThread (moved with moveToThread).
 *   - All public slots are invoked via QueuedConnection from the UI thread.
 *   - Signals are emitted from the worker thread; Qt::QueuedConnection delivers
 *     them safely to UI slots.
 *   - VolumeIntegration::stepOnce() callbacks run on the worker thread and are
 *     forwarded as signals.
 */
class ScannerWorker : public QObject {
    Q_OBJECT
public:
    explicit ScannerWorker(QObject *parent = nullptr);
    ~ScannerWorker() override;

    // Synchronous helpers, safe to call after the worker has been initialized
    // (i.e. after initialized() has fired). All return cached values without
    // touching CUDA, so they are cheap.
    ScanParameters currentParameters() const;
    bool isInitialized() const { return m_initialized; }

public slots:
    /// Construct the VolumeIntegration with the provided parameters. Emits
    /// initialized() on success or error() on failure. Must be called once
    /// after the thread starts.
    void initialize(ScanParameters params);

    /// Begin (or resume) the scanning loop. Idempotent.
    void start();

    /// Pause / unpause integration. Preview frames continue to arrive while
    /// paused, but no TSDF updates happen.
    void setPaused(bool paused);

    /// Signal the scan loop to exit at the next iteration.
    void stop();

    /// Clear the volume and reset pose. Allowed only when stopped or paused.
    void reset();

    /// Apply new parameters. Volume dim / voxel size require a full reset to
    /// take effect; all others apply on the next iteration.
    void applyParameters(ScanParameters params);

    /// Run marching cubes on the current TSDF. Emits meshReady() with the
    /// vertex/color/triangle buffers on completion.
    void extractMesh();

    /// Save the most recently extracted mesh to a PLY file.
    void saveMesh(QString path);

    /// Trigger the legacy OpenCV-based calibration flow. Long-running; blocks
    /// the worker thread until the user finishes / aborts via the OpenCV
    /// window.
    void calibrate();

signals:
    void initialized(ScanParameters effectiveParams);
    void scanStarted();
    void scanStopped();
    void pausedChanged(bool paused);
    void volumeReset();

    /// Live data delivered once per scan iteration.
    void frameReady(QImage rgb, QImage depth,
                    QImage raycastRgb, QImage raycastDepth,
                    QMatrix4x4 pose, float fps, quint64 frameIndex);

    /// Carries the back-projectable raycasted point cloud for the 3D viewer.
    /// Both arrays are owned by the receiver after the queued connection.
    void pointCloudReady(QVector<float> xyz, QVector<unsigned char> rgb,
                         QMatrix4x4 pose);

    void meshReady(QVector<float> vertices,
                   QVector<unsigned char> colors,
                   QVector<unsigned int> indices);
    void simulationMeshReady(QVector<float> vertices,
                             QVector<unsigned char> colors,
                             QVector<unsigned int> indices);

    void statusMessage(QString msg);
    void error(QString msg);

private:
    void runLoop();
    void runSimulationLoop();
    void runActuatedLoop();
    bool waitForActuatorTarget(const ScanParameters &p, float angleDeg, float stageMm,
                               ActuatorState *state);
    void emitFromBundle(const FrameBundle &fb);
    Eigen::Matrix4f poseForTarget(const ScanParameters &p, float angleDeg, float stageMm) const;

    std::unique_ptr<VolumeIntegration> m_scanner;
    ActuatorTcpServer *m_actuatorServer = nullptr;
    bool m_initialized = false;
    bool m_running = false;
    mutable QMutex m_mutex;
};
