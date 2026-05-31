#include "main_window.hpp"

#include "image_panel.hpp"
#include "parameter_panel.hpp"
#include "viewer3d.hpp"
#include "scanner_worker.hpp"
#include "preset_io.hpp"
#include "tcp_control_server.hpp"

#include <cstdio>
#include <algorithm>

#include <QAction>
#include <QApplication>
#include <QDockWidget>
#include <QFileDialog>
#include <QFrame>
#include <QGridLayout>
#include <QJsonArray>
#include <QJsonObject>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QToolBar>
#include <QTcpSocket>
#include <QVBoxLayout>
#include <QWidget>

namespace {
// Mirror the near-face clamp applied in VolumeIntegration::intializeGridPosition:
// the volume's near face is pushed to >= 0.50 m in front of the camera so it
// stays outside the camera frustum widget.
void effectiveVolumeCenter(const ScanParameters &p, float &cx, float &cy, float &cz) {
    if (p.poseSource == PoseSourceSimulation || p.poseSource == PoseSourceActuatedTcp) {
        // External-pose scans define world origin as the turntable / target
        // center. Keep the TSDF volume and viewer box centered there so the
        // camera trajectory visibly orbits the scan volume.
        cx = cy = cz = 0.0f;
        return;
    }
    const float halfZ = (p.zDim * p.voxelSize) * 0.5f;
    const float minNearFaceZ = 0.50f;
    cx = p.gridInitOffsetX;
    cy = p.gridInitOffsetY;
    cz = p.gridInitOffsetZ;
    if (cz - halfZ < minNearFaceZ) cz = halfZ + minNearFaceZ;
}
} // namespace

MainWindow::MainWindow(const AppOptions &options, QWidget *parent)
    : QMainWindow(parent), m_options(options) {
    qRegisterMetaType<ScanParameters>("ScanParameters");
    qRegisterMetaType<QVector<float>>("QVector<float>");
    qRegisterMetaType<QVector<unsigned char>>("QVector<unsigned char>");
    qRegisterMetaType<QVector<unsigned int>>("QVector<unsigned int>");

    setWindowTitle("Kinect 3D Scanner");
    resize(1400, 900);

    buildUi();
    buildToolBar();

    // Worker on its own thread
    m_worker = new ScannerWorker;
    m_worker->moveToThread(&m_workerThread);
    connect(&m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);

    connect(m_worker, &ScannerWorker::initialized,
            this, &MainWindow::onWorkerInitialized);
    connect(m_worker, &ScannerWorker::scanStarted,
            this, &MainWindow::onWorkerStarted);
    connect(m_worker, &ScannerWorker::scanStopped,
            this, &MainWindow::onWorkerStopped);
    connect(m_worker, &ScannerWorker::pausedChanged,
            this, &MainWindow::onWorkerPausedChanged);
    connect(m_worker, &ScannerWorker::frameReady,
            this, &MainWindow::onFrameReady);
    connect(m_worker, &ScannerWorker::pointCloudReady,
            this, &MainWindow::onPointCloudReady);
    connect(m_worker, &ScannerWorker::meshReady,
            this, &MainWindow::onMeshReady);
    connect(m_worker, &ScannerWorker::simulationMeshReady,
            this, &MainWindow::onSimulationMeshReady);
    connect(m_worker, &ScannerWorker::statusMessage,
            this, &MainWindow::onStatus);
    connect(m_worker, &ScannerWorker::error,
            this, &MainWindow::onError);

    connect(m_paramPanel, &ParameterPanel::parametersChanged,
            this, &MainWindow::onParametersChanged);

    ScanParameters initial = m_paramPanel->parameters();
    initial.controlTcpHost = options.controlTcpHost.toStdString();
    initial.controlTcpPort = options.controlTcpPort;
    initial.actuatorTcpHost = options.actuatorTcpHost.toStdString();
    initial.actuatorTcpPort = options.actuatorTcpPort;
    if (options.simulate) {
        initial.poseSource = PoseSourceSimulation;
        initial.simulationEnabled = true;
        initial.simulationAutoStart = true;
        initial.simStlPath = options.simStlPath.toStdString();
        initial.useDisplay = false;
        m_paramPanel->setParameters(initial);
    } else if (options.actuated) {
        initial.poseSource = PoseSourceActuatedTcp;
        initial.actuatorTcpEnabled = true;
        initial.useDisplay = false;
        m_paramPanel->setParameters(initial);
    }

    m_controlServer = new TcpControlServer(this);
    connect(m_controlServer, &TcpControlServer::statusMessage,
            this, &MainWindow::onStatus);
    connect(m_controlServer, &TcpControlServer::commandReceived,
            this, &MainWindow::onTcpCommand);
    if (initial.controlTcpEnabled) {
        QString err;
        if (!m_controlServer->start(QString::fromStdString(initial.controlTcpHost),
                                    initial.controlTcpPort, &err)) {
            onStatus("UI control TCP failed: " + err);
        }
    }

    m_workerThread.start();
    QMetaObject::invokeMethod(m_worker, "initialize", Qt::QueuedConnection,
                              Q_ARG(ScanParameters, initial));

    onStatus("Ready. Press Start to begin scanning.");
}

MainWindow::~MainWindow() {
    if (m_worker) {
        QMetaObject::invokeMethod(m_worker, "stop", Qt::QueuedConnection);
    }
    m_workerThread.quit();
    m_workerThread.wait();
}

void MainWindow::buildUi() {
    auto *central = new QWidget;
    setCentralWidget(central);
    auto *splitter = new QSplitter(Qt::Horizontal, central);
    splitter->setChildrenCollapsible(true);
    auto *outer = new QVBoxLayout(central);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(splitter);

    // Left: 2x2 image grid.
    auto *leftWrap = new QWidget;
    leftWrap->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto *grid = new QGridLayout(leftWrap);
    grid->setContentsMargins(2, 2, 2, 2);
    grid->setSpacing(2);
    m_panelRgb        = new ImagePanel("RGB");
    m_panelDepth      = new ImagePanel("Depth");
    m_panelRaycastRgb = new ImagePanel("Raycast RGB");
    m_panelRaycastDep = new ImagePanel("Raycast Depth");
    grid->addWidget(m_panelRgb,        0, 0);
    grid->addWidget(m_panelDepth,      0, 1);
    grid->addWidget(m_panelRaycastRgb, 1, 0);
    grid->addWidget(m_panelRaycastDep, 1, 1);
    splitter->addWidget(leftWrap);

    // Right: 3D viewer.
    m_viewer3d = new Viewer3D;
    m_viewer3d->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    splitter->addWidget(m_viewer3d);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 2);
    splitter->setSizes({360, 840});

    // Parameter dock (right).
    auto *paramDock = new QDockWidget("Parameters", this);
    paramDock->setFeatures(QDockWidget::DockWidgetMovable |
                           QDockWidget::DockWidgetFloatable |
                           QDockWidget::DockWidgetClosable);
    paramDock->setMinimumWidth(220);
    m_paramPanel = new ParameterPanel;
    m_paramPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    auto *paramScroll = new QScrollArea(paramDock);
    paramScroll->setWidgetResizable(true);
    paramScroll->setFrameShape(QFrame::NoFrame);
    paramScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    paramScroll->setWidget(m_paramPanel);
    paramDock->setWidget(paramScroll);
    addDockWidget(Qt::RightDockWidgetArea, paramDock);
    resizeDocks({paramDock}, {320}, Qt::Horizontal);

    // Log dock (bottom).
    auto *logDock = new QDockWidget("Log", this);
    logDock->setFeatures(QDockWidget::DockWidgetMovable |
                         QDockWidget::DockWidgetFloatable |
                         QDockWidget::DockWidgetClosable);
    m_log = new QPlainTextEdit(logDock);
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(500);
    m_log->setMinimumHeight(50);
    m_log->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    logDock->setWidget(m_log);
    addDockWidget(Qt::BottomDockWidgetArea, logDock);
    resizeDocks({logDock}, {120}, Qt::Vertical);

    // Status bar widgets.
    m_lblState = new QLabel("Idle");
    m_lblFps   = new QLabel("FPS: -");
    m_lblFrame = new QLabel("Frame: 0");
    m_lblPose  = new QLabel("Pose: 0.00, 0.00, 0.00");
    statusBar()->addWidget(m_lblState);
    statusBar()->addPermanentWidget(m_lblFps);
    statusBar()->addPermanentWidget(m_lblFrame);
    statusBar()->addPermanentWidget(m_lblPose);

    // Update bounds box to match volume size.
    ScanParameters p = m_paramPanel->parameters();
    m_viewer3d->setVolumeBounds(p.xDim * p.voxelSize,
                                p.yDim * p.voxelSize,
                                p.zDim * p.voxelSize);
    float cx, cy, cz; effectiveVolumeCenter(p, cx, cy, cz);
    m_viewer3d->setVolumeCenter(cx, cy, cz);
}

void MainWindow::buildToolBar() {
    auto *tb = addToolBar("Main");
    tb->setObjectName("MainToolbar");
    auto *style = this->style();

    m_actStart = tb->addAction(style->standardIcon(QStyle::SP_MediaPlay), "Start",
                               this, &MainWindow::onStart);
    m_actPause = tb->addAction(style->standardIcon(QStyle::SP_MediaPause), "Pause");
    m_actPause->setCheckable(true);
    connect(m_actPause, &QAction::toggled, this, &MainWindow::onTogglePause);
    m_actStop  = tb->addAction(style->standardIcon(QStyle::SP_MediaStop), "Stop",
                               this, &MainWindow::onStop);
    tb->addSeparator();
    m_actReset = tb->addAction(style->standardIcon(QStyle::SP_BrowserReload), "Reset",
                               this, &MainWindow::onReset);
    m_actExtractMesh = tb->addAction(style->standardIcon(QStyle::SP_ArrowUp), "Extract Mesh",
                                     this, &MainWindow::onExtractMesh);
    m_actSaveMesh = tb->addAction(style->standardIcon(QStyle::SP_DialogSaveButton), "Save Mesh",
                                  this, &MainWindow::onSaveMesh);
    tb->addSeparator();
    m_actLoadPreset = tb->addAction(style->standardIcon(QStyle::SP_DialogOpenButton), "Load Preset",
                                    this, &MainWindow::onLoadPreset);
    m_actSavePreset = tb->addAction(style->standardIcon(QStyle::SP_DriveFDIcon), "Save Preset",
                                    this, &MainWindow::onSavePreset);
    tb->addSeparator();
    m_actCalibrate = tb->addAction(style->standardIcon(QStyle::SP_ComputerIcon), "Calibrate",
                                   this, &MainWindow::onCalibrate);

    tb->addSeparator();
    // 3D view mode toggle: Camera Locked (third-person follows the live
    // Kinect pose) vs. Free (user-controlled arcball).
    auto *actCamLock = tb->addAction(style->standardIcon(QStyle::SP_DesktopIcon), "Camera Locked");
    actCamLock->setCheckable(true);
    actCamLock->setChecked(m_viewer3d->cameraLocked());
    actCamLock->setToolTip("Camera Locked: 3D view mirrors the Kinect.\n"
                           "Uncheck for a free-view arcball camera.");
    connect(actCamLock, &QAction::toggled, this, [this, actCamLock](bool on) {
        m_viewer3d->setCameraLocked(on);
        actCamLock->setText(on ? "Camera Locked" : "Free View");
    });

    setRunningState(false);
}

void MainWindow::setRunningState(bool running) {
    m_actStart->setEnabled(!running);
    m_actStop->setEnabled(running);
    m_actPause->setEnabled(running);
    m_actReset->setEnabled(!running);
    m_actExtractMesh->setEnabled(!running);
    m_actSaveMesh->setEnabled(!running && m_meshAvailable);
    m_actCalibrate->setEnabled(!running);
    m_paramPanel->setVolumeEditable(!running);
}

void MainWindow::onStart() {
    fprintf(stderr, "[ui] Start clicked\n"); fflush(stderr);
    QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection);
}

void MainWindow::onStop() {
    QMetaObject::invokeMethod(m_worker, "stop", Qt::QueuedConnection);
}

void MainWindow::onTogglePause(bool paused) {
    QMetaObject::invokeMethod(m_worker, "setPaused", Qt::QueuedConnection,
                              Q_ARG(bool, paused));
}

void MainWindow::onReset() {
    QMetaObject::invokeMethod(m_worker, "reset", Qt::QueuedConnection);
    m_viewer3d->clearAccumulated();
    m_meshAvailable = false;
    m_actSaveMesh->setEnabled(false);
}

void MainWindow::onExtractMesh() {
    QMetaObject::invokeMethod(m_worker, "extractMesh", Qt::QueuedConnection);
}

void MainWindow::onSaveMesh() {
    QString path = QFileDialog::getSaveFileName(
        this, "Save mesh", "scan.ply", "PLY (*.ply)");
    if (path.isEmpty()) return;
    QMetaObject::invokeMethod(m_worker, "saveMesh", Qt::QueuedConnection,
                              Q_ARG(QString, path));
}

void MainWindow::onLoadPreset() {
    QString path = QFileDialog::getOpenFileName(this, "Load preset", QString(),
                                                "JSON (*.json)");
    if (path.isEmpty()) return;
    ScanParameters p = m_paramPanel->parameters();
    QString err;
    if (!preset_io::loadJson(path, p, &err)) {
        QMessageBox::warning(this, "Load preset", err);
        return;
    }
    m_paramPanel->setParameters(p);
    QMetaObject::invokeMethod(m_worker, "applyParameters", Qt::QueuedConnection,
                              Q_ARG(ScanParameters, p));
    onStatus("Preset loaded.");
}

void MainWindow::onSavePreset() {
    QString path = QFileDialog::getSaveFileName(this, "Save preset", "preset.json",
                                                "JSON (*.json)");
    if (path.isEmpty()) return;
    QString err;
    if (!preset_io::saveJson(path, m_paramPanel->parameters(), &err))
        QMessageBox::warning(this, "Save preset", err);
    else
        onStatus("Preset saved.");
}

void MainWindow::onCalibrate() {
    QMetaObject::invokeMethod(m_worker, "calibrate", Qt::QueuedConnection);
}

void MainWindow::onWorkerInitialized(ScanParameters effectiveParams) {
    onStatus("Worker ready.");
    // Reflect any auto-adjustments (e.g. GPU-memory-driven voxel sizing)
    // back into the parameter panel and viewer.
    m_paramPanel->setParameters(effectiveParams);
    m_viewer3d->setVolumeBounds(effectiveParams.xDim * effectiveParams.voxelSize,
                                effectiveParams.yDim * effectiveParams.voxelSize,
                                effectiveParams.zDim * effectiveParams.voxelSize);
    {
        float cx, cy, cz; effectiveVolumeCenter(effectiveParams, cx, cy, cz);
        m_viewer3d->setVolumeCenter(cx, cy, cz);
    }
    if (effectiveParams.simulationAutoStart)
        QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection);
}

void MainWindow::onWorkerStarted() {
    m_running = true;
    m_paused = false;
    setRunningState(true);
    m_lblState->setText("Scanning");
    if (m_controlServer) {
        QJsonObject ev; ev["type"] = "event"; ev["event"] = "scan_started";
        m_controlServer->broadcast(ev);
    }
}

void MainWindow::onWorkerStopped() {
    m_running = false;
    m_paused = false;
    setRunningState(false);
    m_lblState->setText("Idle");
    m_actPause->setChecked(false);
    if (m_controlServer) {
        QJsonObject ev; ev["type"] = "event"; ev["event"] = "scan_stopped";
        m_controlServer->broadcast(ev);
    }
}

void MainWindow::onWorkerPausedChanged(bool paused) {
    m_paused = paused;
    m_lblState->setText(paused ? "Paused" : "Scanning");
}

void MainWindow::onParametersChanged(ScanParameters p, bool requiresReset) {
    QMetaObject::invokeMethod(m_worker, "applyParameters", Qt::QueuedConnection,
                              Q_ARG(ScanParameters, p));
    m_viewer3d->setVolumeBounds(p.xDim * p.voxelSize,
                                p.yDim * p.voxelSize,
                                p.zDim * p.voxelSize);
    {
        float cx, cy, cz; effectiveVolumeCenter(p, cx, cy, cz);
        m_viewer3d->setVolumeCenter(cx, cy, cz);
    }
    if (requiresReset)
        onStatus("Volume/ICP parameter changed - press Reset to apply.");
}

void MainWindow::onFrameReady(QImage rgb, QImage depth,
                              QImage raycastRgb, QImage raycastDepth,
                              QMatrix4x4 pose, float fps, quint64 frameIndex) {
    m_panelRgb->setImage(rgb);
    m_panelDepth->setImage(depth);
    if (!raycastRgb.isNull())   m_panelRaycastRgb->setImage(raycastRgb);
    if (!raycastDepth.isNull()) m_panelRaycastDep->setImage(raycastDepth);
    m_lblFps->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
    m_lblFrame->setText(QString("Frame: %1").arg(frameIndex));
    m_lastFps = fps;
    m_lastFrame = frameIndex;
    m_lastPose = pose;
    m_lblPose->setText(QString("Pose: %1, %2, %3")
                        .arg(pose(0,3), 0, 'f', 2)
                        .arg(pose(1,3), 0, 'f', 2)
                        .arg(pose(2,3), 0, 'f', 2));
}

void MainWindow::onPointCloudReady(QVector<float> xyz,
                                   QVector<unsigned char> rgb,
                                   QMatrix4x4 pose) {
    m_viewer3d->setPointCloud(xyz, rgb, pose);
}

void MainWindow::onMeshReady(QVector<float> v,
                             QVector<unsigned char> c,
                             QVector<unsigned int> i) {
    m_viewer3d->setMesh(v, c, i);
    m_meshAvailable = true;
    m_actSaveMesh->setEnabled(true);
    if (m_controlServer) {
        QJsonObject ev; ev["type"] = "event"; ev["event"] = "mesh_ready";
        ev["meshAvailable"] = true;
        m_controlServer->broadcast(ev);
    }
}

void MainWindow::onSimulationMeshReady(QVector<float> v,
                                       QVector<unsigned char> c,
                                       QVector<unsigned int> i) {
    if (v.isEmpty() || i.isEmpty())
        m_viewer3d->clearSimulationMesh();
    else
        m_viewer3d->setSimulationMesh(v, c, i);
}

void MainWindow::onStatus(QString msg) {
    if (m_log) m_log->appendPlainText(msg);
}

void MainWindow::onError(QString msg) {
    if (m_log) m_log->appendPlainText("[ERROR] " + msg);
    if (m_controlServer) {
        QJsonObject ev; ev["type"] = "event"; ev["event"] = "error"; ev["message"] = msg;
        m_controlServer->broadcast(ev);
    }
    QMessageBox::warning(this, "Scanner error", msg);
}

QJsonObject MainWindow::statusJson() const {
    QJsonObject out;
    out["type"] = "status";
    out["state"] = m_running ? (m_paused ? "paused" : "scanning") : "idle";
    out["frame"] = (double)m_lastFrame;
    out["fps"] = m_lastFps;
    out["meshAvailable"] = m_meshAvailable;
    ScanParameters p = m_paramPanel->parameters();
    out["poseSource"] = p.poseSource == PoseSourceSimulation ? "simulation" :
                         p.poseSource == PoseSourceActuatedTcp ? "actuated_tcp" : "icp";
    QJsonArray pose;
    pose.append(m_lastPose(0,3));
    pose.append(m_lastPose(1,3));
    pose.append(m_lastPose(2,3));
    out["pose"] = pose;
    return out;
}

bool MainWindow::applyJsonParameters(const QJsonObject &params, QString *error) {
    ScanParameters p = m_paramPanel->parameters();
    auto number = [&](const char *key, float &target) {
        if (params.contains(key)) target = (float)params.value(key).toDouble(target);
    };
    if (params.contains("poseSource")) {
        QString s = params.value("poseSource").toString();
        if (s == "simulation") p.poseSource = PoseSourceSimulation;
        else if (s == "actuated_tcp") p.poseSource = PoseSourceActuatedTcp;
        else if (s == "icp") p.poseSource = PoseSourceIcp;
        else {
            if (error) *error = "poseSource must be icp, actuated_tcp, or simulation";
            return false;
        }
        p.simulationEnabled = (p.poseSource == PoseSourceSimulation);
        p.actuatorTcpEnabled = (p.poseSource == PoseSourceActuatedTcp);
    }
    if (params.contains("simStlPath")) p.simStlPath = params.value("simStlPath").toString().toStdString();
    if (params.contains("stlPath")) p.simStlPath = params.value("stlPath").toString().toStdString();
    number("angleStartDeg", p.angleStartDeg);
    number("angleEndDeg", p.angleEndDeg);
    number("angleStepDeg", p.angleStepDeg);
    number("stageStartMm", p.stageStartMm);
    number("stageEndMm", p.stageEndMm);
    number("stageStepMm", p.stageStepMm);
    if (params.contains("framesPerPose"))
        p.framesPerPose = std::max(1, params.value("framesPerPose").toInt(p.framesPerPose));
    number("targetSettleMs", p.targetSettleMs);
    number("angleToleranceDeg", p.angleToleranceDeg);
    number("stageToleranceMm", p.stageToleranceMm);
    number("targetTimeoutMs", p.targetTimeoutMs);
    number("turntableRadiusMm", p.turntableRadiusMm);
    number("turntableHeightMm", p.turntableHeightMm);
    number("kinectOffsetXMm", p.kinectOffsetXMm);
    number("kinectOffsetYMm", p.kinectOffsetYMm);
    number("kinectOffsetZMm", p.kinectOffsetZMm);
    number("kinectRollDeg", p.kinectRollDeg);
    number("kinectPitchDeg", p.kinectPitchDeg);
    number("kinectYawDeg", p.kinectYawDeg);
    number("stageAxisX", p.stageAxisX);
    number("stageAxisY", p.stageAxisY);
    number("stageAxisZ", p.stageAxisZ);
    if (params.contains("actuatorTcpHost"))
        p.actuatorTcpHost = params.value("actuatorTcpHost").toString().toStdString();
    if (params.contains("actuatorTcpPort"))
        p.actuatorTcpPort = params.value("actuatorTcpPort").toInt(p.actuatorTcpPort);
    number("simDepthNoiseMm", p.simDepthNoiseMm);
    number("simDropoutPercent", p.simDropoutPercent);
    number("maxTruncation", p.maxTruncation);
    number("depthEdgeThreshold", p.depthEdgeThreshold);
    number("normalThreshold", p.normalThreshold);
    m_paramPanel->setParameters(p);
    if (m_running) {
        QMetaObject::invokeMethod(m_worker, "applyParameters", Qt::QueuedConnection,
                                  Q_ARG(ScanParameters, p));
    } else {
        m_viewer3d->clearAccumulated();
        m_meshAvailable = false;
        m_actSaveMesh->setEnabled(false);
        QMetaObject::invokeMethod(m_worker, "initialize", Qt::QueuedConnection,
                                  Q_ARG(ScanParameters, p));
    }
    return true;
}

void MainWindow::onTcpCommand(QJsonObject command, QTcpSocket *socket) {
    QString type = command.value("type").toString();
    QJsonObject reply;
    reply["type"] = "ack";
    if (command.contains("seq")) reply["seq"] = command.value("seq");
    reply["ok"] = true;

    auto fail = [&](const QString &code, const QString &message) {
        QJsonObject err;
        err["type"] = "error";
        if (command.contains("seq")) err["seq"] = command.value("seq");
        err["code"] = code;
        err["message"] = message;
        m_controlServer->send(socket, err);
    };

    if (type == "start") {
        onStart();
        reply["message"] = "start accepted";
    } else if (type == "stop") {
        onStop();
        reply["message"] = "stop accepted";
    } else if (type == "pause") {
        bool paused = command.value("paused").toBool(true);
        m_actPause->setChecked(paused);
        onTogglePause(paused);
        reply["message"] = paused ? "pause accepted" : "resume accepted";
    } else if (type == "reset") {
        onReset();
        reply["message"] = "reset accepted";
    } else if (type == "extract_mesh") {
        onExtractMesh();
        reply["message"] = "extract accepted";
    } else if (type == "save_mesh") {
        QString path = command.value("path").toString();
        if (path.isEmpty()) { fail("missing_path", "save_mesh requires path"); return; }
        QMetaObject::invokeMethod(m_worker, "saveMesh", Qt::QueuedConnection,
                                  Q_ARG(QString, path));
        reply["message"] = "save accepted";
    } else if (type == "load_stl") {
        QString path = command.value("path").toString();
        if (path.isEmpty()) { fail("missing_path", "load_stl requires path"); return; }
        QJsonObject params;
        params["poseSource"] = "simulation";
        params["simStlPath"] = path;
        QString err;
        if (!applyJsonParameters(params, &err)) { fail("bad_params", err); return; }
        reply["message"] = "stl loaded";
    } else if (type == "set_params") {
        QJsonObject params = command.value("params").toObject();
        QString err;
        if (!applyJsonParameters(params, &err)) { fail("bad_params", err); return; }
        reply["message"] = "parameters updated";
    } else if (type == "get_status") {
        QJsonObject status = statusJson();
        if (command.contains("seq")) status["seq"] = command.value("seq");
        m_controlServer->send(socket, status);
        return;
    } else {
        fail("unknown_command", QString("Unknown command: %1").arg(type));
        return;
    }
    m_controlServer->send(socket, reply);
}
