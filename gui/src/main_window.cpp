#include "main_window.hpp"

#include "image_panel.hpp"
#include "parameter_panel.hpp"
#include "viewer3d.hpp"
#include "scanner_worker.hpp"
#include "preset_io.hpp"

#include <cstdio>

#include <QAction>
#include <QApplication>
#include <QDockWidget>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
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
    connect(m_worker, &ScannerWorker::statusMessage,
            this, &MainWindow::onStatus);
    connect(m_worker, &ScannerWorker::error,
            this, &MainWindow::onError);

    connect(m_paramPanel, &ParameterPanel::parametersChanged,
            this, &MainWindow::onParametersChanged);

    m_workerThread.start();
    QMetaObject::invokeMethod(m_worker, "initialize", Qt::QueuedConnection,
                              Q_ARG(ScanParameters, m_paramPanel->parameters()));

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
    auto *outer = new QVBoxLayout(central);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(splitter);

    // Left: 2x2 image grid.
    auto *leftWrap = new QWidget;
    auto *grid = new QGridLayout(leftWrap);
    grid->setContentsMargins(2, 2, 2, 2);
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
    splitter->addWidget(m_viewer3d);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 2);

    // Parameter dock (right).
    auto *paramDock = new QDockWidget("Parameters", this);
    m_paramPanel = new ParameterPanel(paramDock);
    paramDock->setWidget(m_paramPanel);
    addDockWidget(Qt::RightDockWidgetArea, paramDock);

    // Log dock (bottom).
    auto *logDock = new QDockWidget("Log", this);
    m_log = new QPlainTextEdit(logDock);
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(500);
    logDock->setWidget(m_log);
    addDockWidget(Qt::BottomDockWidgetArea, logDock);

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

void MainWindow::onWorkerInitialized() {
    onStatus("Worker ready.");
}

void MainWindow::onWorkerStarted() {
    setRunningState(true);
    m_lblState->setText("Scanning");
}

void MainWindow::onWorkerStopped() {
    setRunningState(false);
    m_lblState->setText("Idle");
    m_actPause->setChecked(false);
}

void MainWindow::onWorkerPausedChanged(bool paused) {
    m_lblState->setText(paused ? "Paused" : "Scanning");
}

void MainWindow::onParametersChanged(ScanParameters p, bool requiresReset) {
    QMetaObject::invokeMethod(m_worker, "applyParameters", Qt::QueuedConnection,
                              Q_ARG(ScanParameters, p));
    m_viewer3d->setVolumeBounds(p.xDim * p.voxelSize,
                                p.yDim * p.voxelSize,
                                p.zDim * p.voxelSize);
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
}

void MainWindow::onStatus(QString msg) {
    if (m_log) m_log->appendPlainText(msg);
}

void MainWindow::onError(QString msg) {
    if (m_log) m_log->appendPlainText("[ERROR] " + msg);
    QMessageBox::warning(this, "Scanner error", msg);
}
