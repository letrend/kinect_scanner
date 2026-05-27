#pragma once

#include <QMainWindow>
#include <QThread>
#include <QImage>
#include <QMatrix4x4>
#include "scan_parameters.hpp"

class ImagePanel;
class ParameterPanel;
class Viewer3D;
class ScannerWorker;
class QPlainTextEdit;
class QLabel;
class QAction;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onStart();
    void onStop();
    void onTogglePause(bool paused);
    void onReset();
    void onExtractMesh();
    void onSaveMesh();
    void onLoadPreset();
    void onSavePreset();
    void onCalibrate();

    void onWorkerInitialized();
    void onWorkerStarted();
    void onWorkerStopped();
    void onWorkerPausedChanged(bool paused);
    void onParametersChanged(ScanParameters p, bool requiresReset);

    void onFrameReady(QImage rgb, QImage depth,
                      QImage raycastRgb, QImage raycastDepth,
                      QMatrix4x4 pose, float fps, quint64 frameIndex);
    void onPointCloudReady(QVector<float> xyz,
                           QVector<unsigned char> rgb,
                           QMatrix4x4 pose);
    void onMeshReady(QVector<float> v,
                     QVector<unsigned char> c,
                     QVector<unsigned int> i);
    void onStatus(QString msg);
    void onError(QString msg);

private:
    void buildUi();
    void buildToolBar();
    void setRunningState(bool running);

    // Widgets
    ImagePanel    *m_panelRgb        = nullptr;
    ImagePanel    *m_panelDepth      = nullptr;
    ImagePanel    *m_panelRaycastRgb = nullptr;
    ImagePanel    *m_panelRaycastDep = nullptr;
    Viewer3D      *m_viewer3d        = nullptr;
    ParameterPanel*m_paramPanel      = nullptr;
    QPlainTextEdit*m_log             = nullptr;

    QLabel *m_lblFps    = nullptr;
    QLabel *m_lblFrame  = nullptr;
    QLabel *m_lblPose   = nullptr;
    QLabel *m_lblState  = nullptr;

    QAction *m_actStart = nullptr;
    QAction *m_actPause = nullptr;
    QAction *m_actStop  = nullptr;
    QAction *m_actReset = nullptr;
    QAction *m_actExtractMesh = nullptr;
    QAction *m_actSaveMesh    = nullptr;
    QAction *m_actLoadPreset  = nullptr;
    QAction *m_actSavePreset  = nullptr;
    QAction *m_actCalibrate   = nullptr;

    // Worker
    QThread        m_workerThread;
    ScannerWorker *m_worker = nullptr;
    bool m_meshAvailable = false;
};
