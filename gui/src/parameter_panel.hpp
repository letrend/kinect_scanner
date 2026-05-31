#pragma once

#include <QWidget>
#include <QVector>
#include <QTimer>
#include "scan_parameters.hpp"

class QDoubleSpinBox;
class QSpinBox;
class QCheckBox;
class QGroupBox;
class QComboBox;
class QLineEdit;

/// Builds a form of widgets bound to every field of ScanParameters, debouncing
/// changes for 100 ms before emitting parametersChanged().
class ParameterPanel : public QWidget {
    Q_OBJECT
public:
    explicit ParameterPanel(QWidget *parent = nullptr);

    ScanParameters parameters() const { return m_current; }

public slots:
    /// Replace all widget values from `p` without firing parametersChanged().
    void setParameters(const ScanParameters &p);

    /// Disable widgets that require a reset (volume dims + voxel size).
    void setVolumeEditable(bool editable);

signals:
    /// Emitted (debounced) after a widget change. Volume size + voxel size
    /// changes are tagged with requiresReset=true.
    void parametersChanged(ScanParameters params, bool requiresReset);

private slots:
    void onAnyChanged();
    void onVolumeChanged();
    void flush();

private:
    QGroupBox *buildVolumeGroup();
    QGroupBox *buildPoseSourceGroup();
    QGroupBox *buildTsdfGroup();
    QGroupBox *buildBilateralGroup();
    QGroupBox *buildRaycastGroup();
    QGroupBox *buildIcpGroup();
    QGroupBox *buildMcGroup();

    ScanParameters m_current;
    QTimer m_debounce;
    bool m_volumeDirty = false;
    bool m_loading = false;

    // Pose source / simulation
    QComboBox      *m_poseSource = nullptr;
    QLineEdit      *m_simStlPath = nullptr;
    QDoubleSpinBox *m_angleStart = nullptr;
    QDoubleSpinBox *m_angleEnd = nullptr;
    QDoubleSpinBox *m_angleStep = nullptr;
    QDoubleSpinBox *m_stageStart = nullptr;
    QDoubleSpinBox *m_stageEnd = nullptr;
    QDoubleSpinBox *m_stageStep = nullptr;
    QSpinBox       *m_framesPerPose = nullptr;
    QDoubleSpinBox *m_targetSettle = nullptr;
    QDoubleSpinBox *m_angleTolerance = nullptr;
    QDoubleSpinBox *m_stageTolerance = nullptr;
    QDoubleSpinBox *m_targetTimeout = nullptr;
    QDoubleSpinBox *m_turntableRadius = nullptr;
    QDoubleSpinBox *m_turntableHeight = nullptr;
    QDoubleSpinBox *m_kinectX = nullptr;
    QDoubleSpinBox *m_kinectY = nullptr;
    QDoubleSpinBox *m_kinectZ = nullptr;
    QDoubleSpinBox *m_kinectRoll = nullptr;
    QDoubleSpinBox *m_kinectPitch = nullptr;
    QDoubleSpinBox *m_kinectYaw = nullptr;
    QDoubleSpinBox *m_stageAxisX = nullptr;
    QDoubleSpinBox *m_stageAxisY = nullptr;
    QDoubleSpinBox *m_stageAxisZ = nullptr;
    QLineEdit      *m_actuatorHost = nullptr;
    QSpinBox       *m_actuatorPort = nullptr;
    QDoubleSpinBox *m_depthNoise = nullptr;
    QDoubleSpinBox *m_dropout = nullptr;

    // Volume
    QSpinBox       *m_xDim = nullptr;
    QSpinBox       *m_yDim = nullptr;
    QSpinBox       *m_zDim = nullptr;
    QDoubleSpinBox *m_voxel = nullptr;
    QDoubleSpinBox *m_offsetX = nullptr;
    QDoubleSpinBox *m_offsetY = nullptr;
    QDoubleSpinBox *m_offsetZ = nullptr;
    // TSDF
    QDoubleSpinBox *m_maxTrunc = nullptr;
    QDoubleSpinBox *m_depthEdge = nullptr;
    // Bilateral
    QDoubleSpinBox *m_sigmaD = nullptr;
    QDoubleSpinBox *m_sigmaR = nullptr;
    // Normals
    QDoubleSpinBox *m_normalThresh = nullptr;
    // Raycast
    QDoubleSpinBox *m_rcNear = nullptr;
    QDoubleSpinBox *m_rcFar  = nullptr;
    QDoubleSpinBox *m_rcStep = nullptr;
    // ICP
    QSpinBox       *m_icp0 = nullptr;
    QSpinBox       *m_icp1 = nullptr;
    QSpinBox       *m_icp2 = nullptr;
    QDoubleSpinBox *m_icpDist = nullptr;
    QDoubleSpinBox *m_icpAngle = nullptr;
    // Marching cubes
    QDoubleSpinBox *m_iso = nullptr;
};
