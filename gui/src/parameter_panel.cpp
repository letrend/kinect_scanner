#include "parameter_panel.hpp"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>

ParameterPanel::ParameterPanel(QWidget *parent) : QWidget(parent) {
    m_debounce.setSingleShot(true);
    m_debounce.setInterval(100);
    connect(&m_debounce, &QTimer::timeout, this, &ParameterPanel::flush);

    auto *layout = new QVBoxLayout(this);
    layout->addWidget(buildPoseSourceGroup());
    layout->addWidget(buildVolumeGroup());
    layout->addWidget(buildTsdfGroup());
    layout->addWidget(buildBilateralGroup());
    layout->addWidget(buildRaycastGroup());
    layout->addWidget(buildIcpGroup());
    layout->addWidget(buildMcGroup());
    layout->addStretch();

    // Initialize widgets from defaults.
    setParameters(ScanParameters{});
}

QGroupBox *ParameterPanel::buildPoseSourceGroup() {
    auto *g = new QGroupBox("Pose source / simulation");
    auto *f = new QFormLayout(g);

    m_poseSource = new QComboBox;
    m_poseSource->addItem("ICP", PoseSourceIcp);
    m_poseSource->addItem("Actuated TCP", PoseSourceActuatedTcp);
    m_poseSource->addItem("Simulation", PoseSourceSimulation);
    m_simStlPath = new QLineEdit;
    m_simStlPath->setPlaceholderText("path/to/object.stl");

    auto makeDeg = [] {
        auto *w = new QDoubleSpinBox;
        w->setRange(-36000.0, 36000.0);
        w->setDecimals(2);
        w->setSingleStep(5.0);
        w->setSuffix(" deg");
        return w;
    };
    auto makeMm = [] {
        auto *w = new QDoubleSpinBox;
        w->setRange(-10000.0, 10000.0);
        w->setDecimals(2);
        w->setSingleStep(10.0);
        w->setSuffix(" mm");
        return w;
    };

    m_angleStart = makeDeg();
    m_angleEnd = makeDeg();
    m_angleStep = makeDeg();
    m_stageStart = makeMm();
    m_stageEnd = makeMm();
    m_stageStep = makeMm();
    m_framesPerPose = new QSpinBox;
    m_framesPerPose->setRange(1, 100);
    m_targetSettle = new QDoubleSpinBox;
    m_targetSettle->setRange(0.0, 60000.0);
    m_targetSettle->setDecimals(0);
    m_targetSettle->setSingleStep(50.0);
    m_targetSettle->setSuffix(" ms");
    m_angleTolerance = makeDeg();
    m_angleTolerance->setRange(0.0, 180.0);
    m_stageTolerance = makeMm();
    m_stageTolerance->setRange(0.0, 1000.0);
    m_targetTimeout = new QDoubleSpinBox;
    m_targetTimeout->setRange(1.0, 600000.0);
    m_targetTimeout->setDecimals(0);
    m_targetTimeout->setSingleStep(1000.0);
    m_targetTimeout->setSuffix(" ms");
    m_turntableRadius = makeMm();
    m_turntableRadius->setRange(1.0, 5000.0);
    m_turntableHeight = makeMm();
    m_turntableHeight->setRange(0.0, 5000.0);
    m_kinectX = makeMm();
    m_kinectY = makeMm();
    m_kinectZ = makeMm();
    m_kinectRoll = makeDeg();
    m_kinectPitch = makeDeg();
    m_kinectYaw = makeDeg();
    auto makeAxis = [] {
        auto *w = new QDoubleSpinBox;
        w->setRange(-1.0, 1.0);
        w->setDecimals(3);
        w->setSingleStep(0.1);
        return w;
    };
    m_stageAxisX = makeAxis();
    m_stageAxisY = makeAxis();
    m_stageAxisZ = makeAxis();
    m_actuatorHost = new QLineEdit;
    m_actuatorHost->setPlaceholderText("0.0.0.0");
    m_actuatorPort = new QSpinBox;
    m_actuatorPort->setRange(1, 65535);
    m_depthNoise = makeMm();
    m_depthNoise->setRange(0.0, 100.0);
    m_dropout = new QDoubleSpinBox;
    m_dropout->setRange(0.0, 100.0);
    m_dropout->setDecimals(2);
    m_dropout->setSingleStep(1.0);
    m_dropout->setSuffix(" %");

    f->addRow("Mode", m_poseSource);
    f->addRow("STL path", m_simStlPath);
    f->addRow("Angle start", m_angleStart);
    f->addRow("Angle end", m_angleEnd);
    f->addRow("Angle step", m_angleStep);
    f->addRow("Stage start", m_stageStart);
    f->addRow("Stage end", m_stageEnd);
    f->addRow("Stage step", m_stageStep);
    f->addRow("Frames / pose", m_framesPerPose);
    f->addRow("Settle", m_targetSettle);
    f->addRow("Angle tolerance", m_angleTolerance);
    f->addRow("Stage tolerance", m_stageTolerance);
    f->addRow("Target timeout", m_targetTimeout);
    f->addRow("Turntable radius", m_turntableRadius);
    f->addRow("Turntable height", m_turntableHeight);
    f->addRow("Kinect X", m_kinectX);
    f->addRow("Kinect Y", m_kinectY);
    f->addRow("Kinect Z", m_kinectZ);
    f->addRow("Kinect roll", m_kinectRoll);
    f->addRow("Kinect pitch", m_kinectPitch);
    f->addRow("Kinect yaw", m_kinectYaw);
    f->addRow("Stage axis X", m_stageAxisX);
    f->addRow("Stage axis Y", m_stageAxisY);
    f->addRow("Stage axis Z", m_stageAxisZ);
    f->addRow("Actuator host", m_actuatorHost);
    f->addRow("Actuator port", m_actuatorPort);
    f->addRow("Depth noise", m_depthNoise);
    f->addRow("Dropout", m_dropout);

    connect(m_poseSource, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ParameterPanel::onVolumeChanged);
    connect(m_simStlPath, &QLineEdit::textChanged, this, &ParameterPanel::onVolumeChanged);
    connect(m_actuatorHost, &QLineEdit::textChanged, this, &ParameterPanel::onVolumeChanged);
    for (auto *w : { m_angleStart, m_angleEnd, m_angleStep, m_stageStart,
                     m_stageEnd, m_stageStep, m_turntableRadius, m_turntableHeight,
                     m_kinectX, m_kinectY, m_kinectZ, m_kinectRoll, m_kinectPitch,
                     m_kinectYaw, m_stageAxisX, m_stageAxisY, m_stageAxisZ,
                     m_targetSettle, m_angleTolerance, m_stageTolerance,
                     m_targetTimeout, m_depthNoise, m_dropout }) {
        connect(w, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &ParameterPanel::onVolumeChanged);
    }
    connect(m_framesPerPose, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ParameterPanel::onVolumeChanged);
    connect(m_actuatorPort, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ParameterPanel::onVolumeChanged);
    return g;
}

QGroupBox *ParameterPanel::buildVolumeGroup() {
    auto *g = new QGroupBox("Volume (requires Reset)");
    auto *f = new QFormLayout(g);

    m_xDim = new QSpinBox; m_xDim->setRange(1, 2000); m_xDim->setSuffix(" vx");
    m_yDim = new QSpinBox; m_yDim->setRange(1, 2000); m_yDim->setSuffix(" vx");
    m_zDim = new QSpinBox; m_zDim->setRange(1, 2000); m_zDim->setSuffix(" vx");
    m_voxel = new QDoubleSpinBox; m_voxel->setRange(0.001, 0.5);
    m_voxel->setSingleStep(0.001); m_voxel->setDecimals(4); m_voxel->setSuffix(" m");

    auto makeOffset = [] {
        auto *w = new QDoubleSpinBox;
        w->setRange(-5.0, 5.0); w->setSingleStep(0.05);
        w->setDecimals(2); w->setSuffix(" m");
        return w;
    };
    m_offsetX = makeOffset();
    m_offsetY = makeOffset();
    m_offsetZ = makeOffset();

    f->addRow("xDim", m_xDim);
    f->addRow("yDim", m_yDim);
    f->addRow("zDim", m_zDim);
    f->addRow("Voxel size", m_voxel);
    f->addRow("Init offset X (right)",  m_offsetX);
    f->addRow("Init offset Y (down)",   m_offsetY);
    f->addRow("Init offset Z (forward)", m_offsetZ);

    auto vol = [this]{ onVolumeChanged(); };
    connect(m_xDim,  QOverload<int>::of(&QSpinBox::valueChanged),         this, vol);
    connect(m_yDim,  QOverload<int>::of(&QSpinBox::valueChanged),         this, vol);
    connect(m_zDim,  QOverload<int>::of(&QSpinBox::valueChanged),         this, vol);
    connect(m_voxel, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, vol);
    connect(m_offsetX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, vol);
    connect(m_offsetY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, vol);
    connect(m_offsetZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, vol);
    return g;
}

QGroupBox *ParameterPanel::buildTsdfGroup() {
    auto *g = new QGroupBox("TSDF");
    auto *f = new QFormLayout(g);
    m_maxTrunc = new QDoubleSpinBox;
    m_maxTrunc->setRange(0.001, 0.5); m_maxTrunc->setSingleStep(0.005);
    m_maxTrunc->setDecimals(4); m_maxTrunc->setSuffix(" m");
    m_depthEdge = new QDoubleSpinBox;
    m_depthEdge->setRange(0.0, 0.5); m_depthEdge->setSingleStep(0.005);
    m_depthEdge->setDecimals(4); m_depthEdge->setSuffix(" m");
    f->addRow("Max truncation", m_maxTrunc);
    f->addRow("Depth edge reject", m_depthEdge);
    for (auto *w : { m_maxTrunc, m_depthEdge })
        connect(w, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &ParameterPanel::onAnyChanged);
    return g;
}

QGroupBox *ParameterPanel::buildBilateralGroup() {
    auto *g = new QGroupBox("Bilateral filter");
    auto *f = new QFormLayout(g);
    m_sigmaD = new QDoubleSpinBox;
    m_sigmaD->setRange(0.1, 30.0); m_sigmaD->setSingleStep(0.5);
    m_sigmaR = new QDoubleSpinBox;
    m_sigmaR->setRange(0.1, 50.0); m_sigmaR->setSingleStep(0.5);
    f->addRow("Spatial sigma (reset)", m_sigmaD);
    f->addRow("Range sigma",  m_sigmaR);
    connect(m_sigmaD, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ParameterPanel::onVolumeChanged); // requires reset
    connect(m_sigmaR, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ParameterPanel::onAnyChanged);
    return g;
}

QGroupBox *ParameterPanel::buildRaycastGroup() {
    auto *g = new QGroupBox("Raycast + normals");
    auto *f = new QFormLayout(g);
    m_normalThresh = new QDoubleSpinBox;
    m_normalThresh->setRange(0.0001, 0.5); m_normalThresh->setDecimals(4);
    m_normalThresh->setSingleStep(0.005);
    m_rcNear = new QDoubleSpinBox; m_rcNear->setRange(0.05, 4.0);
    m_rcNear->setSingleStep(0.05); m_rcNear->setSuffix(" m");
    m_rcFar  = new QDoubleSpinBox; m_rcFar->setRange(0.5, 12.0);
    m_rcFar->setSingleStep(0.1); m_rcFar->setSuffix(" m");
    m_rcStep = new QDoubleSpinBox; m_rcStep->setRange(0.001, 0.2);
    m_rcStep->setDecimals(4); m_rcStep->setSingleStep(0.005);
    m_rcStep->setSuffix(" m");
    f->addRow("Normal threshold", m_normalThresh);
    f->addRow("Near", m_rcNear);
    f->addRow("Far",  m_rcFar);
    f->addRow("Step", m_rcStep);
    for (auto *w : { m_normalThresh, m_rcNear, m_rcFar, m_rcStep })
        connect(w, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &ParameterPanel::onAnyChanged);
    return g;
}

QGroupBox *ParameterPanel::buildIcpGroup() {
    auto *g = new QGroupBox("ICP (requires Reset)");
    auto *f = new QFormLayout(g);
    m_icp0 = new QSpinBox; m_icp0->setRange(0, 50);
    m_icp1 = new QSpinBox; m_icp1->setRange(0, 50);
    m_icp2 = new QSpinBox; m_icp2->setRange(0, 50);
    m_icpDist  = new QDoubleSpinBox; m_icpDist->setRange(0.01, 1.0);
    m_icpDist->setDecimals(3); m_icpDist->setSingleStep(0.01);
    m_icpDist->setSuffix(" m");
    m_icpAngle = new QDoubleSpinBox; m_icpAngle->setRange(0.05, 0.95);
    m_icpAngle->setDecimals(3); m_icpAngle->setSingleStep(0.01);
    f->addRow("Iter pyr 0", m_icp0);
    f->addRow("Iter pyr 1", m_icp1);
    f->addRow("Iter pyr 2", m_icp2);
    f->addRow("Dist threshold", m_icpDist);
    f->addRow("sin(angle thr)", m_icpAngle);
    auto needsReset = [this]{ onVolumeChanged(); };
    connect(m_icp0, QOverload<int>::of(&QSpinBox::valueChanged), this, needsReset);
    connect(m_icp1, QOverload<int>::of(&QSpinBox::valueChanged), this, needsReset);
    connect(m_icp2, QOverload<int>::of(&QSpinBox::valueChanged), this, needsReset);
    connect(m_icpDist,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, needsReset);
    connect(m_icpAngle, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, needsReset);
    return g;
}

QGroupBox *ParameterPanel::buildMcGroup() {
    auto *g = new QGroupBox("Marching cubes");
    auto *f = new QFormLayout(g);
    m_iso = new QDoubleSpinBox; m_iso->setRange(-1.0, 1.0);
    m_iso->setSingleStep(0.01); m_iso->setDecimals(3);
    f->addRow("Iso value", m_iso);
    connect(m_iso, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ParameterPanel::onAnyChanged);
    return g;
}

void ParameterPanel::setParameters(const ScanParameters &p) {
    m_loading = true;
    m_current = p;
    int poseIdx = m_poseSource->findData(p.poseSource);
    m_poseSource->setCurrentIndex(poseIdx >= 0 ? poseIdx : 0);
    m_simStlPath->setText(QString::fromStdString(p.simStlPath));
    m_angleStart->setValue(p.angleStartDeg);
    m_angleEnd->setValue(p.angleEndDeg);
    m_angleStep->setValue(p.angleStepDeg);
    m_stageStart->setValue(p.stageStartMm);
    m_stageEnd->setValue(p.stageEndMm);
    m_stageStep->setValue(p.stageStepMm);
    m_framesPerPose->setValue(p.framesPerPose);
    m_targetSettle->setValue(p.targetSettleMs);
    m_angleTolerance->setValue(p.angleToleranceDeg);
    m_stageTolerance->setValue(p.stageToleranceMm);
    m_targetTimeout->setValue(p.targetTimeoutMs);
    m_turntableRadius->setValue(p.turntableRadiusMm);
    m_turntableHeight->setValue(p.turntableHeightMm);
    m_kinectX->setValue(p.kinectOffsetXMm);
    m_kinectY->setValue(p.kinectOffsetYMm);
    m_kinectZ->setValue(p.kinectOffsetZMm);
    m_kinectRoll->setValue(p.kinectRollDeg);
    m_kinectPitch->setValue(p.kinectPitchDeg);
    m_kinectYaw->setValue(p.kinectYawDeg);
    m_stageAxisX->setValue(p.stageAxisX);
    m_stageAxisY->setValue(p.stageAxisY);
    m_stageAxisZ->setValue(p.stageAxisZ);
    m_actuatorHost->setText(QString::fromStdString(p.actuatorTcpHost));
    m_actuatorPort->setValue(p.actuatorTcpPort);
    m_depthNoise->setValue(p.simDepthNoiseMm);
    m_dropout->setValue(p.simDropoutPercent);
    m_xDim->setValue((int)p.xDim);
    m_yDim->setValue((int)p.yDim);
    m_zDim->setValue((int)p.zDim);
    m_voxel->setValue(p.voxelSize);
    m_offsetX->setValue(p.gridInitOffsetX);
    m_offsetY->setValue(p.gridInitOffsetY);
    m_offsetZ->setValue(p.gridInitOffsetZ);
    m_maxTrunc->setValue(p.maxTruncation);
    m_depthEdge->setValue(p.depthEdgeThreshold);
    m_sigmaD->setValue(p.sigma_d);
    m_sigmaR->setValue(p.sigma_r);
    m_normalThresh->setValue(p.normalThreshold);
    m_rcNear->setValue(p.raycastNear);
    m_rcFar->setValue(p.raycastFar);
    m_rcStep->setValue(p.raycastStep);
    m_icp0->setValue(p.icpIterations0);
    m_icp1->setValue(p.icpIterations1);
    m_icp2->setValue(p.icpIterations2);
    m_icpDist->setValue(p.icpDistThresh);
    m_icpAngle->setValue(p.icpAngleThresh);
    m_iso->setValue(p.isoValue);
    m_loading = false;
}

void ParameterPanel::setVolumeEditable(bool editable) {
    m_poseSource->setEnabled(editable);
    m_simStlPath->setEnabled(editable);
    m_angleStart->setEnabled(editable);
    m_angleEnd->setEnabled(editable);
    m_angleStep->setEnabled(editable);
    m_stageStart->setEnabled(editable);
    m_stageEnd->setEnabled(editable);
    m_stageStep->setEnabled(editable);
    m_framesPerPose->setEnabled(editable);
    m_targetSettle->setEnabled(editable);
    m_angleTolerance->setEnabled(editable);
    m_stageTolerance->setEnabled(editable);
    m_targetTimeout->setEnabled(editable);
    m_turntableRadius->setEnabled(editable);
    m_turntableHeight->setEnabled(editable);
    m_kinectX->setEnabled(editable);
    m_kinectY->setEnabled(editable);
    m_kinectZ->setEnabled(editable);
    m_kinectRoll->setEnabled(editable);
    m_kinectPitch->setEnabled(editable);
    m_kinectYaw->setEnabled(editable);
    m_stageAxisX->setEnabled(editable);
    m_stageAxisY->setEnabled(editable);
    m_stageAxisZ->setEnabled(editable);
    m_actuatorHost->setEnabled(editable);
    m_actuatorPort->setEnabled(editable);
    m_depthNoise->setEnabled(editable);
    m_dropout->setEnabled(editable);
    m_xDim->setEnabled(editable);
    m_yDim->setEnabled(editable);
    m_zDim->setEnabled(editable);
    m_voxel->setEnabled(editable);
    m_offsetX->setEnabled(editable);
    m_offsetY->setEnabled(editable);
    m_offsetZ->setEnabled(editable);
    m_sigmaD->setEnabled(editable);
    m_icp0->setEnabled(editable);
    m_icp1->setEnabled(editable);
    m_icp2->setEnabled(editable);
    m_icpDist->setEnabled(editable);
    m_icpAngle->setEnabled(editable);
}

void ParameterPanel::onAnyChanged() {
    if (m_loading) return;
    m_debounce.start();
}

void ParameterPanel::onVolumeChanged() {
    if (m_loading) return;
    m_volumeDirty = true;
    m_debounce.start();
}

void ParameterPanel::flush() {
    m_current.poseSource = m_poseSource->currentData().toInt();
    m_current.simulationEnabled = (m_current.poseSource == PoseSourceSimulation);
    m_current.actuatorTcpEnabled = (m_current.poseSource == PoseSourceActuatedTcp);
    m_current.simStlPath = m_simStlPath->text().toStdString();
    m_current.angleStartDeg = (float)m_angleStart->value();
    m_current.angleEndDeg = (float)m_angleEnd->value();
    m_current.angleStepDeg = (float)m_angleStep->value();
    m_current.stageStartMm = (float)m_stageStart->value();
    m_current.stageEndMm = (float)m_stageEnd->value();
    m_current.stageStepMm = (float)m_stageStep->value();
    m_current.framesPerPose = m_framesPerPose->value();
    m_current.targetSettleMs = (float)m_targetSettle->value();
    m_current.angleToleranceDeg = (float)m_angleTolerance->value();
    m_current.stageToleranceMm = (float)m_stageTolerance->value();
    m_current.targetTimeoutMs = (float)m_targetTimeout->value();
    m_current.turntableRadiusMm = (float)m_turntableRadius->value();
    m_current.turntableHeightMm = (float)m_turntableHeight->value();
    m_current.kinectOffsetXMm = (float)m_kinectX->value();
    m_current.kinectOffsetYMm = (float)m_kinectY->value();
    m_current.kinectOffsetZMm = (float)m_kinectZ->value();
    m_current.kinectRollDeg = (float)m_kinectRoll->value();
    m_current.kinectPitchDeg = (float)m_kinectPitch->value();
    m_current.kinectYawDeg = (float)m_kinectYaw->value();
    m_current.stageAxisX = (float)m_stageAxisX->value();
    m_current.stageAxisY = (float)m_stageAxisY->value();
    m_current.stageAxisZ = (float)m_stageAxisZ->value();
    m_current.actuatorTcpHost = m_actuatorHost->text().toStdString();
    m_current.actuatorTcpPort = m_actuatorPort->value();
    m_current.simDepthNoiseMm = (float)m_depthNoise->value();
    m_current.simDropoutPercent = (float)m_dropout->value();
    m_current.xDim      = (unsigned)m_xDim->value();
    m_current.yDim      = (unsigned)m_yDim->value();
    m_current.zDim      = (unsigned)m_zDim->value();
    m_current.voxelSize = (float)m_voxel->value();
    m_current.gridInitOffsetX = (float)m_offsetX->value();
    m_current.gridInitOffsetY = (float)m_offsetY->value();
    m_current.gridInitOffsetZ = (float)m_offsetZ->value();
    m_current.maxTruncation  = (float)m_maxTrunc->value();
    m_current.depthEdgeThreshold = (float)m_depthEdge->value();
    m_current.sigma_d        = (float)m_sigmaD->value();
    m_current.sigma_r        = (float)m_sigmaR->value();
    m_current.normalThreshold = (float)m_normalThresh->value();
    m_current.raycastNear    = (float)m_rcNear->value();
    m_current.raycastFar     = (float)m_rcFar->value();
    m_current.raycastStep    = (float)m_rcStep->value();
    m_current.icpIterations0 = m_icp0->value();
    m_current.icpIterations1 = m_icp1->value();
    m_current.icpIterations2 = m_icp2->value();
    m_current.icpDistThresh  = (float)m_icpDist->value();
    m_current.icpAngleThresh = (float)m_icpAngle->value();
    m_current.isoValue       = (float)m_iso->value();
    bool requiresReset = m_volumeDirty;
    m_volumeDirty = false;
    emit parametersChanged(m_current, requiresReset);
}
