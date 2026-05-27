#include "parameter_panel.hpp"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>

ParameterPanel::ParameterPanel(QWidget *parent) : QWidget(parent) {
    m_debounce.setSingleShot(true);
    m_debounce.setInterval(100);
    connect(&m_debounce, &QTimer::timeout, this, &ParameterPanel::flush);

    auto *layout = new QVBoxLayout(this);
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

QGroupBox *ParameterPanel::buildVolumeGroup() {
    auto *g = new QGroupBox("Volume (requires Reset)");
    auto *f = new QFormLayout(g);

    m_xDim = new QSpinBox; m_xDim->setRange(1, 2000); m_xDim->setSuffix(" vx");
    m_yDim = new QSpinBox; m_yDim->setRange(1, 2000); m_yDim->setSuffix(" vx");
    m_zDim = new QSpinBox; m_zDim->setRange(1, 2000); m_zDim->setSuffix(" vx");
    m_voxel = new QDoubleSpinBox; m_voxel->setRange(0.001, 0.5);
    m_voxel->setSingleStep(0.001); m_voxel->setDecimals(4); m_voxel->setSuffix(" m");

    f->addRow("xDim", m_xDim);
    f->addRow("yDim", m_yDim);
    f->addRow("zDim", m_zDim);
    f->addRow("Voxel size", m_voxel);

    auto vol = [this]{ onVolumeChanged(); };
    connect(m_xDim,  QOverload<int>::of(&QSpinBox::valueChanged),         this, vol);
    connect(m_yDim,  QOverload<int>::of(&QSpinBox::valueChanged),         this, vol);
    connect(m_zDim,  QOverload<int>::of(&QSpinBox::valueChanged),         this, vol);
    connect(m_voxel, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, vol);
    return g;
}

QGroupBox *ParameterPanel::buildTsdfGroup() {
    auto *g = new QGroupBox("TSDF");
    auto *f = new QFormLayout(g);
    m_maxTrunc = new QDoubleSpinBox;
    m_maxTrunc->setRange(0.001, 0.5); m_maxTrunc->setSingleStep(0.005);
    m_maxTrunc->setDecimals(4); m_maxTrunc->setSuffix(" m");
    f->addRow("Max truncation", m_maxTrunc);
    connect(m_maxTrunc, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
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
    m_xDim->setValue((int)p.xDim);
    m_yDim->setValue((int)p.yDim);
    m_zDim->setValue((int)p.zDim);
    m_voxel->setValue(p.voxelSize);
    m_maxTrunc->setValue(p.maxTruncation);
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
    m_xDim->setEnabled(editable);
    m_yDim->setEnabled(editable);
    m_zDim->setEnabled(editable);
    m_voxel->setEnabled(editable);
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
    m_current.xDim      = (unsigned)m_xDim->value();
    m_current.yDim      = (unsigned)m_yDim->value();
    m_current.zDim      = (unsigned)m_zDim->value();
    m_current.voxelSize = (float)m_voxel->value();
    m_current.maxTruncation  = (float)m_maxTrunc->value();
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
