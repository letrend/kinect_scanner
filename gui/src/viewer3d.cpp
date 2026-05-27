#include "viewer3d.hpp"

#include <QMouseEvent>
#include <QWheelEvent>
#include <cmath>

namespace {
const char *kColorVS = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uMVP;
uniform mat4 uModel;
out vec3 vColor;
void main(){
    vColor = aColor;
    gl_Position = uMVP * uModel * vec4(aPos, 1.0);
    gl_PointSize = 2.0;
}
)";

const char *kColorFS = R"(
#version 330 core
in vec3 vColor;
out vec4 oColor;
void main(){ oColor = vec4(vColor, 1.0); }
)";

const char *kMeshVS = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uMVP;
uniform mat4 uModel;
out vec3 vColor;
out vec3 vPosW;
void main(){
    vColor = aColor;
    vec4 w = uModel * vec4(aPos, 1.0);
    vPosW = w.xyz;
    gl_Position = uMVP * w;
}
)";

const char *kMeshFS = R"(
#version 330 core
in vec3 vColor;
in vec3 vPosW;
out vec4 oColor;
uniform vec3 uLightDir;
void main(){
    vec3 dx = dFdx(vPosW);
    vec3 dy = dFdy(vPosW);
    vec3 n  = normalize(cross(dx, dy));
    float lambert = max(dot(n, normalize(uLightDir)), 0.0);
    vec3 col = vColor * (0.25 + 0.75 * lambert);
    oColor = vec4(col, 1.0);
}
)";
} // namespace

Viewer3D::Viewer3D(QWidget *parent) : QOpenGLWidget(parent) {
    setMinimumSize(320, 240);
    setFocusPolicy(Qt::StrongFocus);
}

Viewer3D::~Viewer3D() {
    makeCurrent();
    m_vboPointsXyz.destroy(); m_vboPointsRgb.destroy();
    m_vboMeshXyz.destroy(); m_vboMeshRgb.destroy(); m_iboMesh.destroy();
    m_vboBox.destroy(); m_vboTraj.destroy();
    m_vaoPoints.destroy(); m_vaoMesh.destroy();
    m_vaoBox.destroy(); m_vaoTraj.destroy();
    doneCurrent();
}

void Viewer3D::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.12f, 0.13f, 0.16f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

    m_progColor.addShaderFromSourceCode(QOpenGLShader::Vertex, kColorVS);
    m_progColor.addShaderFromSourceCode(QOpenGLShader::Fragment, kColorFS);
    m_progColor.link();
    m_progMesh.addShaderFromSourceCode(QOpenGLShader::Vertex, kMeshVS);
    m_progMesh.addShaderFromSourceCode(QOpenGLShader::Fragment, kMeshFS);
    m_progMesh.link();

    auto initVao = [](QOpenGLVertexArrayObject &v) { v.create(); };
    initVao(m_vaoPoints);
    initVao(m_vaoMesh);
    initVao(m_vaoBox);
    initVao(m_vaoTraj);
    m_vboPointsXyz.create();
    m_vboPointsRgb.create();
    m_vboMeshXyz.create();
    m_vboMeshRgb.create();
    m_iboMesh.create();
    m_vboBox.create();
    m_vboTraj.create();
}

void Viewer3D::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void Viewer3D::uploadPoints() {
    m_vaoPoints.bind();
    m_vboPointsXyz.bind();
    m_vboPointsXyz.allocate(m_pointsXyz.constData(),
                            m_pointsXyz.size() * sizeof(float));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    m_vboPointsRgb.bind();
    m_vboPointsRgb.allocate(m_pointsRgb.constData(), m_pointsRgb.size());
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_TRUE, 0, nullptr);
    m_vaoPoints.release();
    m_pointsDirty = false;
}

void Viewer3D::uploadMesh() {
    m_vaoMesh.bind();
    m_vboMeshXyz.bind();
    m_vboMeshXyz.allocate(m_meshXyz.constData(), m_meshXyz.size() * sizeof(float));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    m_vboMeshRgb.bind();
    m_vboMeshRgb.allocate(m_meshRgb.constData(), m_meshRgb.size());
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_TRUE, 0, nullptr);

    m_iboMesh.bind();
    m_iboMesh.allocate(m_meshIdx.constData(),
                       m_meshIdx.size() * sizeof(unsigned int));
    m_vaoMesh.release();
    m_meshDirty = false;
}

void Viewer3D::uploadBounds() {
    const float x = m_boundsM.x() * 0.5f;
    const float y = m_boundsM.y() * 0.5f;
    const float z = m_boundsM.z() * 0.5f;
    const float c = 0.6f; // grey
    // 12 edges = 24 vertices (xyz + rgb per vertex interleaved as separate buffers).
    // For simplicity store xyz and a uniform colour.
    float v[] = {
        -x,-y,-z,  x,-y,-z,    x,-y,-z,  x, y,-z,
         x, y,-z, -x, y,-z,   -x, y,-z, -x,-y,-z,
        -x,-y, z,  x,-y, z,    x,-y, z,  x, y, z,
         x, y, z, -x, y, z,   -x, y, z, -x,-y, z,
        -x,-y,-z, -x,-y, z,    x,-y,-z,  x,-y, z,
         x, y,-z,  x, y, z,   -x, y,-z, -x, y, z,
    };
    QVector<float> data;
    data.reserve(sizeof(v)/sizeof(float) * 2);
    for (size_t i = 0; i < sizeof(v)/sizeof(float); i += 3) {
        data.append(v[i]); data.append(v[i+1]); data.append(v[i+2]);
        data.append(c);    data.append(c);      data.append(c);
    }
    m_vaoBox.bind();
    m_vboBox.bind();
    m_vboBox.allocate(data.constData(), data.size() * sizeof(float));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float),
                          (void*)(3*sizeof(float)));
    m_vaoBox.release();
    m_boundsDirty = false;
}

void Viewer3D::uploadTraj() {
    if (m_trajXyz.isEmpty()) { m_trajDirty = false; return; }
    QVector<float> data;
    data.reserve(m_trajXyz.size() * 2);
    for (int i = 0; i < m_trajXyz.size(); i += 3) {
        data.append(m_trajXyz[i]); data.append(m_trajXyz[i+1]); data.append(m_trajXyz[i+2]);
        data.append(1.0f); data.append(0.9f); data.append(0.2f); // yellow
    }
    m_vaoTraj.bind();
    m_vboTraj.bind();
    m_vboTraj.allocate(data.constData(), data.size() * sizeof(float));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float),
                          (void*)(3*sizeof(float)));
    m_vaoTraj.release();
    m_trajDirty = false;
}

void Viewer3D::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 proj;
    proj.perspective(45.0f, width() / float(qMax(1, height())), 0.05f, 100.0f);

    QMatrix4x4 view;
    QVector3D eye = m_center;
    float rad = m_distance;
    float yaw = m_yawDeg * float(M_PI) / 180.0f;
    float pitch = m_pitchDeg * float(M_PI) / 180.0f;
    eye += QVector3D(rad * std::cos(pitch) * std::sin(yaw),
                     rad * std::sin(pitch),
                     rad * std::cos(pitch) * std::cos(yaw));
    view.lookAt(eye, m_center, QVector3D(0, 1, 0));
    QMatrix4x4 mvp = proj * view;

    if (m_boundsDirty) uploadBounds();
    if (m_pointsDirty && !m_pointsXyz.isEmpty()) uploadPoints();
    if (m_meshDirty   && !m_meshXyz.isEmpty())   uploadMesh();
    if (m_trajDirty   && !m_trajXyz.isEmpty())   uploadTraj();

    // --- Box ---
    {
        m_progColor.bind();
        m_progColor.setUniformValue("uMVP", mvp);
        m_progColor.setUniformValue("uModel", QMatrix4x4{});
        m_vaoBox.bind();
        glDrawArrays(GL_LINES, 0, 24);
        m_vaoBox.release();
        m_progColor.release();
    }
    // --- Points ---
    if (m_showPoints && !m_pointsXyz.isEmpty()) {
        m_progColor.bind();
        m_progColor.setUniformValue("uMVP", mvp);
        m_progColor.setUniformValue("uModel", m_pointsPose);
        m_vaoPoints.bind();
        glDrawArrays(GL_POINTS, 0, m_pointsXyz.size()/3);
        m_vaoPoints.release();
        m_progColor.release();
    }
    // --- Mesh ---
    if (m_showMesh && !m_meshIdx.isEmpty()) {
        m_progMesh.bind();
        m_progMesh.setUniformValue("uMVP", mvp);
        m_progMesh.setUniformValue("uModel", QMatrix4x4{});
        m_progMesh.setUniformValue("uLightDir", QVector3D(0.3f, 0.8f, 0.5f));
        m_vaoMesh.bind();
        glDrawElements(GL_TRIANGLES, m_meshIdx.size(), GL_UNSIGNED_INT, nullptr);
        m_vaoMesh.release();
        m_progMesh.release();
    }
    // --- Trajectory ---
    if (m_showTraj && m_trajXyz.size() >= 6) {
        m_progColor.bind();
        m_progColor.setUniformValue("uMVP", mvp);
        m_progColor.setUniformValue("uModel", QMatrix4x4{});
        m_vaoTraj.bind();
        glDrawArrays(GL_LINE_STRIP, 0, m_trajXyz.size()/3);
        m_vaoTraj.release();
        m_progColor.release();
    }
}

void Viewer3D::setPointCloud(const QVector<float> &xyz,
                              const QVector<unsigned char> &rgb,
                              const QMatrix4x4 &pose) {
    m_pointsXyz = xyz;
    m_pointsRgb = rgb;
    m_pointsPose = pose;
    m_pointsDirty = true;
    // append camera origin to trajectory
    m_trajXyz.append(pose(0,3));
    m_trajXyz.append(pose(1,3));
    m_trajXyz.append(pose(2,3));
    m_trajDirty = true;
    update();
}

void Viewer3D::setMesh(const QVector<float> &vertices,
                        const QVector<unsigned char> &colors,
                        const QVector<unsigned int> &indices) {
    m_meshXyz = vertices;
    m_meshRgb = colors;
    m_meshIdx = indices;
    m_meshDirty = true;
    update();
}

void Viewer3D::setVolumeBounds(float x, float y, float z) {
    m_boundsM = QVector3D(x, y, z);
    m_boundsDirty = true;
    update();
}

void Viewer3D::resetView() {
    m_center = QVector3D(0, 0, 2);
    m_distance = 3.0f;
    m_yawDeg = 0.0f;
    m_pitchDeg = -15.0f;
    update();
}

void Viewer3D::mousePressEvent(QMouseEvent *e) {
    m_lastMouse = e->pos();
    m_dragButton = e->button();
}

void Viewer3D::mouseMoveEvent(QMouseEvent *e) {
    QPoint d = e->pos() - m_lastMouse;
    m_lastMouse = e->pos();
    if (m_dragButton == Qt::LeftButton) {
        m_yawDeg   += d.x() * 0.4f;
        m_pitchDeg += d.y() * 0.4f;
        if (m_pitchDeg > 89.0f)  m_pitchDeg = 89.0f;
        if (m_pitchDeg < -89.0f) m_pitchDeg = -89.0f;
    } else if (m_dragButton == Qt::MiddleButton || m_dragButton == Qt::RightButton) {
        float k = 0.005f * m_distance;
        // Pan in screen plane.
        float yaw = m_yawDeg * float(M_PI) / 180.0f;
        QVector3D right(std::cos(yaw), 0, -std::sin(yaw));
        QVector3D up(0, 1, 0);
        m_center -= right * (d.x() * k);
        m_center += up    * (d.y() * k);
    }
    update();
}

void Viewer3D::wheelEvent(QWheelEvent *e) {
    float delta = e->angleDelta().y() / 120.0f;
    m_distance *= std::pow(0.9f, delta);
    if (m_distance < 0.1f) m_distance = 0.1f;
    update();
}
