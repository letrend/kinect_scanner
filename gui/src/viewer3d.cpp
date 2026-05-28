#include "viewer3d.hpp"

#include <QElapsedTimer>
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
    m_vboBox.destroy(); m_vboTraj.destroy(); m_vboCam.destroy();
    m_vaoPoints.destroy(); m_vaoMesh.destroy();
    m_vaoBox.destroy(); m_vaoTraj.destroy(); m_vaoCam.destroy();
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
    initVao(m_vaoCam);
    m_vboPointsXyz.create();
    m_vboPointsRgb.create();
    m_vboMeshXyz.create();
    m_vboMeshRgb.create();
    m_iboMesh.create();
    m_vboBox.create();
    m_vboTraj.create();
    m_vboCam.create();
    uploadCamera();
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

void Viewer3D::uploadCamera() {
    // Wireframe pyramid in *camera-local* coordinates (OpenCV style:
    // +X right, +Y down, +Z forward). Apex at origin, base at z=d.
    const float d = 0.20f;                          // 20 cm
    const float w = d * std::tan(0.5f * 70.0f * float(M_PI) / 180.0f); // ~RGB hfov
    const float h = d * std::tan(0.5f * 60.0f * float(M_PI) / 180.0f); // ~RGB vfov
    const float r = 0.10f, g = 1.00f, b = 1.00f;     // cyan
    // Apex
    QVector3D A(0, 0, 0);
    // Base corners (TL, TR, BR, BL)
    QVector3D TL(-w, -h, d), TR( w, -h, d), BR( w, h, d), BL(-w, h, d);
    QVector3D U( 0, -h * 1.35f, d); // little "up" tick so orientation is unambiguous
    auto seg = [&](QVector<float> &v, QVector3D a, QVector3D bp) {
        v.append(a.x()); v.append(a.y()); v.append(a.z());
        v.append(r); v.append(g); v.append(b);
        v.append(bp.x()); v.append(bp.y()); v.append(bp.z());
        v.append(r); v.append(g); v.append(b);
    };
    QVector<float> data;
    seg(data, A, TL); seg(data, A, TR); seg(data, A, BR); seg(data, A, BL);
    seg(data, TL, TR); seg(data, TR, BR); seg(data, BR, BL); seg(data, BL, TL);
    seg(data, TL, U);  seg(data, TR, U);   // "up" triangle on top of frustum
    m_camVertexCount = data.size() / 6;
    m_vaoCam.bind();
    m_vboCam.bind();
    m_vboCam.allocate(data.constData(), data.size() * sizeof(float));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float),
                          (void*)(3*sizeof(float)));
    m_vaoCam.release();
}

void Viewer3D::paintGL() {
    QElapsedTimer paintTimer; paintTimer.start();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 proj;
    if (m_cameraLocked && m_camPoseValid) {
        // Match the Kinect v2 IR sensor's vertical FOV (~60.4°). Aspect is
        // forced to the sensor's 512x424 so the view feels like looking
        // through the camera rather than the GL widget aspect.
        const float kinectVFovDeg = 60.4f;
        const float kinectAspect  = 512.0f / 424.0f;
        proj.perspective(kinectVFovDeg, kinectAspect, 0.05f, 100.0f);
        // World is Kinect (X right, Y down, Z forward) with model=flipY,
        // giving a left-handed (X right, Y up, Z forward) frame. GL's
        // lookAt is right-handed, so without correction the world appears
        // X-mirrored (looking left makes points slide right relative to
        // expectation). Pre-mirror the projection's X axis to convert
        // back to a right-handed image. Face culling is disabled, so
        // winding flip has no visual effect.
        proj = QMatrix4x4(-1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1) * proj;
    } else {
        proj.perspective(45.0f, width() / float(qMax(1, height())), 0.05f, 100.0f);
    }

    // Kinect/OpenCV camera frame is +Y down; flip Y to render upright.
    QMatrix4x4 flipY;
    flipY.scale(1.0f, -1.0f, 1.0f);

    QMatrix4x4 view;
    if (m_cameraLocked && m_camPoseValid) {
        // Build an explicit lookAt from the Kinect pose. The point cloud is
        // already in *world* space (camera-space points were transformed via
        // pose.map() on receipt) and drawn with model = flipY, i.e. world
        // coordinates with Y negated to render upright.
        //
        // The Kinect camera basis is (X right, Y down, Z forward). To put
        // the GL camera at the same physical location and aim it the same
        // way, we extract those basis vectors from m_camPose, apply the
        // same Y-flip the world uses, then feed lookAt:
        //   eye      = flipY * cam_origin
        //   forward  = flipY * cam_R * (0,0,1)
        //   up       = flipY * cam_R * (0,-1,0)  // Kinect Y is down -> GL up
        QVector3D camOrigin( m_camPose(0,3), m_camPose(1,3), m_camPose(2,3) );
        QVector3D camFwd  ( m_camPose(0,2), m_camPose(1,2), m_camPose(2,2) );
        QVector3D camUpK  (-m_camPose(0,1),-m_camPose(1,1),-m_camPose(2,1));

        QVector3D eye    ( camOrigin.x(), -camOrigin.y(), camOrigin.z() );
        QVector3D fwd    ( camFwd.x(),    -camFwd.y(),    camFwd.z() );
        QVector3D up     ( camUpK.x(),    -camUpK.y(),    camUpK.z() );
        view.lookAt(eye, eye + fwd, up);
    } else {
        QVector3D eye = m_center;
        float rad = m_distance;
        float yaw = m_yawDeg * float(M_PI) / 180.0f;
        float pitch = m_pitchDeg * float(M_PI) / 180.0f;
        eye += QVector3D(rad * std::cos(pitch) * std::sin(yaw),
                         rad * std::sin(pitch),
                         rad * std::cos(pitch) * std::cos(yaw));
        view.lookAt(eye, m_center, QVector3D(0, 1, 0));
    }
    QMatrix4x4 mvp = proj * view;

    if (m_boundsDirty) uploadBounds();
    if (m_pointsDirty && !m_pointsXyz.isEmpty()) uploadPoints();
    if (m_meshDirty   && !m_meshXyz.isEmpty())   uploadMesh();
    if (m_trajDirty   && !m_trajXyz.isEmpty())   uploadTraj();

    // --- Box ---
    {
        QMatrix4x4 boxModel = flipY;
        boxModel.translate(m_volumeCenterM);
        m_progColor.bind();
        m_progColor.setUniformValue("uMVP", mvp);
        m_progColor.setUniformValue("uModel", boxModel);
        m_vaoBox.bind();
        glDrawArrays(GL_LINES, 0, 24);
        m_vaoBox.release();
        m_progColor.release();
    }
    // --- Points ---
    if (m_showPoints && !m_pointsXyz.isEmpty()) {
        m_progColor.bind();
        m_progColor.setUniformValue("uMVP", mvp);
        m_progColor.setUniformValue("uModel", flipY * m_pointsPose);
        m_vaoPoints.bind();
        glDrawArrays(GL_POINTS, 0, m_pointsXyz.size()/3);
        m_vaoPoints.release();
        m_progColor.release();
    }
    // --- Mesh ---
    if (m_showMesh && !m_meshIdx.isEmpty()) {
        m_progMesh.bind();
        m_progMesh.setUniformValue("uMVP", mvp);
        m_progMesh.setUniformValue("uModel", flipY);
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
        m_progColor.setUniformValue("uModel", flipY);
        m_vaoTraj.bind();
        glDrawArrays(GL_LINE_STRIP, 0, m_trajXyz.size()/3);
        m_vaoTraj.release();
        m_progColor.release();
    }
    // --- Camera frustum at estimated pose ---
    if (m_showCamera && !m_cameraLocked && m_camPoseValid && m_camVertexCount > 0) {
        m_progColor.bind();
        m_progColor.setUniformValue("uMVP", mvp);
        m_progColor.setUniformValue("uModel", flipY * m_camPose);
        m_vaoCam.bind();
        glDrawArrays(GL_LINES, 0, m_camVertexCount);
        m_vaoCam.release();
        m_progColor.release();
    }

    // Adaptive point-cap controller: keep frame-time near 1/30 s.
    // Exponential moving average smooths spikes; geometric step gives fast
    // response in either direction without oscillating.
    const double ms = paintTimer.nsecsElapsed() / 1.0e6;
    m_avgPaintMs = (m_avgPaintMs == 0.0) ? ms : (0.85 * m_avgPaintMs + 0.15 * ms);
    const int kMinPoints = 50000;
    const int kMaxPoints = 20000000;
    if (m_avgPaintMs > kTargetFrameMs * 1.10) {
        // Too slow: shrink cap by 15% and immediately thin the existing
        // cloud (keep 1% as a sparse "memory" of what was scanned) so the
        // next frame benefits.
        int newCap = int(m_maxPoints * 0.85);
        if (newCap < kMinPoints) newCap = kMinPoints;
        if (newCap != m_maxPoints) {
            m_maxPoints = newCap;
            int have = m_pointsXyz.size() / 3;
            if (have > m_maxPoints) {
                const int keep = qMax(1, have / 100);
                QVector<float>         keptXyz; keptXyz.reserve(keep * 3);
                QVector<unsigned char> keptRgb; keptRgb.reserve(keep * 3);
                for (int i = 0; i < keep; ++i) {
                    int src = (int)((qint64(i) * have) / keep);
                    keptXyz.append(m_pointsXyz[3*src + 0]);
                    keptXyz.append(m_pointsXyz[3*src + 1]);
                    keptXyz.append(m_pointsXyz[3*src + 2]);
                    keptRgb.append(m_pointsRgb[3*src + 0]);
                    keptRgb.append(m_pointsRgb[3*src + 1]);
                    keptRgb.append(m_pointsRgb[3*src + 2]);
                }
                m_pointsXyz = std::move(keptXyz);
                m_pointsRgb = std::move(keptRgb);
                m_pointsDirty = true;
            }
        }
    } else if (m_avgPaintMs < kTargetFrameMs * 0.70 && m_maxPoints < kMaxPoints) {
        // Comfortable headroom: grow cap by 10%.
        int newCap = int(m_maxPoints * 1.10) + 1;
        if (newCap > kMaxPoints) newCap = kMaxPoints;
        m_maxPoints = newCap;
    }
}

void Viewer3D::setPointCloud(const QVector<float> &xyz,
                              const QVector<unsigned char> &rgb,
                              const QMatrix4x4 &pose) {
    // Transform incoming camera-space points into world space using the
    // estimated pose, then append to a bounded ring buffer so the viewer
    // shows an *accumulated* reconstruction rather than just the current
    // raycast.
    const int n = xyz.size() / 3;
    if (n > 0 && rgb.size() >= n * 3) {
        // If adding the new frame would exceed the adaptive cap, *thin out*
        // the existing (older) cloud uniformly down to 1% of its size,
        // instead of dropping the oldest contiguous block. That way the
        // user still sees the rough shape of everything scanned so far.
        int have = m_pointsXyz.size() / 3;
        if (have + n > m_maxPoints) {
            const int keep = qMax(1, have / 100); // ~1% of old points
            if (keep < have) {
                QVector<float>         keptXyz; keptXyz.reserve(keep * 3);
                QVector<unsigned char> keptRgb; keptRgb.reserve(keep * 3);
                // Uniform stride keeps a spatially representative subset
                // (per-frame downsampling at emit time made indices
                // already shuffled across the scene).
                for (int i = 0; i < keep; ++i) {
                    int src = (int)((qint64(i) * have) / keep);
                    keptXyz.append(m_pointsXyz[3*src + 0]);
                    keptXyz.append(m_pointsXyz[3*src + 1]);
                    keptXyz.append(m_pointsXyz[3*src + 2]);
                    keptRgb.append(m_pointsRgb[3*src + 0]);
                    keptRgb.append(m_pointsRgb[3*src + 1]);
                    keptRgb.append(m_pointsRgb[3*src + 2]);
                }
                m_pointsXyz = std::move(keptXyz);
                m_pointsRgb = std::move(keptRgb);
            }
        }
        m_pointsXyz.reserve(m_pointsXyz.size() + n * 3);
        m_pointsRgb.reserve(m_pointsRgb.size() + n * 3);
        for (int i = 0; i < n; ++i) {
            float x = xyz[3*i + 0];
            float y = xyz[3*i + 1];
            float z = xyz[3*i + 2];
            QVector3D w = pose.map(QVector3D(x, y, z));
            m_pointsXyz.append(w.x());
            m_pointsXyz.append(w.y());
            m_pointsXyz.append(w.z());
            m_pointsRgb.append(rgb[3*i + 0]);
            m_pointsRgb.append(rgb[3*i + 1]);
            m_pointsRgb.append(rgb[3*i + 2]);
        }
        m_pointsDirty = true;
    }
    m_pointsPose = QMatrix4x4(); // identity: points are already in world
    m_camPose = pose;
    m_camPoseValid = true;
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

void Viewer3D::setVolumeCenter(float x, float y, float z) {
    m_volumeCenterM = QVector3D(x, y, z);
    update();
}

void Viewer3D::resetView() {
    m_center = QVector3D(0, 0, 2);
    m_distance = 3.0f;
    m_yawDeg = 0.0f;
    m_pitchDeg = -15.0f;
    update();
}

void Viewer3D::clearAccumulated() {
    m_pointsXyz.clear();
    m_pointsRgb.clear();
    m_pointsDirty = true;
    m_trajXyz.clear();
    m_trajDirty = true;
    m_camPoseValid = false;
    update();
}

void Viewer3D::mousePressEvent(QMouseEvent *e) {
    if (m_cameraLocked) return; // arcball disabled in camera-lock mode
    m_lastMouse = e->pos();
    m_dragButton = e->button();
}

void Viewer3D::mouseMoveEvent(QMouseEvent *e) {
    if (m_cameraLocked) return;
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
    if (m_cameraLocked) return;
    float delta = e->angleDelta().y() / 120.0f;
    m_distance *= std::pow(0.9f, delta);
    if (m_distance < 0.1f) m_distance = 0.1f;
    update();
}
