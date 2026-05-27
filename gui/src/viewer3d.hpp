#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QVector>
#include <QMatrix4x4>
#include <QVector3D>
#include <QPoint>

/// Interactive 3D viewer:
///   - live point cloud uploaded from the worker each frame (GL_POINTS),
///     transformed into world space using the camera pose,
///   - optional final mesh (GL_TRIANGLES) with Phong-ish shading,
///   - wireframe volume bounding box and camera trajectory polyline,
///   - arcball rotation (LMB drag), pan (MMB drag), wheel zoom.
class Viewer3D : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core {
    Q_OBJECT
public:
    explicit Viewer3D(QWidget *parent = nullptr);
    ~Viewer3D() override;

public slots:
    /// Replace the live point cloud (xyz triples + rgb triples) drawn at pose.
    void setPointCloud(const QVector<float> &xyz,
                       const QVector<unsigned char> &rgb,
                       const QMatrix4x4 &pose);
    /// Replace the persistent mesh.
    void setMesh(const QVector<float> &vertices,
                 const QVector<unsigned char> &colors,
                 const QVector<unsigned int> &indices);
    /// Set the axis-aligned volume bounding-box size in meters.
    void setVolumeBounds(float xMeters, float yMeters, float zMeters);
    /// Reset the user view to look at the volume from the front.
    void resetView();
    /// Toggle visibility layers.
    void setShowPointCloud(bool s) { m_showPoints = s; update(); }
    void setShowMesh(bool s)       { m_showMesh   = s; update(); }
    void setShowTrajectory(bool s) { m_showTraj   = s; update(); }

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

private:
    void uploadPoints();
    void uploadMesh();
    void uploadBounds();
    void uploadTraj();

    // GL resources
    QOpenGLShaderProgram m_progColor; // point/line: vertex colour
    QOpenGLShaderProgram m_progMesh;  // mesh: simple Phong + vertex colour

    QOpenGLVertexArrayObject m_vaoPoints, m_vaoMesh, m_vaoBox, m_vaoTraj;
    QOpenGLBuffer m_vboPointsXyz{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_vboPointsRgb{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_vboMeshXyz{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_vboMeshRgb{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_iboMesh{QOpenGLBuffer::IndexBuffer};
    QOpenGLBuffer m_vboBox{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer m_vboTraj{QOpenGLBuffer::VertexBuffer};

    // CPU-side data
    QVector<float>         m_pointsXyz;
    QVector<unsigned char> m_pointsRgb;
    bool m_pointsDirty = false;
    QMatrix4x4 m_pointsPose;

    QVector<float>         m_meshXyz;
    QVector<unsigned char> m_meshRgb;
    QVector<unsigned int>  m_meshIdx;
    bool m_meshDirty = false;

    QVector<float> m_trajXyz; // appended each pose update
    bool m_trajDirty = false;

    QVector3D m_boundsM{4.0f, 4.0f, 4.0f};
    bool m_boundsDirty = true;

    // Camera (arcball)
    QVector3D m_center{0,0,2};
    float m_distance = 3.0f;
    float m_yawDeg = 0.0f;
    float m_pitchDeg = -15.0f;
    QPoint m_lastMouse;
    Qt::MouseButton m_dragButton = Qt::NoButton;

    bool m_showPoints = true;
    bool m_showMesh   = true;
    bool m_showTraj   = true;
};
