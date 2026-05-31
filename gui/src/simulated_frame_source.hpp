#pragma once

#include "frame_source.hpp"
#include "scan_parameters.hpp"

#include <QVector>
#include <QString>
#include <vector>

class SimulatedFrameSource : public FrameSource {
public:
    explicit SimulatedFrameSource(const ScanParameters &params);

    bool updateFrames() override;
    void getDepthMM(cv::Mat &output) override;
    void getRgbMapped2Depth(cv::Mat &output) override;
    void getVideo(cv::Mat &output) override;
    CameraIntrinsics intrinsics() const override { return m_intrinsics; }
    void setCameraPose(const Eigen::Matrix4f &pose) override;

    QString sourceName() const { return m_sourceName; }
    void exportPreviewMesh(QVector<float> &vertices,
                           QVector<unsigned char> &colors,
                           QVector<unsigned int> &indices) const;

private:
    struct Vec3 {
        Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
        Vec3(float ax, float ay, float az) : x(ax), y(ay), z(az) {}
        float x, y, z;
    };

    struct Triangle {
        Vec3 v0, v1, v2;
        Vec3 normal;
        Vec3 color;
    };

    struct Bounds {
        Vec3 mn;
        Vec3 mx;
    };

    struct BvhNode {
        Bounds bounds;
        int left = -1;
        int right = -1;
        int start = 0;
        int count = 0;
    };

    bool loadStl(const std::string &path, std::vector<Triangle> &triangles) const;
    bool loadBinaryStl(const std::string &path, std::vector<Triangle> &triangles) const;
    bool loadAsciiStl(const std::string &path, std::vector<Triangle> &triangles) const;
    void makeDefaultMesh(std::vector<Triangle> &triangles) const;
    void normalizeMesh(std::vector<Triangle> &triangles) const;
    void addTurntable(std::vector<Triangle> &triangles) const;
    void rebuildBvh();
    int buildNode(int start, int count);
    bool intersectNode(int nodeIdx, const Vec3 &origin, const Vec3 &dir,
                       float &bestT, int &bestTri) const;
    bool intersectTriangle(const Triangle &tri, const Vec3 &origin, const Vec3 &dir,
                           float &t) const;
    bool intersectBounds(const Bounds &b, const Vec3 &origin, const Vec3 &dir,
                         float maxT) const;
    void render();
    float noiseFor(int x, int y, int frame, float amplitude) const;

    static Vec3 add(Vec3 a, Vec3 b);
    static Vec3 sub(Vec3 a, Vec3 b);
    static Vec3 mul(Vec3 a, float s);
    static Vec3 cross(Vec3 a, Vec3 b);
    static float dot(Vec3 a, Vec3 b);
    static Vec3 normalize(Vec3 a);
    static Bounds triangleBounds(const Triangle &t);
    static void grow(Bounds &b, Vec3 p);
    static void grow(Bounds &b, const Bounds &other);

    ScanParameters m_params;
    CameraIntrinsics m_intrinsics;
    Eigen::Matrix4f m_cameraPose = Eigen::Matrix4f::Identity();
    std::vector<Triangle> m_triangles;
    std::vector<int> m_indices;
    std::vector<BvhNode> m_nodes;
    cv::Mat m_depthMM;
    cv::Mat m_rgb;
    int m_frame = 0;
    QString m_sourceName;
};
