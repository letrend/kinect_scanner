#include "simulated_frame_source.hpp"

#include <QFileInfo>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>

namespace {
const float kEps = 1e-6f;
const float kBoundsPad = 1e-4f;

float degToRad(float d) {
    return d * 3.14159265358979323846f / 180.0f;
}
}

SimulatedFrameSource::SimulatedFrameSource(const ScanParameters &params)
    : m_params(params) {
    m_intrinsics.width = 512;
    m_intrinsics.height = 424;
    m_intrinsics.fx = 365.456f;
    m_intrinsics.fy = 365.456f;
    m_intrinsics.cx = 254.878f;
    m_intrinsics.cy = 205.395f;

    std::vector<Triangle> mesh;
    const bool loaded = !params.simStlPath.empty() && loadStl(params.simStlPath, mesh);
    if (!loaded) {
        makeDefaultMesh(mesh);
        m_sourceName = params.simStlPath.empty()
                ? QStringLiteral("procedural default")
                : QStringLiteral("procedural default (failed to load %1)")
                    .arg(QString::fromStdString(params.simStlPath));
    } else {
        m_sourceName = QFileInfo(QString::fromStdString(params.simStlPath)).fileName();
    }

    normalizeMesh(mesh);
    if (m_params.simRenderTurntable)
        addTurntable(mesh);
    m_triangles = mesh;
    rebuildBvh();

    m_depthMM = cv::Mat(m_intrinsics.height, m_intrinsics.width, CV_32FC1);
    m_rgb = cv::Mat(m_intrinsics.height, m_intrinsics.width, CV_32FC3);
}

bool SimulatedFrameSource::updateFrames() {
    render();
    ++m_frame;
    return true;
}

void SimulatedFrameSource::getDepthMM(cv::Mat &output) {
    m_depthMM.copyTo(output);
}

void SimulatedFrameSource::getRgbMapped2Depth(cv::Mat &output) {
    m_rgb.copyTo(output);
}

void SimulatedFrameSource::getVideo(cv::Mat &output) {
    m_rgb.convertTo(output, CV_8UC3, 255.0);
}

void SimulatedFrameSource::setCameraPose(const Eigen::Matrix4f &pose) {
    m_cameraPose = pose;
}

void SimulatedFrameSource::exportPreviewMesh(QVector<float> &vertices,
                                             QVector<unsigned char> &colors,
                                             QVector<unsigned int> &indices) const {
    vertices.clear();
    colors.clear();
    indices.clear();
    vertices.reserve((int)m_triangles.size() * 9);
    colors.reserve((int)m_triangles.size() * 9);
    indices.reserve((int)m_triangles.size() * 3);

    unsigned int idx = 0;
    for (const Triangle &t : m_triangles) {
        const Vec3 verts[3] = {t.v0, t.v1, t.v2};
        for (const Vec3 &v : verts) {
            vertices.append(v.x);
            vertices.append(v.y);
            vertices.append(v.z);
            colors.append((unsigned char)std::max(0, std::min(255, int(t.color.x * 255.0f))));
            colors.append((unsigned char)std::max(0, std::min(255, int(t.color.y * 255.0f))));
            colors.append((unsigned char)std::max(0, std::min(255, int(t.color.z * 255.0f))));
        }
        indices.append(idx++);
        indices.append(idx++);
        indices.append(idx++);
    }
}

bool SimulatedFrameSource::loadStl(const std::string &path, std::vector<Triangle> &triangles) const {
    if (loadBinaryStl(path, triangles))
        return true;
    triangles.clear();
    return loadAsciiStl(path, triangles);
}

bool SimulatedFrameSource::loadBinaryStl(const std::string &path, std::vector<Triangle> &triangles) const {
    std::ifstream f(path.c_str(), std::ios::binary | std::ios::ate);
    if (!f) return false;
    const std::streamoff size = f.tellg();
    if (size < 84) return false;
    f.seekg(80, std::ios::beg);
    uint32_t count = 0;
    f.read(reinterpret_cast<char*>(&count), sizeof(count));
    const std::streamoff expected = 84 + static_cast<std::streamoff>(count) * 50;
    if (expected != size || count == 0) return false;

    triangles.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        float n[3], v[9];
        uint16_t attr = 0;
        f.read(reinterpret_cast<char*>(n), sizeof(n));
        f.read(reinterpret_cast<char*>(v), sizeof(v));
        f.read(reinterpret_cast<char*>(&attr), sizeof(attr));
        if (!f) return false;
        Triangle t;
        t.v0 = {v[0], v[1], v[2]};
        t.v1 = {v[3], v[4], v[5]};
        t.v2 = {v[6], v[7], v[8]};
        t.normal = normalize(cross(sub(t.v1, t.v0), sub(t.v2, t.v0)));
        t.color = {0.70f, 0.74f, 0.82f};
        triangles.push_back(t);
    }
    return !triangles.empty();
}

bool SimulatedFrameSource::loadAsciiStl(const std::string &path, std::vector<Triangle> &triangles) const {
    std::ifstream f(path.c_str());
    if (!f) return false;
    std::string token;
    std::vector<Vec3> verts;
    while (f >> token) {
        if (token == "vertex") {
            Vec3 v;
            f >> v.x >> v.y >> v.z;
            verts.push_back(v);
            if (verts.size() == 3) {
                Triangle t;
                t.v0 = verts[0];
                t.v1 = verts[1];
                t.v2 = verts[2];
                t.normal = normalize(cross(sub(t.v1, t.v0), sub(t.v2, t.v0)));
                t.color = {0.70f, 0.74f, 0.82f};
                triangles.push_back(t);
                verts.clear();
            }
        }
    }
    return !triangles.empty();
}

void SimulatedFrameSource::makeDefaultMesh(std::vector<Triangle> &triangles) const {
    const float sx = 80.0f;
    const float sy = 70.0f;
    const float h = 140.0f;
    const Vec3 v[] = {
        {-sx,-sy,0}, { sx,-sy,0}, { sx, sy,0}, {-sx, sy,0},
        {-sx,-sy,h}, { sx,-sy,h}, { sx, sy,h}, {-sx, sy,h}
    };
    const int faces[][3] = {
        {0,1,2},{0,2,3}, {4,6,5},{4,7,6},
        {0,4,5},{0,5,1}, {1,5,6},{1,6,2},
        {2,6,7},{2,7,3}, {3,7,4},{3,4,0}
    };
    for (const auto &face : faces) {
        Triangle t;
        t.v0 = v[face[0]];
        t.v1 = v[face[1]];
        t.v2 = v[face[2]];
        t.normal = normalize(cross(sub(t.v1, t.v0), sub(t.v2, t.v0)));
        t.color = {0.75f, 0.68f, 0.56f};
        triangles.push_back(t);
    }
}

void SimulatedFrameSource::normalizeMesh(std::vector<Triangle> &triangles) const {
    if (triangles.empty()) return;
    Bounds b;
    b.mn = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    b.mx = {-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max() };
    for (const Triangle &t : triangles) {
        grow(b, t.v0);
        grow(b, t.v1);
        grow(b, t.v2);
    }

    const float cx = 0.5f * (b.mn.x + b.mx.x);
    const float cy = 0.5f * (b.mn.y + b.mx.y);
    const float minZ = b.mn.z;
    const float scale = m_params.simStlScale * 0.001f;
    for (Triangle &t : triangles) {
        Vec3 *vs[] = {&t.v0, &t.v1, &t.v2};
        for (Vec3 *v : vs) {
            const float rawX = v->x;
            const float rawY = v->y;
            const float rawZ = v->z;
            v->x = (rawX - cx) * scale;
            v->y = -(rawZ - minZ) * scale;
            v->z = (rawY - cy) * scale;
        }
        t.normal = normalize(cross(sub(t.v1, t.v0), sub(t.v2, t.v0)));
        const float h = std::max(0.0f, std::min(1.0f, -0.5f * (t.v0.y + t.v1.y + t.v2.y)));
        t.color = {0.55f + 0.25f * h, 0.62f + 0.20f * h, 0.78f};
    }
}

void SimulatedFrameSource::addTurntable(std::vector<Triangle> &triangles) const {
    const int segments = 64;
    const float r = m_params.turntableRadiusMm * 0.001f;
    const float h = m_params.turntableHeightMm * 0.001f;
    const Vec3 topCenter{0, 0, 0};
    const Vec3 bottomCenter{0, h, 0};
    for (int i = 0; i < segments; ++i) {
        const float a0 = 2.0f * 3.14159265358979323846f * float(i) / float(segments);
        const float a1 = 2.0f * 3.14159265358979323846f * float(i + 1) / float(segments);
        const Vec3 t0{r * std::cos(a0), 0, r * std::sin(a0)};
        const Vec3 t1{r * std::cos(a1), 0, r * std::sin(a1)};
        const Vec3 b0{t0.x, h, t0.z};
        const Vec3 b1{t1.x, h, t1.z};
        Triangle top{topCenter, t1, t0, {0, -1, 0}, {0.35f, 0.35f, 0.37f}};
        Triangle side0{t0, t1, b1, normalize(cross(sub(t1, t0), sub(b1, t0))), {0.25f, 0.25f, 0.26f}};
        Triangle side1{t0, b1, b0, normalize(cross(sub(b1, t0), sub(b0, t0))), {0.25f, 0.25f, 0.26f}};
        Triangle bottom{bottomCenter, b0, b1, {0, 1, 0}, {0.18f, 0.18f, 0.19f}};
        triangles.push_back(top);
        triangles.push_back(side0);
        triangles.push_back(side1);
        triangles.push_back(bottom);
    }
}

void SimulatedFrameSource::rebuildBvh() {
    m_indices.resize(m_triangles.size());
    for (int i = 0; i < (int)m_indices.size(); ++i)
        m_indices[i] = i;
    m_nodes.clear();
    if (!m_indices.empty())
        buildNode(0, (int)m_indices.size());
}

int SimulatedFrameSource::buildNode(int start, int count) {
    BvhNode node;
    node.start = start;
    node.count = count;
    node.bounds.mn = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    node.bounds.mx = {-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max() };
    Bounds centroidBounds = node.bounds;
    for (int i = start; i < start + count; ++i) {
        const Triangle &t = m_triangles[m_indices[i]];
        grow(node.bounds, triangleBounds(t));
        Vec3 c = mul(add(add(t.v0, t.v1), t.v2), 1.0f / 3.0f);
        grow(centroidBounds, c);
    }
    const int idx = (int)m_nodes.size();
    m_nodes.push_back(node);
    if (count <= 8) return idx;

    Vec3 ext = sub(centroidBounds.mx, centroidBounds.mn);
    int axis = 0;
    if (ext.y > ext.x && ext.y > ext.z) axis = 1;
    else if (ext.z > ext.x) axis = 2;
    const int mid = start + count / 2;
    std::nth_element(m_indices.begin() + start, m_indices.begin() + mid,
                     m_indices.begin() + start + count,
                     [this, axis](int a, int b) {
        const Triangle &ta = m_triangles[a];
        const Triangle &tb = m_triangles[b];
        Vec3 ca = mul(add(add(ta.v0, ta.v1), ta.v2), 1.0f / 3.0f);
        Vec3 cb = mul(add(add(tb.v0, tb.v1), tb.v2), 1.0f / 3.0f);
        return axis == 0 ? ca.x < cb.x : axis == 1 ? ca.y < cb.y : ca.z < cb.z;
    });
    m_nodes[idx].left = buildNode(start, mid - start);
    m_nodes[idx].right = buildNode(mid, start + count - mid);
    m_nodes[idx].count = 0;
    return idx;
}

bool SimulatedFrameSource::intersectNode(int nodeIdx, const Vec3 &origin, const Vec3 &dir,
                                         float &bestT, int &bestTri) const {
    if (nodeIdx < 0) return false;
    const BvhNode &node = m_nodes[nodeIdx];
    if (!intersectBounds(node.bounds, origin, dir, bestT))
        return false;
    bool hit = false;
    if (node.left < 0 && node.right < 0) {
        for (int i = node.start; i < node.start + node.count; ++i) {
            float t = 0.0f;
            int triIdx = m_indices[i];
            if (intersectTriangle(m_triangles[triIdx], origin, dir, t) && t < bestT) {
                bestT = t;
                bestTri = triIdx;
                hit = true;
            }
        }
        return hit;
    }
    hit = intersectNode(node.left, origin, dir, bestT, bestTri) || hit;
    hit = intersectNode(node.right, origin, dir, bestT, bestTri) || hit;
    return hit;
}

bool SimulatedFrameSource::intersectTriangle(const Triangle &tri, const Vec3 &origin,
                                             const Vec3 &dir, float &t) const {
    Vec3 e1 = sub(tri.v1, tri.v0);
    Vec3 e2 = sub(tri.v2, tri.v0);
    Vec3 p = cross(dir, e2);
    float det = dot(e1, p);
    if (std::fabs(det) < kEps) return false;
    float invDet = 1.0f / det;
    Vec3 tv = sub(origin, tri.v0);
    float u = dot(tv, p) * invDet;
    if (u < 0.0f || u > 1.0f) return false;
    Vec3 q = cross(tv, e1);
    float v = dot(dir, q) * invDet;
    if (v < 0.0f || u + v > 1.0f) return false;
    t = dot(e2, q) * invDet;
    return t > (m_params.simRaycastNearMm * 0.001f);
}

bool SimulatedFrameSource::intersectBounds(const Bounds &b, const Vec3 &origin,
                                           const Vec3 &dir, float maxT) const {
    float tmin = 0.0f;
    float tmax = maxT;
    const float o[3] = {origin.x, origin.y, origin.z};
    const float d[3] = {dir.x, dir.y, dir.z};
    const float mn[3] = {b.mn.x - kBoundsPad, b.mn.y - kBoundsPad, b.mn.z - kBoundsPad};
    const float mx[3] = {b.mx.x + kBoundsPad, b.mx.y + kBoundsPad, b.mx.z + kBoundsPad};
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(d[i]) < kEps) {
            if (o[i] < mn[i] || o[i] > mx[i]) return false;
        } else {
            float invD = 1.0f / d[i];
            float t0 = (mn[i] - o[i]) * invD;
            float t1 = (mx[i] - o[i]) * invD;
            if (t0 > t1) std::swap(t0, t1);
            tmin = std::max(tmin, t0);
            tmax = std::min(tmax, t1);
            if (tmax < tmin) return false;
        }
    }
    return true;
}

void SimulatedFrameSource::render() {
    m_depthMM.setTo(0.0f);
    m_rgb.setTo(cv::Scalar(0.02f, 0.02f, 0.025f));
    const Vec3 origin{m_cameraPose(0,3), m_cameraPose(1,3), m_cameraPose(2,3)};
    const float farM = m_params.simRaycastFarMm * 0.001f;
    const float dropout = std::max(0.0f, std::min(100.0f, m_params.simDropoutPercent));

#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < m_intrinsics.height; ++y) {
        for (int x = 0; x < m_intrinsics.width; ++x) {
            const float cx = (float(x) - m_intrinsics.cx) / m_intrinsics.fx;
            const float cy = (float(y) - m_intrinsics.cy) / m_intrinsics.fy;
            Vec3 dirCam{cx, cy, 1.0f};
            Vec3 dir{
                m_cameraPose(0,0)*dirCam.x + m_cameraPose(0,1)*dirCam.y + m_cameraPose(0,2)*dirCam.z,
                m_cameraPose(1,0)*dirCam.x + m_cameraPose(1,1)*dirCam.y + m_cameraPose(1,2)*dirCam.z,
                m_cameraPose(2,0)*dirCam.x + m_cameraPose(2,1)*dirCam.y + m_cameraPose(2,2)*dirCam.z
            };
            float bestT = farM;
            int bestTri = -1;
            bool hit = !m_nodes.empty() && intersectNode(0, origin, dir, bestT, bestTri);
            if (!hit && m_triangles.size() <= 20000) {
                for (int triIdx = 0; triIdx < (int)m_triangles.size(); ++triIdx) {
                    float t = 0.0f;
                    if (intersectTriangle(m_triangles[triIdx], origin, dir, t) && t < bestT) {
                        bestT = t;
                        bestTri = triIdx;
                        hit = true;
                    }
                }
            }
            if (hit) {
                if (dropout > 0.0f) {
                    float r = 50.0f + 50.0f * noiseFor(x, y, m_frame, 1.0f);
                    if (r < dropout) continue;
                }
                float depth = bestT * 1000.0f;
                if (m_params.simDepthNoiseMm > 0.0f)
                    depth += noiseFor(x, y, m_frame, m_params.simDepthNoiseMm);
                m_depthMM.at<float>(y, x) = std::max(0.0f, depth);
                const Triangle &tri = m_triangles[bestTri];
                Vec3 n = normalize(tri.normal);
                Vec3 view = normalize(mul(dir, -1.0f));
                float shade = std::max(0.15f, std::min(1.0f, 0.25f + 0.75f * std::fabs(dot(n, view))));
                float *pix = m_rgb.ptr<float>(y) + 3*x;
                pix[0] = std::min(1.0f, tri.color.z * shade);
                pix[1] = std::min(1.0f, tri.color.y * shade);
                pix[2] = std::min(1.0f, tri.color.x * shade);
            }
        }
    }
}

float SimulatedFrameSource::noiseFor(int x, int y, int frame, float amplitude) const {
    uint32_t h = 2166136261u;
    h = (h ^ uint32_t(x)) * 16777619u;
    h = (h ^ uint32_t(y)) * 16777619u;
    h = (h ^ uint32_t(frame)) * 16777619u;
    float u = float(h & 0xffffu) / 65535.0f;
    return (u * 2.0f - 1.0f) * amplitude;
}

SimulatedFrameSource::Vec3 SimulatedFrameSource::add(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
SimulatedFrameSource::Vec3 SimulatedFrameSource::sub(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
SimulatedFrameSource::Vec3 SimulatedFrameSource::mul(Vec3 a, float s) { return {a.x*s, a.y*s, a.z*s}; }
float SimulatedFrameSource::dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
SimulatedFrameSource::Vec3 SimulatedFrameSource::cross(Vec3 a, Vec3 b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
SimulatedFrameSource::Vec3 SimulatedFrameSource::normalize(Vec3 a) {
    float n = std::sqrt(std::max(kEps, dot(a, a)));
    return mul(a, 1.0f / n);
}
SimulatedFrameSource::Bounds SimulatedFrameSource::triangleBounds(const Triangle &t) {
    Bounds b;
    b.mn = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    b.mx = {-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max() };
    grow(b, t.v0);
    grow(b, t.v1);
    grow(b, t.v2);
    return b;
}
void SimulatedFrameSource::grow(Bounds &b, Vec3 p) {
    b.mn.x = std::min(b.mn.x, p.x); b.mn.y = std::min(b.mn.y, p.y); b.mn.z = std::min(b.mn.z, p.z);
    b.mx.x = std::max(b.mx.x, p.x); b.mx.y = std::max(b.mx.y, p.y); b.mx.z = std::max(b.mx.z, p.z);
}
void SimulatedFrameSource::grow(Bounds &b, const Bounds &other) {
    grow(b, other.mn);
    grow(b, other.mx);
}
