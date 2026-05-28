#pragma once

#include <vector>
#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <fstream>
#include <map>

typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Matrix<unsigned char, 3, 1> Vec3b;

class MarchingCubes
{
public:
    MarchingCubes(const Vec3i &dimensions, const Vec3 &size);

    ~MarchingCubes();

    bool computeIsoSurface(const float* tsdf, const unsigned char* red, const unsigned char* green, const unsigned char* blue, float isoValue = 0.0f);

    bool savePly(const std::string &filename) const;

protected:
    inline int computeLutIndex(int i, int j, int k, float isoValue);

    Vec3 interpolate(float tsdf0, float tsdf1, const Vec3 &val0, const Vec3 &val1, float isoValue);

    Vec3 getVertex(int i1, int j1, int k1, int i2, int j2, int k2, float isoValue);

    Vec3b getColor(int x1, int y1, int z1, int x2, int y2, int z2, float isoValue);

    void computeTriangles(int cubeIndex,
                            const Vec3 edgePoints[12], const Vec3b edgeColors[12],
                            std::vector<Vec3> &outVerts,
                            std::vector<Vec3b> &outColors,
                            std::vector<Vec3i> &outFaces);

    inline unsigned int addVertex(const Vec3 &v, const Vec3b &c,
                                  std::vector<Vec3> &outVerts,
                                  std::vector<Vec3b> &outColors);

    Vec3 voxelToWorld(int i, int j, int k) const;

    Vec3i m_dim;
    Vec3 m_size;
    Vec3 m_voxelSize;

    const float* m_tsdf;
    const unsigned char* m_red;
    const unsigned char* m_green;
    const unsigned char* m_blue;
public:
    std::vector<Vec3> m_vertices;
    std::vector<Vec3b> m_colors;
    std::vector<Vec3i> m_faces;
};
