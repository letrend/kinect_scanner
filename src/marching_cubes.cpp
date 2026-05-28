#include "marching_cubes.hpp"

#include <cstdio>
#include <cstdint>
#ifdef _OPENMP
#include <omp.h>
#endif

MarchingCubes::MarchingCubes(const Vec3i &dimensions, const Vec3 &size) :
    m_dim(dimensions),
    m_size(size),
    m_voxelSize(Vec3(m_size.cwiseQuotient(m_dim.cast<double>()))),
    m_tsdf(0),
    m_red(0),
    m_green(0),
    m_blue(0)
{
}


MarchingCubes::~MarchingCubes()
{
}


// To find which edges are intersected by the surface, we find the edges in a (trivial) table.
// Table giving the edges intersected by the surface:
int edgeTable[256] = { 0x0, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
        0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00, 0x190, 0x99,
        0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c, 0x99c, 0x895, 0xb9f, 0xa96,
        0xd9a, 0xc93, 0xf99, 0xe90, 0x230, 0x339, 0x33, 0x13a, 0x636, 0x73f,
        0x435, 0x53c, 0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
        0x3a0, 0x2a9, 0x1a3, 0xaa, 0x7a6, 0x6af, 0x5a5, 0x4ac, 0xbac, 0xaa5,
        0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0, 0x460, 0x569, 0x663, 0x76a,
        0x66, 0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963,
        0xa69, 0xb60, 0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff, 0x3f5, 0x2fc,
        0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0, 0x650, 0x759,
        0x453, 0x55a, 0x256, 0x35f, 0x55, 0x15c, 0xe5c, 0xf55, 0xc5f, 0xd56,
        0xa5a, 0xb53, 0x859, 0x950, 0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf,
        0x1c5, 0xcc, 0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
        0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc, 0xcc, 0x1c5,
        0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0, 0x950, 0x859, 0xb53, 0xa5a,
        0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x55, 0x35f, 0x256, 0x55a, 0x453,
        0x759, 0x650, 0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
        0x2fc, 0x3f5, 0xff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0, 0xb60, 0xa69,
        0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x36c, 0x265, 0x16f, 0x66,
        0x76a, 0x663, 0x569, 0x460, 0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af,
        0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa, 0x1a3, 0x2a9, 0x3a0,
        0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 0x53c, 0x435,
        0x73f, 0x636, 0x13a, 0x33, 0x339, 0x230, 0xe90, 0xf99, 0xc93, 0xd9a,
        0xa96, 0xb9f, 0x895, 0x99c, 0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393,
        0x99, 0x190, 0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
        0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0 };


// This table gives the edges forming triangles for the surface.
int triTable[256][16] = { { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1 }, { 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1 },
        { 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1,
                8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1,
                2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, {
                0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, {
                9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, {
                2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1 }, { 3,
                11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, {
                0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, {
                1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, {
                1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1 }, {
                3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
        { 0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1 }, { 3, 9,
                0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1 }, { 9, 8,
                10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 4,
                7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, {
                4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, {
                0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, {
                4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1 }, { 1,
                2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 3,
                4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1 }, { 9, 2,
                10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 }, { 2, 10, 9,
                2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1 }, { 8, 4, 7, 3, 11,
                2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 11, 4, 7, 11, 2,
                4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1 }, { 9, 0, 1, 8, 4, 7,
                2, 3, 11, -1, -1, -1, -1, -1, -1, -1 }, { 4, 7, 11, 9, 4, 11,
                9, 11, 2, 9, 2, 1, -1, -1, -1, -1 }, { 3, 10, 1, 3, 11, 10, 7,
                8, 4, -1, -1, -1, -1, -1, -1, -1 }, { 1, 11, 10, 1, 4, 11, 1,
                0, 4, 7, 11, 4, -1, -1, -1, -1 }, { 4, 7, 8, 9, 0, 11, 9, 11,
                10, 11, 0, 3, -1, -1, -1, -1 }, { 4, 7, 11, 4, 11, 9, 9, 11,
                10, -1, -1, -1, -1, -1, -1, -1 }, { 9, 5, 4, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 9, 5, 4, 0, 8, 3, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 5, 4, 1, 5, 0, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 8, 5, 4, 8, 3, 5, 3, 1,
                5, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 10, 9, 5, 4, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1 }, { 3, 0, 8, 1, 2, 10, 4, 9, 5,
                -1, -1, -1, -1, -1, -1, -1 }, { 5, 2, 10, 5, 4, 2, 4, 0, 2, -1,
                -1, -1, -1, -1, -1, -1 }, { 2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4,
                8, -1, -1, -1, -1 }, { 9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1 }, { 0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1,
                -1, -1, -1, -1, -1 }, { 0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1,
                -1, -1, -1, -1 }, { 2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1,
                -1, -1, -1 }, { 10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1,
                -1, -1, -1 }, { 4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1,
                -1, -1 }, { 5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1,
                -1 }, { 5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1,
                -1 }, { 9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1 },
        { 9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1 }, { 0, 7, 8,
                0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1 }, { 1, 5, 3, 3,
                5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 9, 7, 8, 9,
                5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1 }, { 10, 1, 2, 9, 5,
                0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1 }, { 8, 0, 2, 8, 2, 5, 8,
                5, 7, 10, 5, 2, -1, -1, -1, -1 }, { 2, 10, 5, 2, 5, 3, 3, 5, 7,
                -1, -1, -1, -1, -1, -1, -1 }, { 7, 9, 5, 7, 8, 9, 3, 11, 2, -1,
                -1, -1, -1, -1, -1, -1 }, { 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7,
                11, -1, -1, -1, -1 }, { 2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7,
                -1, -1, -1, -1 }, { 11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1,
                -1, -1, -1, -1 }, { 9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1,
                -1, -1, -1 }, { 5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10,
                0, -1 },
        { 11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1 }, { 11, 10, 5,
                7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 10, 6, 5,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 8,
                3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 9, 0,
                1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 8,
                3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 }, { 1, 6, 5,
                2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 6, 5,
                1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1 }, { 9, 6, 5, 9,
                0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1 }, { 5, 9, 8, 5, 8,
                2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1 }, { 2, 3, 11, 10, 6, 5,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 11, 0, 8, 11, 2, 0,
                10, 6, 5, -1, -1, -1, -1, -1, -1, -1 }, { 0, 1, 9, 2, 3, 11, 5,
                10, 6, -1, -1, -1, -1, -1, -1, -1 }, { 5, 10, 6, 1, 9, 2, 9,
                11, 2, 9, 8, 11, -1, -1, -1, -1 }, { 6, 3, 11, 6, 5, 3, 5, 1,
                3, -1, -1, -1, -1, -1, -1, -1 }, { 0, 8, 11, 0, 11, 5, 0, 5, 1,
                5, 11, 6, -1, -1, -1, -1 }, { 3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5,
                9, -1, -1, -1, -1 }, { 6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1,
                -1, -1, -1, -1 }, { 5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1 }, { 4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1,
                -1, -1, -1 }, { 1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1,
                -1, -1 },
        { 10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1 }, { 6, 1, 2, 6,
                5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 5, 5, 2,
                6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1 }, { 8, 4, 7, 9, 0, 5, 0,
                6, 5, 0, 2, 6, -1, -1, -1, -1 }, { 7, 3, 9, 7, 9, 4, 3, 2, 9,
                5, 9, 6, 2, 6, 9, -1 }, { 3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1,
                -1, -1, -1, -1, -1 }, { 5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11,
                -1, -1, -1, -1 }, { 0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1,
                -1, -1, -1 }, { 9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10,
                6, -1 },
        { 8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1 }, { 5, 1, 11,
                5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1 }, { 0, 5, 9, 0, 6,
                5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1 }, { 6, 5, 9, 6, 9, 11, 4, 7,
                9, 7, 11, 9, -1, -1, -1, -1 }, { 10, 4, 9, 6, 4, 10, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1 }, { 4, 10, 6, 4, 9, 10, 0, 8,
                3, -1, -1, -1, -1, -1, -1, -1 }, { 10, 0, 1, 10, 6, 0, 6, 4, 0,
                -1, -1, -1, -1, -1, -1, -1 }, { 8, 3, 1, 8, 1, 6, 8, 6, 4, 6,
                1, 10, -1, -1, -1, -1 }, { 1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1,
                -1, -1, -1, -1, -1 }, { 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1,
                -1, -1, -1 }, { 0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1 }, { 8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1,
                -1, -1 }, { 10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1,
                -1, -1 }, { 0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1,
                -1 }, { 3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
        { 6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1 }, { 9, 6, 4, 9,
                3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1 }, { 8, 11, 1, 8, 1, 0,
                11, 6, 1, 9, 1, 4, 6, 4, 1, -1 }, { 3, 11, 6, 3, 6, 0, 0, 6, 4,
                -1, -1, -1, -1, -1, -1, -1 }, { 6, 4, 8, 11, 6, 8, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1 }, { 7, 10, 6, 7, 8, 10, 8, 9, 10,
                -1, -1, -1, -1, -1, -1, -1 }, { 0, 7, 3, 0, 10, 7, 0, 9, 10, 6,
                7, 10, -1, -1, -1, -1 }, { 10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8,
                0, -1, -1, -1, -1 }, { 10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1,
                -1, -1, -1, -1 }, { 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1,
                -1, -1 }, { 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1 },
        { 7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1 }, { 7, 3, 2,
                6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 2, 3, 11,
                10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1 }, { 2, 0, 7, 2, 7,
                11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1 }, { 1, 8, 0, 1, 7, 8, 1,
                10, 7, 6, 7, 10, 2, 3, 11, -1 }, { 11, 2, 1, 11, 1, 7, 10, 6,
                1, 6, 7, 1, -1, -1, -1, -1 }, { 8, 9, 6, 8, 6, 7, 9, 1, 6, 11,
                6, 3, 1, 3, 6, -1 }, { 0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1 }, { 7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0,
                -1, -1, -1, -1 }, { 7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1 }, { 7, 6, 11, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1 }, { 3, 0, 8, 11, 7, 6, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1 }, { 0, 1, 9, 11, 7, 6, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1 }, { 8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1,
                -1, -1, -1, -1, -1 }, { 10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1 }, { 1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1,
                -1, -1, -1, -1, -1 }, { 2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1,
                -1, -1, -1, -1, -1 }, { 6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8,
                -1, -1, -1, -1 }, { 7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1 }, { 7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1,
                -1, -1, -1 }, { 2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1,
                -1, -1 },
        { 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1 }, { 10, 7, 6, 10,
                1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1 }, { 10, 7, 6, 1, 7,
                10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1 }, { 0, 3, 7, 0, 7, 10, 0,
                10, 9, 6, 10, 7, -1, -1, -1, -1 }, { 7, 6, 10, 7, 10, 8, 8, 10,
                9, -1, -1, -1, -1, -1, -1, -1 }, { 6, 8, 4, 11, 8, 6, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1 }, { 3, 6, 11, 3, 0, 6, 0, 4, 6,
                -1, -1, -1, -1, -1, -1, -1 }, { 8, 6, 11, 8, 4, 6, 9, 0, 1, -1,
                -1, -1, -1, -1, -1, -1 }, { 9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3,
                6, -1, -1, -1, -1 }, { 6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1,
                -1, -1, -1, -1 }, { 1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1,
                -1, -1, -1 }, { 4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1,
                -1, -1 },
        { 10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1 }, { 8, 2, 3, 8,
                4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1 }, { 0, 4, 2, 4, 6,
                2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 9, 0, 2, 3,
                4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1 }, { 1, 9, 4, 1, 4, 2, 2,
                4, 6, -1, -1, -1, -1, -1, -1, -1 }, { 8, 1, 3, 8, 6, 1, 8, 4,
                6, 6, 10, 1, -1, -1, -1, -1 }, { 10, 1, 0, 10, 0, 6, 6, 0, 4,
                -1, -1, -1, -1, -1, -1, -1 }, { 4, 6, 3, 4, 3, 8, 6, 10, 3, 0,
                3, 9, 10, 9, 3, -1 }, { 10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1 }, { 4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1 }, { 0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1,
                -1, -1, -1, -1 }, { 5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1,
                -1, -1, -1 }, { 11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1,
                -1, -1 }, { 9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1,
                -1, -1 }, { 6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1,
                -1 },
        { 7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1 }, { 3, 4, 8,
                3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1 }, { 7, 2, 3, 7, 6, 2,
                5, 4, 9, -1, -1, -1, -1, -1, -1, -1 }, { 9, 5, 4, 0, 8, 6, 0,
                6, 2, 6, 8, 7, -1, -1, -1, -1 }, { 3, 6, 2, 3, 7, 6, 1, 5, 0,
                5, 4, 0, -1, -1, -1, -1 }, { 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8,
                5, 1, 5, 8, -1 }, { 9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1,
                -1, -1, -1 }, { 1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4,
                -1 }, { 4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1 },
        { 7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1 }, { 6, 9, 5,
                6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1 }, { 3, 6, 11,
                0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1 }, { 0, 11, 8, 0, 5,
                11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1 }, { 6, 11, 3, 6, 3, 5,
                5, 3, 1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 10, 9, 5, 11, 9,
                11, 8, 11, 5, 6, -1, -1, -1, -1 }, { 0, 11, 3, 0, 6, 11, 0, 9,
                6, 5, 6, 9, 1, 2, 10, -1 }, { 11, 8, 5, 11, 5, 6, 8, 0, 5, 10,
                5, 2, 0, 2, 5, -1 }, { 6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3,
                -1, -1, -1, -1 }, { 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1,
                -1, -1 }, { 9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1,
                -1 }, { 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1 }, { 1,
                5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1,
                3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1 }, { 10, 1, 0,
                10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1 }, { 0, 3, 8, 5, 6,
                10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 10, 5, 6, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 11, 5, 10,
                7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 11, 5,
                10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1 }, { 5, 11,
                7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1 }, { 10, 7,
                5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1 }, { 11, 1, 2,
                11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 8, 3, 1,
                2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1 }, { 9, 7, 5, 9, 2, 7,
                9, 0, 2, 2, 11, 7, -1, -1, -1, -1 }, { 7, 5, 2, 7, 2, 11, 5, 9,
                2, 3, 2, 8, 9, 8, 2, -1 }, { 2, 5, 10, 2, 3, 5, 3, 7, 5, -1,
                -1, -1, -1, -1, -1, -1 }, { 8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2,
                5, -1, -1, -1, -1 }, { 9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2,
                -1, -1, -1, -1 }, { 9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5,
                2, -1 }, { 1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1 }, { 0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1,
                -1 },
        { 9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1 }, { 9, 8, 7,
                5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 5, 8, 4,
                5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1 }, { 5, 0, 4,
                5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1 }, { 0, 1, 9, 8,
                4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1 }, { 10, 11, 4, 10,
                4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1 }, { 2, 5, 1, 2, 8, 5, 2,
                11, 8, 4, 5, 8, -1, -1, -1, -1 }, { 0, 4, 11, 0, 11, 3, 4, 5,
                11, 2, 11, 1, 5, 1, 11, -1 }, { 0, 2, 5, 0, 5, 9, 2, 11, 5, 4,
                5, 8, 11, 8, 5, -1 }, { 9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1 }, { 2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4,
                -1, -1, -1, -1 }, { 5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1,
                -1, -1, -1 }, { 3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9,
                -1 }, { 5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1 },
        { 8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 4, 5,
                1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 8, 4, 5,
                8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1 }, { 9, 4, 5, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 4, 11, 7, 4, 9,
                11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1 }, { 0, 8, 3, 4, 9,
                7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1 }, { 1, 10, 11, 1, 11,
                4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1 }, { 3, 1, 4, 3, 4, 8, 1,
                10, 4, 7, 4, 11, 10, 11, 4, -1 }, { 4, 11, 7, 9, 11, 4, 9, 2,
                11, 9, 1, 2, -1, -1, -1, -1 }, { 9, 7, 4, 9, 11, 7, 9, 1, 11,
                2, 11, 1, 0, 8, 3, -1 }, { 11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1,
                -1, -1, -1, -1, -1 }, { 11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4,
                -1, -1, -1, -1 }, { 2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1,
                -1, -1, -1 }, { 9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7,
                -1 }, { 3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1 },
        { 1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 4, 9,
                1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1 }, { 4, 9, 1,
                4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1 }, { 4, 0, 3, 7, 4,
                3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 4, 8, 7, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 9, 10, 8, 10,
                11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 3, 0, 9, 3,
                9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1 }, { 0, 1, 10, 0,
                10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1 }, { 3, 1, 10, 11,
                3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 2, 11, 1,
                11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1 }, { 3, 0, 9, 3, 9,
                11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1 }, { 0, 2, 11, 8, 0, 11,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 3, 2, 11, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 2, 3, 8, 2, 8,
                10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1 }, { 9, 10, 2, 0, 9,
                2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 2, 3, 8, 2, 8,
                10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1 }, { 1, 10, 2, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 1, 3, 8, 9, 1,
                8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 9, 1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { 0, 3, 8, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }, { -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } };


bool MarchingCubes::computeIsoSurface(const float* tsdf, const unsigned char* red, const unsigned char* green, const unsigned char* blue, float isoValue)
{
    if (!tsdf || !red || !green || !blue)
        return false;

    m_tsdf = tsdf;
    m_red = red;
    m_green = green;
    m_blue = blue;

    m_vertices.clear();
    m_colors.clear();
    m_faces.clear();

    // Each OpenMP thread accumulates into its own private buffers; we
    // concatenate at the end with per-thread vertex-index offsets so that
    // faces correctly reference the merged vertex array. This avoids any
    // contention on m_vertices/m_faces during the inner loops.
    const int zMax = m_dim[2] - 2;
    const int yMax = m_dim[1] - 2;
    const int xMax = m_dim[0] - 2;

#ifdef _OPENMP
    const int nThreads = std::max(1, omp_get_max_threads());
#else
    const int nThreads = 1;
#endif
    std::vector<std::vector<Vec3>>  tVerts(nThreads);
    std::vector<std::vector<Vec3b>> tColors(nThreads);
    std::vector<std::vector<Vec3i>> tFaces(nThreads);

    #pragma omp parallel
    {
#ifdef _OPENMP
        const int tid = omp_get_thread_num();
#else
        const int tid = 0;
#endif
        std::vector<Vec3>  &outVerts  = tVerts[tid];
        std::vector<Vec3b> &outColors = tColors[tid];
        std::vector<Vec3i> &outFaces  = tFaces[tid];

        Vec3  edgePoints[12];
        Vec3b edgeColors[12];

        #pragma omp for schedule(dynamic, 4) nowait
        for (int z = 0; z < zMax; z++)
        {
            for (int y = 0; y < yMax; y++)
            {
                for (int x = 0; x < xMax; x++)
                {
                    int cubeindex = computeLutIndex(x, y, z, isoValue);
                    if (cubeindex == 0 || cubeindex == 255)
                        continue;

                    if (edgeTable[cubeindex] & 1)
                        edgePoints[0] = getVertex(x + 1, y + 1, z, x + 1, y, z, isoValue),
                        edgeColors[0] = getColor(x + 1, y + 1, z, x + 1, y, z, isoValue);
                    if (edgeTable[cubeindex] & 2)
                        edgePoints[1] = getVertex(x + 1, y, z, x, y, z, isoValue),
                        edgeColors[1] = getColor(x + 1, y, z, x, y, z, isoValue);
                    if (edgeTable[cubeindex] & 4)
                        edgePoints[2] = getVertex(x, y, z, x, y + 1, z, isoValue),
                        edgeColors[2] = getColor(x, y + 1, z, x, y, z, isoValue);
                    if (edgeTable[cubeindex] & 8)
                        edgePoints[3] = getVertex(x, y + 1, z, x + 1, y + 1, z, isoValue),
                        edgeColors[3] = getColor(x + 1, y + 1, z, x, y + 1, z, isoValue);
                    if (edgeTable[cubeindex] & 16)
                        edgePoints[4] = getVertex(x + 1, y + 1, z + 1, x + 1, y, z + 1, isoValue),
                        edgeColors[4] = getColor(x + 1, y + 1, z + 1, x + 1, y, z + 1, isoValue);
                    if (edgeTable[cubeindex] & 32)
                        edgePoints[5] = getVertex(x + 1, y, z + 1, x, y, z + 1, isoValue),
                        edgeColors[5] = getColor(x + 1, y, z + 1, x, y, z + 1, isoValue);
                    if (edgeTable[cubeindex] & 64)
                        edgePoints[6] = getVertex(x, y, z + 1, x, y + 1, z + 1, isoValue),
                        edgeColors[6] = getColor(x, y + 1, z + 1, x, y, z + 1, isoValue);
                    if (edgeTable[cubeindex] & 128)
                        edgePoints[7] = getVertex(x, y + 1, z + 1, x + 1, y + 1, z + 1, isoValue),
                        edgeColors[7] = getColor(x + 1, y + 1, z + 1, x, y + 1, z + 1, isoValue);
                    if (edgeTable[cubeindex] & 256)
                        edgePoints[8] = getVertex(x + 1, y + 1, z, x + 1, y + 1, z + 1, isoValue),
                        edgeColors[8] = getColor(x + 1, y + 1, z, x + 1, y + 1, z + 1, isoValue);
                    if (edgeTable[cubeindex] & 512)
                        edgePoints[9] = getVertex(x + 1, y, z, x + 1, y, z + 1, isoValue),
                        edgeColors[9] = getColor(x + 1, y, z, x + 1, y, z + 1, isoValue);
                    if (edgeTable[cubeindex] & 1024)
                        edgePoints[10] = getVertex(x, y, z, x, y, z + 1, isoValue),
                        edgeColors[10] = getColor(x, y, z, x, y, z + 1, isoValue);
                    if (edgeTable[cubeindex] & 2048)
                        edgePoints[11] = getVertex(x, y + 1, z, x, y + 1, z + 1, isoValue),
                        edgeColors[11] = getColor(x, y + 1, z, x, y + 1, z + 1, isoValue);

                    computeTriangles(cubeindex, edgePoints, edgeColors,
                                     outVerts, outColors, outFaces);
                }
            }
        }
    } // omp parallel

    // Merge: copy each thread's vertices/colors and offset its face indices.
    size_t totalV = 0, totalF = 0;
    for (int t = 0; t < nThreads; ++t) { totalV += tVerts[t].size(); totalF += tFaces[t].size(); }
    m_vertices.reserve(totalV);
    m_colors.reserve(totalV);
    m_faces.reserve(totalF);

    for (int t = 0; t < nThreads; ++t) {
        const unsigned int base = (unsigned int)m_vertices.size();
        m_vertices.insert(m_vertices.end(), tVerts[t].begin(),  tVerts[t].end());
        m_colors  .insert(m_colors  .end(), tColors[t].begin(), tColors[t].end());
        if (base == 0) {
            m_faces.insert(m_faces.end(), tFaces[t].begin(), tFaces[t].end());
        } else {
            for (const Vec3i &f : tFaces[t])
                m_faces.emplace_back(f[0] + (int)base, f[1] + (int)base, f[2] + (int)base);
        }
    }

    return true;
}


// Finding which vertices are below zero.
// Input: Integers i,j and k
// Output: A vector with integer with value between 0 and 256.
inline int MarchingCubes::computeLutIndex(int i, int j, int k, float isoValue)
{
    int cubeindex = 0;
    int offZ = m_dim[0] * m_dim[1];
    int offY = m_dim[0];
    if (m_tsdf[k * offZ + (j + 1) * offY + (i + 1)] > isoValue)
        cubeindex |= 1;
    if (m_tsdf[k * offZ + j * offY + (i + 1)] > isoValue)
        cubeindex |= 2;
    if (m_tsdf[k * offZ + j * offY + i] > isoValue)
        cubeindex |= 4;
    if (m_tsdf[k * offZ + (j + 1) * offY + i] > isoValue)
        cubeindex |= 8;
    if (m_tsdf[(k + 1) * offZ + (j + 1) * offY + i + 1] > isoValue)
        cubeindex |= 16;
    if (m_tsdf[(k + 1) * offZ + j * offY + i + 1] > isoValue)
        cubeindex |= 32;
    if (m_tsdf[(k + 1) * offZ + j * offY + i] > isoValue)
        cubeindex |= 64;
    if (m_tsdf[(k + 1) * offZ + (j + 1) * offY + i] > isoValue)
        cubeindex |= 128;
    return cubeindex;
}


Vec3 MarchingCubes::interpolate(float tsdf0, float tsdf1, const Vec3 &val0, const Vec3 &val1, float isoValue)
{
    if (std::fabs(isoValue - tsdf0) < 1e-7)
        return val0;
    if (std::fabs(isoValue - tsdf1) < 1e-7)
        return val1;
    if (std::fabs(tsdf0 - tsdf1) < 1e-7)
        return val0;

    double mu = (isoValue - tsdf0) / (tsdf1 - tsdf0);
    if(mu > 1.0)
        mu = 1.0;
    else if (mu < 0)
        mu = 0.0;

    Vec3 val;
    val[0] = val0[0] + mu * (val1[0] - val0[0]);
    val[1] = val0[1] + mu * (val1[1] - val0[1]);
    val[2] = val0[2] + mu * (val1[2] - val0[2]);
    return val;
}


Vec3 MarchingCubes::getVertex(int i1, int j1, int k1, int i2, int j2, int k2, float isoValue)
{
    float v1 = m_tsdf[k1 * m_dim[0] * m_dim[1] + j1 * m_dim[0] + i1];
    Vec3 p1 = voxelToWorld(i1, j1, k1);
    float v2 = m_tsdf[k2 * m_dim[0] * m_dim[1] + j2 * m_dim[0] + i2];
    Vec3 p2 = voxelToWorld(i2, j2, k2);
    return interpolate(v1, v2, p1, p2, isoValue);
}


Vec3b MarchingCubes::getColor(int x1, int y1, int z1, int x2, int y2, int z2, float isoValue)
{
    Vec3b c(0, 0, 0);
    // linear interpolation
    size_t idx1 = z1 * m_dim[0] * m_dim[1] + y1 * m_dim[0] + x1;
    size_t idx2 = z2 * m_dim[0] * m_dim[1] + y2 * m_dim[0] + x2;
    float v1 = m_tsdf[idx1];
    float v2 = m_tsdf[idx2];
    Vec3b c1b(m_red[idx1], m_green[idx1+1], m_blue[idx1+2]);
    Vec3b c2b(m_red[idx2], m_green[idx2+1], m_blue[idx2+2]);
    Vec3 c1 = c1b.cast<double>() / 255.0;
    Vec3 c2 = c2b.cast<double>() / 255.0;
    Vec3 cVal = interpolate(v1, v2, c1, c2, isoValue);
    // convert double to unsigned char
    c = (cVal * 255.0).cast<unsigned char>();
    return c;
}


void MarchingCubes::computeTriangles(int cubeIndex, const Vec3 edgePoints[12], const Vec3b edgeColors[12],
                                     std::vector<Vec3> &outVerts,
                                     std::vector<Vec3b> &outColors,
                                     std::vector<Vec3i> &outFaces)
{
    std::vector<Vec3> pts;
    pts.resize(3);
    for (int i = 0; triTable[cubeIndex][i] != -1; i += 3)
    {
        Vec3 p1 = edgePoints[triTable[cubeIndex][i]];
        pts[0] = p1;
        Vec3 p2 = edgePoints[triTable[cubeIndex][i + 1]];
        pts[1] = p2;
        Vec3 p3 = edgePoints[triTable[cubeIndex][i + 2]];
        pts[2] = p3;

        if (p1 != p2 && p1 != p3 && p2 != p3)
        {
            // add vertices
            Vec3i vIdx;
            for (int t = 0; t < 3; ++t)
            {
                Vec3b c = edgeColors[triTable[cubeIndex][i + t]];
                vIdx[t] = addVertex(pts[t], c, outVerts, outColors);
            }

            // add face
            Vec3i faceVerts(vIdx[0], vIdx[1], vIdx[2]);
            outFaces.push_back(faceVerts);
        }
    }
}


inline unsigned int MarchingCubes::addVertex(const Vec3 &v, const Vec3b &c,
                                             std::vector<Vec3> &outVerts,
                                             std::vector<Vec3b> &outColors)
{
    unsigned int vIdx = (unsigned int)outVerts.size();
    outVerts.push_back(v);
    outColors.push_back(c);
    return vIdx;
}


Vec3 MarchingCubes::voxelToWorld(int i, int j, int k) const
{
    Vec3 pt = Vec3i(i, j, k).cast<double>().cwiseProduct(m_voxelSize) - m_size*0.5;
    return pt;
}


bool MarchingCubes::savePly(const std::string &filename) const
{
    if (m_vertices.empty())
        return false;

    // Binary little-endian PLY: dramatically faster to write than ASCII
    // (no per-number formatting, no std::endl flush per line) and produces
    // a much smaller file. No precision is lost vs. the previous ASCII
    // writer because PLY's "float" is 32-bit IEEE-754, which is what the
    // previous code emitted as a decimal string from a double anyway.

    FILE *fp = std::fopen(filename.c_str(), "wb");
    if (!fp)
        return false;

    // Larger I/O buffer for fewer write() syscalls.
    static char io_buf[1 << 20];
    std::setvbuf(fp, io_buf, _IOFBF, sizeof(io_buf));

    std::fprintf(fp,
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex %zu\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "element face %zu\n"
        "property list uchar int vertex_indices\n"
        "end_header\n",
        m_vertices.size(), m_faces.size());

    // Pack each vertex as 12 bytes xyz (float32) + 3 bytes rgb.
    #pragma pack(push, 1)
    struct VertexRec { float x, y, z; unsigned char r, g, b; };
    #pragma pack(pop)
    static_assert(sizeof(VertexRec) == 15, "PLY vertex record must be 15 bytes");

    {
        std::vector<VertexRec> buf(m_vertices.size());
        for (size_t i = 0; i < m_vertices.size(); ++i) {
            buf[i].x = static_cast<float>(m_vertices[i][0]);
            buf[i].y = static_cast<float>(m_vertices[i][1]);
            buf[i].z = static_cast<float>(m_vertices[i][2]);
            buf[i].r = m_colors[i][0];
            buf[i].g = m_colors[i][1];
            buf[i].b = m_colors[i][2];
        }
        std::fwrite(buf.data(), sizeof(VertexRec), buf.size(), fp);
    }

    // Pack each face as 1 byte count + 3 * int32 indices.
    #pragma pack(push, 1)
    struct FaceRec { unsigned char n; int32_t a, b, c; };
    #pragma pack(pop)
    static_assert(sizeof(FaceRec) == 13, "PLY face record must be 13 bytes");

    {
        std::vector<FaceRec> buf(m_faces.size());
        for (size_t i = 0; i < m_faces.size(); ++i) {
            buf[i].n = 3;
            buf[i].a = static_cast<int32_t>(m_faces[i][0]);
            buf[i].b = static_cast<int32_t>(m_faces[i][1]);
            buf[i].c = static_cast<int32_t>(m_faces[i][2]);
        }
        std::fwrite(buf.data(), sizeof(FaceRec), buf.size(), fp);
    }

    std::fclose(fp);
    return true;
}
