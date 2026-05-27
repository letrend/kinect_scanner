#pragma once

#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <string>
#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include "marching_cubes.hpp"
#include "kinect.hpp"
#include "icp_wrapper.hpp"
#include "calibration.hpp"
#include "scan_parameters.hpp"
#include <memory>
#include <mutex>
#include <atomic>
#include <functional>

/**
 * Bundle of host-side frame data emitted to the GUI via callback once per step.
 * Mats are deep-copied owners so the GUI can hold them on another thread.
 */
struct FrameBundle {
    cv::Mat rgb;            // CV_32FC3, 512x424, range [0,1]
    cv::Mat depthMM;        // CV_32FC1, depth in millimeters
    cv::Mat raycastRGB;     // CV_32FC3, raycasted color (empty until 2nd frame)
    cv::Mat raycastDepth;   // CV_32FC1, raycasted depth in mm (empty until 2nd frame)
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    unsigned long long frameIndex = 0;
    float fps = 0.0f;
};

#define STR1(x)  #x
#define STR(x)  STR1(x)

using namespace std;

//! cuda error checking
void cuda_check(string file, int line);
#define CUDA_CHECK cuda_check(__FILE__,__LINE__)

//! intrinsic matrix in constant memory on device
#ifdef __CUDACC__
__constant__ float c_k[3 * 3];
//! inverse intrinsic matrix in constant memory on device
__constant__ float c_kinv[3 * 3];
//! camera pose in constant memory on device
__constant__ float c_cameraPose[4 * 3];
//! inverse camera pose in constant memory on device
__constant__ float c_cameraPose_inv[4 * 3];
//! raycasting camera pose in constant memory on device
__constant__ float c_raycastPose[4 * 3];
//! location of the cubus within the global grid in constant memory on device
__constant__ float c_gridLocation[3];
//! domain kernel (Bilateral Filter)
__constant__ float c_domainKernel[1000];

__global__ void deviceCalculateLocalCoordinates(float *d_depth, float3 *d_v, size_t pWidth, size_t pHeight);
__global__ void deviceCalculateLocalNormals(float3 *d_v, float3 *d_normals, size_t w, size_t h, float normalThreshold);
__global__ void deviceCalculateTSDF(float *d_depth, float *d_color, float3 *d_normals, size_t pWidth, size_t pHeight, float maxTruncation,
                                    float *d_voxelTSDF, float *d_voxelWeight, float *d_voxelWeightColor, unsigned char *d_voxelRed, unsigned char *d_voxelGreen,
                                    unsigned char *d_voxelBlue, float voxelSize, size_t vWidth, size_t vHeight, size_t slice);
__global__ void bilateralFilterKernel(float *img, float *res, int img_width, int img_height,
                           int kernel_width, int kernel_height, float sigma_r);
__device__ __inline__ float deviceTriliniearInterpolation(float *d_voxelGrid, float3 location,
                                                          size_t vWidth, size_t vHeight, size_t slices);
__device__ __inline__ float deviceTriliniearInterpolation(unsigned char *d_voxelGrid, float3 location,
                                                          size_t vWidth, size_t vHeight, size_t slices);
__device__ __inline__ float deviceTSDFBoundaryCheck(float *d_voxelTSDF, float3 location,
                                                    size_t vWidth, size_t vHeight, size_t slices);
__device__ __inline__ float3 deviceGetVoxelGridCoordinates(float3 camera, float voxelSize);
__global__ void deviceRaycast(float *d_voxelTSDF, float *d_depthModel, unsigned char *d_voxelRed, unsigned char *d_voxelGreen,
                              unsigned char *d_voxelBlue, size_t pWidth, size_t pHeight, size_t vWidth,
                              size_t vHeight, size_t slices, float voxelSize, float near, float far, float step, float *d_img);
#endif

class VolumeIntegration{
public:
    VolumeIntegration(uint xDim=400, uint yDim=400, uint zDim=400, float voxelsize=0.01f);
    ~VolumeIntegration();
    /**
     * Initializes the voxel grid position depending on the depth data.
     * In GUI mode (useDisplay=false) the grid is centred on the first valid frame
     * automatically without blocking on user input.
     */
    bool intializeGridPosition();
    /**
     * Run the full scan loop until ESC (CLI) or requestStop() (GUI).
     */
    void scan();
    /**
     * Run a single iteration of the scan loop. Returns false on capture error,
     * pause, or stop request. Intended for the GUI worker thread.
     */
    bool stepOnce();
    /**
     * Signal scan()/stepOnce() loops to exit at the next opportunity.
     */
    void requestStop();
    /**
     * When paused, stepOnce() still pulls frames so the preview keeps updating
     * but skips ICP/TSDF integration.
     */
    void setPaused(bool paused);
    bool isPaused() const { return m_paused.load(); }
    /**
     * Clear the TSDF/weight/color volumes and reset pose to identity.
     * Volume dimension/voxelSize changes in m_params take effect via reset()
     * by re-allocating GPU buffers.
     */
    void reset();
    /**
     * Thread-safe parameter setter/getter.
     */
    void setParameters(const ScanParameters &p);
    ScanParameters getParameters() const;
    /**
     * Callback installed by the GUI worker. Invoked once per stepOnce() with
     * deep-copied cv::Mats. Pass an empty std::function to disable.
     */
    using FrameCallback = std::function<void(const FrameBundle&)>;
    using StatusCallback = std::function<void(const std::string&)>;
    void setOnFrame(FrameCallback cb);
    void setOnStatus(StatusCallback cb);
    /**
     * marching cubes extracts the isosurface
     */
    void extractMesh();
    /**
     * the mesh is saved in ply format
     */
    bool saveMesh(string name="mesh.ply");
    /**
     * calibrate this function calibrates the webcam using opencvs chessboard calibration
     */
    bool calibrate();

    // ---- Read-only accessors used by the GUI ----
    Eigen::Matrix4f currentPose() const { return pose; }
    unsigned long long frameCount() const { return m_frame; }
    size_t volumeWidth()  const { return vWidth; }
    size_t volumeHeight() const { return vHeight; }
    size_t volumeDepth()  const { return slices; }
    float  volumeVoxelSize() const { return voxelSize; }
    const float* hostTSDF()  const { return tsdf; }
    const unsigned char* hostRed()   const { return red; }
    const unsigned char* hostGreen() const { return green; }
    const unsigned char* hostBlue()  const { return blue; }
    const MarchingCubes* mesh() const { return mc; }
    MyFreenectDevice* kinect() { return device; }

private:
    void calculateVoxelGridPosition(float3 *voxels, float* depth, size_t n, float vWidth,
                                    float vHeight, float slices, float voxelSize, float *gridLocation);
    void domainKernel(float *kernel, int cols, int rows, float sigma_d);

    // opencv helpers
    void convert_layered_to_interleaved(float *aOut, const float *aIn, int w, int h, int nc);
    void convert_layered_to_mat(cv::Mat &mOut, const float *aIn);
    void convert_interleaved_to_layered(float *aOut, const float *aIn, int w, int h, int nc);
    void convert_mat_to_layered(float *aOut, const cv::Mat &mIn);

    //! vertices of result
    std::vector<float> vertices;
    //! colors of result
    std::vector<float> colors;
    //! triangles of result
    std::vector<unsigned int> triangles;
    //! volume width, height, depth
    size_t vWidth, vHeight, slices;
    float maxTruncation;
    //! voxelSize in meters
    float voxelSize;
    //! tunables shared with GUI; guarded by m_paramsMutex
    mutable std::mutex m_paramsMutex;
    ScanParameters m_params;
    //! lifecycle flags
    std::atomic<bool> m_stopRequested{false};
    std::atomic<bool> m_paused{false};
    //! frame counter (visible to GUI)
    std::atomic<unsigned long long> m_frame{0};
    //! optional GUI callbacks (installed under m_paramsMutex)
    FrameCallback  m_onFrame;
    StatusCallback m_onStatus;
    //! FPS smoothing
    double m_lastFrameTimeSec = 0.0;
    float  m_fps = 0.0f;
    //! kinect device
    MyFreenectDevice *device;
    //! savePath
    string dataFolder;
    //! input image size and color channels
    size_t pWidth, pHeight, nc;
    //! voxel grid size using float
    size_t voxelGridBytesFloat;
    //! voxel grid size using byte
    size_t voxelGridBytes;

    float *tsdf, *weight, *weightColor;
    unsigned char *red, *green, *blue;

    float *d_voxelTSDF, *d_voxelWeight, *d_voxelWeightColor;
    unsigned char *d_voxelRed, *d_voxelGreen, *d_voxelBlue;

    float *d_depth, *d_depthModel, *d_depthFiltered, *d_color, *d_imgColorRayCast;
    float3 *d_v, *d_normals;

    size_t bytesFloat, bytesFloatColor, bytesFloat3;


public:
    int domain_kernel_width, domain_kernel_height;
    float* domain_kernel;
    float h_k[9];
    float h_kinv[9];
private:
    // block and grid setup
    dim3 block, grid, gridVoxel;
    size_t smBytes;

    int xDim, yDim, arraySize;

    // setup color and depth arrays on host
    float *imgDepth, *imgDepthFiltered, *imgColor, *imgColorRayCast, *depthModel;

    // host camera pose arraycudaMemcpyHostToDevice
    float *cameraPose, *cameraPose_inv ;
    // Mat setup
    cv::Mat color, depth0, depth1, depthFiltered, mOut;
    //! icpcuda
    std::shared_ptr<ICPCUDA> icp;

    //! camera pose
    Eigen::Matrix4f pose;
    Eigen::Matrix4f pose_inv;

    //! voxel grid location
    float gridLocation[3];

    //! marching cubes class
    MarchingCubes *mc;
};