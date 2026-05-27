#include "VolumeIntegration.cuh"

// cuda error checking
string prev_file = "";
int prev_line = 0;
void cuda_check(string file, int line)
{
    cudaError_t e = cudaGetLastError();
    if (e != cudaSuccess)
    {
        cout << endl << file << ", line " << line << ": " << cudaGetErrorString(e) << " (" << e << ")" << endl;
        if (prev_line>0) cout << "Previous CUDA call:" << endl << prev_file << ", line " << prev_line << endl;
        exit(1);
    }
    prev_file = file;
    prev_line = line;
}

VolumeIntegration::VolumeIntegration(uint xDim, uint yDim, uint zDim, float voxelsize):
        vWidth(xDim), vHeight(yDim), slices(zDim), voxelSize(voxelsize){
    // initialize tunables that come from constructor args
    m_params.xDim      = xDim;
    m_params.yDim      = yDim;
    m_params.zDim      = zDim;
    m_params.voxelSize = voxelsize;

    pose     = Eigen::Matrix4f::Identity();
    pose_inv = Eigen::Matrix4f::Identity();

    // initialize cuda context
    cudaDeviceSynchronize();
    CUDA_CHECK;

    // image resolution and number of color channels
    pWidth = 512;
    pHeight = 424;
    nc = 3;

    // Initialize Kinect
    fprintf(stderr, "[VI] before MyFreenectDevice\n"); fflush(stderr);
    device = new MyFreenectDevice;
    fprintf(stderr, "[VI] after MyFreenectDevice\n"); fflush(stderr);

    dataFolder = "/home/roboy/workspace/kinect_scanner/build/data/";//string(STR(TSDF_CUDA_SOURCE_DIR))+ "/data/";

    // initialize intrinsic and inverse intrinsic matrix
    Eigen::Matrix3f K;
    K << device->irCameraParams.fx, 0.0, device->irCameraParams.cx, 0.0, device->irCameraParams.fy, device->irCameraParams.cy, 0.0, 0.0, 1.0;
    Eigen::Matrix3f Kinv = K.inverse();
    float *h_k = new float[3 * 3];
    float *h_kinv = new float[3 * 3];
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            h_k[x + 3 * y] = K(y, x);
            h_kinv[x + 3 * y] = Kinv(y, x);
        }
    }
    cudaMemcpyToSymbol(c_k, h_k, 3 * 3 * sizeof(float));
    CUDA_CHECK;
    cudaMemcpyToSymbol(c_kinv, h_kinv, 3 * 3 * sizeof(float));
    CUDA_CHECK;
    delete[] h_k;
    delete[] h_kinv;

    maxTruncation = 0.03f;
    m_params.maxTruncation = maxTruncation;

    voxelGridBytesFloat = vWidth * vHeight * slices * sizeof(float);
    voxelGridBytes = vWidth * vHeight * slices * sizeof(unsigned char);

    cudaMalloc(&d_voxelTSDF, voxelGridBytesFloat);
    CUDA_CHECK;
    cudaMalloc(&d_voxelWeight, voxelGridBytesFloat);
    CUDA_CHECK;
    cudaMalloc(&d_voxelWeightColor, voxelGridBytesFloat);
    CUDA_CHECK;
    cudaMalloc(&d_voxelRed, voxelGridBytes);
    CUDA_CHECK;
    cudaMalloc(&d_voxelGreen, voxelGridBytes);
    CUDA_CHECK;
    cudaMalloc(&d_voxelBlue, voxelGridBytes);
    CUDA_CHECK;

    tsdf = new float[vWidth * vHeight * slices];
    weight = new float[vWidth * vHeight * slices];
    weightColor = new float[vWidth * vHeight * slices];
    red = new unsigned char[vWidth * vHeight * slices];
    green = new unsigned char[vWidth * vHeight * slices];
    blue = new unsigned char[vWidth * vHeight * slices];

    fill_n(tsdf, vWidth * vHeight * slices, -1.0f);
    fill_n(weight, vWidth * vHeight * slices, 0.0f);
    fill_n(weightColor, vWidth * vHeight * slices, 0.0f);
    fill_n(red, vWidth * vHeight * slices, 0);
    fill_n(green, vWidth * vHeight * slices, 0);
    fill_n(blue, vWidth * vHeight * slices, 0);

    cudaMemcpy(d_voxelTSDF, tsdf, voxelGridBytesFloat, cudaMemcpyHostToDevice);
    CUDA_CHECK;
    cudaMemcpy(d_voxelWeight, weight, voxelGridBytesFloat, cudaMemcpyHostToDevice);
    CUDA_CHECK;
    cudaMemcpy(d_voxelWeightColor, weightColor, voxelGridBytesFloat, cudaMemcpyHostToDevice);
    CUDA_CHECK;
    cudaMemcpy(d_voxelRed, red, voxelGridBytes, cudaMemcpyHostToDevice);
    CUDA_CHECK;
    cudaMemcpy(d_voxelGreen, green, voxelGridBytes, cudaMemcpyHostToDevice);
    CUDA_CHECK;
    cudaMemcpy(d_voxelBlue, blue, voxelGridBytes, cudaMemcpyHostToDevice);
    CUDA_CHECK;

    bytesFloat = pWidth * pHeight * sizeof(float);
    bytesFloatColor = pWidth * pHeight * nc * sizeof(float);
    bytesFloat3 = pWidth * pHeight * sizeof(float3);

    cudaMalloc(&d_depth, bytesFloat);
    CUDA_CHECK;
    cudaMalloc(&d_depthModel, bytesFloat);
    CUDA_CHECK;
    cudaMalloc(&d_depthFiltered, bytesFloat);
    CUDA_CHECK;

    cudaMalloc(&d_color, bytesFloatColor);
    CUDA_CHECK;
    cudaMalloc(&d_imgColorRayCast, bytesFloatColor);
    CUDA_CHECK;
    cudaMalloc(&d_v, bytesFloat3);
    CUDA_CHECK;
    cudaMalloc(&d_normals, bytesFloat3);
    CUDA_CHECK;

    cudaMemset(d_depth, 0, bytesFloat);
    CUDA_CHECK;
    cudaMemset(d_depthModel, 0, bytesFloat);
    CUDA_CHECK;
    cudaMemset(d_depthFiltered, 0, bytesFloat);
    CUDA_CHECK;
    cudaMemset(d_color, 0, bytesFloatColor);
    CUDA_CHECK;
    cudaMemset(d_imgColorRayCast, 0, bytesFloatColor);
    CUDA_CHECK;
    cudaMemset(d_v, 0, bytesFloat3);
    CUDA_CHECK;
    cudaMemset(d_normals, 0, bytesFloat3);
    CUDA_CHECK;

    int r = ceil(3 * m_params.sigma_d);
    domain_kernel_width = 2 * r + 1;
    domain_kernel_height = 2 * r + 1;
    domain_kernel = new float[domain_kernel_width * domain_kernel_height];
    domainKernel(domain_kernel, domain_kernel_width, domain_kernel_height, m_params.sigma_d);
    cudaMemcpyToSymbol(c_domainKernel, domain_kernel, domain_kernel_width * domain_kernel_height * sizeof(float));
    CUDA_CHECK;

    // block and grid setup
    block = dim3(16, 8, 1);
    grid = dim3((pWidth + block.x - 1) / block.x,
                (pHeight + block.y - 1) / block.y, 1);
    gridVoxel = dim3((vWidth + block.x - 1) / block.x,
                     (vHeight + block.y - 1) / block.y, 1);
    smBytes = (block.x + domain_kernel_width - 1)
              * (block.y + domain_kernel_height - 1) * sizeof(float);

    arraySize = (block.x + 2) * (block.y + 2);

    // setup color and depth arrays on host
    imgDepth = new float[pWidth * pHeight];
    imgDepthFiltered = new float[pWidth * pHeight];
    imgColor = new float[pWidth * pHeight * nc];
    imgColorRayCast = new float[pWidth * pHeight * nc];
    depthModel = new float[pWidth * pHeight];

    // host camera pose arraycudaMemcpyHostToDevice
    cameraPose = new float[4 * 3];
    cameraPose_inv = new float[4 * 3];

    // Mat setup
    color = cv::Mat(pHeight, pWidth, CV_32FC3);
    depth0 = cv::Mat(pHeight, pWidth, CV_32FC1);
    depth1 = cv::Mat(pHeight, pWidth, CV_32FC1);
    depthFiltered = cv::Mat(pHeight, pWidth, CV_32FC1);
    mOut = cv::Mat(pHeight, pWidth, CV_32FC3);

    // initialize icpcuda
    fprintf(stderr, "[VI] init ICPCUDA\n"); fflush(stderr);
    icp = std::shared_ptr<ICPCUDA>(new ICPCUDA(pWidth, pHeight, device->irCameraParams.cx,
                      device->irCameraParams.cy, device->irCameraParams.fx,
                      device->irCameraParams.fy,
                      m_params.icpDistThresh, m_params.icpAngleThresh));
    fprintf(stderr, "[VI] ctor done (skipping blocking first updateFrames)\n"); fflush(stderr);
}
VolumeIntegration::~VolumeIntegration(){
    cudaFree(d_depth);
    CUDA_CHECK;
    cudaFree(d_depthFiltered);
    CUDA_CHECK;
    cudaFree(d_color);
    CUDA_CHECK;
    cudaFree(d_imgColorRayCast);
    CUDA_CHECK;
    cudaFree(d_v);
    CUDA_CHECK;
    cudaFree(d_normals);
    CUDA_CHECK;
    cudaFree(d_voxelTSDF);
    CUDA_CHECK;
    cudaFree(d_voxelWeight);
    CUDA_CHECK;
    cudaFree(d_voxelWeightColor);
    CUDA_CHECK;
    cudaFree(d_voxelRed);
    CUDA_CHECK;
    cudaFree(d_voxelGreen);
    CUDA_CHECK;
    cudaFree(d_voxelBlue);
    CUDA_CHECK;

    delete[] imgDepth;
    delete[] imgColor;
    delete[] imgColorRayCast;
    delete[] depthModel;
    delete[] cameraPose;
    delete[] cameraPose_inv;
    delete[] tsdf;
    delete[] weight;
    delete[] weightColor;
    delete[] red;
    delete[] green;
    delete[] blue;
    delete device;
}
bool VolumeIntegration::intializeGridPosition(){
    const bool useDisplay = getParameters().useDisplay;
    if (useDisplay) {
        cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("color", 100, 0);
        cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("depth", 100 + pWidth + 40, 0);
    }
    int settleFrames = 0;
    while (true) {
        if (m_stopRequested.load())
            return false;
        if(!device->updateFrames())
            continue;
        device->getDepthMM(depth1);
        device->getRgbMapped2Depth(color);

        if (useDisplay) {
            cv::imshow("color", color);
            cv::imshow("depth", depth1 / 255.0f / 4.0f);
            char k = cv::waitKey(1);
            if (k == 32) {
                break;
            }else if(k == 20){
                return false;
            }
        } else {
            // headless / GUI: emit the live frame and auto-confirm after a few
            // settle frames so the depth pipeline has stable data.
            if (m_onFrame) {
                FrameBundle fb;
                fb.rgb        = color.clone();
                fb.depthMM    = depth1.clone();
                fb.pose       = pose;
                fb.frameIndex = m_frame.load();
                fb.fps        = m_fps;
                m_onFrame(fb);
            }
            if (++settleFrames >= 5) {
                break;
            }
        }
    }

    // calculate centroid for grid position
    float3 *voxels = new float3[pWidth * pHeight];
    convert_mat_to_layered(imgDepth, depth1 / 1000.0f);
    cudaMemcpy(d_depth, imgDepth, bytesFloat, cudaMemcpyHostToDevice);
    CUDA_CHECK;
    deviceCalculateLocalCoordinates<<<grid, block>>>(d_depth, d_v, pWidth, pHeight);
    CUDA_CHECK;
    cudaMemcpy(voxels, d_v, bytesFloat3, cudaMemcpyDeviceToHost);
    CUDA_CHECK;
    calculateVoxelGridPosition(voxels, imgDepth, pWidth * pHeight, vWidth,
                               vHeight, slices, voxelSize, gridLocation);
    delete[] voxels;

    cudaMemcpyToSymbol(c_gridLocation, gridLocation, 3 * sizeof(float));
    CUDA_CHECK;
    return true;
}

void VolumeIntegration::calculateVoxelGridPosition(float3 *voxels, float* depth, size_t n, float vWidth,
                                float vHeight, float slices, float voxelSize, float *gridLocation)
{
    double3 centroid;
    centroid.x = 0.0f;
    centroid.y = 0.0f;
    centroid.z = 0.0f;

    int count = 0;
    vector<float3> centroidPoints;
    for(size_t i = 0; i < n; i++) {
        if(depth[i]==0.0f) {
            continue;
        }

        if(depth[i]<1.5f){

            count++;

            centroid.x += voxels[i].x;
            centroid.y += voxels[i].y;
            centroid.z += voxels[i].z;

            centroidPoints.push_back(voxels[i]);
        }
    }

    centroid.x /= count;
    centroid.y /= count;
    centroid.z /= count;

    gridLocation[0] = centroid.x - (vWidth * voxelSize) / 2.0f;
    gridLocation[1] = centroid.y - (vHeight * voxelSize) / 2.0f;
    gridLocation[2] = centroid.z - (slices * voxelSize) / 2.0f;
}

__global__ void deviceCalculateLocalCoordinates(float *d_depth, float3 *d_v, size_t pWidth, size_t pHeight)
{
    size_t x = threadIdx.x + blockDim.x * blockIdx.x;
    size_t y = threadIdx.y + blockDim.y * blockIdx.y;

    if(x < pWidth && y < pHeight) {
        size_t idx = x + pWidth*y;
        float D = d_depth[idx];

        // immediately abort if the depth value is 0
        if(D == 0.0f) {
            d_v[idx].x = 0.0f;
            d_v[idx].y = 0.0f;
            d_v[idx].z = 0.0f;
            return;
        }

        // x
        float tmpVal = 0.0f;
        tmpVal += c_kinv[0 + 3*0]*(float)x;
        tmpVal += c_kinv[1 + 3*0]*(float)y;
        tmpVal += c_kinv[2 + 3*0];
        tmpVal *= D;
        d_v[idx].x = tmpVal;

        // y
        tmpVal = 0.0f;
        tmpVal += c_kinv[0 + 3*1]*(float)x;
        tmpVal += c_kinv[1 + 3*1]*(float)y;
        tmpVal += c_kinv[2 + 3*1];
        tmpVal *= D;
        d_v[idx].y = tmpVal;

        // z
        tmpVal = 0.0f;
        tmpVal += c_kinv[0 + 3*2]*(float)x;
        tmpVal += c_kinv[1 + 3*2]*(float)y;
        tmpVal += c_kinv[2 + 3*2];
        tmpVal *= D;
        d_v[idx].z = tmpVal;
    }
}

__global__ void deviceCalculateLocalNormals(float3 *d_v, float3 *d_normals, size_t w, size_t h, float normalThreshold)
{
    size_t x = threadIdx.x + blockDim.x * blockIdx.x;
    size_t y = threadIdx.y + blockDim.y * blockIdx.y;

    extern __shared__ float3 s_v[];
    if(x < w && y < h) {
        size_t idx = x + w*y;

        size_t xDim = (blockDim.x+2);
        size_t yDim = (blockDim.y+2);
        size_t arraySize = xDim*yDim;

        size_t stepSize = blockDim.x*blockDim.y;

        size_t startX = blockDim.x * blockIdx.x - 1;
        size_t startY = blockDim.y * blockIdx.y - 1;

        // setup shared memory
        for(size_t smIdx = threadIdx.x + blockDim.x*threadIdx.y; smIdx < arraySize; smIdx += stepSize) {
            size_t offsetX = smIdx % xDim;
            size_t offsetY = smIdx / xDim;

            size_t globalX = llmax(llmin(w-1, startX+offsetX), 0);
            size_t globalY = llmax(llmin(h-1, startY+offsetY), 0);
            size_t globalIdx = globalX + (size_t)w*globalY;

            s_v[smIdx] = d_v[globalIdx];
        }

        __syncthreads();

        //
        // Shared Memory Index Calculation
        //

        // x and y coordinates in shared memory
        size_t sX = threadIdx.x + 1;
        size_t sY = threadIdx.y + 1;

        // xp = x+1; xm = x-1; yp = y+1; ym = y-1
        size_t x0yp = (sX  ) + xDim * (sY+1);
        size_t x0ym = (sX  ) + xDim * (sY-1);
        size_t xpy0 = (sX+1) + xDim * (sY  );
        size_t xpyp = (sX+1) + xDim * (sY+1);
        size_t xpym = (sX+1) + xDim * (sY-1);
        size_t xmy0 = (sX-1) + xDim * (sY  );
        size_t xmyp = (sX-1) + xDim * (sY+1);
        size_t xmym = (sX-1) + xDim * (sY-1);

        // calculate central differences in two different ways
        float3 n1;
        n1.x = (s_v[xpy0].x - s_v[xmy0].x) * 0.5f;
        n1.y = (s_v[xpy0].y - s_v[xmy0].y) * 0.5f;
        n1.z = (s_v[xpy0].z - s_v[xmy0].z) * 0.5f;
        float n1Len = sqrtf(n1.x*n1.x + n1.y*n1.y + n1.z*n1.z);

        float3 n2;
        n2.x = (s_v[x0yp].x - s_v[x0ym].x) * 0.5f;
        n2.y = (s_v[x0yp].y - s_v[x0ym].y) * 0.5f;
        n2.z = (s_v[x0yp].z - s_v[x0ym].z) * 0.5f;
        float n2Len = sqrtf(n2.x*n2.x + n2.y*n2.y + n2.z*n2.z);

        float3 n3;
        n3.x = (s_v[xpyp].x - s_v[xmym].x) * 0.5f;
        n3.y = (s_v[xpyp].y - s_v[xmym].y) * 0.5f;
        n3.z = (s_v[xpyp].z - s_v[xmym].z) * 0.5f;
        float n3Len = sqrtf(n3.x*n3.x + n3.y*n3.y + n3.z*n3.z);

        float3 n4;
        n4.x = (s_v[xmyp].x - s_v[xpym].x) * 0.5f;
        n4.y = (s_v[xmyp].y - s_v[xpym].y) * 0.5f;
        n4.z = (s_v[xmyp].z - s_v[xpym].z) * 0.5f;
        float n4Len = sqrtf(n4.x*n4.x + n4.y*n4.y + n4.z*n4.z);

        // calculate the first normal from n1 and n2 using the cross product
        float3 normal1;
        if(n1Len > normalThreshold || n2Len > normalThreshold) {
            normal1.x = 0.0f;
            normal1.y = 0.0f;
            normal1.z = 0.0f;
        } else {
            normal1.x = n1.y * n2.z - n1.z * n2.y;
            normal1.y = n1.z * n2.x - n1.x * n2.z;
            normal1.z = n1.x * n2.y - n1.y * n2.x;
        }

        // calculate the second normal from n3 and n4 using the cross product
        float3 normal2;
        if(n3Len > normalThreshold || n4Len > normalThreshold) {
            normal2.x = 0.0f;
            normal2.y = 0.0f;
            normal2.z = 0.0f;
        } else {
            normal2.x = n3.y * n4.z - n3.z * n4.y;
            normal2.y = n3.z * n4.x - n3.x * n4.z;
            normal2.z = n3.x * n4.y - n3.y * n4.x;
        }

        if(normal1.x == 0.0f || normal1.y == 0.0f || normal1.z == 0.0f) {
            normal1.x = normal2.x;
            normal1.y = normal2.y;
            normal1.z = normal2.z;
        } else if(normal2.x == 0.0f || normal2.y == 0.0f || normal2.z == 0.0f) {
            normal2.x = normal1.x;
            normal2.y = normal1.y;
            normal2.z = normal1.z;
        }

        // average both normals for the final normal
        float3 normal;
        normal.x = (normal1.x + normal2.x) * 0.5f;
        normal.y = (normal1.y + normal2.y) * 0.5f;
        normal.z = (normal1.z + normal2.z) * 0.5f;

        // normalize the normal
        float normalizationFactor = rsqrtf(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
        normal.x *= normalizationFactor;
        normal.y *= normalizationFactor;
        normal.z *= normalizationFactor;

        if(normal.x != normal.x || normal.y != normal.y || normal.z != normal.z) {
            normal.x = 0.0f;
            normal.y = 0.0f;
            normal.z = 0.0f;
        }

        // write to global memory
        d_normals[idx] = normal;
    }
}

__global__ void deviceCalculateTSDF(float *d_depth, float *d_color, float3 *d_normals, size_t pWidth, size_t pHeight, float maxTruncation,
                                    float *d_voxelTSDF, float *d_voxelWeight, float *d_voxelWeightColor, unsigned char *d_voxelRed, unsigned char *d_voxelGreen,
                                    unsigned char *d_voxelBlue, float voxelSize, size_t vWidth, size_t vHeight, size_t slice)
{
    size_t x = threadIdx.x + blockDim.x * blockIdx.x;
    size_t y = threadIdx.y + blockDim.y * blockIdx.y;

    // index in voxel arrays
    size_t ind_voxel = x + vWidth*y + vWidth*vHeight*slice;

    // near and far clipping plane
    float near = 0.3f;
    float far  = 3.0f;

    if(x < vWidth && y < vHeight) {
        // Calculate position of each voxel in Voxel(vx) Coordinate system
        float voxelHalfSize = voxelSize * 0.5f;
        float3 vx;
        vx.x = x     * voxelSize + voxelHalfSize;
        vx.y = y     * voxelSize + voxelHalfSize;
        vx.z = slice * voxelSize + voxelHalfSize;

        // Calculate position of each voxel in Global(g) Coordinate system
        float3 vg;
        vg.x = vx.x + c_gridLocation[0];
        vg.y = vx.y + c_gridLocation[1];
        vg.z = vx.z + c_gridLocation[2];

        // Calculate position of each voxel in Camera(c) Coordinate system
        float3 v;
        v.x = 0.0f;
        v.y = 0.0f;
        v.z = 0.0f;

        // x
        v.x += c_cameraPose_inv[0 + 4*0] * vg.x;
        v.x += c_cameraPose_inv[1 + 4*0] * vg.y;
        v.x += c_cameraPose_inv[2 + 4*0] * vg.z;
        v.x += c_cameraPose_inv[3 + 4*0];

        // y
        v.y += c_cameraPose_inv[0 + 4*1] * vg.x;
        v.y += c_cameraPose_inv[1 + 4*1] * vg.y;
        v.y += c_cameraPose_inv[2 + 4*1] * vg.z;
        v.y += c_cameraPose_inv[3 + 4*1];

        // z
        v.z += c_cameraPose_inv[0 + 4*2] * vg.x;
        v.z += c_cameraPose_inv[1 + 4*2] * vg.y;
        v.z += c_cameraPose_inv[2 + 4*2] * vg.z;
        v.z += c_cameraPose_inv[3 + 4*2];

        // Perspective projection using intrinsic matrix K
        float3 p;
        p.x = 0.0f;
        p.y = 0.0f;
        p.z = 0.0f;

        // x
        p.x += c_k[0 + 3*0]*v.x;
        p.x += c_k[1 + 3*0]*v.y;
        p.x += c_k[2 + 3*0]*v.z;

        // y
        p.y += c_k[0 + 3*1]*v.x;
        p.y += c_k[1 + 3*1]*v.y;
        p.y += c_k[2 + 3*1]*v.z;

        // z
        p.z += c_k[0 + 3*2]*v.x;
        p.z += c_k[1 + 3*2]*v.y;
        p.z += c_k[2 + 3*2]*v.z;

        // homogenize: divide by z
        p.x /= p.z;
        p.y /= p.z;

        // index in image arrays (depth only - add pWidth*pHeight*c for color channel)
        ssize_t pX = llrintf(p.x);
        ssize_t pY = llrintf(p.y);
        size_t ind_depth = pX + pWidth*pY;

        // if voxel is visible in camera frustum AND has a valid depth (greater than 0 and not NaN)
        if(pX >= 0 && pX < pWidth && pY >= 0 && pY < pHeight && p.z > near && p.z <= far
           && d_depth[ind_depth] > 0.0f ) {//&& isfinite(d_depth[ind_depth])

            float sdf = p.z	- d_depth[ind_depth];
            if(sdf <= maxTruncation) {
                // calculate preliminary tsdf value
                float tsdf;
#if defined(TSDF_WEIGHTING) || defined(TSDF_EXP_WEIGHTING)
                float tsdfWeight;
#endif
                if(sdf >= 0) {
                    tsdf = fminf(1.0f, sdf/maxTruncation);
#if defined(TSDF_WEIGHTING) || defined(TSDF_EXP_WEIGHTING)
                    tsdfWeight = 1.0f - tsdf;
#endif
                }
                else {
                    tsdf = fmaxf(-1.0f, sdf/maxTruncation);
#if defined(TSDF_WEIGHTING) || defined(TSDF_EXP_WEIGHTING)
                    tsdfWeight = 1.0f;
#endif
                }

                // calculate preliminary color values (multiply by 255)
                float red   = d_color[ind_depth + (size_t)pWidth*pHeight*0]*255.0f ;
                float green = d_color[ind_depth + (size_t)pWidth*pHeight*1]*255.0f ;
                float blue  = d_color[ind_depth + (size_t)pWidth*pHeight*2]*255.0f ;

                // calculate weight
                float oldWeight = d_voxelWeight[ind_voxel];
                float newWeight = d_normals[ind_depth].z;
                float weight = oldWeight + newWeight;
                float invWeight = 1.0f / weight;

                // calculate color weight
                float oldWeightColor = d_voxelWeightColor[ind_voxel];
                float newWeightColor = d_normals[ind_depth].z;
#if defined(TSDF_WEIGHTING)
                newWeightColor *= tsdfWeight;
#elif defined(TSDF_EXP_WEIGHTING)
                newWeightColor *= expf(tsdfWeight);
#endif
                float weightColor = oldWeightColor + newWeightColor;
                float invWeightColor = 1.0f / weightColor;

                // calculate tsdf and write it to global memory
                if(newWeight > 0.0f && newWeight == newWeight) {
                    d_voxelTSDF[ind_voxel]  = (d_voxelTSDF[ind_voxel]  * (float)oldWeight + tsdf  * (float)newWeight) *
                                              (float)invWeight;

                    d_voxelWeight[ind_voxel] = weight;
                }

                // calculate color values and write them to global memory
                if(newWeightColor > 0.0f && newWeightColor == newWeightColor) {
                    d_voxelRed[ind_voxel]   = lrintf((d_voxelRed[ind_voxel]   * (float)oldWeightColor + red   * (float)newWeightColor) *
                                                     (float)invWeightColor);
                    d_voxelGreen[ind_voxel] = lrintf((d_voxelGreen[ind_voxel] * (float)oldWeightColor + green * (float)newWeightColor) *
                                                     (float)invWeightColor);
                    d_voxelBlue[ind_voxel]  = lrintf((d_voxelBlue[ind_voxel]  * (float)oldWeightColor + blue  * (float)newWeightColor) *
                                                     (float)invWeightColor);

                    d_voxelWeightColor[ind_voxel] = weightColor;
                }
            }
        }
    }
}

__device__ __inline__ float deviceTriliniearInterpolation(float *d_voxelGrid, float3 location,
                                                          size_t vWidth, size_t vHeight, size_t slices)
{
    int3 fLocation, cLocation;
    fLocation.x = (int)floorf(location.x);
    fLocation.y = (int)floorf(location.y);
    fLocation.z = (int)floorf(location.z);
    cLocation.x = (int)ceilf(location.x);
    cLocation.y = (int)ceilf(location.y);
    cLocation.z = (int)ceilf(location.z);

    float x0y0z0 = d_voxelGrid[fLocation.x + vWidth*fLocation.y + vWidth*vHeight*fLocation.z];
    float x0y1z0 = d_voxelGrid[fLocation.x + vWidth*cLocation.y + vWidth*vHeight*fLocation.z];
    float x0y0z1 = d_voxelGrid[fLocation.x + vWidth*fLocation.y + vWidth*vHeight*cLocation.z];
    float x0y1z1 = d_voxelGrid[fLocation.x + vWidth*cLocation.y + vWidth*vHeight*cLocation.z];
    float x1y0z0 = d_voxelGrid[cLocation.x + vWidth*fLocation.y + vWidth*vHeight*fLocation.z];
    float x1y1z0 = d_voxelGrid[cLocation.x + vWidth*cLocation.y + vWidth*vHeight*fLocation.z];
    float x1y0z1 = d_voxelGrid[cLocation.x + vWidth*fLocation.y + vWidth*vHeight*cLocation.z];
    float x1y1z1 = d_voxelGrid[cLocation.x + vWidth*cLocation.y + vWidth*vHeight*cLocation.z];

    float3 d;
    d.x = (location.x - fLocation.x) / (cLocation.x - fLocation.x);
    d.y = (location.y - fLocation.y) / (cLocation.y - fLocation.y);
    d.z = (location.z - fLocation.z) / (cLocation.z - fLocation.z);

    float c00 = x0y0z0 * (1 - d.x) + x1y0z0 * d.x;
    float c10 = x0y1z0 * (1 - d.x) + x1y1z0 * d.x;
    float c01 = x0y0z1 * (1 - d.x) + x1y0z1 * d.x;
    float c11 = x0y1z1 * (1 - d.x) + x1y1z1 * d.x;

    float c0  = c00    * (1 - d.y) + c10    * d.y;
    float c1  = c01    * (1 - d.y) + c11    * d.y;

    return c0 * (1 - d.z) + c1 * d.z;
}

__device__ __inline__ float deviceTriliniearInterpolation(unsigned char *d_voxelGrid, float3 location,
                                                          size_t vWidth, size_t vHeight, size_t slices)
{
    int3 fLocation, cLocation;
    fLocation.x = (int)floorf(location.x);
    fLocation.y = (int)floorf(location.y);
    fLocation.z = (int)floorf(location.z);
    cLocation.x = (int)ceilf(location.x);
    cLocation.y = (int)ceilf(location.y);
    cLocation.z = (int)ceilf(location.z);

    float colorFactor = 1.0f / 255.0f;
    float x0y0z0 = (float)d_voxelGrid[fLocation.x + vWidth*fLocation.y + vWidth*vHeight*fLocation.z] * colorFactor;
    float x0y1z0 = (float)d_voxelGrid[fLocation.x + vWidth*cLocation.y + vWidth*vHeight*fLocation.z] * colorFactor;
    float x0y0z1 = (float)d_voxelGrid[fLocation.x + vWidth*fLocation.y + vWidth*vHeight*cLocation.z] * colorFactor;
    float x0y1z1 = (float)d_voxelGrid[fLocation.x + vWidth*cLocation.y + vWidth*vHeight*cLocation.z] * colorFactor;
    float x1y0z0 = (float)d_voxelGrid[cLocation.x + vWidth*fLocation.y + vWidth*vHeight*fLocation.z] * colorFactor;
    float x1y1z0 = (float)d_voxelGrid[cLocation.x + vWidth*cLocation.y + vWidth*vHeight*fLocation.z] * colorFactor;
    float x1y0z1 = (float)d_voxelGrid[cLocation.x + vWidth*fLocation.y + vWidth*vHeight*cLocation.z] * colorFactor;
    float x1y1z1 = (float)d_voxelGrid[cLocation.x + vWidth*cLocation.y + vWidth*vHeight*cLocation.z] * colorFactor;

    float3 d;
    d.x = (location.x - fLocation.x) / (cLocation.x - fLocation.x);
    d.y = (location.y - fLocation.y) / (cLocation.y - fLocation.y);
    d.z = (location.z - fLocation.z) / (cLocation.z - fLocation.z);

    float c00 = x0y0z0 * (1 - d.x) + x1y0z0 * d.x;
    float c10 = x0y1z0 * (1 - d.x) + x1y1z0 * d.x;
    float c01 = x0y0z1 * (1 - d.x) + x1y0z1 * d.x;
    float c11 = x0y1z1 * (1 - d.x) + x1y1z1 * d.x;

    float c0  = c00    * (1 - d.y) + c10    * d.y;
    float c1  = c01    * (1 - d.y) + c11    * d.y;

    return c0 * (1 - d.z) + c1 * d.z;
}

__device__ __inline__ float deviceTSDFBoundaryCheck(float *d_voxelTSDF, float3 location,
                                                    size_t vWidth, size_t vHeight, size_t slices)
{
    if(floorf(location.x) >= 0 && ceilf(location.x) < vWidth && floorf(location.y) >= 0 && ceilf(location.y) < vHeight
       && floorf(location.z) >= 0 && ceilf(location.z) < slices) {
        return deviceTriliniearInterpolation(d_voxelTSDF, location, vWidth, vHeight, slices);
    } else {
        return -1.0f;
    }
}

__device__ __inline__ float3 deviceGetVoxelGridCoordinates(float3 camera, float voxelSize)
{
// dehomogenize
float3 pixel;
pixel.x = camera.x * camera.z;
pixel.y = camera.y * camera.z;
pixel.z = camera.z;

// camera coordinates
float3 frustum;
frustum.x=0.0f;
frustum.y=0.0f;
frustum.z=0.0f;

// x
frustum.x += c_kinv[0 + 3*0] * pixel.x;
frustum.x += c_kinv[1 + 3*0] * pixel.y;
frustum.x += c_kinv[2 + 3*0] * pixel.z;

// y
frustum.y += c_kinv[0 + 3*1] * pixel.x;
frustum.y += c_kinv[1 + 3*1] * pixel.y;
frustum.y += c_kinv[2 + 3*1] * pixel.z;

// z
frustum.z += c_kinv[0 + 3*2] * pixel.x;
frustum.z += c_kinv[1 + 3*2] * pixel.y;
frustum.z += c_kinv[2 + 3*2] * pixel.z;

// global coordinates
float3 v;
v.x=0.0f;
v.y=0.0f;
v.z=0.0f;

// x
v.x += c_cameraPose[0 + 4*0] * frustum.x;
v.x += c_cameraPose[1 + 4*0] * frustum.y;
v.x += c_cameraPose[2 + 4*0] * frustum.z;
v.x += c_cameraPose[3 + 4*0];

// y
v.y += c_cameraPose[0 + 4*1] * frustum.x;
v.y += c_cameraPose[1 + 4*1] * frustum.y;
v.y += c_cameraPose[2 + 4*1] * frustum.z;
v.y += c_cameraPose[3 + 4*1];

// z
v.z += c_cameraPose[0 + 4*2] * frustum.x;
v.z += c_cameraPose[1 + 4*2] * frustum.y;
v.z += c_cameraPose[2 + 4*2] * frustum.z;
v.z += c_cameraPose[3 + 4*2];

// convert to global coordinates
float3 cglobal;
cglobal.x = v.x - c_gridLocation[0];
cglobal.y = v.y - c_gridLocation[1];
cglobal.z = v.z - c_gridLocation[2];

float voxelHalfSize = voxelSize * 0.5f;
float inverseVoxelSize = 1.0f / voxelSize;
float3 voxelgrid;
voxelgrid.x = (cglobal.x - voxelHalfSize) * inverseVoxelSize;
voxelgrid.y = (cglobal.y - voxelHalfSize) * inverseVoxelSize;
voxelgrid.z = (cglobal.z - voxelHalfSize) * inverseVoxelSize;

return voxelgrid;
}

__global__ void deviceRaycast(float *d_voxelTSDF, float *d_depthModel, unsigned char *d_voxelRed, unsigned char *d_voxelGreen,
                              unsigned char *d_voxelBlue, size_t pWidth, size_t pHeight, size_t vWidth,
                              size_t vHeight, size_t slices, float voxelSize, float near, float far, float step, float *d_img)
{
    size_t x = threadIdx.x + blockDim.x * blockIdx.x;
    size_t y = threadIdx.y + blockDim.y * blockIdx.y;

    if(x < pWidth && y < pHeight){
        size_t ind_img = x + pWidth*y;

        float tsdfThreshold = 0.001;

        bool terminated = false;
        for(float i = near; i < far; i += step) {
            float3 pixel, voxelgrid;
            pixel.x = x;
            pixel.y = y;
            pixel.z = i;

            voxelgrid = deviceGetVoxelGridCoordinates(pixel, voxelSize);

            float tsdf = deviceTSDFBoundaryCheck(d_voxelTSDF, voxelgrid, vWidth, vHeight, slices);
            if(tsdf >= -tsdfThreshold) {
                float adaptiveStep = step * 0.5;

                int c = 0;
                while ((tsdf > tsdfThreshold || tsdf < -tsdfThreshold) && c++ < 10) {
                    if(tsdf > 0) {
                        pixel.z -= adaptiveStep;
                    } else {
                        pixel.z += adaptiveStep;
                    }
                    voxelgrid = deviceGetVoxelGridCoordinates(pixel, voxelSize);

                    tsdf = deviceTSDFBoundaryCheck(d_voxelTSDF, voxelgrid, vWidth, vHeight, slices);

                    if(tsdf < 1.0 && tsdf > -1.0) {
                        adaptiveStep *= 0.75;
                    }
                }

                d_img[ind_img + pWidth*pHeight*0] = deviceTriliniearInterpolation(d_voxelRed,
                                                                                  voxelgrid, vWidth, vHeight, slices);
                d_img[ind_img + pWidth*pHeight*1] = deviceTriliniearInterpolation(d_voxelGreen,
                                                                                  voxelgrid, vWidth, vHeight, slices);
                d_img[ind_img + pWidth*pHeight*2] = deviceTriliniearInterpolation(d_voxelBlue,
                                                                                  voxelgrid, vWidth, vHeight, slices);

                d_depthModel[ind_img] = pixel.z;

                terminated = true;
                break;
            }

        }

        if(!terminated){
            d_img[ind_img + pWidth*pHeight*0] = 0.0f;
            d_img[ind_img + pWidth*pHeight*1] = 0.0f;
            d_img[ind_img + pWidth*pHeight*2] = 0.0f;
        }
    }
}

void VolumeIntegration::scan(){
    const bool useDisplay = getParameters().useDisplay;
    if (useDisplay) {
        cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("color", 100, 0);
        cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("depth", 100 + pWidth + 40, 0);
        cv::namedWindow("depth Model", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("depth Model", 100 + pWidth + 40, pHeight + 100);
        cv::namedWindow("raycasted", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("raycasted", 100, pHeight + 100);
    }
    m_stopRequested.store(false);
    while (!m_stopRequested.load()) {
        if (!stepOnce()) {
            if (m_paused.load()) {
                // paused: keep UI responsive (CLI) / yield (GUI)
                if (useDisplay) {
                    char k = cv::waitKey(30);
                    if (k == 27) break;
                }
                continue;
            }
            // capture error -> exit loop
            break;
        }
        if (useDisplay) {
            char k = cv::waitKey(1);
            if (k == 27) break;
        }
    }
    if (useDisplay) cv::destroyAllWindows();

    // download GPU volume data
    cudaMemcpy(tsdf, d_voxelTSDF, voxelGridBytesFloat,
               cudaMemcpyDeviceToHost);
    CUDA_CHECK;
    cudaMemcpy(red, d_voxelRed, voxelGridBytes, cudaMemcpyDeviceToHost);
    CUDA_CHECK;
    cudaMemcpy(green, d_voxelGreen, voxelGridBytes, cudaMemcpyDeviceToHost);
    CUDA_CHECK;
    cudaMemcpy(blue, d_voxelBlue, voxelGridBytes, cudaMemcpyDeviceToHost);
    CUDA_CHECK;
}

bool VolumeIntegration::stepOnce(){
    // Snapshot tunables once per step.
    ScanParameters p = getParameters();
    const bool useDisplay = p.useDisplay;

    // get new kinect frames
    if(!device->updateFrames())
        return false;
    device->getDepthMM(depth1);
    device->getRgbMapped2Depth(color);

    if (useDisplay) {
        cv::imshow("color", color);
        cv::imshow("depth", depth1 / 255.0f / 10.0f);
    }

    // When paused, still publish the live RGB/depth to keep the preview alive
    // but skip all integration / pose update.
    if (m_paused.load()) {
        if (m_onFrame) {
            FrameBundle fb;
            fb.rgb        = color.clone();
            fb.depthMM    = depth1.clone();
            fb.pose       = pose;
            fb.frameIndex = m_frame.load();
            fb.fps        = m_fps;
            m_onFrame(fb);
        }
        return false;
    }

    const unsigned long long frame = m_frame.load();

    // convert and copy to device
    convert_mat_to_layered(imgDepth, depth1 / 1000.0f);
    convert_mat_to_layered(imgColor, color);
    cudaMemcpy(d_depth, imgDepth, bytesFloat, cudaMemcpyHostToDevice);
    CUDA_CHECK;
    cudaMemcpy(d_color, imgColor, bytesFloatColor,cudaMemcpyHostToDevice);
    CUDA_CHECK;

    // Step 0: Bilateral Filter depth image
    bilateralFilterKernel<<<grid, block, smBytes>>>(d_depth,
            d_depthFiltered, pWidth, pHeight, domain_kernel_width,
            domain_kernel_height, p.sigma_r);
    CUDA_CHECK;
    cudaDeviceSynchronize();
    CUDA_CHECK;

    // Step 1: calculate local coordinates and corresponding normals
    deviceCalculateLocalCoordinates<<<grid, block>>>(d_depth,
            d_v, pWidth, pHeight);
    CUDA_CHECK;
    cudaDeviceSynchronize();
    CUDA_CHECK;
    deviceCalculateLocalNormals<<<grid, block,
            arraySize * sizeof(float3)>>>(d_v, d_normals, pWidth,
                    pHeight, p.normalThreshold);
    CUDA_CHECK;
    cudaDeviceSynchronize();
    CUDA_CHECK;

    if (frame > 0) {
        cudaMemcpy(imgDepthFiltered, d_depthFiltered, bytesFloat,
                   cudaMemcpyDeviceToHost);
        CUDA_CHECK;
        convert_layered_to_mat(depthFiltered, imgDepthFiltered);
        depthFiltered *= 1000.0f;
        // ICP
        icp->getPoseFromDepth(depth0, depthFiltered);
        pose = icp->getPose().cast<float>();
        pose_inv = icp->getPose_inv().cast<float>();
    }

    // set (inverse) camera pose and write it to constant device memory
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 4; x++) {
            size_t idx = x + 4 * y;
            cameraPose[idx] = pose(y, x);
            cameraPose_inv[idx] = pose_inv(y, x);
        }
    }
    cudaMemcpyToSymbol(c_cameraPose, cameraPose, 4 * 3 * sizeof(float));
    CUDA_CHECK;
    cudaMemcpyToSymbol(c_cameraPose_inv, cameraPose_inv,
                       4 * 3 * sizeof(float));
    CUDA_CHECK;

    // sweep through the voxel grid slice by slice
    for (size_t slice = 0; slice < slices; slice++) {
        deviceCalculateTSDF<<<gridVoxel, block>>>(d_depth, d_color,
                d_normals, pWidth, pHeight, p.maxTruncation, d_voxelTSDF,
                d_voxelWeight, d_voxelWeightColor, d_voxelRed, d_voxelGreen,
                d_voxelBlue,voxelSize, vWidth, vHeight, slice);
        cudaDeviceSynchronize();
        CUDA_CHECK;
    }

    // raycast the current model
    deviceRaycast<<<grid, block>>>(d_voxelTSDF, d_depthModel,
            d_voxelRed, d_voxelGreen, d_voxelBlue, pWidth, pHeight,
            vWidth, vHeight, slices, voxelSize,
            p.raycastNear, p.raycastFar, p.raycastStep,
            d_imgColorRayCast);
    CUDA_CHECK;
    cudaDeviceSynchronize();
    CUDA_CHECK;

    cudaMemcpy(imgColorRayCast, d_imgColorRayCast, bytesFloatColor,
               cudaMemcpyDeviceToHost);
    CUDA_CHECK;
    convert_layered_to_mat(mOut, imgColorRayCast);
    if (useDisplay) cv::imshow("raycasted", mOut);

    cudaMemcpy(depthModel, d_depthModel, bytesFloat,
               cudaMemcpyDeviceToHost);
    CUDA_CHECK;
    cudaMemset(d_depthModel, 0, bytesFloat);
    CUDA_CHECK;

    convert_layered_to_mat(depth0, depthModel);
    depth0 *= 1000.0f;
    if (useDisplay) cv::imshow("depth Model", depth0 / 255.0f / 10.0f);

    // update fps (simple exponential moving average)
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double now = ts.tv_sec + ts.tv_nsec * 1e-9;
        if (m_lastFrameTimeSec > 0.0) {
            float instFps = 1.0f / (float)(now - m_lastFrameTimeSec);
            m_fps = (m_fps <= 0.0f) ? instFps : (0.9f * m_fps + 0.1f * instFps);
        }
        m_lastFrameTimeSec = now;
    }

    // increment frame counter
    m_frame.fetch_add(1);

    // publish to GUI
    if (m_onFrame) {
        FrameBundle fb;
        fb.rgb          = color.clone();
        fb.depthMM      = depth1.clone();
        fb.raycastRGB   = mOut.clone();
        fb.raycastDepth = depth0.clone();
        fb.pose         = pose;
        fb.frameIndex   = m_frame.load();
        fb.fps          = m_fps;
        m_onFrame(fb);
    }
    return true;
}

void VolumeIntegration::requestStop(){
    m_stopRequested.store(true);
}

void VolumeIntegration::setPaused(bool paused){
    m_paused.store(paused);
}

void VolumeIntegration::setParameters(const ScanParameters &p){
    std::lock_guard<std::mutex> lock(m_paramsMutex);
    m_params = p;
}

ScanParameters VolumeIntegration::getParameters() const{
    std::lock_guard<std::mutex> lock(m_paramsMutex);
    return m_params;
}

void VolumeIntegration::setOnFrame(FrameCallback cb){
    std::lock_guard<std::mutex> lock(m_paramsMutex);
    m_onFrame = std::move(cb);
}

void VolumeIntegration::setOnStatus(StatusCallback cb){
    std::lock_guard<std::mutex> lock(m_paramsMutex);
    m_onStatus = std::move(cb);
}

void VolumeIntegration::reset(){
    // clear host volumes
    std::fill_n(tsdf,        vWidth * vHeight * slices, -1.0f);
    std::fill_n(weight,      vWidth * vHeight * slices,  0.0f);
    std::fill_n(weightColor, vWidth * vHeight * slices,  0.0f);
    std::fill_n(red,         vWidth * vHeight * slices,  (unsigned char)0);
    std::fill_n(green,       vWidth * vHeight * slices,  (unsigned char)0);
    std::fill_n(blue,        vWidth * vHeight * slices,  (unsigned char)0);

    // upload cleared volumes
    cudaMemcpy(d_voxelTSDF,        tsdf,        voxelGridBytesFloat, cudaMemcpyHostToDevice); CUDA_CHECK;
    cudaMemcpy(d_voxelWeight,      weight,      voxelGridBytesFloat, cudaMemcpyHostToDevice); CUDA_CHECK;
    cudaMemcpy(d_voxelWeightColor, weightColor, voxelGridBytesFloat, cudaMemcpyHostToDevice); CUDA_CHECK;
    cudaMemcpy(d_voxelRed,   red,   voxelGridBytes, cudaMemcpyHostToDevice); CUDA_CHECK;
    cudaMemcpy(d_voxelGreen, green, voxelGridBytes, cudaMemcpyHostToDevice); CUDA_CHECK;
    cudaMemcpy(d_voxelBlue,  blue,  voxelGridBytes, cudaMemcpyHostToDevice); CUDA_CHECK;

    // reset pose
    pose     = Eigen::Matrix4f::Identity();
    pose_inv = Eigen::Matrix4f::Identity();
    m_frame.store(0);
    m_fps = 0.0f;
    m_lastFrameTimeSec = 0.0;

    // re-create ICP so it starts from a clean state
    icp = std::shared_ptr<ICPCUDA>(new ICPCUDA(pWidth, pHeight,
          device->irCameraParams.cx, device->irCameraParams.cy,
          device->irCameraParams.fx, device->irCameraParams.fy,
          m_params.icpDistThresh, m_params.icpAngleThresh));
}

void VolumeIntegration::domainKernel(float *kernel, int cols, int rows, float sigma_d) {
    float invSigma = 1.0f / sigma_d;

    int i=0;
    for(int row = -rows/2; row <= rows/2; row++) {
        int j=0;
        for(int col =- cols/2; col <= cols/2; col++) {
            kernel[j + i*cols] = expf(-0.5f * (sqrtf(row*row + col*col) * invSigma) * (sqrtf(row*row + col*col) * invSigma));
            j++;
        }
        i++;
    }
}

__global__ void bilateralFilterKernel(float *img, float *res, int img_width, int img_height,
                                      int kernel_width, int kernel_height, float sigma_r){
    int sw = blockDim.x + kernel_width - 1;
    int sh = blockDim.y + kernel_height - 1;
    // how many times does the block fit into shared memory
    int n = (sw*sh+blockDim.x*blockDim.y-1)/(blockDim.x*blockDim.y);
    int x = threadIdx.x + blockDim.x * blockIdx.x;
    int y = threadIdx.y + blockDim.y * blockIdx.y;

    extern __shared__ float sh_data[];
    for (int i=0; i<n; i++) {
        int sh_ind = (i * blockDim.x * blockDim.y) + threadIdx.x + threadIdx.y * blockDim.x;
        if (sh_ind < sw*sh){
            int img_ind_x = -kernel_width/2 + blockIdx.x * blockDim.x + sh_ind % sw;
            int img_ind_y = -kernel_height/2 + blockIdx.y * blockDim.y + sh_ind / sw;

            // clamping border
            int xc=max(min(img_width-1,img_ind_x),0);
            int yc=max(min(img_height-1,img_ind_y),0);
            sh_data[sh_ind] = img[xc + img_width * yc];
        }
    }

    __syncthreads();

    if(x < img_width && y < img_height) {
        float invSigma = 1.0f / sigma_r;

        float val = 0.0f;
        float norm = 0.0f;
        float range_kernel = 0.0;
        int sh_ind_img_neighbor, sh_ind, ind_img;
        int yy = 0;
        int l = 0;
        sh_ind = (threadIdx.x + kernel_width/2) + (threadIdx.y + kernel_height/2) * sw; // central pixel
        ind_img = x + img_width*y;
        if (sh_data[sh_ind] != 0.0f){
            for (int i = -kernel_width/2; i <= kernel_width/2; i++){
                int xx = 0;
                for (int j = -kernel_height/2; j <= kernel_height/2; j++){
                    sh_ind_img_neighbor = (threadIdx.x+xx) + (threadIdx.y+yy) * sw; // neighboring pixel

                    if (sh_data[sh_ind_img_neighbor] != 0.0f) { // only use non-zero values
                        range_kernel = sh_data[sh_ind_img_neighbor] - sh_data[sh_ind];
                        range_kernel = expf(-0.5f * (fabsf(range_kernel) * invSigma));
                        norm += range_kernel * c_domainKernel[l];
                        val  += range_kernel * c_domainKernel[l] * sh_data[sh_ind_img_neighbor];
                    }
                    l++;
                    xx++;
                }
                yy++;
            }
            if (norm != 0.0f ) {//&& isfinite(norm)
                res[ind_img] = val / norm;
            } else {
                res[ind_img] = 0.0f;
            }
        }else{
            res[ind_img] = 0.0f;
        }
    }
}

void VolumeIntegration::extractMesh(){
    // extract mesh using marching cubes
    Eigen::Vector3d volumeSize(vWidth, vHeight, slices);
    mc = new MarchingCubes(Eigen::Vector3i(vWidth, vHeight, slices), volumeSize.normalized());
    mc->computeIsoSurface(tsdf, red, green, blue, getParameters().isoValue);
}

bool VolumeIntegration::saveMesh(string name){
    return mc->savePly(dataFolder + name);
}

bool VolumeIntegration::calibrate(){
    //! [file_read]
    Settings s;
    const string inputSettingsFile = "/home/letrend/workspace/3d-kinect-scanner/default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file
    //! [file_read]

    //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
    //fout << "Settings" << s;

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    //! [get_input]
    for(;;)
    {
        Mat view;
        bool blinkOutput = false;

        if(!device->updateFrames())
            continue;
        device->getVideo(view);
        resize(view,view,Size(512, 424));

        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if( mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames )
        {
            if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
                mode = CALIBRATED;
            else
                mode = DETECTION;
        }
        if(view.empty())          // If there are no more images stop the loop
        {
            // if calibration threshold was not reached yet, calibrate now
            if( mode != CALIBRATED && !imagePoints.empty() )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
            break;
        }
        //! [get_input]

        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );

        //! [find_pattern]
        vector<Point2f> pointBuf;

        bool found;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
            case Settings::CHESSBOARD:
                found = findChessboardCorners( view, s.boardSize, pointBuf,
                                               CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
                break;
            case Settings::CIRCLES_GRID:
                found = findCirclesGrid( view, s.boardSize, pointBuf );
                break;
            case Settings::ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
                break;
            default:
                found = false;
                break;
        }
        //! [find_pattern]
        //! [pattern_found]
        if ( found)                // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if( s.calibrationPattern == Settings::CHESSBOARD)
            {
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                cornerSubPix( viewGray, pointBuf, Size(11,11),
                              Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
            }

            if( mode == CAPTURING &&  // For camera only take new samples after delay time
                (clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
            {
                imagePoints.push_back(pointBuf);
                prevTimestamp = clock();
            }

            // Draw the corners.
            drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
        }
        //! [pattern_found]
        //----------------------------- Output Text ------------------------------------------------
        //! [output_text]
        string msg = (mode == CAPTURING) ? "100/100" :
                     mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(view, view);
        //! [output_text]
        //------------------------- Video capture  output  undistorted ------------------------------
        //! [output_undistorted]
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }
        //! [output_undistorted]
        //------------------------------ Show image and check for input commands -------------------
        //! [await_input]
        imshow("Image View", view);
        char key = (char)waitKey(1);

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
            s.showUndistorsed = !s.showUndistorsed;

        if(  key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
        //! [await_input]
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    //! [show_results]
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                imageSize, CV_16SC2, map1, map2);

        for(size_t i = 0; i < s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], 1);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }
    //! [show_results]

    return 0;
}

// opencv helpers
void VolumeIntegration::convert_layered_to_interleaved(float *aOut, const float *aIn, int w, int h, int nc)
{
    if (nc==1) { memcpy(aOut, aIn, w*h*sizeof(float)); return; }
    size_t nOmega = (size_t)w*h;
    for (int y=0; y<h; y++)
    {
        for (int x=0; x<w; x++)
        {
            for (int c=0; c<nc; c++)
            {
                aOut[(nc-1-c) + nc*(x + (size_t)w*y)] = aIn[x + (size_t)w*y + nOmega*c];
            }
        }
    }
}
void VolumeIntegration::convert_layered_to_mat(cv::Mat &mOut, const float *aIn)
{
    convert_layered_to_interleaved((float*)mOut.data, aIn, mOut.cols, mOut.rows, mOut.channels());
}

void VolumeIntegration::convert_interleaved_to_layered(float *aOut, const float *aIn, int w, int h, int nc)
{
    if (nc==1) { memcpy(aOut, aIn, w*h*sizeof(float)); return; }
    size_t nOmega = (size_t)w*h;
    for (int y=0; y<h; y++)
    {
        for (int x=0; x<w; x++)
        {
            for (int c=0; c<nc; c++)
            {
                aOut[x + (size_t)w*y + nOmega*c] = aIn[(nc-1-c) + nc*(x + (size_t)w*y)];
            }
        }
    }
}
void VolumeIntegration::convert_mat_to_layered(float *aOut, const cv::Mat &mIn)
{
    convert_interleaved_to_layered(aOut, (float*)mIn.data, mIn.cols, mIn.rows, mIn.channels());
}
