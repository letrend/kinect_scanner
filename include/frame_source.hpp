#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include "kinect.hpp"

struct CameraIntrinsics {
    int width = 512;
    int height = 424;
    float fx = 365.456f;
    float fy = 365.456f;
    float cx = 254.878f;
    float cy = 205.395f;
};

class FrameSource {
public:
    virtual ~FrameSource() {}
    virtual bool updateFrames() = 0;
    virtual void getDepthMM(cv::Mat &output) = 0;
    virtual void getRgbMapped2Depth(cv::Mat &output) = 0;
    virtual void getVideo(cv::Mat &output) = 0;
    virtual CameraIntrinsics intrinsics() const = 0;
    virtual void setCameraPose(const Eigen::Matrix4f &) {}
};

class FreenectFrameSource : public FrameSource {
public:
    FreenectFrameSource() : m_device(new MyFreenectDevice) {}

    bool updateFrames() override { return m_device->updateFrames(); }
    void getDepthMM(cv::Mat &output) override { m_device->getDepthMM(output); }
    void getRgbMapped2Depth(cv::Mat &output) override { m_device->getRgbMapped2Depth(output); }
    void getVideo(cv::Mat &output) override { m_device->getVideo(output); }

    CameraIntrinsics intrinsics() const override {
        CameraIntrinsics c;
        c.width = 512;
        c.height = 424;
        c.fx = m_device->irCameraParams.fx;
        c.fy = m_device->irCameraParams.fy;
        c.cx = m_device->irCameraParams.cx;
        c.cy = m_device->irCameraParams.cy;
        return c;
    }

private:
    std::unique_ptr<MyFreenectDevice> m_device;
};
