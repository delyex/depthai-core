#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Optional. If set (true), the ColorCamera is downscaled from 1080p to 720p.
// Otherwise (false), the aligned depth is automatically upscaled to 1080p
static std::atomic<bool> downscaleColor{true};
static constexpr int fps = 30;
// The disparity is computed at this resolution, then upscaled to RGB resolution
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    std::vector<std::string> queueNames;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();

    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");
    depthOut->setStreamName("depth");
    queueNames.push_back("depth");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(fps);
    if(downscaleColor) camRgb->setIspScale(2, 3);
    // For now, RGB needs fixed focus to properly align with depth.
    // This value was used during calibration
    camRgb->initialControl.setManualFocus(135);

    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(fps);

    stereo->initialConfig.setConfidenceThreshold(245);
    // LR-check is required for depth alignment
    stereo->setLeftRightCheck(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->disparity.link(depthOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Sets queues size and behavior
    for(const auto& name : queueNames) {
        device.getOutputQueue(name, 4, false);
    }

    std::unordered_map<std::string, cv::Mat> frame;

    while(true) {
        std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

        auto queueEvents = device.getQueueEvents(queueNames);
        for(const auto& name : queueEvents) {
            auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
            auto count = packets.size();
            if(count > 0) {
                latestPacket[name] = packets[count - 1];
            }
        }

        for(const auto& name : queueNames) {
            if(latestPacket.find(name) != latestPacket.end()) {
                if(name == "depth") {
                    frame[name] = latestPacket[name]->getFrame();
                    auto maxDisparity = stereo->initialConfig.getMaxDisparity();
                    // Optional, extend range 0..95 -> 0..255, for a better visualisation
                    if(1) frame[name].convertTo(frame[name], CV_8UC1, 255. / maxDisparity);
                    // Optional, apply false colorization
                    if(1) cv::applyColorMap(frame[name], frame[name], cv::COLORMAP_HOT);
                } else {
                    frame[name] = latestPacket[name]->getCvFrame();
                }

                cv::imshow(name, frame[name]);
            }
        }

        // Blend when both received
        if(frame.find("rgb") != frame.end() && frame.find("depth") != frame.end()) {
            // Need to have both frames in BGR format before blending
            if(frame["depth"].channels() < 3) {
                cv::cvtColor(frame["depth"], frame["depth"], cv::COLOR_GRAY2BGR);
            }
            cv::Mat blended;
            cv::addWeighted(frame["rgb"], 0.6, frame["depth"], 0.4, 0, blended);
            cv::imshow("rgb-depth", blended);
            frame.clear();
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
