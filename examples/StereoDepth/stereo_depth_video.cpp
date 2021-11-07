#include <iostream>

// Inludes common necessary includes for development using depthai library
#include <Converter.h>
#include <System.h>

#include "depthai/depthai.hpp"

static std::atomic<bool> withDepth{true};

static std::atomic<bool> outputDepth{false};
static std::atomic<bool> outputRectified{true};
static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};

int main(int argc, char** argv) {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = withDepth ? pipeline.create<dai::node::StereoDepth>() : nullptr;

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifL = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifR = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    if(withDepth) {
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    if(withDepth) {
        // Linking
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);

        if(outputRectified) {
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }
    } else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("left", 8, false);
    auto rightQueue = device.getOutputQueue("right", 8, false);
    auto rectifLeftQueue = withDepth ? device.getOutputQueue("rectified_left", 8, false) : nullptr;
    auto rectifRightQueue = withDepth ? device.getOutputQueue("rectified_right", 8, false) : nullptr;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    while(true) {
        if(withDepth) {
            if(outputRectified) {
                auto rectifL = rectifLeftQueue->get<dai::ImgFrame>();
                cv::Mat imLeftRect = rectifL->getFrame();

                auto rectifR = rectifRightQueue->get<dai::ImgFrame>();
                cv::Mat imRightRect = rectifR->getFrame();
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
                cout << "运行时间: " << tframe << endl;
                cv::Mat Tcw = SLAM.TrackStereo(imLeftRect, imRightRect, tframe);
            }
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            SLAM.Shutdown();
            return 0;
        }
    }
    return 0;
}
