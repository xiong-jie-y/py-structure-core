#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <mutex>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "ST/CaptureSession.h"

namespace py = pybind11;

std::mutex frame_lock;
std::mutex depth_lock;

void alignDepthPointToVisFrame(const int *depth_point, int *visible_point, double *depth_3d_point, const double depth_in_meters,
                               const ST::Intrinsics &depth_intrinsics, const ST::Intrinsics &color_intrinsics, const ST::Matrix4 &depth_color_transform)
{
    depth_3d_point[0] = depth_in_meters * (depth_point[0] - depth_intrinsics.cx) / depth_intrinsics.fx;
    depth_3d_point[1] = depth_in_meters * (depth_point[1] - depth_intrinsics.cy) / depth_intrinsics.fy;
    depth_3d_point[2] = depth_in_meters;

    ST::Vector3f xyz_depth(depth_3d_point[0], depth_3d_point[1], depth_3d_point[2]);

    auto xyz_visible = depth_color_transform * xyz_depth;

    float float_visible_col = (xyz_visible.x * color_intrinsics.fx / xyz_visible.z) + color_intrinsics.cx;
    float float_visible_row = (xyz_visible.y * color_intrinsics.fy / xyz_visible.z) + color_intrinsics.cy;

    visible_point[0] = std::round(float_visible_col);
    visible_point[1] = std::round(float_visible_row);
}

py::array_t<float> GetAlignDepthFrameToColor(const ST::DepthFrame &depthFrame, const ST::ColorFrame &visFrame)
{
    int output_image_size = visFrame.width() * visFrame.height();
    py::array_t<float> registered_depth(output_image_size);
    py::buffer_info buf2_2 = registered_depth.request();
    float *registered_data = (float *)buf2_2.ptr;

    for (int i = 0; i < output_image_size; i++)
    {
        registered_data[i] = -1;
    }

    const auto visible_from_depth = depthFrame.visibleCameraPoseInDepthCoordinateFrame();
    ST::Intrinsics depth_intrinsics = depthFrame.intrinsics();
    assert(depth_intrinsics.k1k2k3p1p2AreZero());

    const ST::ColorFrame undistorted_color_frame = visFrame.undistorted();
    ST::Intrinsics color_intrinsics = undistorted_color_frame.intrinsics();

    const float *buf_depth = depthFrame.depthInMillimeters();
    double to_meters_multiplier = 0.001;

    for (int y = 0; y < depthFrame.height(); y++)
    {
        for (int x = 0; x < depthFrame.width(); x++)
        {
            std::size_t pixel_offset = (y * depthFrame.width()) + x;
            auto depth_in_meters = buf_depth[pixel_offset] * to_meters_multiplier;

            if (!std::isnan(depth_in_meters))
            {
                int depth_point[2] = {x, y};
                int visible_point[2] = {0};
                double depth_3d_point[3] = {0};

                alignDepthPointToVisFrame(depth_point, visible_point, depth_3d_point, depth_in_meters,
                                          depth_intrinsics, color_intrinsics, visible_from_depth);

                int pixel_color_offset = (visible_point[1] * undistorted_color_frame.width()) + visible_point[0];

                if (pixel_color_offset > 0 && pixel_color_offset < depthFrame.height() * depthFrame.width())
                {
                    registered_data[pixel_color_offset] = depth_in_meters;
                }
            }
        }
    }
    return registered_depth;
}

struct SessionDelegate : ST::CaptureSessionDelegate
{
    void captureSessionEventDidOccur(ST::CaptureSession *session, ST::CaptureSessionEventId event) override
    {
        printf("Received capture session event %d (%s)\n", (int)event, ST::CaptureSessionSample::toString(event));
        switch (event)
        {
        case ST::CaptureSessionEventId::Booting:
            break;
        case ST::CaptureSessionEventId::Connected:
            printf("Starting streams...\n");
            printf("Sensor Serial Number is %s \n ", session->sensorInfo().serialNumber);
            session->startStreaming();
            break;
        case ST::CaptureSessionEventId::Disconnected:
        case ST::CaptureSessionEventId::Error:
            printf("Capture session error\n");
            exit(1);
            break;
        default:
            printf("Capture session event unhandled\n");
        }
    }

    void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample &sample) override
    {
        if (sample.type == ST::CaptureSessionSample::Type::SynchronizedFrames &&
            sample.visibleFrame.isValid() && sample.depthFrame.isValid())
        {
            std::lock_guard<std::mutex> lk(frame_lock);
            latest_color_frame = sample;
            current_id++;
        }
    }

    ST::CaptureSessionSample latest_color_frame;

    long current_id;
};

class StructureCoreReader
{
public:
    StructureCoreReader(int depth_range_mode, int depth_resolution)
    {
        settings.source = ST::CaptureSessionSourceId::StructureCore;
        settings.applyExpensiveCorrection = true;
        settings.structureCore.depthEnabled = true;
        settings.structureCore.visibleEnabled = true;
        settings.structureCore.infraredEnabled = true;
        settings.dispatcher = ST::CaptureSessionDispatcherId::BackgroundThread;
        settings.frameSyncEnabled = true;
        settings.structureCore.accelerometerEnabled = true;
        settings.structureCore.gyroscopeEnabled = true;
        settings.structureCore.depthRangeMode = static_cast<ST::StructureCoreDepthRangeMode>(depth_range_mode);
        settings.structureCore.depthResolution = static_cast<ST::StructureCoreDepthResolution>(depth_resolution);
        settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::AccelAndGyro_100Hz;

        session.setDelegate(&delegate);
        if (!session.startMonitoring(settings))
        {
            printf("Failed to initialize capture session!\n");
            exit(1);
        }
        delegate = delegate;
    }

    py::tuple GetLatestFrame()
    {
        {
            std::lock_guard<std::mutex> lk(frame_lock);

            py::tuple result;
            const auto &sample = delegate.latest_color_frame;
            int size = 3 * sample.visibleFrame.width() * sample.visibleFrame.height();
            py::array_t<unsigned char> result1(size);
            py::buffer_info buf2 = result1.request();
            unsigned char *dstPt = (unsigned char *)buf2.ptr;

            auto pt = sample.visibleFrame.rgbData();

            for (int i = 0; i < size; i++)
            {
                dstPt[i] = pt[i];
            }

            py::array_t<float> result2 =
                GetAlignDepthFrameToColor(sample.depthFrame, sample.visibleFrame);

            previous_id = delegate.current_id;

            return py::make_tuple(result1, result2);
        }
    }
    bool NewFrameArrived()
    {
        return previous_id != delegate.current_id;
    }

    py::dict GetIntrinsic()
    {
        py::dict dict;
        std::lock_guard<std::mutex> lk(frame_lock);
        const auto &sample = delegate.latest_color_frame;
        auto &depth = sample.visibleFrame;

        py::list intrinsic;
        // first col.
        intrinsic.append(depth.intrinsics().fx);
        intrinsic.append(0);
        intrinsic.append(0);

        // second col.
        intrinsic.append(0);
        intrinsic.append(depth.intrinsics().fx);
        intrinsic.append(0);

        // 3 col.
        intrinsic.append(depth.intrinsics().cx);
        intrinsic.append(depth.intrinsics().cy);
        intrinsic.append(1);

        dict["width"] = depth.intrinsics().width;
        dict["height"] = depth.intrinsics().height;
        dict["intrinsic_matrix"] = intrinsic;

        return dict;
    }

    ST::CaptureSessionSettings settings;
    SessionDelegate delegate;
    ST::CaptureSession session;
    long previous_id = 0;
};

PYBIND11_MODULE(structure_core, m)
{
    py::class_<StructureCoreReader>(m, "StructureCoreReader")
        .def(py::init<int, int>())
        .def("get_latest_frame", &StructureCoreReader::GetLatestFrame)
        .def("get_intrinsic", &StructureCoreReader::GetIntrinsic)
        .def("new_frame_arrived", &StructureCoreReader::NewFrameArrived);
}