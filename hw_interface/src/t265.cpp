#include "t265.h"
#include "transforms.h"
#include "cl_color.h"
#include <map>

using namespace std::chrono_literals;
using namespace std::placeholders;

static bool device_with_streams(const std::vector <rs2_stream>& stream_requests, std::string& out_serial)
{
    rs2::context ctx;
    auto devs = ctx.query_devices();
    std::vector <rs2_stream> unavailable_streams = stream_requests;
    for (auto dev : devs)
    {
        std::map<rs2_stream, bool> found_streams;
        for (auto& type : stream_requests)
        {
            found_streams[type] = false;
            for (auto& sensor : dev.query_sensors())
            {
                for (auto& profile : sensor.get_stream_profiles())
                {
                    if (profile.stream_type() == type)
                    {
                        found_streams[type] = true;
                        unavailable_streams.erase(std::remove(unavailable_streams.begin(), unavailable_streams.end(), type), unavailable_streams.end());
                        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
                            out_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    }
                }
            }
        }
        // Check if all streams are found in current device
        bool found_all_streams = true;
        for (auto& stream : found_streams)
        {
            if (!stream.second)
            {
                found_all_streams = false;
                break;
            }
        }
        if (found_all_streams)
            return true;
    }
    // After scanning all devices, not all requested streams were found
    return false;
}

T265::T265() : ready{false}, camera_to_base_link{Transforms::t265_to_base_link} {
    std::string serial;
    assert(device_with_streams({ RS2_STREAM_POSE}, serial) || !fprintf(stderr, "T265 not found!\n"));

    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    pipe.start(cfg, std::bind(&T265::frame_cb, this, _1));
}

T265::~T265() {
    pipe.stop();
}

void T265::wait_ready() const {
    while (!ready) {
        std::this_thread::sleep_for(10ms);
    }
    std::cout << CL_BOLDGREEN << "t265 booted successfully" << CL_RESET << std::endl;
}

void T265::updateSensorValues() {
    std::scoped_lock<std::mutex> lock { data_mutex };

    Eigen::Matrix3f R_camera_to_base_link = camera_to_base_link.block<3, 3>(0, 0);
    Eigen::Vector3f t_camera_to_base_link = camera_to_base_link.block<3, 1>(0, 3);
    Eigen::Quaternionf q_camera_to_base_link(R_camera_to_base_link);

    base_translation = R_camera_to_base_link * translation + t_camera_to_base_link;
    base_velocity = R_camera_to_base_link * velocity;
    base_acceleration = R_camera_to_base_link * acceleration;
    base_rotation = q_camera_to_base_link * rotation;
    base_angular_velocity = R_camera_to_base_link * angular_velocity;
    base_angular_acceleration = R_camera_to_base_link * angular_acceleration;
    base_rpy = base_rotation.toRotationMatrix().eulerAngles(0, 1, 2);
}

void T265::dataBusWrite(DataBus &busIn) {
    std::scoped_lock<std::mutex> lock { data_mutex };

    busIn.rpy[0] = base_rpy[0];
    busIn.rpy[1] = base_rpy[1];
    busIn.rpy[2] = base_rpy[2];

    busIn.basePos[0] = base_translation[0];
    busIn.basePos[1] = base_translation[1];
    busIn.basePos[2] = base_translation[2];

    busIn.baseLinVel[0] = base_velocity[0];
    busIn.baseLinVel[1] = base_velocity[1];
    busIn.baseLinVel[2] = base_velocity[2];

    busIn.baseAcc[0] = base_acceleration[0];
    busIn.baseAcc[1] = base_acceleration[1];
    busIn.baseAcc[2] = base_acceleration[2];

    busIn.baseAngVel[0] = base_angular_velocity[0];
    busIn.baseAngVel[1] = base_angular_velocity[1];
    busIn.baseAngVel[2] = base_angular_velocity[2];

    busIn.updateQ();
}

void T265::frame_cb(const rs2::frame& frame) {
    if (auto fp = frame.as<rs2::pose_frame>()) {
        rs2_pose pose_data = fp.get_pose_data();

        {
            std::scoped_lock<std::mutex> lock{data_mutex};

            translation[0] = pose_data.translation.x;
            translation[1] = pose_data.translation.y;
            translation[2] = pose_data.translation.z;

            velocity[0] = pose_data.velocity.x;
            velocity[1] = pose_data.velocity.y;
            velocity[2] = pose_data.velocity.z;

            acceleration[0] = pose_data.acceleration.x;
            acceleration[1] = pose_data.acceleration.y;
            acceleration[2] = pose_data.acceleration.z;

            rotation.w() = pose_data.rotation.w;
            rotation.x() = pose_data.rotation.x;
            rotation.y() = pose_data.rotation.y;
            rotation.z() = pose_data.rotation.z;

            angular_velocity[0] = pose_data.angular_velocity.x;
            angular_velocity[1] = pose_data.angular_velocity.y;
            angular_velocity[2] = pose_data.angular_velocity.z;

            angular_acceleration[0] = pose_data.angular_acceleration.x;
            angular_acceleration[1] = pose_data.angular_acceleration.y;
            angular_acceleration[2] = pose_data.angular_acceleration.z;

            ready = true;
        }
    }
}
