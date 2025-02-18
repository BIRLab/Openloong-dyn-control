#pragma once
#include "hw_plugin.h"
#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include <thread>
#include <mutex>

class T265 : public HWPlugin {
public:
    explicit T265();
    ~T265();
    void waitReady() override;
    void updateSensorValues() override;
    void dataBusWrite(DataBus &busIn) override;

private:
    bool ready;
    rs2::pipeline pipe;

    // transform
    const Eigen::Isometry3f camera_odometry_to_odometry;
    const Eigen::Isometry3f camera_to_base_link;

    // camera feedback
    std::mutex data_mutex;
    Eigen::Vector3f translation;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    Eigen::Quaternionf rotation;
    Eigen::Vector3f angular_velocity;
    Eigen::Vector3f angular_acceleration;

    // output
    Eigen::Vector3f base_translation;
    Eigen::Vector3f base_velocity;
    Eigen::Vector3f base_acceleration;
    Eigen::Vector3f base_rpy;
    Eigen::Vector3f base_angular_velocity;
    Eigen::Vector3f base_angular_acceleration;

    void frame_cb(const rs2::frame& frame);
};
