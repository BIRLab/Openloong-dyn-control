#pragma once
#include "robot_interface.h"
#include <Eigen/Geometry>
#include <thread>
#include <mutex>

class T265 {
public:
    explicit T265();
    ~T265();
    void wait_ready() const;
    void updateSensorValues();
    void dataBusWrite(DataBus &busIn);

private:
    bool running;
    bool ready;
    std::thread loop_thread;

    // transform
    const Eigen::Matrix4f camera_to_base_link;

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
    Eigen::Quaternionf base_rotation;
    Eigen::Vector3f base_angular_velocity;
    Eigen::Vector3f base_angular_acceleration;
    Eigen::Vector3f base_rpy;

    void processData();
};
