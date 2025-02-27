#pragma once
#include "hw_plugin.h"
#include "serial.h"
#include <thread>
#include <mutex>

class AHRS : public HWPlugin {
public:
    explicit AHRS();
    ~AHRS();
    void waitReady() override;
    void updateSensorValues() override;
    void dataBusWrite(DataBus &busIn) override;

private:
    serial::Serial sp;
    bool running;
    bool ready;
    std::thread loop_thread;

    // Mutex
    std::mutex data_mutex;

    // IMU
    float gyroscope_x{};          //unit: rad/s
    float gyroscope_y{};          //unit: rad/s
    float gyroscope_z{};          //unit: rad/s
    float accelerometer_x{};      //unit: m/s^2
    float accelerometer_y{};      //unit: m/s^2
    float accelerometer_z{};      //unit: m/s^2

    // AHRS
    float roll_speed{};           //unit: rad/s
    float pitch_speed{};          //unit: rad/s
    float yaw_speed{};            //unit: rad/s
    float roll{};                 //unit: rad
    float pitch{};                //unit: rad
    float yaw{};                  //unit: rad

    // INS
    float velocity_x{};           //unit: m/s
    float velocity_y{};           //unit: m/s
    float velocity_z{};           //unit: m/s

    void process_data();
};
