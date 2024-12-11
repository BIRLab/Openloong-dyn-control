#pragma once
#include "robot_interface.h"
#include "serial.h"
#include <thread>

class AHRS {
public:
    explicit AHRS();
    ~AHRS();
    void dataBusWrite(DataBus &busIn);

private:
    serial::Serial sp;
    bool running;
    std::thread loop_thread;

    // IMU
    float gyroscope_x;          //unit: rad/s
    float gyroscope_y;          //unit: rad/s
    float gyroscope_z;          //unit: rad/s
    float accelerometer_x;      //unit: m/s^2
    float accelerometer_y;      //unit: m/s^2
    float accelerometer_z;      //unit: m/s^2

    // AHRS
    float roll_speed;           //unit: rad/s
    float pitch_speed;          //unit: rad/s
    float yaw_speed;            //unit: rad/s
    float roll;                 //unit: rad
    float pitch;                //unit: rad
    float yaw;                  //unit: rad

    // INS
    float velocity_x;           //unit: m/s
    float velocity_y;           //unit: m/s
    float velocity_z;           //unit: m/s

    void process_data();
};
