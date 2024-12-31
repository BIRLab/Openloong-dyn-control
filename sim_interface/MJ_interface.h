/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include <mujoco/mujoco.h>
#include <string>
#include "robot_interface.h"

class MJ_Interface : RobotInterface {
public:
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;
    std::vector<double> motor_tor;
    double rpy[3]{0}; // roll,pitch and yaw of baselink
    double baseQuat[4]{0}; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    double f3d[3][2]{0}; // 3D foot-end contact force, L for 1st col, R for 2nd col
    double basePos[3]{0}; // position of baselink, in world frame
    double baseAcc[3]{0};  // acceleration of baselink, in body frame
    double baseAngVel[3]{0}; // angular velocity of baselink, in body frame
    double baseLinVel[3]{0}; // linear velocity of baselink, in body frame
    const std::vector<std::string> JointName={ 
                                               
                                               "J_hip_l_roll", "J_hip_l_yaw", "J_hip_l_pitch", "J_knee_l_pitch","J_ankle_l_pitch",
                                               "J_hip_r_roll", "J_hip_r_yaw","J_hip_r_pitch", "J_knee_r_pitch", "J_ankle_r_pitch","J_waist_yaw"}; // joint name in XML file, the corresponds motors name should be M_*, ref to line 29 of MJ_Interface.cpp
    const std::string baseName="base_link";
    const std::string orientationSensorName="baselink-quat"; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    const std::string velSensorName="baselink-velocity";
    const std::string gyroSensorName="baselink-gyro";
    const std::string accSensorName="baselink-baseAcc";

    MJ_Interface(mjModel *mj_modelIn, mjData  *mj_dataIn);
    void updateSensorValues() override;
    void setMotorsTorque(std::vector<double> &tauIn) override;
    void dataBusWrite(DataBus &busIn) override;

private:
    mjModel *mj_model;
    mjData  *mj_data;
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;

    int orientataionSensorId;
    int velSensorId;
    int gyroSensorId;
    int accSensorId;
    int baseBodyId;

    double timeStep{0.001}; // second
    bool isIni{false};
};



