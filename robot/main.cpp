#include "HW_Interface.h"
#include "useful_math.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"
#include "wbc_priority.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
const double timestep = 1e-3;

int main(int argc, const char** argv)
{
    // ini classes
    HW_Interface hw_interface;
    Pin_KinDyn kinDynSolver("../models/bip4_des/bip4_des.urdf");
    DataBus RobotState(kinDynSolver.model_nv);
    WBC_priority WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, timestep);
    GaitScheduler gaitScheduler(0.4, timestep);
    PVT_Ctr pvtCtr(timestep,"../common/joint_ctrl_config.json");
    FootPlacement footPlacement;
    JoyStickInterpreter jsInterp(timestep);
    DataLogger logger("../record/datalog.log");

    // variables ini
    double stand_legLength = 0.76;  // 1.01)desired baselink height
    double foot_height = 0.07;      // distance between the foot ankel joint and the bottom
    double xv_des = 0.4;            // desired velocity in x direction

    RobotState.width_hips = 0.229;
    footPlacement.kp_vx = 0.03;
    footPlacement.kp_vy = 0.035;
    footPlacement.kp_wz = 0.03;
    footPlacement.stepHeight = 0.25;
    footPlacement.legLength = stand_legLength;
    int model_nv = kinDynSolver.model_nv;

    // ini position and posture for foot-end and hand
    std::vector<double> motors_pos_des(model_nv - 6, 0);
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);
    Eigen::Vector3d fe_l_pos_L_des={-0.05, 0.085, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des={-0.05, -0.085, -stand_legLength};

    Eigen::Vector3d fe_l_eul_L_des={-0.000, -0.008, -0.000};
    Eigen::Vector3d fe_r_eul_L_des={0.000, -0.008, 0.000};
    Eigen::Matrix3d fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

    auto resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);

    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(18, 1);
    qIniDes.block(7, 0, 11, 1) = resLeg.jointPosRes;
    WBC_solv.setQini(qIniDes, RobotState.q);

    // register variable name for data logger
    logger.addIterm("simTime", 1);
    logger.addIterm("motors_pos_cur", model_nv - 6);
    logger.addIterm("motors_vel_cur", model_nv - 6);
    logger.addIterm("rpy", 3);
    logger.addIterm("fL", 3);
    logger.addIterm("fR", 3);
    logger.addIterm("basePos", 3);
    logger.addIterm("baseLinVel", 3);
    logger.addIterm("baseAcc", 3);
    logger.addIterm("baseAngVel", 3);
    logger.finishItermAdding();

    /// stage time
    double startSteppingTime = 60;
    double startWalkingTime = 120;
    auto startTime = std::chrono::steady_clock::now();
    auto wakeUpTime = startTime + 1ms;

    while (true)
    {
        auto currentTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
        if (currentTime > startSteppingTime) {
            break;
        }

        hw_interface.updateSensorValues();
        hw_interface.dataBusWrite(RobotState);

        // update kinematics and dynamics info
        kinDynSolver.dataBusRead(RobotState);
        kinDynSolver.computeJ_dJ();
        kinDynSolver.computeDyn();
        kinDynSolver.dataBusWrite(RobotState);

        if (currentTime > startWalkingTime) {
            jsInterp.setWzDesLPara(0, 1);
            jsInterp.setVxDesLPara(xv_des, 2.0); // jsInterp.setVxDesLPara(0.9,1);
            RobotState.motionState = DataBus::Walk; // start walking
        } else
            jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));//Set initial position.

        jsInterp.step();
        RobotState.js_pos_des(2) = stand_legLength + foot_height; // pos z is not assigned in jyInterp
        jsInterp.dataBusWrite(RobotState); // only pos x, pos y, theta z, vel x, vel y , omega z are rewrote.

        if (currentTime >= startSteppingTime) {
            // gait scheduler
            gaitScheduler.dataBusRead(RobotState);
            gaitScheduler.step();
            gaitScheduler.dataBusWrite(RobotState);

            footPlacement.dataBusRead(RobotState);
            footPlacement.getSwingPos();
            footPlacement.dataBusWrite(RobotState);
        }

        // ------------- WBC ------------
        // WBC input
        RobotState.Fr_ff = Eigen::VectorXd::Zero(12); //feedforwrd force (ground reaction force) computed by MPC, is the first horizon result of fe_react_tau_cmd.
        RobotState.des_ddq = Eigen::VectorXd::Zero(17);
        RobotState.des_dq = Eigen::VectorXd::Zero(17);
        RobotState.des_delta_q = Eigen::VectorXd::Zero(17);
        RobotState.base_rpy_des << 0, 0, jsInterp.thetaZ;
        RobotState.base_pos_des(2) = stand_legLength + foot_height;

        RobotState.Fr_ff << 0, 0, 360, 0, 0, 0, 0, 0, 360, 0, 0, 0;

        // adjust des_delata_q, des_dq and des_ddq to achieve forward walking
        if (currentTime > startWalkingTime + 1) {
            RobotState.des_delta_q.block<2, 1>(0, 0) << jsInterp.vx_W * timestep, jsInterp.vy_W * timestep;
            RobotState.des_delta_q(5) = jsInterp.wz_L * timestep;
            RobotState.des_dq.block<2, 1>(0, 0) << jsInterp.vx_W, jsInterp.vy_W;
            RobotState.des_dq(5) = jsInterp.wz_L;

            double k = 5;
            RobotState.des_ddq.block<2, 1>(0, 0) << k * (jsInterp.vx_W - RobotState.dq(0)), k * (jsInterp.vy_W - RobotState.dq(1));
            RobotState.des_ddq(5) = k * (jsInterp.wz_L - RobotState.dq(5));
        }

        // WBC Calculation
        WBC_solv.dataBusRead(RobotState);
        WBC_solv.computeDdq(kinDynSolver);
        WBC_solv.computeTau();
        WBC_solv.dataBusWrite(RobotState);

        // get the final joint command
        if (currentTime <= startSteppingTime) {
            RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes);
            RobotState.motors_vel_des = motors_vel_des;
            RobotState.motors_tor_des = motors_tau_des;
        } else {
            // RobotState.wbc_delta_q_final = Eigen::VectorXd::Zero(mj_model->nv);
            Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(
            RobotState.q,
            RobotState.wbc_delta_q_final); // intergrate the position q with
                                            // velocity dq, for floating base
                                            // dynamics
            RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, model_nv - 6, 1));
            // RobotState.motors_pos_des= eigen2std(resLeg.jointPosRes);
            RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
            RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
        }

        // ------------- pvt ------------
        pvtCtr.dataBusRead(RobotState);
        if (currentTime <= 3)
        {
          pvtCtr.calMotorsPVT(100.0 / 1000.0 / 180.0 * 3.1415);
        } else {
            pvtCtr.setJointPD(100, 10, "J_ankle_l_pitch");
            pvtCtr.setJointPD(100, 10, "J_ankle_r_pitch");
            pvtCtr.setJointPD(1000, 100, "J_knee_l_pitch");
            pvtCtr.setJointPD(1000, 100, "J_knee_r_pitch");
            pvtCtr.calMotorsPVT();
        }
        pvtCtr.dataBusWrite(RobotState);

        hw_interface.setMotorsTorque(RobotState.motors_tor_out);

        logger.startNewLine();
        logger.recItermData("simTime", currentTime);
        logger.recItermData("motors_pos_cur",RobotState.motors_pos_cur);
        logger.recItermData("motors_vel_cur",RobotState.motors_vel_cur);
        logger.recItermData("rpy",RobotState.rpy);
        logger.recItermData("fL",RobotState.fL);
        logger.recItermData("fR",RobotState.fR);
        logger.recItermData("basePos",RobotState.basePos);
        logger.recItermData("baseLinVel",RobotState.baseLinVel);
        logger.recItermData("baseAcc",RobotState.baseAcc);
        logger.recItermData("baseAngVel",RobotState.baseAngVel);
        logger.finishLine();

        std::this_thread::sleep_until(wakeUpTime);
        wakeUpTime += 1ms;
    }
    return 0;
}
