#include "HW_Interface.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"
#include "scheduler.h"

const double timestep = 1e-3;

int main(int argc, const char** argv)
{
    // ini classes
    HW_Interface hw_interface;
    Pin_KinDyn kinDynSolver("../models/bip4_des/bip4_des.urdf");
    DataBus RobotState(kinDynSolver.model_nv);
    PVT_Ctr pvtCtr(timestep,"../common/joint_ctrl_config.json");
    DataLogger logger("../record/datalog.log");

    int model_nv = kinDynSolver.model_nv;

    std::vector<double> motors_pos_des(model_nv - 6, 0.1);
    std::vector<double> motors_pos_cur(model_nv - 6, 0);
    std::vector<double> motors_vel_des(model_nv - 6, 0);
    std::vector<double> motors_vel_cur(model_nv - 6, 0);
    std::vector<double> motors_tau_des(model_nv - 6, 0);
    std::vector<double> motors_tau_cur(model_nv - 6, 0);

    // register variable name for data logger
    logger.addIterm("time", 1);
    logger.addIterm("frequency", 1);
    logger.addIterm("motors_pos_cur", model_nv - 6);
    logger.addIterm("motors_vel_cur", model_nv - 6);
    logger.addIterm("motors_tor_cur", model_nv - 6);
    logger.addIterm("motors_tor_out", model_nv - 6);
    logger.finishItermAdding();

    // stage time
    double steppingTime = 2;
    double runningTime = 10;

    // reset pvt controller
    hw_interface.updateSensorValues();
    hw_interface.dataBusWrite(RobotState);
    pvtCtr.motor_pos_des_old = RobotState.motors_pos_cur;

    // loop rate
    Rate rate(1 / timestep);

    while (true)
    {
        double currentTime = rate.now();
        if (currentTime > runningTime) {
            break;
        }

        // update sensor values
        hw_interface.updateSensorValues();
        hw_interface.dataBusWrite(RobotState);

        if (currentTime < steppingTime) {
            RobotState.motors_pos_des = motors_pos_cur;
            RobotState.motors_vel_des = motors_vel_cur;
            RobotState.motors_tor_des = motors_tau_cur;
        } else {
            RobotState.motors_pos_des = motors_pos_des;
            RobotState.motors_vel_des = motors_vel_des;
            RobotState.motors_tor_des = motors_tau_des;
        }

        pvtCtr.dataBusRead(RobotState);
        pvtCtr.calMotorsPVT();
        pvtCtr.dataBusWrite(RobotState);

        hw_interface.setMotorsTorque(RobotState.motors_tor_out);

        logger.startNewLine();
        logger.recItermData("time", currentTime);
        logger.recItermData("frequency", rate.frequency());
        logger.recItermData("motors_pos_cur",RobotState.motors_pos_cur);
        logger.recItermData("motors_vel_cur",RobotState.motors_vel_cur);
        logger.recItermData("motors_tor_cur",RobotState.motors_tor_cur);
        logger.recItermData("motors_tor_out",RobotState.motors_tor_out);
        logger.finishLine();

        rate.sleep();
    }
    return 0;
}
