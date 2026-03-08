/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-27 20:38:05
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-03-08 02:59:00
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
//the cofficients for phi calculation
#define PHI_K0 22506.0f
#define PHI_K1 475976946.0f
#define PHI_K2 296955904.0f
#define PHI_K3 179021042.0f
#define PHI_K4 18922.0f
//the cofficients for position in polar coordinates
#define POLAR_K0 236976927.0f
#define POLAR_K1 21059.0f
#define POLAR_K2 946100000.0f
#define POLAR_K3 100000.0f
//the cofficients for VMC transformation matrix
#define TRANS_K0 21059.0f
#define TRANS_K1 100000.0f
//the offsets of motors, which is the angle between the zero point of motor and
// the forward direction of robot, counter clockwise is positive(rad)
#define R_MOTOR1_OFFSET 2.66f
#define R_MOTOR2_OFFSET 3.979092653f
#define L_MOTOR1_OFFSET -1.323f
#define L_MOTOR2_OFFSET 1.226f
// the cofficients of lqr gain, 48 values in total, every value has 3 cofficients,
#define LQR_GAIN \
-0.0103, 0.6830, -1.7189, 1.6066,-0.1995, 13.9194, -34.9509, 32.6658,-3.8813, 7.6184, 0.4808, -11.9552,-1.8284, 3.5294, 0.2086, -5.4955,-0.5019, -63.9886, 126.0836, -108.6725,0.0410, -7.6610, 9.3843, -8.8623,-0.2301, -0.2284, 2.9825, -4.3880,-4.7276, -4.5028, 60.4132, -89.0854,-0.4622, -55.8088, 123.7312, -101.4324,-0.0636, -26.1096, 57.4619, -46.7770,8.1935, 83.5282, -370.5331, 449.8109,1.9071, 9.6819, -43.8559, 54.2105
#include "pyro_wl_chassis.h"

using namespace pyro;

float infantry2_lqr_coef[48] = {LQR_GAIN};
wl_chassis_cfg_t infantry2_chassis_cfg = {
    .phi_k = {
        .k0 = PHI_K0,
        .k1 = PHI_K1,
        .k2 = PHI_K2,
        .k3 = PHI_K3,
        .k4 = PHI_K4,
    },
    .polar_k = {
        .k0 = POLAR_K0,
        .k1 = POLAR_K1,
        .k2 = POLAR_K2,
        .k3 = POLAR_K3,
    },
    .vmc_k = {
        .k0 = TRANS_K0,
        .k1 = TRANS_K1,
    },
    .joint_motor_cfg = {
        {
            .tx_id = 0x01,
            .rx_id = 0x11,
            .can = can_hub_t::can1,
            .offset_angle = R_MOTOR1_OFFSET,
        },
        {
            .tx_id = 0x02,
            .rx_id = 0x12,
            .can = can_hub_t::can1,
            .offset_angle = R_MOTOR2_OFFSET,
        },
        {
            .tx_id = 0x03,
            .rx_id = 0x13,
            .can = can_hub_t::can2,
            .offset_angle = L_MOTOR1_OFFSET,
        },
        {
            .tx_id = 0x04,
            .rx_id = 0x14,
            .can = can_hub_t::can2,
            .offset_angle = L_MOTOR2_OFFSET,
        }
    },
    .wheel_motor_cfg = {
        {
            .tx_id = dji_motor_tx_frame_t::id_3,
            .can = can_hub_t::can1,
        },
        {
            .tx_id = dji_motor_tx_frame_t::id_2,
            .can = can_hub_t::can2,
        }
    },
    .T_pid_cfg = {
        {
            .kp = 6.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        },
        {
            .kp = 6.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        }
    },
    .d_T_pid_cfg = {
        {
            .kp = 2.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        },
        {
            .kp = 2.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        }
    },
    .F_pid_cfg = {
        {
            .kp = 200.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 200.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        }
    },
    .d_F_pid_cfg = {
        {
            .kp = 40.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 40.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        }
    },
    .yaw_pid_cfg = {
        .kp = 4.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 0.0f,
        .max_out = 20.0f,
    },
    .g_yaw_pid_cfg = {
        .kp = 0.0005f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 0.0f,
        .max_out =0.1f
    },
    .lqr_coef = infantry2_lqr_coef,
    .wheel_radius = 0.06f,
    .reduction_ratio = 13.94f,
    .rotate_min = -45.0f,
    .rotate_max = 45.0f,
    .position_min = -12.5f,
    .position_max = 12.5f,
    .torque_min = -54.0f,
    .torque_max = 54.0f,
};