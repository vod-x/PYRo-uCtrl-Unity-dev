/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-27 20:38:05
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-04-25 03:50:44
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
#define R_MOTOR1_OFFSET  1.30f
#define R_MOTOR2_OFFSET  -4.34f
#define L_MOTOR1_OFFSET -1.43f
#define L_MOTOR2_OFFSET  2.02f

#define WHEEL_DISTANCE 0.424f
#define CONTROL_PERIOD 0.001f
// the cofficients of lqr gain, 48 values in total, every value has 3 cofficients,
#define LQR_GAIN \
-1.5092, 38.5249, -91.7685, 80.9149,-1.4554, 49.9621, -119.6671, 108.2999,-21.0728, 24.3836, 84.8969, -167.8870,-4.0004, 3.2525, 19.0257, -33.9699,5.9353, -187.3058, 387.9502, -334.2119,0.9422, -21.2931, 23.9205, -19.3273,-3.8155, -131.2068, 590.4204, -733.9559,-7.6376, -158.3335, 745.6554, -943.4668,-4.2340, -420.5455, 794.6226, -510.1013,0.9008, -70.0920, 107.8495, -42.1720,7.2318, 675.5969, -2794.1211, 3363.0720,-0.6028, 106.4836, -386.8161, 441.5956
#define LQR_COEF_OVER_STEP \
-0.2673, 10.7832, -26.7550, 24.1743,-0.3308, 16.2297, -39.3264, 35.6179,-17.6527, 32.9779, 37.5834, -113.8599,-2.6193, 5.1268, 2.1380, -11.2595,0.5865, -66.8949, 131.2213, -108.6102,0.2422, -8.9337, 11.4009, -9.2969,-4.0884, -11.4273, 93.5874, -133.4049,-6.6605, -14.5746, 133.5190, -193.3530,-0.2295, -419.1380, 950.2799, -772.4684,0.6058, -50.6901, 107.3506, -80.7207,9.6239, 146.6775, -681.6507, 848.0795,2.3120, 23.1996, -107.4647, 133.3937
#include "pyro_wl_chassis.h"



using namespace pyro;

float infantry2_lqr_coef[48] = {LQR_GAIN};
float infantry2_lqr_coef_over_step[48] = {LQR_COEF_OVER_STEP};
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
    .yaw_motor_cfg = {
        .tx_id = dji_motor_tx_frame_t::id_5,
        .can = can_hub_t::can3,
    },
    .yaw_offset = 1.92f,
    .T_pid_cfg = {
        {
            .kp = 15.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 40.0f,
        },
        {
            .kp = 15.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 40.0f,
        }
    },
    .d_T_pid_cfg = {
        {
            .kp = 20.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        },
        {
            .kp = 20.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        }
    },
    .F_pid_cfg = {
        {
            .kp = 40.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 40.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .max_out = 200.0f,
        }
    },
    .d_F_pid_cfg = {
        {
            .kp = 240.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 240.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        }
    },
    .yaw_pid_cfg = {
        .kp = 8.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 0.0f,
        .max_out = 30.0f,
    },
    .g_yaw_pid_cfg = {
        .kp = 6.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 0.0f,
        .max_out = 40.0f
    },
    .delta_pid_cfg = {
          .kp = 10.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .integral_limit = 0.0f,
          .max_out = 20.0f,
     },
    .d_delta_pid_cfg = {
          .kp = 8.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .integral_limit = 0.0f,
          .max_out = 20.0f,
     },
    .wheel_kf_cfg = {
        {
            .x_init = (float[3]){0.0f, 0.0f, 0.0f},
            .P_init = (float[9]){
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            },
            .A = (float[9]){
                1.0f,  CONTROL_PERIOD, 0.0f,
                0.0f,  1.0f,   0.0f,
                0.0f,  0.0f,   1.0f
            },
            .B = (float[3]){
                0.0f, 0.0f, 0.0f
            },
            .H = (float[9]){
                1.0f,  0.0f, 0.0f,
                0.0f,  1.0f, 0.0f,
                0.0f,  0.0f, 1.0f
            },
            .G = (float[6]){
                CONTROL_PERIOD * CONTROL_PERIOD / 2.0f, 0.0f,
                CONTROL_PERIOD,                         0.0f,
                0.0f,                                   CONTROL_PERIOD
            },
            .Q = (float[4]){
                10000.0f, 0.0f,
                0.0f,     10000.0f
            },
            .R = (float[9]){
                0.05f, 0.0f,  0.0f,
                0.0f,  0.5f,  0.0f,
                0.0f,  0.0f,  0.005f
            }
        },
        {
            .x_init = (float[3]){0.0f, 0.0f, 0.0f},
            .P_init = (float[9]){
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            },
            .A = (float[9]){
                1.0f,  CONTROL_PERIOD,  0.0f,
                0.0f,  1.0f,   0.0f,
                0.0f,  0.0f,   1.0f
            },
            .B = (float[3]){
                0.0f, 0.0f, 0.0f
            },
            .H = (float[9]){
                1.0f,  0.0f, 0.0f,
                0.0f,  1.0f, 0.0f,
                0.0f,  0.0f, 1.0f
            },
            .G = (float[6]){
                CONTROL_PERIOD * CONTROL_PERIOD / 2.0f, 0.0f,
                CONTROL_PERIOD,                         0.0f,
                0.0f,                                   CONTROL_PERIOD
            },
            .Q = (float[4]){
                10000.0f, 0.0f,
                0.0f,     10000.0f
            },
            .R = (float[9]){
                0.05f, 0.0f,  0.0f,
                0.0f,  0.5f,  0.0f,
                0.0f,  0.0f,  0.005f
            },
        }
    },
    .lqr_coef = infantry2_lqr_coef,
    .lqr_coef_over_step = infantry2_lqr_coef_over_step,
    .wheel_radius = 0.06f,
    .reduction_ratio = 13.94f,
    .rotate_min = -45.0f,
    .rotate_max = 45.0f,
    .position_min = -12.5f,
    .position_max = 12.5f,
    .torque_min = -54.0f,
    .torque_max = 54.0f,
};