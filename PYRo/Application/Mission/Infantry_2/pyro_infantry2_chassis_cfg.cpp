/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-27 20:38:05
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-19 14:51:32
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
// -0.5536 0.770
#if ROBOT_ID == INFANTRY1_ID
#define R_MOTOR1_OFFSET  -2.9376f
#define R_MOTOR2_OFFSET  2.064f
#define L_MOTOR1_OFFSET  1.2824f
#define L_MOTOR2_OFFSET  -0.6760f
#elif ROBOT_ID == INFANTRY2_ID
#define R_MOTOR1_OFFSET 1.3954f
#define R_MOTOR2_OFFSET 0.325f
#define L_MOTOR1_OFFSET -1.3868f
#define L_MOTOR2_OFFSET 2.9350f
#endif


#define WHEEL_DISTANCE 0.424f
#define CONTROL_PERIOD 0.001f
// the cofficients of lqr gain, 48 values in total, every value has 3 cofficients,
#define LQR_GAIN \
-1.5115, 42.7061, -95.2936, 80.0163,-0.7882, 39.0952, -84.2765, 71.9086,-16.4163, 11.8271, 78.1420, -132.3314,-2.6142, 2.0286, 9.3880, -16.3876,2.3388, -121.5314, 208.8887, -166.5677,0.4561, -13.1334, 4.3057, -2.3368,-10.0811, -134.1174, 631.2455, -783.5426,-12.3847, -112.6701, 560.5343, -708.7229,-3.3483, -299.8473, 486.8654, -242.6445,1.6891, -39.2584, 47.2859, -3.9819,13.8432, 456.1463, -1818.4553, 2130.7907,0.6589, 75.2115, -250.7503, 274.1828
#define LQR_COEF_OVER_STEP \
-0.9554, 66.1323, -165.7462, 154.5218,0.2570, 48.6392, -110.4089, 101.5732,-24.0850, 45.5699, 8.9053, -81.4426,-2.9647, 4.8903, 0.0668, -6.7078,0.1855, -121.0960, 184.5589, -154.2871,0.2188, -11.1556, -9.9992, 8.5742,-23.3047, -18.2274, 275.3656, -409.4914,-20.5748, -1.0771, 166.9874, -264.0747,-8.1442, -343.9401, 762.2338, -625.7714,2.0788, -38.6943, 78.5594, -59.1337,24.7244, 156.6254, -687.3910, 833.4701,1.3319, 34.4804, -97.0465, 100.3895
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
#if ROBOT_ID == INFANTRY1_ID
    .yaw_offset = -2.49f,
#elif ROBOT_ID == INFANTRY1_ID 
    .yaw_offset = 0.384f,
#endif
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
            .kp = 80.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 80.0f,
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
    .roll_pid_cfg = {
          .kp = 0.0f,
          .ki = 0.0f,
          .kd = 0.0f,
          .integral_limit = 0.0f,
          .max_out = 30.0f,
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
    .position_min = -3.141593f,
    .position_max = 3.141593f,
    .torque_min = -54.0f,
    .torque_max = 54.0f,
#if ROBOT_ID == INFANTRY1_ID
    .power_ctrl_cfg = {
        .k1 = {0.115f, 0.167f},
        .k2 = {2.44f, 2.26f},
        .k3 = {2.0f, 2.0f},
        .energy_kp = 0.0f,
        .energy_kd = 0.0f,
        .min_max_power = 0.0f,
        .cap_max_bonus = 0.0f,
    },
#elif ROBOT_ID == INFANTRY2_ID
#endif
};