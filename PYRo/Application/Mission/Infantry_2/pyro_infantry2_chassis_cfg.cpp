/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-27 20:38:05
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-04-22 14:33:30
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
#define L_MOTOR1_OFFSET 1.40f
#define L_MOTOR2_OFFSET  0.15f

#define WHEEL_DISTANCE 0.424f
#define CONTROL_PERIOD 0.001f
// the cofficients of lqr gain, 48 values in total, every value has 3 cofficients,
#define LQR_GAIN \
-1.1697, 47.6121, -124.6271, 120.7091,-0.7118, 52.4322, -134.5380, 132.2344,-23.3583, 49.5935, -8.8666, -59.0511,-3.4806, 5.8068, 2.3636, -11.9952,2.7471, -161.0435, 337.2613, -317.0778,0.6436, -21.8952, 28.7529, -30.1870,-7.6056, -63.0852, 330.7837, -428.2632,-10.0225, -63.4379, 351.3147, -462.3101,-16.1708, -296.4998, 652.3987, -531.2115,-0.5676, -37.4369, 69.0840, -44.2849,12.2982, 287.2180, -1193.1275, 1436.3471,1.4468, 50.8587, -184.5215, 212.9088
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
    .wheel_radius = 0.06f,
    .reduction_ratio = 13.94f,
    .rotate_min = -45.0f,
    .rotate_max = 45.0f,
    .position_min = -12.5f,
    .position_max = 12.5f,
    .torque_min = -54.0f,
    .torque_max = 54.0f,
};