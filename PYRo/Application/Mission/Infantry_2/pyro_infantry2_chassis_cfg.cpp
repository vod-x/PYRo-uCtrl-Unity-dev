/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-27 20:38:05
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-03-14 17:37:03
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
#define R_MOTOR1_OFFSET 2.1898f
#define R_MOTOR2_OFFSET -1.595f
#define L_MOTOR1_OFFSET -1.0176f
#define L_MOTOR2_OFFSET -0.7211f
// the cofficients of lqr gain, 48 values in total, every value has 3 cofficients,
#define LQR_GAIN \
-0.9011, 28.0123, -73.7301, 71.8956,-1.1297, 47.5880, -122.3684, 119.7066,-6.9477, 11.6042, 6.1111, -26.4958,-3.0261, 4.6318, 3.7137, -12.5568,1.3602, -130.6724, 250.9476, -229.9261,0.4183, -17.9931, 21.1370, -23.2696,-5.2101, -22.4893, 136.1133, -181.4845,-9.4292, -37.9112, 232.0343, -310.6141,-1.9928, -81.2538, 170.7247, -132.4417,-0.4345, -35.1011, 71.8893, -54.0596,8.5519, 206.4342, -807.6488, 951.0299,1.6627, 32.8605, -115.5700, 133.4264
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
            .kp = 8.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 40.0f,
        },
        {
            .kp = 8.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 40.0f,
        }
    },
    .d_T_pid_cfg = {
        {
            .kp = 200.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        },
        {
            .kp = 200.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 20.0f,
        }
    },
    .F_pid_cfg = {
        {
            .kp = 120.0f,
            .ki = 0.5f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 120.0f,
            .ki = 0.5f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .max_out = 200.0f,
        }
    },
    .d_F_pid_cfg = {
        {
            .kp = 3.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 0.0f,
            .max_out = 200.0f,
        },
        {
            .kp = 3.0f,
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