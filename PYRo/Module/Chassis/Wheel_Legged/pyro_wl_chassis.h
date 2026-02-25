/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-07 15:14:47
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-08 14:49:58
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_kin.wl.h"

 #include "pyro_dm_motor_drv.h"
 #include "pyro_dji_motor_drv.h"

namespace pyro
{
/* DM joint motor configuration structure. */
struct wl_dm_motor_cfg_t
{
    /* can tx id of motor*/
    uint8_t tx_id;
    /* can rx id of motor*/
    uint8_t rx_id;
    /* Which CAN bus the motor is connected to */
    can_hub_t::which_can can;
    /* Offset angle to eliminate the bias of installation(rad)*/
    float offset_angle;
};

/* DJI 3508 configuration structure.*/
struct wl_dji_motor_cfg_t
{
    /* can tx id of motor*/
    dji_motor_tx_frame_t::register_id_t tx_id;
    /* Which CAN bus the motor is connected to */
    can_hub_t::which_can can;
};

/* Configuration structure for wheel-legged chassis, every parameter
   should be set here */
struct wl_chassis_cfg_t
{
    /* Kinematic coefficients for the chassis. */
    wheel_legged_kin_t::phi_k_t phi_k;
    wheel_legged_kin_t::polar_k_t polar_k;
    wheel_legged_kin_t::vmc_k_t vmc_k;

    /* Joint motor configuration. The order of array is: front-right,
       rear-right, front-left, rear-left */
    wl_dm_motor_cfg_t joint_motor_cfg[4];
    /* Wheel motor configuration. The order of array is: right wheel, left
       wheel */
    wl_dji_motor_cfg_t wheel_motor_cfg[2];
    /* LQR coefficients for the chassis control. 2 raw x 6 column, 12 values
       in total. Every value has 3 coefficients.*/
    float lqr_coef[36];
    /* Wheel radius(m)*/
    float wheel_radius;
    /* Reduction ratio of the motor */
    float reduction_ratio;
    /* Rotation range of the motor(rad/s)*/
    float rotate_min;
    float rotate_max;
    /* Position range of the motor(rad)*/
    float position_min;
    float position_max;
    /* Torque range of the motor(Nm)*/
    float torque_min;
    float torque_max;
};
//command structure for wheel-legged chassis
struct wl_cmd_t final : public cmd_base_t
{
    /* Linear velocity in x direction(m/s), positive toward the front of chassis
       negative toward the back of chassis */
    float vx;   
    /* Linear velocity in y direction(m/s), positive toward the left of chassis
       negative toward the right of chassis */
    float vy;  
    /* Angular velocity in z direction(rad/s), positive for counter-clockwise
       negative for clockwise */
    float vz;
    /* Leg length of both sides after normalization, value is between 0 and 1*/
    float l_leg;
    float r_leg;

    /* Construct function, set zero values */
    wl_cmd_t() : vx(0), vy(0), vz(0), l_leg(0), r_leg(0)
    {
    }
};

class wl_chassis_t final : public module_base_t<wl_chassis_t, wl_cmd_t, wl_chassis_cfg_t>
{
   friend class module_base_t<wl_chassis_t, wl_cmd_t, wl_chassis_cfg_t>;

public:
   
    wl_chassis_t(const wl_chassis_t &)            = delete;
    wl_chassis_t &operator=(const wl_chassis_t &) = delete;

private:
    /**
     * @description:
       Construct function, initialize the kinematic solver with given 
       cofficients. 
    *  @param {cfg_t*} cfg: configuration structure pointer, all parameters for 
       chassis should be set in this structure.
     */
    wl_chassis_t();
    ~wl_chassis_t() override = default;

    /* base interface define */
    void _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    /* private function*/

    /* Kinematic solver, provide kinematic calculations and VMC matrix update
       function. */
    wheel_legged_kin_t _kinematic_solver;

    /* LQR coefficients for the chassis control. 2 raw x 6 column, 12 values
       in total. Every value has 3 coefficients.*/
    float _lqr_cof[36];

    /* DM joint motors driver, the order of array is: front-right, rear-right, 
       front-left, rear-left */
    dm_motor_drv_t *_motor_drv[4];

    /* DJI wheel motors driver, the order of array is: right wheel, left wheel*/
    dji_m3508_motor_drv_t *_wheel_drv[2];
    /* Linear velocity of wheel equals angular velocity of wheel x wheel radius
       x reduction ratio */
    float _wheel_radius;
    float _reduction_ratio;
    /* Offset angle to eliminate the bias of installation(rad), order is same
       as _motor_drv array */
    float _motor_offset[4];

    
};
}
#endif // __PYRO_WL_CHASSIS_H__