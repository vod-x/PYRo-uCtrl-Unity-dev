/*
 * @Author: Vod vod0575@outlook
 * @Date: 2026-02-06 15:27:37
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-02-08 14:52:32
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */

 #include "pyro_wl_chassis.h"


 namespace pyro
 {
wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis")
{
}

void wl_chassis_t::_init()
{
    /* Initialize kinematic solver with given coefficients. */
    _kinematic_solver.init(&_config.phi_k, &_config.polar_k, 
                                                   &_config.vmc_k);
    /* Save LQR coefficients */
    memcpy(_lqr_cof, _config.lqr_coef, sizeof(float) * 36);

    /* Save wheel radius and reduction ratio */
    _wheel_radius = _config.wheel_radius;
    _reduction_ratio = _config.reduction_ratio;

    /* Initialize joint motor driver */
    for(uint8_t i = 0; i < 4; i++)
    {
        _motor_drv[i] = new dm_motor_drv_t(_config.joint_motor_cfg[i].tx_id,
                                           _config.joint_motor_cfg[i].rx_id,
                                           _config.joint_motor_cfg[i].can);
        _motor_offset[i] = _config.joint_motor_cfg[i].offset_angle;
        _motor_drv[i]->set_rotate_range(_config.rotate_min, 
                                                  _config.rotate_max);
        _motor_drv[i]->set_position_range(_config.position_min,
                                                  _config.position_max);
        _motor_drv[i]->set_torque_range(_config.torque_min,
                                                  _config.torque_max);
    }
    /* Initialize wheel motor driver */
    for(uint8_t i = 0; i < 2; i++)
    {
        _wheel_drv[i] = 
               new dji_m3508_motor_drv_t(_config.wheel_motor_cfg[i].tx_id,
                                     _config.wheel_motor_cfg[i].can);
    }

}
 }