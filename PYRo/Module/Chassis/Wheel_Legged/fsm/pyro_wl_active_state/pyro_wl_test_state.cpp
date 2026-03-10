/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:55:50
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-03-10 13:50:42
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"
namespace pyro
{
void wl_chassis_t::fsm_active_t::state_test_t::enter(wl_chassis_t *owner)
{
}

void wl_chassis_t::fsm_active_t::state_test_t::execute(wl_chassis_t *owner)
{
    owner->_wheel_drv[wl_chassis_t::R]->send_torque(0.0f);
    owner->_wheel_drv[wl_chassis_t::L]->send_torque(0.0f);

    
    owner->_motor_drv[wl_chassis_t::RF]->send_torque(0.0f);
    owner->_motor_drv[wl_chassis_t::RB]->send_torque(0.0f);
    owner->_motor_drv[wl_chassis_t::LF]->send_torque(0.0f);
    owner->_motor_drv[wl_chassis_t::LB]->send_torque(0.0f);
}
void wl_chassis_t::fsm_active_t::state_test_t::exit(wl_chassis_t *owner)
{
}
}

