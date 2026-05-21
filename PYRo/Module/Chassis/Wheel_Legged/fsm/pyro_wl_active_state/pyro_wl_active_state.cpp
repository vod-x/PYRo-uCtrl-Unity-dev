/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 20:03:11
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-14 22:49:40
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"
namespace pyro
{
void wl_chassis_t::fsm_active_t::on_enter(wl_chassis_t *owner)
{ 
    for(uint8_t i = 0; i < 4; i++)
    {
        if(dm_motor_drv_t::ok != owner->_motor_drv[i]->get_error_code())
        {
            owner->_motor_drv[i]->clear_error();
        }
        owner->_motor_drv[i]->enable();
    }
    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_wheel_drv[i]->enable();
    }
    
    owner->_leg_data[wl_chassis_t::R].x = 0.0f;
    owner->_leg_data[wl_chassis_t::L].x = 0.0f;
    owner->_leg_data[wl_chassis_t::R].x_gain = 0.0f;
    owner->_leg_data[wl_chassis_t::L].x_gain = 0.0f;
    owner->_leg_data[wl_chassis_t::R].kf_x = 0.0f;
    owner->_leg_data[wl_chassis_t::L].kf_x = 0.0f;
    owner->_wheel_kf[wl_chassis_t::R].reset();
    owner->_wheel_kf[wl_chassis_t::L].reset();
}

void wl_chassis_t::fsm_active_t::on_execute(wl_chassis_t *owner)
{
    if(owner->_cmd->active_mode == wl_cmd_t::REVERSE)
    {
        this->change_state(&_state_reverse);
    }
    if(owner->_cmd->active_mode == wl_cmd_t::READY)
    {
        if(owner->_cmd->last_active_mode == wl_cmd_t::OVER_STEP_READY)
        {
            owner->_active_mode_flag.ready = 1;
            return;
        }
        this->change_state(&_state_ready);
    }
    if(owner->_cmd->active_mode == wl_cmd_t::NORMAL)
    {
        this->change_state(&_state_normal);
    }
    if(owner->_cmd->active_mode == wl_cmd_t::TEST)
    {
        this->change_state(&_state_test);
    }
    if(owner->_cmd->active_mode == wl_cmd_t::OVER_STEP)
    {
        this->change_state(&_state_over_step);
    }
    if(owner->_cmd->active_mode == wl_cmd_t::OVER_STEP_READY)
    {
        this->change_state(&_state_over_step_ready);
    }
    if(owner->_cmd->active_mode == wl_cmd_t::CONTROL)
    {
        this->change_state(&_state_control);
    }
     if(owner->_cmd->active_mode == wl_cmd_t::SPIN)
    {
        this->change_state(&_state_spin);
    }
}

void wl_chassis_t::fsm_active_t::on_exit(wl_chassis_t *owner)
{
}
}
