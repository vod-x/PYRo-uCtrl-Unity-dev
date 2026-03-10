/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 20:03:11
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-03-10 14:14:26
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
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
}

void wl_chassis_t::fsm_active_t::on_execute(wl_chassis_t *owner)
{
    if(owner->_cmd->active_mode == wl_cmd_t::REVERSE)
    {
        this->change_state(&_state_reverse);
    }
    if(owner->_cmd->active_mode == wl_cmd_t::READY)
    {
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
     
}

void wl_chassis_t::fsm_active_t::on_exit(wl_chassis_t *owner)
{
}
}
