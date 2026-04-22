/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 19:51:12
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-04-19 13:30:51
 * @Description: Wheel-legged chassis passive state implementation
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_pid.h"

namespace pyro
{
    
static uint8_t wheel_disable_flag[2] = {0, 0};
pid_t wheel_disable_pid[2] = {
    pid_t(1.0f, 0.0f, 0.0f, 0.0f, 3.0f), 
    pid_t(1.0f, 0.0f, 0.0f, 0.0f, 3.0f)};
/**
 * @description: 
   When wheel-legged chassis enter passive state, it will call this function.
   In this function, all the motors of the chassis will be disabled.
 * @param {wl_chassis_t} *owner
   Pointer to the wheel-legged chassis instance.
 * @return {*}
 */
void wl_chassis_t::state_passive_t::enter(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        owner->_motor_drv[i]->disable();
    }
    for(uint8_t i = 0; i < 2; i++)
    {
        wheel_disable_flag[i] = 1;
    }
}

void wl_chassis_t::state_passive_t::execute(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        if(1 == wheel_disable_flag[i])
        {
            float t = wheel_disable_pid[i].calculate(0.0f,
                 owner->_wheel_drv[i]->get_current_rotate());
            owner->_wheel_drv[i]->send_torque(t);
        }
        else 
        {
            owner->_wheel_drv[i]->send_torque(0.0f);
        
        }
        if(0.1f > abs(owner->_wheel_drv[i]->get_current_rotate()))
        {
            wheel_disable_flag[i] = 0;
            owner->_wheel_drv[i]->disable();
        }
    }
    for(uint8_t i = 0; i < 4; i++)
    {
        owner->_motor_drv[i]->send_torque(0.0f);
    }
}

void wl_chassis_t::state_passive_t::exit(wl_chassis_t *owner)
{
}

}