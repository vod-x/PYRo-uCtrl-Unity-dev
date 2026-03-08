/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:55:50
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-03-07 18:48:15
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
namespace pyro
{
void wl_chassis_t::fsm_active_t::state_test_t::enter(wl_chassis_t *owner)
{
}

void wl_chassis_t::fsm_active_t::state_test_t::execute(wl_chassis_t *owner)
{
    // for(uint8_t i = 0; i < 2; i++)
    // {
    //     owner->_wheel_drv[i]->send_torque(0);
    // }
    // for(uint8_t i = 0; i < 4; i++)
    // {
    //     owner->_motor_drv[i]->send_torque(0);
    // }    owner->_leg_data[wl_chassis_t::R].ref_d_l=
    float angle = PI/2.0f;
    float leg = 0.15f;
    owner->_leg_data[wl_chassis_t::R].ref_d_l=
            owner->_F_pid[wl_chassis_t::R]->
        calculate(leg, 
        owner->_leg_data[wl_chassis_t::R].l);
    owner->_leg_data[wl_chassis_t::R].F[0]=
            owner->_F_pid[wl_chassis_t::R]->
             calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, 
        owner->_leg_data[wl_chassis_t::R].d_l);
    /* Left leg */
    owner->_leg_data[wl_chassis_t::L].ref_d_l=
        owner->_F_pid[wl_chassis_t::L]->
        calculate(leg,
        owner->_leg_data[wl_chassis_t::L].l);
    owner->_leg_data[wl_chassis_t::L].F[0]=
        owner->_F_pid[wl_chassis_t::L]->
        calculate(owner->_leg_data[wl_chassis_t::L].ref_d_l,
        owner->_leg_data[wl_chassis_t::L].d_l);

    /* Calculate the target torque of VMC for each leg */
    /* Right leg */
    owner->_leg_data[wl_chassis_t::R].ref_d_alpha=
        owner->_T_pid[wl_chassis_t::R]->
        calculate(angle,
        owner->_leg_data[wl_chassis_t::R].alpha);
    owner->_leg_data[wl_chassis_t::R].F[1]=
        owner->_T_pid[wl_chassis_t::R]->
        calculate(owner->_leg_data[wl_chassis_t::R].ref_d_alpha,
        owner->_leg_data[wl_chassis_t::R].d_alpha);
    /* Left leg */
    owner->_leg_data[wl_chassis_t::L].ref_d_alpha=
        owner->_T_pid[wl_chassis_t::L]->
        calculate(angle,
        owner->_leg_data[wl_chassis_t::L].alpha);
    owner->_leg_data[wl_chassis_t::L].F[1]=
        owner->_T_pid[wl_chassis_t::L]->
        calculate(owner->_leg_data[wl_chassis_t::L].ref_d_alpha,
        owner->_leg_data[wl_chassis_t::L].d_alpha);
    
    /* Transfer the force and torque of virtual rod to the practical torque of
       motors by VMC matrix. */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_vec_mult_f32(&owner->_leg_data[i].T_mat, 
                            owner->_leg_data[i].F, 
                            owner->_leg_data[i].T);
    }

    /* Send torque to motors. The direction of right motors is opposite to the
       torque direction due to installation*/
    owner->_motor_drv[wl_chassis_t::RF]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[0]);
    owner->_motor_drv[wl_chassis_t::RB]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[1]);
    owner->_motor_drv[wl_chassis_t::LF]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[0]);
    owner->_motor_drv[wl_chassis_t::LB]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[1]);
}
void wl_chassis_t::fsm_active_t::state_test_t::exit(wl_chassis_t *owner)
{
}

}