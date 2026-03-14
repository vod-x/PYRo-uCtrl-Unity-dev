/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:55:50
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-03-14 15:27:50
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"

#define LENGTH_SPEED (0.1f/1000.0f)
#define ANGLE_SPEED (2.0f * PI/1000.0f)
#define TARGET_LENGTH 0.17f
#define TARGET_ANGLE (-2.0f * PI/3.0f)
namespace pyro
{

 static float target_length[2] = {0.0f, 0.0f};
 static float target_angle[2] = {0.0f, 0.0f};
 static float cur_length[2] = {0.0f, 0.0f};
 static float cur_angle[2] = {0.0f, 0.0f};
void wl_chassis_t::fsm_active_t::state_over_step_t::enter(wl_chassis_t *owner)
{
    /* record the current angle and length of the legs */
    owner->get_cur_angle(&cur_angle[wl_chassis_t::R], 
                        &cur_angle[wl_chassis_t::L]);
    owner->get_cur_length(&cur_length[wl_chassis_t::R], 
                    &cur_length[wl_chassis_t::L]);
    /* set the target angle and length of the legs as the current values */
    target_length[wl_chassis_t::R] = cur_length[wl_chassis_t::R];
    target_length[wl_chassis_t::L] = cur_length[wl_chassis_t::L];
    target_angle[wl_chassis_t::R] = cur_angle[wl_chassis_t::R];
    target_angle[wl_chassis_t::L] = cur_angle[wl_chassis_t::L];
    
    owner->_active_mode_flag.over_step = 0;
}

void wl_chassis_t::fsm_active_t::state_over_step_t::execute(wl_chassis_t *owner)
{


    calc_target_value(owner);

    /* Calculate the target force of VMC for each leg */
    /* Right leg */
    owner->_leg_data[wl_chassis_t::R].ref_d_l=
            owner->_F_pid[wl_chassis_t::R]->
        calculate(target_length[wl_chassis_t::R], 
        owner->_leg_data[wl_chassis_t::R].l);
    owner->_leg_data[wl_chassis_t::R].F[0]=
            owner->_F_pid[wl_chassis_t::R]->
             calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, 
        owner->_leg_data[wl_chassis_t::R].d_l);
    /* Left leg */
    owner->_leg_data[wl_chassis_t::L].ref_d_l=
        owner->_F_pid[wl_chassis_t::L]->
        calculate(target_length[wl_chassis_t::L],
        owner->_leg_data[wl_chassis_t::L].l);
    owner->_leg_data[wl_chassis_t::L].F[0]=
        owner->_F_pid[wl_chassis_t::L]->
        calculate(owner->_leg_data[wl_chassis_t::L].ref_d_l,
        owner->_leg_data[wl_chassis_t::L].d_l);

    /* Calculate the target torque of VMC for each leg */
    /* Right leg */
    float diff;
    if(target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha > PI)
    {
        diff = -2 * PI + (target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha);
    }
    else if(target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha < -PI)
    {
        diff = 2 * PI + (target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha);
    }
    else
    {
        diff = target_angle[wl_chassis_t::R] - owner->_leg_data[wl_chassis_t::R].alpha;
    }
    owner->_leg_data[wl_chassis_t::R].ref_d_alpha=
        owner->_T_pid[wl_chassis_t::R]->
        calculate(owner->_leg_data[wl_chassis_t::R].alpha + diff,
        owner->_leg_data[wl_chassis_t::R].alpha);
    owner->_leg_data[wl_chassis_t::R].F[1]=
        owner->_T_pid[wl_chassis_t::R]->
        calculate(owner->_leg_data[wl_chassis_t::R].ref_d_alpha,
        owner->_leg_data[wl_chassis_t::R].d_alpha);
    /* Left leg */
    if(target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha > PI)
    {
        diff = -2 * PI + (target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha);
    }
    else if(target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha < -PI)
    {
        diff = 2 * PI + (target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha);
    }
    else
    {
        diff = target_angle[wl_chassis_t::L] - owner->_leg_data[wl_chassis_t::L].alpha;
    }
    owner->_leg_data[wl_chassis_t::L].ref_d_alpha=
        owner->_T_pid[wl_chassis_t::L]->
        calculate(owner->_leg_data[wl_chassis_t::L].alpha+diff,
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
void wl_chassis_t::fsm_active_t::state_over_step_t::exit(wl_chassis_t *owner)
{
}

void wl_chassis_t::fsm_active_t::state_over_step_t::calc_target_value(wl_chassis_t *owner)
{
    /* calculate the target angle and length of the legs, add a small bias in 
        each period */
    /* target length */
    for(uint8_t i = 0; i < 2; i++)
    {
        if(0.01f > abs(target_length[i] - TARGET_LENGTH))
        {
            continue;
        }
        if(cur_length[i] < TARGET_LENGTH)
        {
            target_length[i] += LENGTH_SPEED;
        }
        else if(cur_length[i] > TARGET_LENGTH)
        {
            target_length[i] -= LENGTH_SPEED;
        }
    }
    /* target angle */
    for(uint8_t i = 0; i < 2; i++)
    {
        if(0.3f > abs(target_angle[i] - TARGET_ANGLE))
        {
            continue;
        }
        // if((cur_angle[i] < -TARGET_ANGLE) || (cur_angle[i] > TARGET_ANGLE))
        // {
        //     target_angle[i] -= ANGLE_SPEED;
        //     target_angle[i] = wrap2pi_f32(target_angle[i]);
        // }
        // else if((cur_angle[i] > -TARGET_ANGLE) && (cur_angle[i] < TARGET_ANGLE))
        // {
             target_angle[i] += ANGLE_SPEED;
             target_angle[i] = wrap2pi_f32(target_angle[i]);
        // }
    }
    if((0.01f > abs(target_length[wl_chassis_t::R] - TARGET_LENGTH)) &&
       (0.01f > abs(target_length[wl_chassis_t::L] - TARGET_LENGTH)) &&
       (0.05f > abs(target_angle[wl_chassis_t::R] - TARGET_ANGLE)) &&
       (0.05f > abs(target_angle[wl_chassis_t::L] - TARGET_ANGLE)))
    {
        owner->_active_mode_flag.over_step = 1;
    }
}

}