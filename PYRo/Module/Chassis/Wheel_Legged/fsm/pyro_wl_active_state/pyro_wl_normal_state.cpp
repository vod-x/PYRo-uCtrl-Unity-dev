/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 13:11:52
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-03-07 21:05:21
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"
namespace pyro
{
void wl_chassis_t::fsm_active_t::state_normal_t::enter(wl_chassis_t *owner)
{
}

void wl_chassis_t::fsm_active_t::state_normal_t::execute(wl_chassis_t *owner)
{
    calc_support_force(owner);
    if(owner->_leg_data[wl_chassis_t::R].P < 0 || owner->_leg_data[wl_chassis_t::L].P < 0)
    {
        /* If the support force is negative, it means the leg is in the air, which may cause instability. */
        owner->_cnt.solver_error++;
    }
    /* Calculate target force of VMC for each legs. */
    /* Right leg */
    owner->_leg_data[wl_chassis_t::R].ref_d_l=
        owner->_F_pid[wl_chassis_t::R]->
        calculate(owner->_cmd->r_leg, 
        owner->_leg_data[wl_chassis_t::R].l);
    owner->_leg_data[wl_chassis_t::R].F[0]=
            owner->_F_pid[wl_chassis_t::R]->
             calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, 
        owner->_leg_data[wl_chassis_t::R].d_l);
    /* Left leg */
    owner->_leg_data[wl_chassis_t::L].ref_d_l=
        owner->_F_pid[wl_chassis_t::L]->
        calculate(owner->_cmd->l_leg,
        owner->_leg_data[wl_chassis_t::L].l);
    owner->_leg_data[wl_chassis_t::L].F[0]=
        owner->_F_pid[wl_chassis_t::L]->
        calculate(owner->_leg_data[wl_chassis_t::L].ref_d_l,
        owner->_leg_data[wl_chassis_t::L].d_l);
    // owner->_leg_data[wl_chassis_t::R].F[0] = 0.0f;
    // owner->_leg_data[wl_chassis_t::L].F[0] = 0.0f;
    // owner->_leg_data[wl_chassis_t::R].F[1] = 2.0f;
    // owner->_leg_data[wl_chassis_t::L].F[1] = 0.0f;
    /* Calculate the target torque of VMC for each leg */
    for(uint8_t i = 0; i < 2; i++)
    {
        float l = owner->_leg_data[i].l;
        for(uint8_t j = 0; j < 2; j++)
        {
            for(uint8_t k = 0; k < 6; k++)
            {
                
                owner->_leg_data[i].lqr_gain[j * 6 + k] = owner->_lqr_cof[(j * 6 + k) * 4]  +
                                                          owner->_lqr_cof[(j * 6 + k) * 4 + 1] * l +
                                                          owner->_lqr_cof[(j * 6 + k) * 4 + 2] * l * l +
                                                          owner->_lqr_cof[(j * 6 + k) * 4 + 3] * l * l * l ;
            }
        }
        owner->_leg_data[i].T_w = -(owner->_leg_data[i].lqr_gain[0] * owner->_leg_data[i].x + 
                                  owner->_leg_data[i].lqr_gain[1] * owner->_leg_data[i].dx + 
                                  owner->_leg_data[i].lqr_gain[2] * owner->_leg_data[i].gamma + 
                                  owner->_leg_data[i].lqr_gain[3] * owner->_leg_data[i].d_gamma + 
                                  owner->_leg_data[i].lqr_gain[4] * owner->_leg_data[i].beta + 
                                  owner->_leg_data[i].lqr_gain[5] * owner->_leg_data[i].d_beta)
                                  / owner->_reduction_ratio /0.3f * (3591.0f/187.0f);
        owner->_leg_data[i].F[1] = owner->_leg_data[i].lqr_gain[6] * owner->_leg_data[i].x + 
                                  owner->_leg_data[i].lqr_gain[7] * owner->_leg_data[i].dx + 
                                  owner->_leg_data[i].lqr_gain[8] * owner->_leg_data[i].gamma + 
                                  owner->_leg_data[i].lqr_gain[9] * owner->_leg_data[i].d_gamma + 
                                  owner->_leg_data[i].lqr_gain[10] * owner->_leg_data[i].beta + 
                                  owner->_leg_data[i].lqr_gain[11] * owner->_leg_data[i].d_beta;
                                  
    }
    

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
    owner->_wheel_drv[wl_chassis_t::R]->send_torque(fp32_constrain(-owner->_leg_data[wl_chassis_t::R].T_w, -20.0f, 20.0f));
    owner->_wheel_drv[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_leg_data[wl_chassis_t::L].T_w, -20.0f, 20.0f));
    owner->_motor_drv[wl_chassis_t::RF]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[0]);
    owner->_motor_drv[wl_chassis_t::RB]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[1]);
    owner->_motor_drv[wl_chassis_t::LF]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[0]);
    owner->_motor_drv[wl_chassis_t::LB]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[1]);
    
    // owner->_wheel_drv[wl_chassis_t::L]->send_torque(0.0f);
    // owner->_wheel_drv[wl_chassis_t::R]->send_torque(-0.0f);
    // owner->_motor_drv[wl_chassis_t::RF]->send_torque(
    //                     0.0f);
    // owner->_motor_drv[wl_chassis_t::RB]->send_torque(
    //                     0.0f);
    // owner->_motor_drv[wl_chassis_t::LF]->send_torque(
    //                     0.0f);
    // owner->_motor_drv[wl_chassis_t::LB]->send_torque(
    //                     0.0f);
    // vTaskDelay(1);
}
void wl_chassis_t::fsm_active_t::state_normal_t::exit(wl_chassis_t *owner)
{
}

void wl_chassis_t::fsm_active_t::state_normal_t::calc_support_force(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        /* The support force is calculated by the projection of VMC output force
           in vertical direction. */
        owner->_leg_data[i].P 
         = owner->_leg_data[i].F[0] * arm_cos_f32(owner->_leg_data[i].beta)
         + owner->_leg_data[i].F[1] * arm_sin_f32(owner->_leg_data[i].beta) / owner->_leg_data[i].l
         + owner->a_z 
         - owner->_leg_data[i].d2_l * arm_cos_f32(owner->_leg_data[i].beta)
         + owner->_leg_data[i].d_l * owner->_leg_data[i].d_beta * arm_sin_f32(owner->_leg_data[i].beta)
         + owner->_leg_data[i].l * owner->_leg_data[i].d2_beta * arm_sin_f32(owner->_leg_data[i].beta)
         + owner->_leg_data[i].l * owner->_leg_data[i].d_beta * owner->_leg_data[i].d_beta * arm_cos_f32(owner->_leg_data[i].beta);
        
    }
}
}