/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:55:50
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-20 11:32:09
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"

#define LENGTH_SPEED (0.1f/1000.0f)
#define ANGLE_SPEED (PI/2000.0f)
#define TARGET_LENGTH 0.18f
// #define TARGET_ANGLE (2.0f * PI/3.0f)
#define TARGET_ANGLE ((PI/2.0f) + 0.6f)
namespace pyro
{
extern pid_t wheel_disable_pid[2];
static uint8_t wheel_disable_flag[2] = {0, 0};
static float tmp_angle = 0.0f;
static uint8_t state_flag[2] = {0, 0};

static float target_length[2] = {0.0f, 0.0f};
static float target_angle[2] = {0.0f, 0.0f};
static float cur_length[2] = {0.0f, 0.0f};
static float cur_angle[2] = {0.0f, 0.0f};

static uint8_t ready_flag = 0;
const float beta_bias = 0.3f;
const float gamma_bias = 0.3f;
const uint32_t ready_time = 1;

void wl_chassis_t::fsm_active_t::state_ready_t::enter(wl_chassis_t *owner)
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
    for(uint8_t i = 0; i < 2; i++)
    {
        wheel_disable_flag[i] = 1;
        state_flag[i] = 0;
    }
    ready_flag = 0;
    owner->_active_mode_flag.ready = 0;
}

void wl_chassis_t::fsm_active_t::state_ready_t::execute(wl_chassis_t *owner)
{

    if((0.01f > abs(owner->_leg_data[wl_chassis_t::R].l - TARGET_LENGTH)) &&
       (0.01f > abs(owner->_leg_data[wl_chassis_t::L].l - TARGET_LENGTH)) &&
       (0.05f > abs(owner->_leg_data[wl_chassis_t::R].alpha - TARGET_ANGLE)) &&
       (0.05f > abs(owner->_leg_data[wl_chassis_t::L].alpha - TARGET_ANGLE)))
    {
        ready_flag = 1;
        owner->_leg_data[wl_chassis_t::R].x = 0.0f;
        owner->_leg_data[wl_chassis_t::L].x = 0.0f;
        owner->_leg_data[wl_chassis_t::R].x_gain = 0.0f;
        owner->_leg_data[wl_chassis_t::L].x_gain = 0.0f;
        owner->_leg_data[wl_chassis_t::R].kf_x = 0.0f;
        owner->_leg_data[wl_chassis_t::R].kf_v = 0.0f;
        owner->_leg_data[wl_chassis_t::L].kf_v = 0.0f;
        owner->_leg_data[wl_chassis_t::L].kf_x = 0.0f;
        owner->_wheel_kf[wl_chassis_t::R].reset();
        owner->_wheel_kf[wl_chassis_t::L].reset();
    }
    if(0 == ready_flag)
    {
        /* calculate the target angle and length of the legs, add a small bias in 
            each period */
        calc_target_value(owner);

        /* Calculate the target force of VMC for each leg */
        /* Right leg */
        owner->_leg_data[wl_chassis_t::R].ref_d_l=
                owner->_F_pid[wl_chassis_t::R]->
            calculate(target_length[wl_chassis_t::R], 
            owner->_leg_data[wl_chassis_t::R].l);
        owner->_leg_data[wl_chassis_t::R].F[0]=
                owner->_d_F_pid[wl_chassis_t::R]->
                 calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, 
            owner->_leg_data[wl_chassis_t::R].d_l);
        /* Left leg */
        owner->_leg_data[wl_chassis_t::L].ref_d_l=
            owner->_F_pid[wl_chassis_t::L]->
            calculate(target_length[wl_chassis_t::L],
            owner->_leg_data[wl_chassis_t::L].l);
        owner->_leg_data[wl_chassis_t::L].F[0]=
            owner->_d_F_pid[wl_chassis_t::L]->
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
            owner->_d_T_pid[wl_chassis_t::R]->
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
            owner->_d_T_pid[wl_chassis_t::L]->
            calculate(owner->_leg_data[wl_chassis_t::L].ref_d_alpha,
            owner->_leg_data[wl_chassis_t::L].d_alpha);
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
    }
    else 
    {
        for(uint8_t i = 0; i < 2; i++)
        {

            /* Calculate lqr gain by length of each leg */
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

            owner->_leg_data[i].ref_l = TARGET_LENGTH;
            /* Right leg */
            owner->_leg_data[wl_chassis_t::R].ref_d_l=
                owner->_F_pid[wl_chassis_t::R]->
                calculate(owner->_leg_data[wl_chassis_t::R].ref_l, 
                owner->_leg_data[wl_chassis_t::R].l);
            owner->_leg_data[wl_chassis_t::R].F[0]=
                    owner->_d_F_pid[wl_chassis_t::R]->
                     calculate(owner->_leg_data[wl_chassis_t::R].ref_d_l, 
                owner->_leg_data[wl_chassis_t::R].d_l);
            /* Left leg */
            owner->_leg_data[wl_chassis_t::L].ref_d_l=
                owner->_F_pid[wl_chassis_t::L]->
                calculate(owner->_leg_data[wl_chassis_t::L].ref_l,
                owner->_leg_data[wl_chassis_t::L].l);
            owner->_leg_data[wl_chassis_t::L].F[0]=
                owner->_d_F_pid[wl_chassis_t::L]->
                calculate(owner->_leg_data[wl_chassis_t::L].ref_d_l,
                owner->_leg_data[wl_chassis_t::L].d_l);

            owner->_leg_data[i].T_w_balance = (
                                      owner->_leg_data[i].lqr_gain[2] * (0 - owner->_leg_data[i].gamma) + 
                                      owner->_leg_data[i].lqr_gain[3] * (0 - owner->_leg_data[i].d_gamma) + 
                                      owner->_leg_data[i].lqr_gain[4] * (0.0f - owner->_leg_data[i].beta) + 
                                      owner->_leg_data[i].lqr_gain[5] * (0 - owner->_leg_data[i].d_beta));

            owner->_leg_data[i].F[1] = -(
                                      owner->_leg_data[i].lqr_gain[8] * (0 - owner->_leg_data[i].gamma) + 
                                      owner->_leg_data[i].lqr_gain[9] * (0 - owner->_leg_data[i].d_gamma) + 
                                      owner->_leg_data[i].lqr_gain[10] * (0.0f - owner->_leg_data[i].beta) + 
                                      owner->_leg_data[i].lqr_gain[11] * (0 - owner->_leg_data[i].d_beta));

        }

        static uint32_t time_count = 0;
        if((beta_bias > abs(owner->_leg_data[wl_chassis_t::R].beta)) &&
           (gamma_bias > abs(owner->_leg_data[wl_chassis_t::R].gamma)) &&
           (beta_bias > abs(owner->_leg_data[wl_chassis_t::L].beta)) &&
           (gamma_bias > abs(owner->_leg_data[wl_chassis_t::L].gamma)))
        {
            time_count += 1;
            for(uint8_t i = 0; i < 2; i++)
            {

                // owner->_leg_data[i].T_w_balance = wheel_disable_pid[i].calculate(0.0f,
                //      owner->_wheel_drv[i]->get_current_rotate());
                // owner->_wheel_drv[i]->send_torque(owner->_leg_data[i].T_w_balance);

            }

        }
        else 
        {
            time_count = 0;
            // owner->_wheel_drv[wl_chassis_t::R]->send_torque(fp32_constrain(-owner->_leg_data[wl_chassis_t::R].T_w / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
            // owner->_wheel_drv[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_leg_data[wl_chassis_t::L].T_w / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));

        }
        if(time_count > ready_time)
        {
            owner->_active_mode_flag.ready = 1;
        }
        owner->_leg_data[wl_chassis_t::R].F[0] = fp32_constrain(owner->_leg_data[wl_chassis_t::R].F[0], -200.0f, 200.0f);
        owner->_leg_data[wl_chassis_t::L].F[0] = fp32_constrain(owner->_leg_data[wl_chassis_t::L].F[0], -200.0f, 200.0f);

        owner->_leg_data[wl_chassis_t::R].T_w = owner->_leg_data[wl_chassis_t::R].T_w_balance;
        owner->_leg_data[wl_chassis_t::L].T_w = owner->_leg_data[wl_chassis_t::L].T_w_balance;

            owner->_wheel_drv[wl_chassis_t::R]->send_torque(fp32_constrain(-owner->_leg_data[wl_chassis_t::R].T_w / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
            owner->_wheel_drv[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_leg_data[wl_chassis_t::L].T_w / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
    
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
    owner->_motor_drv[wl_chassis_t::RF]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[0]);
    owner->_motor_drv[wl_chassis_t::RB]->send_torque(
                        -owner->_leg_data[wl_chassis_t::R].T[1]);
    owner->_motor_drv[wl_chassis_t::LF]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[0]);
    owner->_motor_drv[wl_chassis_t::LB]->send_torque(
                        owner->_leg_data[wl_chassis_t::L].T[1]);
    
}
void wl_chassis_t::fsm_active_t::state_ready_t::exit(wl_chassis_t *owner)
{
}

void wl_chassis_t::fsm_active_t::state_ready_t::calc_target_value(wl_chassis_t *owner)
{
    /* calculate the target angle and length of the legs, add a small bias in 
        each period */
    /* 1. make alpha in the range [-PI/2, -PI] or [PI/2, PI] */
    if((0 == state_flag[wl_chassis_t::R]) || (0 == state_flag[wl_chassis_t::L]))
    {
        for(uint8_t i = 0; i < 2; i++)
        {
            /* if alpha is not in the desired range, set the target angle to 
                -PI/2 */
            if(((0.0f < cur_angle[i]) && (cur_angle[i] < PI/2)) ||
               ((-PI/2 < cur_angle[i]) && (cur_angle[i] < 0.0f)))
            {
                /* add target angle smoothly to avoid sudden changes */
                if(0.05f < abs(target_angle[i] - (-PI/2.0f)))
                {
                    target_angle[i] -= ANGLE_SPEED;
                }
                /* when the target angle is close enough to the desired angle, 
                    set the state flag as 1 */
                else 
                {
                    target_angle[i] = -PI/2.0f;
                    state_flag[i] = 1;
                }
            }
            /* if alpha is in the desired range, set the state flag as 2 */
            else 
            {
                state_flag[i] = 2;
            }
        }
    }
    /* 2. Make alpha of both legs equa. 
       (2.1. If any flag equal to 0, it means that step 1 has not been completed,
              do not execute step 2;
        2.2. If every flags equal to 1, it means alpha of both legs equal -PI/2,
              do not need to execute step 2;
        2.3. If any flag equal to 2, it means that alpha of one leg is in the \
              desired range, but the other leg is not, execute step 2 to make 
              alpha of both legs equal.) */
    if(((2 == state_flag[wl_chassis_t::R]) || (2 == state_flag[wl_chassis_t::L]))
        || ((0 != state_flag[wl_chassis_t::R]) && (0 != state_flag[wl_chassis_t::L])))
    {
        /* choose the angle which is closer to target angle as tmp_angle */
        if((cur_angle[wl_chassis_t::R] > 0.0f) && (cur_angle[wl_chassis_t::L] > 0.0f))
        {
            if(cur_angle[wl_chassis_t::R] < cur_angle[wl_chassis_t::L])
            {
                tmp_angle = cur_angle[wl_chassis_t::R];
            }
            else 
            {
                tmp_angle = cur_angle[wl_chassis_t::L];
            }
        
        }
        else 
        {
            if(cur_angle[wl_chassis_t::R] < cur_angle[wl_chassis_t::L])
            {
                tmp_angle = cur_angle[wl_chassis_t::L];
            }
            else 
            {
                tmp_angle = cur_angle[wl_chassis_t::R];
            }
        }
        for(uint8_t i = 0; i < 2; i++)
        {
            if(0.05f < abs(target_angle[i] - tmp_angle))
            {
                target_angle[i] -= ANGLE_SPEED;
                target_angle[i] = wrap2pi_f32(target_angle[i]);
            }
        }
        if((0.05f > abs(target_angle[wl_chassis_t::R] - tmp_angle)) &&
           (0.05f > abs(target_angle[wl_chassis_t::L] - tmp_angle)))
        {
            state_flag[wl_chassis_t::R] = 3;
            state_flag[wl_chassis_t::L] = 3;
            tmp_angle = 0.0f;
        }
    }
    /* 3. move to target length and angle */
    if(((3 == state_flag[wl_chassis_t::R]) && (3 == state_flag[wl_chassis_t::L]))
        ||((1 == state_flag[wl_chassis_t::R]) && (1 == state_flag[wl_chassis_t::L])))
    {
        for(uint8_t i = 0; i < 2; i++)
        {
            if(target_length[i] < TARGET_LENGTH)
            {
                target_length[i] += LENGTH_SPEED;
            }
            else if(target_length[i] > TARGET_LENGTH)
            {
                target_length[i] -= LENGTH_SPEED;
            }
            if(0.01f > abs(target_length[i] - TARGET_LENGTH))
            {
                target_length[i] = TARGET_LENGTH;
            }

            if(0.05f < fabsf(target_angle[i] - TARGET_ANGLE))
            {
                target_angle[i] -= ANGLE_SPEED;
                target_angle[i] = wrap2pi_f32(target_angle[i]);
            }
            else {

                target_angle[i] = TARGET_ANGLE;
            
            }
        }
    }
}

}