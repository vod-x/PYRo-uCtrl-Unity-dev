#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"
namespace pyro
{
extern pid_t wheel_disable_pid[2];

void wl_chassis_t::fsm_active_t::state_spin_t::enter(wl_chassis_t *owner)
{
    owner->_flag.aerial_cnt = 0;
    owner->_flag.is_aerial = 0;
    owner->_flag.test = 0;
    for(uint8_t i = 0; i < 2; i++)
    {
        owner->_leg_data[i].ref_l = owner->_cmd->l_leg;
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
void wl_chassis_t::fsm_active_t::state_spin_t::execute(wl_chassis_t *owner)
{

    

    /* Calculate Tw turn */
    float yaw_ref, g_yaw_ref, diff;
    yaw_ref = owner->_cmd->yaw;
    if(yaw_ref - owner->yaw > PI)
    {
        diff = -2 * PI + (yaw_ref - owner->yaw);
    }
    else if(yaw_ref - owner->yaw < -PI)
    {
        diff = 2 * PI + (yaw_ref - owner->yaw);
    }
    else
    {
        diff = yaw_ref - owner->yaw;
    }
    g_yaw_ref = owner->_yaw_pid->calculate(owner->yaw + diff, owner->yaw);
    owner->_yaw_ref = yaw_ref;
    owner->_g_yaw_ref = g_yaw_ref;
    owner->_T_w_gain = owner->_g_yaw_pid->calculate(g_yaw_ref, owner->g_yaw);

    // g_yaw_ref = owner->_yaw_pid->calculate(0.0f, -owner->gimbal_yaw);
    // owner->_yaw_ref = yaw_ref;
    // owner->_g_yaw_ref = g_yaw_ref;
    // owner->_T_w_gain = owner->_g_yaw_pid->calculate(g_yaw_ref, -owner->gimbal_g_yaw); 

    owner->_leg_data[wl_chassis_t::R].T_w_turn = owner->_T_w_gain;
    owner->_leg_data[wl_chassis_t::L].T_w_turn = owner->_T_w_gain;

    /* Calculate roll gain to make sure roll angle equal 0 */
    owner->_delta_mea = owner->_leg_data[wl_chassis_t::R].alpha 
                            - owner->_leg_data[wl_chassis_t::L].alpha;
    owner->_d_delta_mea = owner->_leg_data[wl_chassis_t::R].d_alpha 
                            - owner->_leg_data[wl_chassis_t::L].d_alpha;
    owner->_d_delta_ref = owner->_d_delta_pid->calculate(
                                0.0f, owner->_delta_mea);
    owner->T_l_gain = owner->_d_delta_pid->calculate(owner->_d_delta_ref,
                                                 owner->_d_delta_mea);
    owner->roll_gain = owner->_roll_pid->calculate(0.0f, owner->roll);



    /* Calculate target x */
    static float last_d_x_gain[2] = {0.0f, 0.0f};

    owner->_leg_data[wl_chassis_t::R].d_x_gain = owner->_cmd->vx;
    owner->_leg_data[wl_chassis_t::L].d_x_gain = owner->_cmd->vx;
    owner->_leg_data[wl_chassis_t::R].x_gain += (
        owner->_leg_data[wl_chassis_t::R].d_x_gain 
        + last_d_x_gain[wl_chassis_t::R]) / 2.0f /1000.0f;
    owner->_leg_data[wl_chassis_t::L].x_gain += (
        owner->_leg_data[wl_chassis_t::L].d_x_gain 
        + last_d_x_gain[wl_chassis_t::L]) / 2.0f /1000.0f;

    last_d_x_gain[wl_chassis_t::R] = owner->_leg_data[wl_chassis_t::R].d_x_gain;
    last_d_x_gain[wl_chassis_t::L] = owner->_leg_data[wl_chassis_t::L].d_x_gain;


    /* Calculate the target torque of VMC for each leg */
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

                owner->_cmd->r_leg -= owner->roll_gain;
                owner->_cmd->l_leg += owner->roll_gain;

                if(0.005f < abs(owner->_cmd->r_leg - owner->_leg_data[wl_chassis_t::R].ref_l))
                {
                    if(owner->_cmd->r_leg < owner->_leg_data[wl_chassis_t::R].ref_l)
                    {
                        owner->_leg_data[wl_chassis_t::R].ref_l -= 0.0001f;
                    }
                    else
                    {
                        owner->_leg_data[wl_chassis_t::R].ref_l += 0.0001f;
                    }
                }
                else
                {
                    owner->_leg_data[wl_chassis_t::R].ref_l = owner->_cmd->r_leg;
                }
                if(0.005f < abs(owner->_cmd->l_leg - owner->_leg_data[wl_chassis_t::L].ref_l))
                {
                    if(owner->_cmd->l_leg < owner->_leg_data[wl_chassis_t::L].ref_l)
                    {
                        owner->_leg_data[wl_chassis_t::L].ref_l -= 0.0001f;
                    }
                    else
                    {
                        owner->_leg_data[wl_chassis_t::L].ref_l += 0.0001f;
                    }
                }
                else
                {
                    owner->_leg_data[wl_chassis_t::L].ref_l = owner->_cmd->l_leg;
                }
            
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
                                      owner->_leg_data[i].lqr_gain[4] * (0.1f - owner->_leg_data[i].beta) + 
                                      owner->_leg_data[i].lqr_gain[5] * (0 - owner->_leg_data[i].d_beta));
            owner->_leg_data[i].T_w_move = (
                                      owner->_leg_data[i].lqr_gain[0] * (owner->_leg_data[i].x_gain - owner->_leg_data[i].kf_x) + 
                                      owner->_leg_data[i].lqr_gain[1] * (owner->_leg_data[i].d_x_gain - owner->_leg_data[i].kf_v));
                                    //   owner->_leg_data[i].lqr_gain[0] * (owner->_leg_data[i].x_gain - owner->_leg_data[i].x) + 
                                    //   owner->_leg_data[i].lqr_gain[1] * (owner->_leg_data[i].d_x_gain - owner->_leg_data[i].dx) + 

            owner->_leg_data[i].F[1] = -(
                                      owner->_leg_data[i].lqr_gain[6] * (owner->_leg_data[i].x_gain - owner->_leg_data[i].kf_x) + 
                                      owner->_leg_data[i].lqr_gain[7] * (owner->_leg_data[i].d_x_gain - owner->_leg_data[i].kf_v) + 
                                    //   owner->_leg_data[i].lqr_gain[6] * (owner->_leg_data[i].x_gain - owner->_leg_data[i].x) + 
                                    //   owner->_leg_data[i].lqr_gain[7] * (owner->_leg_data[i].d_x_gain - owner->_leg_data[i].dx) + 
                                      owner->_leg_data[i].lqr_gain[8] * (0 - owner->_leg_data[i].gamma) + 
                                      owner->_leg_data[i].lqr_gain[9] * (0 - owner->_leg_data[i].d_gamma) + 
                                      owner->_leg_data[i].lqr_gain[10] * (-0.1f - owner->_leg_data[i].beta) + 
                                      owner->_leg_data[i].lqr_gain[11] * (0 - owner->_leg_data[i].d_beta));
                                  
    }
    owner->_leg_data[wl_chassis_t::R].F[0] = fp32_constrain(owner->_leg_data[wl_chassis_t::R].F[0], -200.0f, 200.0f);
    owner->_leg_data[wl_chassis_t::L].F[0] = fp32_constrain(owner->_leg_data[wl_chassis_t::L].F[0], -200.0f, 200.0f);
    owner->_leg_data[wl_chassis_t::R].F[1] += owner->T_l_gain;
    owner->_leg_data[wl_chassis_t::L].F[1] -= owner->T_l_gain;


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
    if(owner->_flag.is_aerial)
    {
        for(uint8_t i = 0; i < 2; i++)
        {
            // owner->_wheel_drv[i]->send_torque(wheel_disable_pid[i].calculate(0.0f,
            //   owner->_wheel_drv[i]->get_current_rotate()));
            owner->_wheel_drv[i]->send_torque(0.0f);

        }
    }
    else
    {
        owner->_leg_data[wl_chassis_t::R].T_w = owner->_leg_data[wl_chassis_t::R].T_w_balance + owner->_leg_data[wl_chassis_t::R].T_w_move - owner->_T_w_gain;
        owner->_leg_data[wl_chassis_t::L].T_w = owner->_leg_data[wl_chassis_t::L].T_w_balance + owner->_leg_data[wl_chassis_t::L].T_w_move + owner->_T_w_gain;
        // owner->_leg_data[wl_chassis_t::R].T_w = owner->_leg_data[wl_chassis_t::R].T_w_balance + owner->_leg_data[wl_chassis_t::R].T_w_move ;
        // owner->_leg_data[wl_chassis_t::L].T_w = owner->_leg_data[wl_chassis_t::L].T_w_balance + owner->_leg_data[wl_chassis_t::L].T_w_move ;
        owner->_power_ctrl.set_max_power(1000.0f);
        float T[2];
        wl_wheel_cmd_t cmd[2];
        cmd[wl_chassis_t::R].tau_balance = -owner->_leg_data[wl_chassis_t::R].T_w_balance;
        cmd[wl_chassis_t::L].tau_balance = owner->_leg_data[wl_chassis_t::L].T_w_balance;
        cmd[wl_chassis_t::R].tau_motion = -owner->_leg_data[wl_chassis_t::R].T_w_move + owner->_T_w_gain;
        cmd[wl_chassis_t::L].tau_motion = owner->_leg_data[wl_chassis_t::L].T_w_move + owner->_T_w_gain;
        cmd[wl_chassis_t::R].omega = owner->_leg_data[wl_chassis_t::R].w;
        cmd[wl_chassis_t::L].omega = owner->_leg_data[wl_chassis_t::L].w;
        owner->_power_ctrl.update(cmd, T); 
        for(uint8_t i = 0; i < 2; i++)
        {
            owner->_leg_data[i].T_w_out = T[i];
        }
        owner->_leg_data[wl_chassis_t::R].predict_power = owner->_power_ctrl.predict_power(wl_chassis_t::R, -owner->_leg_data[wl_chassis_t::R].T_w, owner->_leg_data[wl_chassis_t::R].w);
        owner->_leg_data[wl_chassis_t::L].predict_power = owner->_power_ctrl.predict_power(wl_chassis_t::L, owner->_leg_data[wl_chassis_t::L].T_w, owner->_leg_data[wl_chassis_t::L].w);
        // owner->_wheel_drv[wl_chassis_t::R]->send_torque(fp32_constrain(owner->_leg_data[wl_chassis_t::R].T_w_out / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
        // owner->_wheel_drv[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_leg_data[wl_chassis_t::L].T_w_out / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
        owner->_wheel_drv[wl_chassis_t::R]->send_torque(fp32_constrain(-owner->_leg_data[wl_chassis_t::R].T_w / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
        owner->_wheel_drv[wl_chassis_t::L]->send_torque(fp32_constrain(owner->_leg_data[wl_chassis_t::L].T_w / owner->_reduction_ratio /0.3f * (3591.0f/187.0f), -20.0f, 20.0f));
    }
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
}
void wl_chassis_t::fsm_active_t::state_spin_t::exit(wl_chassis_t *owner)
{
    owner->_active_mode_flag.ready = 0;
}

}