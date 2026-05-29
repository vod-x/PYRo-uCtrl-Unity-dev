#include "pyro_wl_chassis.h"
#include "pyro_algo_common.h"
#include "pyro_referee.h"
namespace pyro
{
extern pid_t wheel_disable_pid[2];
extern referee_drv_t *referee_drv;
pid_t wheel_turn_pid[2] = {
    pid_t(5.0, 0.0f, 0.0f, 2.0f, 20.0f), 
    pid_t(5.0, 0.0f, 0.0f, 2.0f, 20.0f)};
 const float SPIN_SPEED =8.0f   ;
void wl_chassis_t::fsm_active_t::state_spin_t::enter(wl_chassis_t *owner)
{
   
    owner->_leg_data[wl_chassis_t::R].x_gain = owner->_leg_data[wl_chassis_t::R].kf_x;
    owner->_leg_data[wl_chassis_t::L].x_gain = owner->_leg_data[wl_chassis_t::L].kf_x;

    owner->_leg_data[wl_chassis_t::R].d_x_gain = 0.0f;
    owner->_leg_data[wl_chassis_t::L].d_x_gain = 0.0f;
     owner->_active_mode_flag.ready = 1;
    // owner->_leg_data[wl_chassis_t::R].x = 0.0f;
    // owner->_leg_data[wl_chassis_t::L].x = 0.0f;
    // owner->_leg_data[wl_chassis_t::R].x_gain = 0.0f;
    // owner->_leg_data[wl_chassis_t::L].x_gain = 0.0f;
    // owner->_leg_data[wl_chassis_t::R].kf_x = 0.0f;
    // owner->_leg_data[wl_chassis_t::L].kf_x = 0.0f;
    // owner->_wheel_kf[wl_chassis_t::R].reset();
    // owner->_wheel_kf[wl_chassis_t::L].reset();

}

void wl_chassis_t::fsm_active_t::state_spin_t::execute(wl_chassis_t *owner)
{

    // owner->_leg_data[wl_chassis_t::L].ref_l = owner->_cmd->l_leg / cosf(owner->_leg_data[wl_chassis_t::L].gamma);
    // owner->_leg_data[wl_chassis_t::R].ref_l = owner->_cmd->r_leg / cosf(owner->_leg_data[wl_chassis_t::R].gamma);
     /* Calculate turn torque */
    owner->_leg_data[wl_chassis_t::R].T_w_turn =  wheel_turn_pid[wl_chassis_t::R].calculate(SPIN_SPEED , owner->g_yaw); 
    owner->_leg_data[wl_chassis_t::L].T_w_turn =  wheel_turn_pid[wl_chassis_t::L].calculate(SPIN_SPEED , owner->g_yaw); 

    /* Calculate roll gain to make sure roll angle equal 0 */
    owner->_delta_mea = owner->_leg_data[wl_chassis_t::R].alpha 
                            - owner->_leg_data[wl_chassis_t::L].alpha;
    owner->_d_delta_mea = owner->_leg_data[wl_chassis_t::R].d_alpha 
                            - owner->_leg_data[wl_chassis_t::L].d_alpha;
    owner->_d_delta_ref = owner->_d_delta_pid->calculate(
                                0.0f, owner->_delta_mea);

    owner->T_l_gain = owner->_d_delta_pid->calculate(owner->_d_delta_ref,
                                                 owner->_d_delta_mea);
    // owner->roll_gain = owner->_roll_pid->calculate(0.0f, owner->roll);
owner->roll_gain = 0.0f;

    owner->_leg_data[wl_chassis_t::R].d_x_gain = 0.0f;
    owner->_leg_data[wl_chassis_t::L].d_x_gain = 0.0f;

    owner->_leg_data[wl_chassis_t::R].x_gain = owner->_leg_data[wl_chassis_t::R].kf_x;
    owner->_leg_data[wl_chassis_t::L].x_gain = owner->_leg_data[wl_chassis_t::L].kf_x;
    float chassis_v = (owner->_leg_data[wl_chassis_t::L].dx + owner->_leg_data[wl_chassis_t::R].dx) / 2.0f;

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
                                      owner->_leg_data[i].lqr_gain[4] * (0 - owner->_leg_data[i].beta) + 
                                      owner->_leg_data[i].lqr_gain[5] * (0 - owner->_leg_data[i].d_beta));
            // owner->_leg_data[i].T_w_move = (
            //                           owner->_leg_data[i].lqr_gain[0] * (owner->_leg_data[i].x_gain - owner->_leg_data[i].kf_x) + 
            //                           owner->_leg_data[i].lqr_gain[1] * (owner->_leg_data[i].d_x_gain - owner->_leg_data[i].kf_v));
            //                         //   owner->_leg_data[i].lqr_gain[0] * (owner->_leg_data[i].x_gain - owner->_leg_data[i].x) + 
            //                         //   owner->_leg_data[i].lqr_gain[1] * (owner->_leg_data[i].d_x_gain - owner->_leg_data[i].dx) + 

            // owner->_leg_data[i].F[1] = -(
            //                         //   owner->_leg_data[i].lqr_gain[6] * (owner->_leg_data[i].x_gain - owner->_leg_data[i].kf_x) + 
            //                         //   owner->_leg_data[i].lqr_gain[7] * (owner->_leg_data[i].d_x_gain - owner->_leg_data[i].kf_v) + 
            //                         //   owner->_leg_data[i].lqr_gain[6] * (owner->_leg_data[i].x_gain - owner->_leg_data[i].x) + 
            //                         //   owner->_leg_data[i].lqr_gain[7] * (owner->_leg_data[i].d_x_gain - owner->_leg_data[i].dx) + 
            //                           owner->_leg_data[i].lqr_gain[8] * (0 - owner->_leg_data[i].gamma) + 
            //                           owner->_leg_data[i].lqr_gain[9] * (0 - owner->_leg_data[i].d_gamma) + 
            //                           owner->_leg_data[i].lqr_gain[10] * (0 - owner->_leg_data[i].beta) + 
            //                           owner->_leg_data[i].lqr_gain[11] * (0 - owner->_leg_data[i].d_beta));
            owner->_leg_data[i].T_w_move = owner->_leg_data[i].lqr_gain[1] * (0.0f - chassis_v);

        owner->_leg_data[i].F[1] = -(
                                  owner->_leg_data[i].lqr_gain[7] * (0.0f - chassis_v) + 
                                  owner->_leg_data[i].lqr_gain[8] * (0 - owner->_leg_data[i].gamma) + 
                                  owner->_leg_data[i].lqr_gain[9] * (0 - owner->_leg_data[i].d_gamma) + 
                                  owner->_leg_data[i].lqr_gain[10] * (-0.04f - owner->_leg_data[i].beta) + 
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
  
        owner->_leg_data[wl_chassis_t::R].T_w = owner->_leg_data[wl_chassis_t::R].T_w_balance + owner->_leg_data[wl_chassis_t::R].T_w_move - owner->_leg_data[wl_chassis_t::R].T_w_turn ;
        owner->_leg_data[wl_chassis_t::L].T_w = owner->_leg_data[wl_chassis_t::L].T_w_balance + owner->_leg_data[wl_chassis_t::L].T_w_move + owner->_leg_data[wl_chassis_t::L].T_w_turn ;
        // owner->_leg_data[wl_chassis_t::R].T_w = owner->_leg_data[wl_chassis_t::R].T_w_balance + owner->_leg_data[wl_chassis_t::R].T_w_move ;
        // owner->_leg_data[wl_chassis_t::L].T_w = owner->_leg_data[wl_chassis_t::L].T_w_balance + owner->_leg_data[wl_chassis_t::L].T_w_move ;
        // owner->_power_ctrl.set_max_power(1000.0f);
        float T[2];
        wl_wheel_cmd_t cmd[2];
        cmd[wl_chassis_t::R].tau_balance = -owner->_leg_data[wl_chassis_t::R].T_w_balance;
        cmd[wl_chassis_t::L].tau_balance = owner->_leg_data[wl_chassis_t::L].T_w_balance;
        cmd[wl_chassis_t::R].tau_motion = -owner->_leg_data[wl_chassis_t::R].T_w_move + owner->_leg_data[wl_chassis_t::R].T_w_turn;
        cmd[wl_chassis_t::L].tau_motion = owner->_leg_data[wl_chassis_t::L].T_w_move + owner->_leg_data[wl_chassis_t::L].T_w_turn;
        cmd[wl_chassis_t::R].omega = owner->_leg_data[wl_chassis_t::R].w;
        cmd[wl_chassis_t::L].omega = owner->_leg_data[wl_chassis_t::L].w;
        owner->_power_ctrl.update(cmd, T); 
        owner->_power_ctrl.set_max_power(referee_drv->get_data().robot_status.chassis_power_limit);
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
    owner->_leg_data[wl_chassis_t::R].x_gain = owner->_leg_data[wl_chassis_t::R].kf_x;
    owner->_leg_data[wl_chassis_t::L].x_gain = owner->_leg_data[wl_chassis_t::L].kf_x;
    owner->_active_mode_flag.ready = 1;
}

}