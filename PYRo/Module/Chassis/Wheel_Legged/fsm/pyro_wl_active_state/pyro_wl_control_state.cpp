/*
 * @Author: vod-x vod_x@outlook.com
 * @Date: 2026-04-18 15:06:20
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-04-22 17:21:45
 * @FilePath: \Wheel-Legged-Robot\embedded_system\PYRo\Module\Chassis\Wheel_Legged\fsm\pyro_wl_active_state\pyro_wl_control_state.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pyro_wl_chassis.h"
namespace pyro
{
extern pid_t wheel_disable_pid[2];
void wl_chassis_t::fsm_active_t::state_control_t::enter(wl_chassis_t *owner)
{
    
}

void wl_chassis_t::fsm_active_t::state_control_t::execute(wl_chassis_t *owner)
{
    for(uint8_t i = 0; i < 2; i++)
    {
            float t = wheel_disable_pid[i].calculate(0.0f,
                 owner->_wheel_drv[i]->get_current_rotate());
            owner->_wheel_drv[i]->send_torque(t);
    }
    
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

    /* Calculate the target torque of VMC for each leg */
    /* Right leg */
    float diff;
    if(owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha > PI)
    {
        diff = -2 * PI + (owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha);
    }
    else if(owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha < -PI)
    {
        diff = 2 * PI + (owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha);
    }
    else
    {
        diff = owner->_cmd->r_angle - owner->_leg_data[wl_chassis_t::R].alpha;
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
    if(owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha > PI)
    {
        diff = -2 * PI + (owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha);
    }
    else if(owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha < -PI)
    {
        diff = 2 * PI + (owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha);
    }
    else
    {
        diff = owner->_cmd->l_angle - owner->_leg_data[wl_chassis_t::L].alpha;
    }
    owner->_leg_data[wl_chassis_t::L].ref_d_alpha=
        owner->_T_pid[wl_chassis_t::L]->
        calculate(owner->_leg_data[wl_chassis_t::L].alpha + diff,
        owner->_leg_data[wl_chassis_t::L].alpha);
    owner->_leg_data[wl_chassis_t::L].F[1]=
        owner->_d_T_pid[wl_chassis_t::L]->
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
void wl_chassis_t::fsm_active_t::state_control_t::exit(wl_chassis_t *owner)
{
}


}