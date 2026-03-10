/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 20:18:33
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-03-10 09:46:46
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_rc_hub.h"
#include "pyro_ins.h"
#include "pyro_algo_common.h"

using namespace pyro;
wl_chassis_t *infantry2_chassis_ptr = nullptr;
wl_cmd_t     *infantry2_chassis_cmd_ptr = nullptr;
dr16_drv_t::dr16_ctrl_t const *infantry2_rc_ctrl_ptr = nullptr;
extern wl_chassis_cfg_t infantry2_chassis_cfg;
extern "C"
{
void infantry2_chassis_rc2cmd(void const *rc_ctrl)
{
    static pyro::cmd_base_t::mode_t last_mode = pyro::cmd_base_t::mode_t::PASSIVE; 
    pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(
         pyro::rc_hub_t::DR16)->get_lock());
   static auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
    
    if(last_mode == pyro::cmd_base_t::mode_t::PASSIVE && 
        p_ctrl->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_MID)
    {
        infantry2_chassis_ptr->get_cur_angle(&infantry2_chassis_cmd_ptr->r_angle,
                              &infantry2_chassis_cmd_ptr->l_angle);
        infantry2_chassis_ptr->get_cur_leg(&infantry2_chassis_cmd_ptr->r_leg,
                            &infantry2_chassis_cmd_ptr->l_leg);
    }
    if(pyro::dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_r.state || pyro::dr16_drv_t::sw_state_t::SW_DOWN == p_ctrl->rc.s_r.state)
    {
        infantry2_chassis_cmd_ptr->l_angle = PI/2.0f;
        infantry2_chassis_cmd_ptr->r_angle = PI/2.0f;
        infantry2_chassis_cmd_ptr->l_leg = 0;
        infantry2_chassis_cmd_ptr->r_leg = 0;
        infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        float yaw, pitch, roll;
        ins_drv_t::get_instance()->get_rads_b(&yaw, &pitch, &roll);
        infantry2_chassis_cmd_ptr->yaw = yaw;
            
        last_mode = infantry2_chassis_cmd_ptr->mode;
        return;
    }
    


    infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
    if(p_ctrl->rc.s_l.state == pyro::dr16_drv_t::sw_state_t::SW_UP)
    {
        infantry2_chassis_cmd_ptr->r_angle += (p_ctrl->rc.ch_rx * PI / 2000.0f);
        infantry2_chassis_cmd_ptr->l_angle += (p_ctrl->rc.ch_lx * PI / 2000.0f);
        infantry2_chassis_cmd_ptr->r_angle = loop_fp32_constrain(
            infantry2_chassis_cmd_ptr->r_angle, -PI, PI);
        infantry2_chassis_cmd_ptr->l_angle = loop_fp32_constrain(
            infantry2_chassis_cmd_ptr->l_angle, -PI, PI);

        infantry2_chassis_cmd_ptr->r_leg += (p_ctrl->rc.ch_ry / 2000.0f);
        infantry2_chassis_cmd_ptr->l_leg += (p_ctrl->rc.ch_ly / 2000.0f);
        infantry2_chassis_cmd_ptr->r_leg = fp32_constrain(
           infantry2_chassis_cmd_ptr->r_leg, 0.14f, 0.33f);
        infantry2_chassis_cmd_ptr->l_leg = fp32_constrain(
           infantry2_chassis_cmd_ptr->l_leg, 0.14f, 0.33f);
        infantry2_chassis_cmd_ptr->active_mode = 2;
    }
    else if(p_ctrl->rc.s_l.state == pyro::dr16_drv_t::sw_state_t::SW_MID)
    {
        infantry2_chassis_cmd_ptr->active_mode = 1;

        infantry2_chassis_cmd_ptr->l_leg = 0.27f;
        infantry2_chassis_cmd_ptr->r_leg = 0.27f;

        infantry2_chassis_cmd_ptr->yaw += (p_ctrl->rc.ch_lx * PI / 4000.0f);
        infantry2_chassis_cmd_ptr->yaw = loop_fp32_constrain(
            infantry2_chassis_cmd_ptr->yaw, -PI, PI);

    }
    else
    {
        infantry2_chassis_cmd_ptr->active_mode = 0;
    }

    last_mode = infantry2_chassis_cmd_ptr->mode;
}
void infantry2_chassis_main_tread(void *argument)
{
    status_t ret = infantry2_chassis_ptr->start();
    while(1)
    {
        infantry2_chassis_rc2cmd(infantry2_rc_ctrl_ptr);
        infantry2_chassis_ptr->set_command(*infantry2_chassis_cmd_ptr);
        vTaskDelay(2);
    }
}

status_t infantry2_chassis_init(void *argument)
{
    infantry2_chassis_cmd_ptr = new wl_cmd_t();
    infantry2_chassis_ptr = wl_chassis_t::instance();
    infantry2_rc_ctrl_ptr = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
    BaseType_t ret = 
        xTaskCreate(infantry2_chassis_main_tread, 
            "Infantry2 Chassis", 512, 
            nullptr, 1, nullptr);
    CHECK_OS_RET(ret);
    infantry2_chassis_ptr->configure(infantry2_chassis_cfg);
    return status_t::PYRO_OK;
}

}
