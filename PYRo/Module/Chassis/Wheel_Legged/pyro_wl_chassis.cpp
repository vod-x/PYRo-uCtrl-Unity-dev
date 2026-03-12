/*
 * @Author: Vod vod0575@outlook
 * @Date: 2026-02-06 15:27:37
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-03-10 09:33:37
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */

#include "pyro_wl_chassis.h"
#include "pyro_dwt_drv.h"

 namespace pyro
 {
wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis", 0, 512)
{
}

status_t wl_chassis_t::get_cur_angle(float *r_angle, float *l_angle)
{
    if(!l_angle || !r_angle)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_angle = _leg_data[R].alpha;
    *l_angle = _leg_data[L].alpha;
    return PYRO_OK;
}

status_t wl_chassis_t::get_cur_length(float *r_leg, float *l_leg)
{
    if(!l_leg || !r_leg)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_leg = _leg_data[R].l;
    *l_leg = _leg_data[L].l;
    return PYRO_OK;
}

status_t wl_chassis_t::get_cur_p_torque(float *r_torque, float *l_torque)
{
    if(!r_torque || !l_torque)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_torque = _leg_data[R].F[1];
    *l_torque = _leg_data[L].F[1];
    return PYRO_OK;
}

uint8_t wl_chassis_t::get_status_flag(wl_cmd_t::active_mode_t mode)
{
    switch(mode)
    {
        case wl_cmd_t::READY:
            return _active_mode_flag.ready;
        case wl_cmd_t::TEST:
            return _active_mode_flag.test;
        case wl_cmd_t::REVERSE:
            return _active_mode_flag.reverse;
        case wl_cmd_t::OVER_STEP:
            return _active_mode_flag.over_step;
        case wl_cmd_t::NORMAL:
            return _active_mode_flag.normal;
        default:
            return 0;
    }
}

status_t wl_chassis_t::_init()
{
    status_t ret;

    /* Initialize kinematic solver with given coefficients. */
    ret = _kinematic_solver.init(&_module_deps.phi_k, 
             &_module_deps.polar_k, &_module_deps.vmc_k);
    CHECK_PYRO_RET(ret);
    /* Save LQR coefficients */
    memcpy(_lqr_cof, _module_deps.lqr_coef, sizeof(float) * 48);

    /* Save wheel radius and reduction ratio */
    _wheel_radius = _module_deps.wheel_radius;
    _reduction_ratio = _module_deps.reduction_ratio;

    /* Initialize joint motor driver */
    for(uint8_t i = 0; i < 4; i++)
    {
        _motor_drv[i] = new dm_motor_drv_t(_module_deps.joint_motor_cfg[i].tx_id,
                                           _module_deps.joint_motor_cfg[i].rx_id,
                                           _module_deps.joint_motor_cfg[i].can);
        if(!_motor_drv[i])
        {
            return PYRO_NO_MEMORY;
        }
        _motor_offset[i] = _module_deps.joint_motor_cfg[i].offset_angle;
        _motor_drv[i]->set_rotate_range(_module_deps.rotate_min, 
                                                  _module_deps.rotate_max);
        _motor_drv[i]->set_position_range(_module_deps.position_min,
                                                  _module_deps.position_max);
        _motor_drv[i]->set_torque_range(_module_deps.torque_min,
                                                  _module_deps.torque_max);
    }
    /* Initialize wheel motor driver */
    for(uint8_t i = 0; i < 2; i++)
    {
        _wheel_drv[i] = 
               new dji_m3508_motor_drv_t(_module_deps.wheel_motor_cfg[i].tx_id,
                                     _module_deps.wheel_motor_cfg[i].can);
        if(!_wheel_drv[i])
        {
            return PYRO_NO_MEMORY;
        }
    }
    /* Initialize VMC matrix */
    for(uint8_t i = 0; i < 2; i++)
    {
        arm_mat_init_f32(&_leg_data[i].T_mat, 2, 2, 
                                            _leg_data[i].T_mat_val);
    }
    /* Initialize PID controllers */
    for(uint8_t i = 0; i < 2; i++)
    {
        /* Init T pid */
        _T_pid[i] = new pid_t(_module_deps.T_pid_cfg[i].kp, _module_deps.T_pid_cfg[i].ki, 
                            _module_deps.T_pid_cfg[i].kd, 
                            _module_deps.T_pid_cfg[i].integral_limit,
                            _module_deps.T_pid_cfg[i].max_out);
        if(!_T_pid[i])
        {
            return PYRO_NO_MEMORY;
        }
        /* Init d_T pid */
        _d_T_pid[i] = new pid_t(_module_deps.F_pid_cfg[i].kp, _module_deps.F_pid_cfg[i].ki, 
                            _module_deps.F_pid_cfg[i].kd, 
                            _module_deps.F_pid_cfg[i].integral_limit,
                            _module_deps.F_pid_cfg[i].max_out);
        if(!_d_T_pid[i])
        {
            return PYRO_NO_MEMORY;
        }
        /* Init F pid */
        _F_pid[i] = new pid_t(_module_deps.d_F_pid_cfg[i].kp, _module_deps.d_F_pid_cfg[i].ki, 
                            _module_deps.d_F_pid_cfg[i].kd, 
                            _module_deps.d_F_pid_cfg[i].integral_limit,
                            _module_deps.d_F_pid_cfg[i].max_out);
        if(!_F_pid[i])        
        {
            return PYRO_NO_MEMORY;
        }
        /* Init d_F pid */
        _d_F_pid[i] = new pid_t(_module_deps.d_T_pid_cfg[i].kp, _module_deps.d_T_pid_cfg[i].ki, 
                            _module_deps.d_T_pid_cfg[i].kd, 
                            _module_deps.d_T_pid_cfg[i].integral_limit,
                            _module_deps.d_T_pid_cfg[i].max_out);
        if(!_d_F_pid[i])
        {
            return PYRO_NO_MEMORY;
        }
    }
    _yaw_pid = new pid_t(_module_deps.yaw_pid_cfg.kp, _module_deps.yaw_pid_cfg.ki, 
                            _module_deps.yaw_pid_cfg.kd, 
                            _module_deps.yaw_pid_cfg.integral_limit,
                            _module_deps.yaw_pid_cfg.max_out);
    if(!_yaw_pid)
    {
        return PYRO_NO_MEMORY;
    }
    _g_yaw_pid = new pid_t(_module_deps.g_yaw_pid_cfg.kp, _module_deps.g_yaw_pid_cfg.ki, 
                            _module_deps.g_yaw_pid_cfg.kd, 
                            _module_deps.g_yaw_pid_cfg.integral_limit,
                            _module_deps.g_yaw_pid_cfg.max_out);
    if(!_g_yaw_pid)    
    {
        return PYRO_NO_MEMORY;
    }
    _delta_pid = new pid_t(_module_deps.delta_pid_cfg.kp, _module_deps.delta_pid_cfg.ki, 
                            _module_deps.delta_pid_cfg.kd, 
                            _module_deps.delta_pid_cfg.integral_limit,
                            _module_deps.delta_pid_cfg.max_out);
    if(!_delta_pid)    
    {
        return PYRO_NO_MEMORY;
    }
    _d_delta_pid = new pid_t(_module_deps.d_delta_pid_cfg.kp, _module_deps.d_delta_pid_cfg.ki, 
                            _module_deps.d_delta_pid_cfg.kd, 
                            _module_deps.d_delta_pid_cfg.integral_limit,
                            _module_deps.d_delta_pid_cfg.max_out);
    if(!_d_delta_pid)
    {
        return PYRO_NO_MEMORY;
    }
    /* get INS drv */
    _ins_drv = ins_drv_t::get_instance();

    return ret;
}

void wl_chassis_t::_update_feedback()
{
    static uint32_t dwt_cnt;
    static float last_dx[2];
    /* Update INS data */
    if(_ins_drv)
    {
        _ins_drv->get_rads_b(&yaw, &pitch, &roll);
        _ins_drv->get_gyro_b(&g_yaw, &g_pitch, &g_roll);
        _ins_drv->get_acc_without_g_n(&a_x, &a_y, &a_z);
    }
    /* Update the feedback of joint motors and wheel motors. */
    for(uint8_t i = 0; i < 4; i++)
    {
        _motor_drv[i]->update_feedback();
    }
    /* angle theta takes clockwise as positive, with the forward direction as
       the fixed side. When the legged overlap with forward direction, the value
       of theta is 0, so we need to add offset to fix the installation bias. 
       Due to the direction of right motor is opposite to the direction of 
       theta, we need to invert the sign for the right leg motors. */
    _leg_data[R].theta1 = -_motor_drv[RF]->get_current_position() 
                                                        + _motor_offset[RF];
    _leg_data[R].theta2 = -_motor_drv[RB]->get_current_position() 
                                                        + _motor_offset[RB];
    _leg_data[L].theta1 = _motor_drv[LF]->get_current_position() 
                                                        + _motor_offset[LF];
    _leg_data[L].theta2 = _motor_drv[LB]->get_current_position() 
                                                        + _motor_offset[LB];
    /* The direction of differential of theta is same as theta */
    _leg_data[R].d_theta1 = -_motor_drv[RF]->get_current_rotate();
    _leg_data[R].d_theta2 = -_motor_drv[RB]->get_current_rotate();
    _leg_data[L].d_theta1 = _motor_drv[LF]->get_current_rotate();
    _leg_data[L].d_theta2 = _motor_drv[LB]->get_current_rotate();

    /* Update wheel feedback, state x is from the intefration of x, due to 
       the direction of installation, the direction of left wheel if opposite to
    the direction of forward, so its dx has minus */
    _wheel_drv[R]->update_feedback();
    _leg_data[R].dx = _wheel_drv[R]->get_current_rotate() * _wheel_radius   
                                        / _reduction_ratio;
    _leg_data[R].x += (_leg_data[R].dx + last_dx[R])/2 * 0.001f;
    last_dx[R] = _leg_data[R].dx;
    _wheel_drv[L]->update_feedback();
    _leg_data[L].dx = - _wheel_drv[L]->get_current_rotate() * _wheel_radius 
                                        / _reduction_ratio;
    _leg_data[L].x += (_leg_data[L].dx + last_dx[L])/2 * 0.001f;
    last_dx[L] = _leg_data[L].dx;

    /* kinematic solve the current states of the chassis */
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret;
        float time;
        float last_d_l = _leg_data[i].d_l;
        float last_d_beta = _leg_data[i].d_beta;
        ret = _kinematic_solver.solve(_leg_data[i].theta1, 
                                      _leg_data[i].theta2,
                                    _leg_data[i].d_theta1,
                                    _leg_data[i].d_theta2,
                                        &_leg_data[i].phi1,
                                        &_leg_data[i].phi2,
                                      &_leg_data[i].alpha,
                                       &_leg_data[i].l,
                                    &_leg_data[i].d_l,
                                     &_leg_data[i].d_alpha,
                                     &_leg_data[i].d_jx,
                                     &_leg_data[i].d_jy,
                                     &_leg_data[i].jx,
                                     &_leg_data[i].jy);
        if(ret != PYRO_OK)
        {
            _cnt.solver_error++;
        }
         
        _leg_data[i].beta = PI / 2 - _leg_data[i].alpha - pitch;
        _leg_data[i].d_beta = -_leg_data[i].d_alpha - g_pitch;
        _leg_data[i].gamma = -pitch;
        _leg_data[i].d_gamma = -g_pitch;

         /* The second differential of beta is calculated by data, which may be
            noisy but can reflect the real dynamic of the chassis. */
        time = dwt_drv_t::get_delta_t(&dwt_cnt);
        _leg_data[i].d2_beta = (_leg_data[i].d_beta - last_d_beta) / time;
         /* The second differential of l is calculated by data, which may be
            noisy but can reflect the real dynamic of the chassis. */
        _leg_data[i].d2_l = (_leg_data[i].d_l - last_d_l) / time;
    }

    /* update VMC matrix */
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret;

        ret = _kinematic_solver.get_VMC_value(_leg_data[i].theta1, 
                                             _leg_data[i].theta2,
                                             _leg_data[i].phi1,
                                             _leg_data[i].phi2,
                                             _leg_data[i].l,
                                             _leg_data[i].alpha,
                                             _leg_data[i].T_mat.pData);
        if(ret != PYRO_OK)
        {
            _cnt.solver_error++;
        }
    }

}

void wl_chassis_t::_fsm_execute()
{
    _cmd = &_current_cmd;
    if (cmd_base_t::mode_t::PASSIVE == _cmd->mode)
        _fsm.change_state(&_state_passive)  ;
    else if (cmd_base_t::mode_t::ACTIVE == _cmd->mode)
        _fsm.change_state(&_state_active);
    _fsm.execute(this);
}
 }