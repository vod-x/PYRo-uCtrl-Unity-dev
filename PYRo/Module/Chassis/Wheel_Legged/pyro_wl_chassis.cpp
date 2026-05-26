/*
 * @Author: Vod vod0575@outlook
 * @Date: 2026-02-06 15:27:37
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-23 03:42:32
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */

#include "pyro_wl_chassis.h"
#include "pyro_dwt_drv.h"
#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"
#include "pyro_vofa.h"
#include "pyro_referee.h"

#define WHEEL_DISTANCE 0.424f
#define SUPPORT_FORCE_ACC_LPF_RC 0.01f
/* IMU offset from yaw rotation center (midpoint of two wheels) along body x-axis.
   Positive = IMU is in front of wheel axis. Measure and adjust this value. */
#define IMU_OFFSET_X  0.2f

 namespace pyro
{
float time;
float last_time;
float test_wl_chassis_power_cap{};
float test_wl_cap_power_cap{};
float test_wl_cap_vot{};
supercap_drv_t::cap_feedback_t test_wl_cap_feedback{};

wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis", 0, 2048),
    _wheel_kf{kf_t(3, 1, 3, 2), kf_t(3, 1, 3, 2)}
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

status_t wl_chassis_t::get_cur_d_angle(float *r_d_angle, float *l_d_angle)
{
    if(!l_d_angle || !r_d_angle)
    {
        return PYRO_PARAM_ERROR;
    }
    *r_d_angle = _leg_data[R].d_alpha;
    *l_d_angle = _leg_data[L].d_alpha;
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

status_t wl_chassis_t::get_cur_ins_yaw(float* temp_yaw)
{
    if(!temp_yaw)
    {
        return PYRO_PARAM_ERROR;
    }
    *temp_yaw = yaw;
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
        case wl_cmd_t::CONTROL:
            return _active_mode_flag.control;
        default:
            return 0;
    }
}

status_t wl_chassis_t::clear_status_flag(wl_cmd_t::active_mode_t mode)
{
    switch(mode)
    {
        case wl_cmd_t::READY:
            _active_mode_flag.ready = 0;
            break;
        case wl_cmd_t::TEST:
            _active_mode_flag.test = 0;
            break;
        case wl_cmd_t::REVERSE:
            _active_mode_flag.reverse = 0;
            break;
        case wl_cmd_t::OVER_STEP:
            _active_mode_flag.over_step = 0;
            break;
        case wl_cmd_t::NORMAL:
            _active_mode_flag.normal = 0;
            break;
        case wl_cmd_t::CONTROL:
            _active_mode_flag.control = 0;
            break;
        default:
            return PYRO_PARAM_ERROR;
    }
    return PYRO_OK;
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
    memcpy(_lqr_cof_over_step, _module_deps.lqr_coef_over_step, sizeof(float) * 48);

    /* Save wheel radius and reduction ratio */
    _wheel_radius = _module_deps.wheel_radius;
    _reduction_ratio = _module_deps.reduction_ratio;

    _power_ctrl.init(&_module_deps.power_ctrl_cfg);

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
    /* Initialize gimbal motor driver */
    _yaw_motor_drv = new dji_gm_6020_motor_drv_t(_module_deps.yaw_motor_cfg.tx_id,
                                                  _module_deps.yaw_motor_cfg.can);
    _yaw_offset = _module_deps.yaw_offset;
    if(!_yaw_motor_drv)
    {
        return PYRO_NO_MEMORY;
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
        _d_T_pid[i] = new pid_t(_module_deps.d_T_pid_cfg[i].kp, _module_deps.d_T_pid_cfg[i].ki, 
                            _module_deps.d_T_pid_cfg[i].kd, 
                            _module_deps.d_T_pid_cfg[i].integral_limit,
                            _module_deps.d_T_pid_cfg[i].max_out);
        if(!_d_T_pid[i])
        {
            return PYRO_NO_MEMORY;
        }
        /* Init F pid */
        _F_pid[i] = new pid_t(_module_deps.F_pid_cfg[i].kp, _module_deps.F_pid_cfg[i].ki, 
                            _module_deps.F_pid_cfg[i].kd, 
                            _module_deps.F_pid_cfg[i].integral_limit,
                            _module_deps.F_pid_cfg[i].max_out);
        if(!_F_pid[i])        
        {
            return PYRO_NO_MEMORY;
        }
        /* Init d_F pid */
        _d_F_pid[i] = new pid_t(_module_deps.d_F_pid_cfg[i].kp, _module_deps.d_F_pid_cfg[i].ki, 
                            _module_deps.d_F_pid_cfg[i].kd, 
                            _module_deps.d_F_pid_cfg[i].integral_limit,
                            _module_deps.d_F_pid_cfg[i].max_out);
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

    _roll_pid = new pid_t(_module_deps.roll_pid_cfg.kp, _module_deps.roll_pid_cfg.ki, 
                            _module_deps.roll_pid_cfg.kd, 
                            _module_deps.roll_pid_cfg.integral_limit,
                            _module_deps.roll_pid_cfg.max_out);
    if(!_roll_pid)
    {
        return PYRO_NO_MEMORY;
    }
    /* Initialize Kalman filter for the wheel velocity */
    for(uint8_t i = 0; i < 2; i++)
    {
        ret = _wheel_kf[i].init(_module_deps.wheel_kf_cfg[i].A, 
                                 _module_deps.wheel_kf_cfg[i].B, 
                                 _module_deps.wheel_kf_cfg[i].H, 
                                 _module_deps.wheel_kf_cfg[i].G,
                                 _module_deps.wheel_kf_cfg[i].Q, 
                                 _module_deps.wheel_kf_cfg[i].R,
                                 _module_deps.wheel_kf_cfg[i].x_init,
                                 _module_deps.wheel_kf_cfg[i].P_init);
        CHECK_PYRO_RET(ret);
    }
    /* get INS drv */
    _ins_drv = ins_drv_t::get_instance();
    yaw = pitch = roll = 0.0f;
    g_yaw = g_pitch = g_roll = 0.0f;
    a_x = a_y = a_z = 0.0f;
    a_forward = a_upward = a_upward_lpf = 0.0f;

    return ret;
}

float power;
float energy;
float limit;

extern referee_drv_t *referee_drv;
void wl_chassis_t::_update_feedback()
{
    power = referee_drv->get_data().robot_status.chassis_power_limit;
    last_time = dwt_drv_t::get_timeline_ms();

    _supercap_cmd.power_referee = 0;
    _supercap_cmd.power_limit_referee = referee_drv->get_data().robot_status.chassis_power_limit;
    _supercap_cmd.power_buffer_limit_referee = 60.0f;
    _supercap_cmd.power_buffer_referee = referee_drv->get_data().power_heat.buffer_energy;
    _supercap_cmd.kill_chassis_user = 0;
    _supercap_cmd.speed_up_user_now = 0;
   
    _cap_feedback = supercap_drv_t::get_instance()->get_feedback();

    test_wl_cap_feedback = _cap_feedback;
    test_wl_chassis_power_cap = test_wl_cap_feedback.chassis_power_cap / 100.0f; 
    test_wl_cap_power_cap = test_wl_cap_feedback.cap_power_cap / 100.0f - 250; 
    test_wl_cap_vot = test_wl_cap_feedback.vot_cap / 100.0f; 


    static uint32_t dwt_cnt;
    static float last_dx[2];
    /* Update INS data */
    if(_ins_drv)
    {
        _ins_drv->get_rads_b(&yaw, &pitch, &roll);
        _ins_drv->get_gyro_b(&g_yaw, &g_pitch, &g_roll);
        _ins_drv->get_acc_without_g_b(&a_x, &a_y, &a_z);
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
    _leg_data[R].w = _wheel_drv[R]->get_current_rotate() / _reduction_ratio;
    _leg_data[R].T_w_real = _wheel_drv[R]->get_current_torque()*0.3f/(3591.0f/187.0f) * _reduction_ratio;
    _leg_data[R].dx = _leg_data[R].w * _wheel_radius;
    _leg_data[R].x += (_leg_data[R].dx + last_dx[R])/2 * 0.001f;
    last_dx[R] = _leg_data[R].dx;
    _wheel_drv[L]->update_feedback();
    _leg_data[L].w = _wheel_drv[L]->get_current_rotate() / _reduction_ratio;
    _leg_data[L].T_w_real = _wheel_drv[L]->get_current_torque()*0.3f/(3591.0f/187.0f) * _reduction_ratio;
    _leg_data[L].dx = - _leg_data[L].w * _wheel_radius;
    _leg_data[L].x += (_leg_data[L].dx + last_dx[L])/2 * 0.001f;
    last_dx[L] = _leg_data[L].dx;

    _yaw_motor_drv->update_feedback();
    gimbal_yaw = wrap2pi_f32(_yaw_motor_drv->get_current_position() + _yaw_offset);
    gimbal_g_yaw = _yaw_motor_drv->get_current_rotate();

    
    /* kinematic solve the current states of the chassis */
    time = dwt_drv_t::get_delta_t(&dwt_cnt);
    for(uint8_t i = 0; i < 2; i++)
    {
        status_t ret;
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
    /* update Kalman filter for wheel velocity */
    float kf_u = 0.0f;
    float kf_z[3] = {0.0f, 0.0f, 0.0f};
    float kf_estimated[3] = {0.0f, 0.0f, 0.0f};
     /* Project body-frame acceleration to ground-aligned axes.
         Pitch is defined as nose-down positive, so the body x-axis gains a
         downward component as pitch increases. Resolve the measured
         acceleration into forward and upward components before using it in
         observers or support force estimation. */
     a_forward = a_x * arm_cos_f32(pitch)
                  + a_z * arm_sin_f32(pitch)
                  + g_yaw * g_yaw * IMU_OFFSET_X;
     a_upward = -a_x * arm_sin_f32(pitch)
                 + a_z * arm_cos_f32(pitch);
    a_upward_lpf = a_upward_lpf * SUPPORT_FORCE_ACC_LPF_RC / (time + SUPPORT_FORCE_ACC_LPF_RC)
                 + a_upward * time / (time + SUPPORT_FORCE_ACC_LPF_RC);
    /* Compensate centripetal acceleration caused by IMU offset from yaw axis.
       a_x_measured = a_x_linear - w^2 * r_x  →  a_x_linear = a_x + w^2 * r_x */
    /* Average left/right wheel speed to obtain v_center directly.
       dx_R = v + (d/2)*w,  dx_L = v - (d/2)*w  →  mean = v
       Rotation cancels exactly, no gyro involved, immune to gyro bias. */
    float v_obs = (_leg_data[R].dx + _leg_data[L].dx) / 2.0f;
    for(uint8_t i = 0; i < 2; i++)
    {
        kf_u = 0.0f;
        kf_z[0] = v_obs;
        kf_z[1] = a_forward;
        kf_z[2] = g_yaw;   
        _wheel_kf[i].update(kf_z, &kf_u, kf_estimated);
        _leg_data[i].kf_v = kf_estimated[0];
        _leg_data[i].kf_a = kf_estimated[1];
        _leg_data[i].kf_w = kf_estimated[2];
        _leg_data[i].kf_x += _leg_data[i].kf_v * 0.001f;
    }
}

void wl_chassis_t::_fsm_execute()
{
    _cmd = &_current_cmd;
    if (cmd_base_t::mode_t::PASSIVE == _cmd->mode)
        _fsm.change_state(&_state_passive)  ;
    else if (cmd_base_t::mode_t::ACTIVE == _cmd->mode)
        _fsm.change_state(&_state_active);

        _decide_cap();
    _fsm.execute(this);
    time = dwt_drv_t::get_timeline_ms() - last_time;
}
void wl_chassis_t::_send_supercap_command() const
{
    supercap_drv_t::get_instance()->send_cmd(_supercap_cmd);
}

void wl_chassis_t::_decide_cap()
{
    static bool _last_status = false;
    static uint32_t _timer   = 0;
    static bool _delay_done  = false;

    // 从裁判系统获取底盘电源输出状态
    bool current_status = referee_drv->get_data().robot_status.power_management_chassis_output;

    if (current_status)
    {
        // --- 情况 A：底盘电源有输出 ---
        if (!_last_status)
        {
            // 刚切到有输出状态：重置计时器和延迟标志
            _timer      = 0;
            _delay_done = false;
        }

        if (!_delay_done)
        {
            // 1. 处理 1000 tick 的初始延迟
            if (++_timer >= 1000)
            {
                _delay_done           = true;
                _timer                = 0; 
                _supercap_cmd.use_cap = 1;
                _send_supercap_command();
            }
        }
        else
        {
            // 2. 延迟结束后，以 10 tick 为周期发送
            if (++_timer >= 10)
            {
                _timer                = 0;
                _supercap_cmd.use_cap = 1;
                _send_supercap_command();
            }
        }
    }
    else
    {
        // --- 情况 B：底盘电源无输出 ---
        if (_last_status)
        {
            // 刚切换到无输出状态：立刻发送 use_cap = 0
            _supercap_cmd.use_cap = 0;
            _send_supercap_command();

            // 重置状态位，防止重复发送
            _delay_done = false;
            _timer      = 0;
        }
    }

    _last_status = current_status;
}
 }