/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-07 15:14:47
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-08 12:58:48
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#ifndef __PYRO_WL_CHASSIS_H__
#define __PYRO_WL_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_kin.wl.h"

#include "pyro_dm_motor_drv.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_ins.h"
#include "pyro_algo_pid.h"
#include "kf.h"
#include "pyro_wl_power_ctrl.h"

namespace pyro
{
/* DM joint motor configuration structure. */
struct wl_dm_motor_cfg_t
{
    /* can tx id of motor*/
    uint8_t tx_id;
    /* can rx id of motor*/
    uint8_t rx_id;
    /* Which CAN bus the motor is connected to */
    can_hub_t::which_can can;
    /* Offset angle to eliminate the bias of installation(rad)*/
    float offset_angle;
};

/* DJI 3508 configuration structure.*/
struct wl_dji_motor_cfg_t
{
    /* can tx id of motor*/
    dji_motor_tx_frame_t::register_id_t tx_id;
    /* Which CAN bus the motor is connected to */
    can_hub_t::which_can can;
};

struct wl_pid_cfg_t
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float max_out;
};

struct wl_kf_cfg_t
{
   float *x_init;
   float *P_init;
   float *A;
   float *B;
   float *H;
   float *G;
   float *Q;
   float *R;
};

/* Configuration structure for wheel-legged chassis, every parameter
   should be set here */
struct wl_chassis_cfg_t
{
    /* Kinematic coefficients for the chassis. */
    wheel_legged_kin_t::phi_k_t phi_k;
    wheel_legged_kin_t::polar_k_t polar_k;
    wheel_legged_kin_t::vmc_k_t vmc_k;

    /* Joint motor configuration. The order of array is: front-right,
       rear-right, front-left, rear-left */
    wl_dm_motor_cfg_t joint_motor_cfg[4];
    /* Wheel motor configuration. The order of array is: right wheel, left
       wheel */
    wl_dji_motor_cfg_t wheel_motor_cfg[2];
    wl_dji_motor_cfg_t  yaw_motor_cfg;
    float yaw_offset;
    /* PID configuration for the chassis control. The order of array is: 
right leg, left leg */
    wl_pid_cfg_t T_pid_cfg[2];
    wl_pid_cfg_t d_T_pid_cfg[2];
    wl_pid_cfg_t F_pid_cfg[2];
    wl_pid_cfg_t d_F_pid_cfg[2];

    /* Yaw PID configuration for the chassis control.*/
    wl_pid_cfg_t yaw_pid_cfg;
    wl_pid_cfg_t g_yaw_pid_cfg;
   /* PID controllers to control the bias between right leg angle and left 
      leg angle */
   wl_pid_cfg_t delta_pid_cfg;
   wl_pid_cfg_t d_delta_pid_cfg;

   wl_pid_cfg_t roll_pid_cfg;
   
   /* Kalman filter configuration for the wheel velocity */
   wl_kf_cfg_t wheel_kf_cfg[2]; 
    /* LQR coefficients for the chassis control. 2 raw x 6 column, 12 values
       in total. Every value has 3 coefficients.*/
    float *lqr_coef;
    float *lqr_coef_over_step;
    /* Wheel radius(m)*/
    float wheel_radius;
    /* Reduction ratio of the motor */
    float reduction_ratio;
    /* Rotation range of the motor(rad/s)*/
    float rotate_min;
    float rotate_max;
    /* Position range of the motor(rad)*/
    float position_min;
    float position_max;
    /* Torque range of the motor(Nm)*/
    float torque_min;
    float torque_max;
   wl_power_ctrl_cfg_t power_ctrl_cfg;
};
//command structure for wheel-legged chassis
struct wl_cmd_t final : public cmd_base_t
{
    /* Linear velocity in x direction(m/s), positive toward the front of chassis
       negative toward the back of chassis */
    float vx;   
    /* Linear velocity in y direction(m/s), positive toward the left of chassis
       negative toward the right of chassis */
    float vy;  
    /* Angular velocity in z direction(rad/s), positive for counter-clockwise
       negative for clockwise */
    float vz;
    /* Yaw angle of the chassis(rad), counter-clockwise positive */
    float yaw;
    /* Leg length of both sides after normalization, value is between 0 and 1*/
    float l_leg;
    float r_leg;

    float l_angle;
    float r_angle;
    enum active_mode_t
    {
        NORMAL = 0,
        READY = 1,
        TEST = 2,
        REVERSE = 3,
        OVER_STEP = 4,
        OVER_STEP_READY = 5,
        CONTROL = 6,
    }active_mode, last_active_mode;

    /* Construct function, set zero values */
    wl_cmd_t() : vx(0), vy(0), vz(0), yaw(0), l_leg(0), r_leg(0),l_angle(0), r_angle(0), active_mode(TEST)
    {
    }
};

class wl_chassis_t final : public module_base_t<wl_chassis_t, wl_cmd_t, wl_chassis_cfg_t>
{
   friend class module_base_t<wl_chassis_t, wl_cmd_t, wl_chassis_cfg_t>;
   friend class vofa_drv_t;

public:
   
   wl_chassis_t(const wl_chassis_t &)            = delete;
   wl_chassis_t &operator=(const wl_chassis_t &) = delete;

   status_t get_cur_angle(float *r_angle, float *l_angle);
   status_t get_cur_d_angle(float *r_angle, float *l_angle);
   status_t get_cur_length(float *r_leg, float *l_leg);
   status_t get_cur_p_torque(float *r_torque, float *l_torque);
   uint8_t get_status_flag(wl_cmd_t::active_mode_t mode);
   status_t clear_status_flag(wl_cmd_t::active_mode_t mode);
private:
    /**
     * @description:
       Construct function, initialize the kinematic solver with given 
       cofficients. 
    *  @param {cfg_t*} cfg: configuration structure pointer, all parameters for 
       chassis should be set in this structure.
     */
    wl_chassis_t();
    ~wl_chassis_t() override = default;

    /* base interface define */
    /**
     * @description: 
       Initialize the wheel-legged chassis module, this function will be called in
       the function 'start', if this function not return PYRO_OK, function 'start'
       will not create main tread of this module.
     * @return {status_t} 
       PYRO_OK if initialize successfully, otherwise return error code.
     */
    status_t _init() override;
    /**
     * @description: 
       Update the feedback of the chassis, include the data of ins and motor. 
       Call function '_kinematics_solve' to calculate the current states of 
       the chassis, if the kinematic solver not return PYRO_OK, cnt 
       'solver_error' will add 1. This function will be called in the main tread
       of this module.
     * @return {*}
     */
    void _update_feedback() override;
    void _fsm_execute() override;

    /* private function*/

    /* the ID of motor array */
    enum
    {
        RF = 0,
        RB = 1,
        LF = 2,
        LB = 3,
    };
    /* the ID of leg array */
    enum
    {
        R = 0,
        L = 1,
    };

    /* Kinematic solver, provide kinematic calculations and VMC matrix update
       function. */
    wheel_legged_kin_t _kinematic_solver;
   
   wl_power_ctrl_t _power_ctrl;

    /* INS Drv */
    ins_drv_t *_ins_drv;

    float yaw, pitch, roll;
    float g_yaw, g_pitch, g_roll;
   float a_x, a_y, a_z, a_forward, a_upward, a_upward_lpf;
   float gimbal_yaw, gimbal_g_yaw;

    pid_t* _yaw_pid;
    pid_t* _g_yaw_pid;
    float _yaw_ref;
    float _g_yaw_ref;

   /* PID controllers to control the bias between right leg angle and left 
      leg angle */
   pid_t* _delta_pid;
   pid_t* _d_delta_pid;
   float _delta_mea;
   float _d_delta_mea;
   float _d_delta_ref;

   pid_t* _roll_pid;
    /* LQR coefficients for the chassis control. 2 raw x 6 column, 12 values
       in total. Every value has 3 coefficients.*/
   /* coeffient in normal state */
   float _lqr_cof[48];
   /* coeffient in over step state */
   float _lqr_cof_over_step[48];
   

    /* DM joint motors driver, the order of array is: front-right, rear-right, 
       front-left, rear-left */
    dm_motor_drv_t *_motor_drv[4];

    /* DJI wheel motors driver, the order of array is: right wheel, left wheel*/
    dji_m3508_motor_drv_t *_wheel_drv[2];
    /* gimbal motor driver */
    dji_gm_6020_motor_drv_t *_yaw_motor_drv;
    /* Linear velocity of wheel equals angular velocity of wheel x wheel radius
       x reduction ratio */
    float _wheel_radius;
    float _reduction_ratio;
    float _yaw_offset;
    /* Offset angle to eliminate the bias of installation(rad), order is same
       as _motor_drv array */
    float _motor_offset[4];
    /* data of each leg */
    float _T_w_gain;
    float _x_gain;
   float T_l_gain; 
   float roll_gain;
    struct leg_data_t
    {
        /* Angle between big rod and direction of movement(rad) */
        float theta1, theta2;
        /* Differential of angle between big rod and direction of movement(rad/s) */
        float d_theta1, d_theta2;
        /* Angle between little rod and direction of movement(rad) */
        float phi1, phi2;
        /* Polar radius of j9(m) */
        float l;
        float ref_l;
        /* Differential of polar radius of j9(m/s) */
        float d_l;
        /* Second differential of polar radius of j9(m/s^2), calculated by data. */
        float d2_l;
        /* Reference differential of polar radius of j9(m/s), which calculated
           by pid controller. */
        float ref_d_l;
        /* Polar angle of j9(rad), clockwise is positive with forward direction
           as fixed side. */
        float alpha;
        /* Differential of polar angle of j9(rad/s) */
        float d_alpha;
        /* Reference differential of polar angle of j9(rad/s), which calculated
           by pid controller. */
        float ref_d_alpha;
        /* The angle between the vertical direction(towards ground) and the 
           leg(rad), it equal pi/2 - alpha - pitch angle of chassis */
        float beta;
        /* Differential of the angle between the vertical direction and the leg(rad/s) */
        float d_beta;
        /* Second differential of the angle between the vertical direction and 
           the leg(rad/s^2), calculated by data. */
        float d2_beta;
        float gamma;
        float d_gamma;
        /* Displacement distance of a j9(m) */
        float x;
        /* Velocity of the displacement of j9(m/s) */
        float dx;
         /* Velocity of rotation of each wheel(rad/s) */
        float w;
        /* VMC transfotm matrix */
        arm_matrix_instance_f32 T_mat;
        /* VMC transform matrix value, the order is [T11, T12, T21, T22] */
        float T_mat_val[4];
        /* Target torque of motors, front is 0, rear is 1 */
        float T[2];
        /* VMC output force and torque, force is 0, torque is 1*/
        float F[2];
        /* Target wheel torque */
        float T_w;
        float T_w_balance;
        float T_w_move;
        float T_w_turn;
        float T_w_real;
        float T_w_out;
        /* LQR gain for the leg, which is calculated by leg length, 2 x 6 matrix */
        float lqr_gain[12];
        float x_gain;
        float d_x_gain;
        /* Support force of the leg in vertical direction */
        float P;
        float jx,jy;
        float d_jx, d_jy;

        /* Kalman filter output for velocity, acceleration, and angular velocity */
        float kf_v;
        float kf_a;
        float kf_w;
        /* position which is integrated from kf_v */
        float kf_x;
        float predict_power;
    } _leg_data[2];
   /* Kalman filter for the wheel velocity */
   kf_t _wheel_kf[2];
    pid_t *_T_pid[2];
    pid_t *_d_T_pid[2];
    pid_t *_F_pid[2];
    pid_t *_d_F_pid[2];

    /* CAN bus configuration for motors, the order is same as _motor_drv array */
    struct
    {
        uint16_t solver_error;
    }_cnt;
   struct
   {
      uint8_t is_aerial = 0;
      uint8_t aerial_cnt = 0;
      float test = 0;
   }_flag;

   struct
   {
      uint8_t normal = 0;
      uint8_t ready = 0;
      uint8_t test = 0;
      uint8_t reverse = 0;
      uint8_t over_step = 0;
      uint8_t control = 0;
   }_active_mode_flag;

    class fsm_active_t : public fsm_t<wl_chassis_t>
    {
    public:
        class state_test_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        }_state_test;

        class state_ready_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_target_value(wl_chassis_t *owner);
        }_state_ready;
        class state_normal_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_support_force(wl_chassis_t *owner);
        }_state_normal;
        class state_reverse_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        }_state_reverse;
        class state_over_step_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
            void calc_target_value(wl_chassis_t *owner);
        }_state_over_step;

        class state_over_step_ready_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        }_state_over_step_ready;
        class state_control_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        }_state_control;

        class state_spin_t : public state_t<wl_chassis_t>
        {
            void enter(wl_chassis_t *owner) override;
            void execute(wl_chassis_t *owner) override;
            void exit(wl_chassis_t *owner) override;
        }_state_spin;

        void on_enter(wl_chassis_t *owner) override;
        void on_execute(wl_chassis_t *owner) override;
        void on_exit(wl_chassis_t *owner) override;
    }_state_active;

    class state_passive_t : public state_t<wl_chassis_t>
    {
    public:
        void enter(wl_chassis_t *owner) override;
        void execute(wl_chassis_t *owner) override;
        void exit(wl_chassis_t *owner) override;
    }_state_passive;

    friend class fsm_active_t;
    friend class state_passive_t;
    fsm_t<wl_chassis_t> _fsm;
    wl_cmd_t *_cmd;
};

}
#endif // __PYRO_WL_CHASSIS_H__