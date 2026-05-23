/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-26 20:18:33
 * @LastEditors: vod-x vod_x@outlook.com
 * @LastEditTime: 2026-05-23 07:53:58
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#include "pyro_wl_chassis.h"
#include "pyro_rc_hub.h"
#include "pyro_ins.h"
#include "pyro_algo_common.h"
#include "pyro_com_canrx.h"
#include "pyro_com_cantx.h"
#include "pyro_referee.h"

namespace pyro
{

const float control_acc = 0.006f;
const float control_max_velocity = 1.0f;
const float control_leg_length[3] = {0.18f, 0.27f, 0.33f};

extern referee_drv_t *referee_drv;
extern can_drv_t *can3_drv;
wl_chassis_t *infantry2_chassis_ptr = nullptr;
wl_cmd_t     *infantry2_chassis_cmd_ptr = nullptr;
wl_cmd_t     *last_infantry2_chassis_cmd_ptr = nullptr;
dr16_drv_t::dr16_ctrl_t const *infantry2_rc_ctrl_ptr = nullptr;
}

using namespace pyro;
#define USE_GIMBAL_COM
// #define USE_DR16
#if defined(USE_GIMBAL_COM) && defined(USE_DR16)
#error "Gimbal COM and DR16 cannot be used at the same time"   
#endif

extern wl_chassis_cfg_t infantry2_chassis_cfg;

union GimbalToChassisComm {

    __attribute__((packed)) struct {
        int32_t vx    : 6; //  正方向： 向前
        int32_t vy    : 6; // 正方向： 向左
        uint32_t mode : 4;
        uint32_t shootEn  : 1;
        uint32_t resetUI  : 1;
        uint32_t fn1Switch: 1;
        uint32_t turboMode    : 1; // [R] 飞坡
        uint32_t stepClimb    : 1; // [E] 上台阶
        uint32_t legLength    : 2; // [Z] 腿长 (0/1/2)
        uint32_t selfRescue   : 1; // [G] 自救
        uint32_t manualRescue : 1; // [Ctrl] 手动自救
        uint32_t gimbalReverse: 1; // [X] 调头
        uint32_t jump         : 1; // [V] 跳跃
        uint32_t capSwitch    : 1; // [C] 超级电容开关
        uint32_t fireState    : 4; // 发射机构 FSM 状态 (FireState)
        uint32_t aimMode      : 2; // [B] 自瞄模式 (0~3)
        int16_t yawvel     ;
    } msg;

    std::array<uint8_t, 8> buffer;
}gimbal_rx;

union ChassisToGimbalComm {

    __attribute__((packed)) struct {
        // 将 float (4字节) 压缩为 uint16_t (2字节) 传初速度，乘以 100 发送，云台除以 100
        uint32_t initialSpeedX100      : 16; // 弹丸初速度 * 100 (2 Bytes)
        uint32_t shooter17mmBarrelHeat : 16; // 17mm 枪口当前热量 (2 Bytes)
        uint32_t heatLimit             : 9; // 热量上限 (如 150, 240, 360)
        uint32_t coolingRate           : 7; // 冷却速率 (如 40, 60, 80)
        uint8_t robotId;                    // 机器人 ID (1 Byte)
        int8_t chassisYawSpeed;
    } msg;

    std::array<uint8_t, 8> buffer;
}gimbal_tx;
struct cmd
{
    float vx;
    float vy;
    float turn_angle;
    float v;
    float yaw_vel;
    enum
    {
        PASSIVE = 0x00,
        ACTIVE = 0x01,
        SPIN = 0x02,
         STEP_CLIMB= 0x03
    }mode;
    uint8_t leg_length_mode;
}cmd, last_cmd;

extern "C"
{

void passive_mode(void const *rc_ctrl);
void test_mode(void const *rc_ctrl);
void ready_mode(void const *rc_ctrl);
void normal_mode(void const *rc_ctrl);
void reverse_mode(void const *rc_ctrl);
void over_step_mode(void const *rc_ctrl);
void over_step_ready_mode(void const *rc_ctrl);
void control_mode(void const *rc_ctrl);
void spin_mode(void const *rc_ctrl);
void infantry2_chassis_rc2cmd(void const *rc_ctrl)
{

#if defined(USE_GIMBAL_COM)
    can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can3, 0x100,gimbal_rx.buffer);
    memcpy(&last_cmd, &cmd, sizeof(cmd));
    if(0 != gimbal_rx.msg.vx)
    {
        if(gimbal_rx.msg.vx > 0)
        {
            cmd.vx += control_acc;
        }
        else
        {
            cmd.vx -= control_acc;
        }
    }
    else 
    {
        if(cmd.vx > control_acc)
        {
            cmd.vx -= control_acc;
        }
        else if(cmd.vx < -control_acc)
        {
            cmd.vx += control_acc;
        }

        if(fabsf(cmd.vx) <= control_acc)
        {
            cmd.vx = 0.0f;
        }
    }
    cmd.vx = fp32_constrain(cmd.vx, -control_max_velocity, control_max_velocity);
    cmd.vy = fp32_constrain(cmd.vy, -control_max_velocity, control_max_velocity);
    cmd.v = sqrtf(cmd.vx * cmd.vx + cmd.vy * cmd.vy);
    cmd.turn_angle = atan2f(cmd.vy, cmd.vx);
    
    cmd.yaw_vel = (float)gimbal_rx.msg.yawvel / 100000.0f;
    if(gimbal_rx.msg.mode == cmd::PASSIVE)
    {
        cmd.mode = cmd::PASSIVE;
    }
    else if(gimbal_rx.msg.mode == cmd::ACTIVE)
    {
        cmd.mode = cmd::ACTIVE;
        if(1 == gimbal_rx.msg.stepClimb)
        {
            cmd.mode = cmd::STEP_CLIMB;
        }
    }
    else if(gimbal_rx.msg.mode == cmd::SPIN) 
    {
        cmd.mode = cmd::SPIN;
    }
    cmd.leg_length_mode = gimbal_rx.msg.legLength;
#endif
#if defined(USE_DR16)
    pyro::read_scope_lock lock(
            pyro::rc_hub_t::get_instance(
         pyro::rc_hub_t::DR16)->get_lock());
   static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
#endif
#if defined (USE_GIMBAL_COM)
    // if(last_mode == pyro::cmd_base_t::mode_t::PASSIVE && 
    //     cmd.mode == cmd::ACTIVE)
    // {
    //     infantry2_chassis_ptr->get_cur_angle(&infantry2_chassis_cmd_ptr->r_angle,
    //                           &infantry2_chassis_cmd_ptr->l_angle);
    //     infantry2_chassis_ptr->get_cur_length(&infantry2_chassis_cmd_ptr->r_leg,
    //                         &infantry2_chassis_cmd_ptr->l_leg);
    // }
    if(cmd.mode == cmd::PASSIVE)
    {
        passive_mode(rc_ctrl);
    }
    else if(cmd.mode == cmd::ACTIVE)
    {
    
        infantry2_chassis_cmd_ptr->l_leg = control_leg_length[cmd.leg_length_mode];
        infantry2_chassis_cmd_ptr->r_leg = control_leg_length[cmd.leg_length_mode];
        // infantry2_chassis_cmd_ptr->l_leg = 0.25f;
        // infantry2_chassis_cmd_ptr->r_leg = 0.25f;
        infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        if(0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::READY))
        {
            ready_mode(rc_ctrl);
        }
      
        else
        {
            normal_mode(rc_ctrl);
        }
                // over_step_mode(rc_ctrl);
    }
    else if(cmd.mode == cmd::SPIN) 
       { 
        infantry2_chassis_cmd_ptr->l_leg = control_leg_length[cmd.leg_length_mode];
        infantry2_chassis_cmd_ptr->r_leg = control_leg_length[cmd.leg_length_mode];
        infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        
        
        if(0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::READY))
        {
            ready_mode(rc_ctrl);
        }
        else
        {
            spin_mode(rc_ctrl);
        }
    }
    else if(cmd.mode == cmd::STEP_CLIMB)
    {
        if(last_cmd.mode == cmd::ACTIVE)
        {
            static float temp_yaw;
            infantry2_chassis_ptr->get_cur_ins_yaw(&temp_yaw); 
            infantry2_chassis_cmd_ptr->yaw = temp_yaw;
        }
        infantry2_chassis_cmd_ptr->yaw += cmd.yaw_vel;
        infantry2_chassis_cmd_ptr->l_leg = 0.33f;
        infantry2_chassis_cmd_ptr->r_leg = 0.33f;
        constexpr float TEST_FORCE = -22.0f;
        constexpr float TEST_ANGLE = 2.0f;
        constexpr float TEST_d_ANGLE = 0.1f;
        static uint8_t over_step_flag = 0;
        static float temp_torque[2] = {0.0f, 0.0f};
        static float temp_angle[2] = {0.0f, 0.0f};
        static float temp_d_angle[2] = {0.0f, 0.0f};
        infantry2_chassis_ptr->get_cur_angle(&temp_angle[0], &temp_angle[1]);
        infantry2_chassis_ptr->get_cur_d_angle(&temp_d_angle[0], &temp_d_angle[1]);
        infantry2_chassis_ptr->get_cur_p_torque(&temp_torque[0],
                                    &temp_torque[1]);
        // if((0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP)) &&
        //     ((temp_torque[0] < TEST_FORCE) && (abs(temp_d_angle[0]) < TEST_d_ANGLE) && (temp_torque[1] < TEST_FORCE) && (abs(temp_d_angle[1]) < TEST_d_ANGLE)))
        // if((0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP)) &&
        //     ((temp_angle[0] > TEST_ANGLE) || (temp_angle[1] > TEST_ANGLE)))
        if((0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP)) &&
            ((temp_torque[0] < TEST_FORCE) || (temp_torque[1] < TEST_FORCE)))
        {
            over_step_flag = 1;
        }
        if(0 == over_step_flag)
        {
            over_step_ready_mode(rc_ctrl);
        }
        else 
        {
            if(0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP))
            {
                over_step_mode(rc_ctrl);
            }
            else 
            {
                ready_mode(rc_ctrl);
            }
            
            if(1 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::READY))
            {
                over_step_flag = 0;
                infantry2_chassis_ptr->clear_status_flag(wl_cmd_t::OVER_STEP);
            }
        }
    }
#endif

#if defined(USE_DR16) 
    if( infantry2_chassis_cmd_ptr->last_active_mode != wl_cmd_t::CONTROL && 
        p_ctrl->rc.s_r.state == pyro::dr16_drv_t::sw_state_t::SW_DOWN)
    {
        infantry2_chassis_ptr->get_cur_angle(&infantry2_chassis_cmd_ptr->r_angle,
                              &infantry2_chassis_cmd_ptr->l_angle);
        infantry2_chassis_ptr->get_cur_length(&infantry2_chassis_cmd_ptr->r_leg,
                            &infantry2_chassis_cmd_ptr->l_leg);
        
    }
    switch (p_ctrl->rc.s_r.state) 
    {
        case dr16_drv_t::sw_state_t::SW_UP:
            passive_mode(rc_ctrl);
            break;
        case dr16_drv_t::sw_state_t::SW_DOWN:
            infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
            if(p_ctrl->rc.s_l.state == dr16_drv_t::sw_state_t::SW_UP)
            {
                test_mode(rc_ctrl);
            }
            else
            {
                control_mode(rc_ctrl);
            }
            break;
        case dr16_drv_t::sw_state_t::SW_MID:
            infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
            if(p_ctrl->rc.s_l.state == dr16_drv_t::sw_state_t::SW_UP)
            {
                ready_mode(rc_ctrl);
            }
            else if(p_ctrl->rc.s_l.state == dr16_drv_t::sw_state_t::SW_MID)
            {

                if(0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::READY))
                {
                    ready_mode(rc_ctrl);
                }
                else
                {
                    normal_mode(rc_ctrl);
                }
            }
            else
            {
                constexpr float TEST_FORCE = -15.0f;
                static uint8_t over_step_flag = 0;
                static float temp_torque[2] = {0.0f, 0.0f};
                infantry2_chassis_ptr->get_cur_p_torque(&temp_torque[0],
                                            &temp_torque[1]);
                if((0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP)) &&
                    ((temp_torque[0] < TEST_FORCE) || (temp_torque[1] < TEST_FORCE)))
                {
                    over_step_flag = 1;

                }

                if(0 == over_step_flag)
                {
                    over_step_ready_mode(rc_ctrl);
                }
                else 
                {
                    if(0 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::OVER_STEP))
                    {
                        over_step_mode(rc_ctrl);
                    }
                    else 
                    {
                        ready_mode(rc_ctrl);
                    }
                    
                    if(1 == infantry2_chassis_ptr->get_status_flag(wl_cmd_t::READY))
                    {
                        over_step_flag = 0;
                        infantry2_chassis_ptr->clear_status_flag(wl_cmd_t::OVER_STEP);
                    }
                }
            }
            break;
        default:
            break;
    
    }
#endif

    infantry2_chassis_cmd_ptr->last_active_mode = infantry2_chassis_cmd_ptr->active_mode;
}
void infantry2_chassis_main_tread(void *argument)
{
    status_t ret = infantry2_chassis_ptr->start();
    while(1)
    {
        gimbal_tx.msg.initialSpeedX100 = (uint16_t)(referee_drv->get_data().shoot.initial_speed* 100.0f);
        gimbal_tx.msg.shooter17mmBarrelHeat = referee_drv->get_data().power_heat.shooter_17mm_barrel_heat;
        gimbal_tx.msg.heatLimit = referee_drv->get_data().robot_status.shooter_barrel_heat_limit;
        gimbal_tx.msg.coolingRate = referee_drv->get_data().robot_status.shooter_barrel_cooling_value;
        gimbal_tx.msg.robotId = referee_drv->get_robot_id();
        gimbal_tx.msg.chassisYawSpeed = (int8_t)(infantry2_chassis_cmd_ptr->yaw * 100.0f);
        can_tx_drv_t::instance()->clear(0x101);
        can_tx_drv_t::instance()->add_data_raw(0x101, 64, &gimbal_tx);
        can_tx_drv_t::instance()->send(0x101, can3_drv);
        infantry2_chassis_rc2cmd(infantry2_rc_ctrl_ptr);
        infantry2_chassis_ptr->set_command(*infantry2_chassis_cmd_ptr);
        vTaskDelay(1);
    }
}

status_t infantry2_chassis_init(void *argument)
{

    can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can3, 0x100);
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

void passive_mode(void const *rc_ctrl)
{
    infantry2_chassis_cmd_ptr->l_angle = PI/2.0f;
    infantry2_chassis_cmd_ptr->r_angle = PI/2.0f;
    infantry2_chassis_cmd_ptr->l_leg = 0;
    infantry2_chassis_cmd_ptr->r_leg = 0;
    infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
    float yaw, pitch, roll;
    ins_drv_t::get_instance()->get_rads_b(&yaw, &pitch, &roll);
    infantry2_chassis_cmd_ptr->yaw = yaw;

}

void test_mode(void const *rc_ctrl)
{
   static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::TEST;
}
void spin_mode(void const *rc_ctrl)
{
    static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
    
    
    infantry2_chassis_cmd_ptr->l_leg = control_leg_length[cmd.leg_length_mode];
    infantry2_chassis_cmd_ptr->r_leg = control_leg_length[cmd.leg_length_mode];

   
    infantry2_chassis_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
    
  
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::SPIN;
}
void ready_mode(void const *rc_ctrl)
{
   static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
    infantry2_chassis_cmd_ptr->l_angle = PI/2.0f;
    infantry2_chassis_cmd_ptr->r_angle = PI/2.0f;
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::READY;
}
void normal_mode(void const *rc_ctrl)
{
   static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::NORMAL;

#if defined (USE_GIMBAL_COM)
    infantry2_chassis_cmd_ptr->vx = cmd.vx;
#endif
#if defined (USE_DR16)
    infantry2_chassis_cmd_ptr->r_leg += (p_ctrl->rc.ch_ry / 2000.0f);
    infantry2_chassis_cmd_ptr->l_leg += (p_ctrl->rc.ch_ry / 2000.0f);

    infantry2_chassis_cmd_ptr->yaw -= (p_ctrl->rc.ch_lx * PI / 500.0f);
    infantry2_chassis_cmd_ptr->vx = (p_ctrl->rc.ch_ly * 2.0f);
    infantry2_chassis_cmd_ptr->yaw = loop_fp32_constrain(
        infantry2_chassis_cmd_ptr->yaw, -PI, PI);
#endif
}
void reverse_mode(void const *rc_ctrl)
{
   static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::REVERSE;
}
void control_mode(void const *rc_ctrl)
{

   static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
    
    
    infantry2_chassis_cmd_ptr->r_angle += (p_ctrl->rc.ch_rx * PI / 2000.0f);
    infantry2_chassis_cmd_ptr->l_angle += (p_ctrl->rc.ch_lx * PI / 2000.0f);
    infantry2_chassis_cmd_ptr->r_angle = loop_fp32_constrain(
        infantry2_chassis_cmd_ptr->r_angle, -PI, PI);
    infantry2_chassis_cmd_ptr->l_angle = loop_fp32_constrain(
        infantry2_chassis_cmd_ptr->l_angle, -PI, PI);

    infantry2_chassis_cmd_ptr->r_leg += (p_ctrl->rc.ch_ry / 2000.0f);
    infantry2_chassis_cmd_ptr->l_leg += (p_ctrl->rc.ch_ly / 2000.0f);
    infantry2_chassis_cmd_ptr->r_leg = fp32_constrain(
       infantry2_chassis_cmd_ptr->r_leg, 0.13f, 0.37f);
    infantry2_chassis_cmd_ptr->l_leg = fp32_constrain(
       infantry2_chassis_cmd_ptr->l_leg, 0.13f, 0.37f);
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::CONTROL;
}
void over_step_mode(void const *rc_ctrl)
{
   static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::OVER_STEP;
}
void over_step_ready_mode(void const *rc_ctrl)
{
   static auto *p_ctrl =
            static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);  
    infantry2_chassis_cmd_ptr->active_mode = wl_cmd_t::OVER_STEP_READY;

#if defined (USE_GIMBAL_COM)
    infantry2_chassis_cmd_ptr->vx = cmd.vx;
#endif
#if defined (USE_DR16)
    infantry2_chassis_cmd_ptr->r_leg += (p_ctrl->rc.ch_ry / 2000.0f);
    infantry2_chassis_cmd_ptr->l_leg += (p_ctrl->rc.ch_ry / 2000.0f);

    infantry2_chassis_cmd_ptr->yaw -= (p_ctrl->rc.ch_lx * PI / 500.0f);
    infantry2_chassis_cmd_ptr->vx = (p_ctrl->rc.ch_ly * 2.0f);
    infantry2_chassis_cmd_ptr->yaw = loop_fp32_constrain(
        infantry2_chassis_cmd_ptr->yaw, -PI, PI);
#endif
}

}
