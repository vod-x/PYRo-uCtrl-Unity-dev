/*
 * @Author: vod vod_x@outlook.com
 * @Date: 2026-02-28 15:55:32
 * @LastEditors: vod vod_x@outlook.com
 * @LastEditTime: 2026-03-03 14:21:10
 * @Description: 
 * 
 * Copyright (c) 2026 by PeiYangRobot, All Rights Reserved. 
 */
#ifndef PYRO_INS_H
#define PYRO_INS_H

#include "pyro_core_config.h"
#include "pyro_core_def.h"
#include "BMI088_driver.h"
#include "FreeRTOS.h"
#include "task.h"
namespace pyro {

class ins_drv_t {
private:
    IMU_Data_t imu_data;
    //quaternion=q0+i*q1+j*q2+k*q3
    float _q[4];
    float _gyro_b[3];
    float _gyro_n[3];
    float _acc_b[3];
    float _acc_n[3];
    float _acc_without_g_b[3];
    float _acc_without_g_n[3];
    float _angle_b[3];
    float _angle_n[3];
    float _gravity_b[3];
    float _gravity_n[3];

    float _dt;
    float _t;
    uint32_t _dwt_cnt;
    static TaskHandle_t _ins_task_handle;
    void __ins_task();
    static void __static_ins_task(void *argument);
    status_t __transform_b2n(float* v_b, float* v_n, float*b2n_q);
    status_t __transform_n2b(float* v_n, float* v_b, float*b2n_q); 
public:
    static ins_drv_t* get_instance(void);
    status_t init();
    status_t get_angles_b(float *yaw, float *pitch, float *roll);
    status_t get_angles_n(float *yaw, float *pitch, float *roll);
    status_t get_rads_b(float* rad_yaw, float* rad_pitch, float* rad_roll);
    status_t get_rads_n(float* rad_yaw, float* rad_pitch, float* rad_roll);
    status_t get_gyro_b(float* g_yaw, float* g_pitch, float* g_roll);
    status_t get_gyro_n(float* g_yaw, float* g_pitch, float* g_roll);
    status_t get_acc_b(float* a_x, float* a_y, float* a_z);
    status_t get_acc_n(float* a_x, float* a_y, float* a_z);
    status_t get_acc_without_g_b(float* a_x, float* a_y, float* a_z);
    status_t get_acc_without_g_n(float* a_x, float* a_y, float* a_z);


};
}
#endif
