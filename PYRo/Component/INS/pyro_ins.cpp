#include "pyro_ins.h"
#include "pyro_dwt_drv.h"
#include "QuaternionEKF.h"
using namespace pyro;

#define X            0
#define Y            1
#define Z            2

#define IMU_DIRECT_1 1
#define IMU_DIRECT_2 2
#define IMU_DIRECT_3 3
#define IMU_DIRECT_4 4
#define IMU_DIRECT_5 5
#define IMU_DIRECT_6 6
#define IMU_DIRECT_7 7
#define IMU_DIRECT_8 8

#if ROBOT_ID == TEST_ROBOT_ID
#define IMU_DIRECT IMU_DIRECT_8
#elif ROBOT_ID == HERO_ID
#define IMU_DIRECT IMU_DIRECT_4
#elif ROBOT_ID == SUB_HERO_ID
#define IMU_DIRECT IMU_DIRECT_1
#elif ROBOT_ID == ENGINEER_ID
#define IMU_DIRECT IMU_DIRECT_1
#elif ROBOT_ID == SUB_ENGINEER_ID
#define IMU_DIRECT IMU_DIRECT_1
#elif ROBOT_ID == INFANTRY1_ID
#define IMU_DIRECT IMU_DIRECT_3
#elif ROBOT_ID == INFANTRY2_ID
#define IMU_DIRECT IMU_DIRECT_1
#elif ROBOT_ID == SUB_INFANTRY_ID
#define IMU_DIRECT IMU_DIRECT_1
#elif ROBOT_ID == SENTRY_ID
#define IMU_DIRECT IMU_DIRECT_2
#elif ROBOT_ID == SUB_SENTRY_ID
#define IMU_DIRECT IMU_DIRECT_1
#elif ROBOT_ID == UAV_ID
#define IMU_DIRECT IMU_DIRECT_1
#elif ROBOT_ID == DARTS_ID
#define IMU_DIRECT IMU_DIRECT_1
#endif

#if IMU_DIRECT == IMU_DIRECT_1
#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2
#define X_DIR 0 // 0 means positive, 1 means negative
#define Y_DIR 0
#define Z_DIR 0
#elif IMU_DIRECT == IMU_DIRECT_2
#define IMU_X 1
#define IMU_Y 0
#define IMU_Z 2
#define X_DIR 0 // 0 means positive, 1 means negative
#define Y_DIR 1
#define Z_DIR 0
#elif IMU_DIRECT == IMU_DIRECT_3
#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2
#define X_DIR 1 // 0 means positive, 1 means negative
#define Y_DIR 1
#define Z_DIR 0
#elif IMU_DIRECT == IMU_DIRECT_4
#define IMU_X 1
#define IMU_Y 0
#define IMU_Z 2
#define X_DIR 1 // 0 means positive, 1 means negative
#define Y_DIR 0
#define Z_DIR 0
#elif IMU_DIRECT == IMU_DIRECT_5
#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2
#define X_DIR 0 // 0 means positive, 1 means negative
#define Y_DIR 1
#define Z_DIR 1
#elif IMU_DIRECT == IMU_DIRECT_6
#define IMU_X 1
#define IMU_Y 0
#define IMU_Z 2
#define X_DIR 1 // 0 means positive, 1 means negative
#define Y_DIR 1
#define Z_DIR 1
#elif IMU_DIRECT == IMU_DIRECT_7
#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2
#define X_DIR 1 // 0 means positive, 1 means negative
#define Y_DIR 0
#define Z_DIR 1
#elif IMU_DIRECT == IMU_DIRECT_8
#define IMU_X 1
#define IMU_Y 0
#define IMU_Z 2
#define X_DIR 0 // 0 means positive, 1 means negative
#define Y_DIR 0
#define Z_DIR 1

#endif

IMU_Data_t *cimu_data;

// Define static TaskHandle_t declared in pyro::ins_drv_t
TaskHandle_t pyro::ins_drv_t::_ins_task_handle = nullptr;

ins_drv_t *ins_drv_t::get_instance(void)
{
    static ins_drv_t instance;
    return &instance;
}

status_t ins_drv_t::init()
{
    cimu_data = &imu_data;
    for (uint16_t count = 0; BMI088_init(&hspi2, IMU_CALIBRATION_EN,
                                         &imu_data) != BMI088_NO_ERROR &&
                             count < 10;
         count++)
    {
        if (count >= 255)
        {
            return PYRO_ERROR;
        }
    }

    if (_ins_task_handle == nullptr)
    {
        xTaskCreate(__static_ins_task, "ins_task", 512, this, 2,
                    &_ins_task_handle);
    }
    else
    {
        return PYRO_ERROR;
    }
    if (IMU_CALIBRATION_EN)
    {
        return PYRO_WARNING;
    }
    return PYRO_OK;
}

void ins_drv_t::__static_ins_task(void *argument)
{
    ((ins_drv_t *)argument)->__ins_task();
}

void ins_drv_t::__ins_task()
{
    _dwt_cnt = 0;
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 0.9996, 0.001);
    _gravity_n[0] = 0.0f;
    _gravity_n[1] = 0.0f;
    _gravity_n[2] = imu_data.gNorm;
    while (1)
    {
        _dt = dwt_drv_t::get_delta_t(&_dwt_cnt);
        _t += _dt;
        BMI088_Read(&imu_data);
        if (X_DIR)
        {
            _acc_b[X]  = -imu_data.Accel[IMU_X];
            _gyro_b[X] = -imu_data.Gyro[IMU_X];
        }
        else
        {
            _acc_b[X]  = imu_data.Accel[IMU_X];
            _gyro_b[X] = imu_data.Gyro[IMU_X];
        }
        if (Y_DIR)
        {
            _acc_b[Y]  = -imu_data.Accel[IMU_Y];
            _gyro_b[Y] = -imu_data.Gyro[IMU_Y];
        }
        else
        {
            _acc_b[Y]  = imu_data.Accel[IMU_Y];
            _gyro_b[Y] = imu_data.Gyro[IMU_Y];
        }
        if (Z_DIR)
        {
            _acc_b[Z]  = -imu_data.Accel[IMU_Z];
            _gyro_b[Z] = -imu_data.Gyro[IMU_Z];
        }
        else
        {
            _acc_b[Z]  = imu_data.Accel[IMU_Z];
            _gyro_b[Z] = imu_data.Gyro[IMU_Z];
        }


        /* Calculate the angle in body coordinate. */
        IMU_QuaternionEKF_Update(_gyro_b[X], _gyro_b[Y], _gyro_b[Z], _acc_b[X],
                                 _acc_b[Y], _acc_b[Z], _dt);
        memcpy(_q, QEKF_INS.q, sizeof(QEKF_INS.q));


        /* TBD, transform the angles to navigation coordinate */
        _angle_n[X] = QEKF_INS.Roll;
        _angle_n[Y] = QEKF_INS.Pitch;
        _angle_n[Z] = QEKF_INS.Yaw;

        /* Transform the gravity from navigation coordinate to body coordinate */
        __transform_n2b(_gravity_n, _gravity_b, _q);
        
        /* Eliminate the gravity component from the accelerometer readings */
        _acc_without_g_b[X] = _acc_b[X] - _gravity_b[X];
        _acc_without_g_b[Y] = _acc_b[Y] - _gravity_b[Y];
        _acc_without_g_b[Z] = _acc_b[Z] - _gravity_b[Z];

        /* Transform the accelerometer readings to navigation coordinate */
        __transform_b2n(_acc_b, _acc_n, _q);
        __transform_b2n(_acc_without_g_b, _acc_without_g_n, _q);

        vTaskDelay(1);
    }
}

status_t ins_drv_t::get_angles_b(float *yaw, float *pitch, float *roll)
{
    if (roll == nullptr || pitch == nullptr || yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *roll  = _angle_n[X];
    *pitch = _angle_n[Y];
    *yaw   = _angle_n[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_angles_n(float *yaw, float *pitch, float *roll)
{
    if (roll == nullptr || pitch == nullptr || yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *roll  = _angle_n[X];
    *pitch = _angle_n[Y];
    *yaw   = _angle_n[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_rads_b(float *rad_yaw, float *rad_pitch,
                               float *rad_roll)
{
    if (rad_roll == nullptr || rad_pitch == nullptr || rad_yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *rad_roll  = _angle_n[X] * PI / 180.0f;
    *rad_pitch = _angle_n[Y] * PI / 180.0f;
    *rad_yaw   = _angle_n[Z] * PI / 180.0f;
    return PYRO_OK;
}

status_t ins_drv_t::get_rads_n(float *rad_yaw, float *rad_pitch,
                               float *rad_roll)
{
    if (rad_roll == nullptr || rad_pitch == nullptr || rad_yaw == nullptr)
    {
        return PYRO_ERROR;
    }
    *rad_roll  = _angle_n[X] * PI / 180.0f;
    *rad_pitch = _angle_n[Y] * PI / 180.0f;
    *rad_yaw   = _angle_n[Z] * PI / 180.0f;
    return PYRO_OK;
}

status_t ins_drv_t::get_gyro_b(float *g_yaw, float *g_pitch, float *g_roll)
{
    if (g_yaw == nullptr || g_pitch == nullptr || g_roll == nullptr)
    {
        return PYRO_ERROR;
    }
    *g_roll  = _gyro_b[X];
    *g_pitch = _gyro_b[Y];
    *g_yaw   = _gyro_b[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_gyro_n(float *g_yaw, float *g_pitch, float *g_roll)
{
    if (g_yaw == nullptr || g_pitch == nullptr || g_roll == nullptr)
    {
        return PYRO_ERROR;
    }
    *g_roll  = _gyro_b[X];
    *g_pitch = _gyro_b[Y];
    *g_yaw   = _gyro_b[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_acc_b(float *a_x, float *a_y, float *a_z)
{
    if (a_x == nullptr || a_y == nullptr || a_z == nullptr)
    {
        return PYRO_ERROR;
    }
    *a_x = _acc_b[X];
    *a_y = _acc_b[Y];
    *a_z = _acc_b[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_acc_n(float *a_x, float *a_y, float *a_z)
{
    if (a_x == nullptr || a_y == nullptr || a_z == nullptr)
    {
        return PYRO_ERROR;
    }
    *a_x = _acc_n[X];
    *a_y = _acc_n[Y];
    *a_z = _acc_n[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_acc_without_g_b(float *a_x, float *a_y,
                                               float *a_z)
{
    if (a_x == nullptr || a_y == nullptr || a_z == nullptr)
    {
        return PYRO_ERROR;
    }
    *a_x = _acc_without_g_b[X];
    *a_y = _acc_without_g_b[Y];
    *a_z = _acc_without_g_b[Z];
    return PYRO_OK;
}

status_t ins_drv_t::get_acc_without_g_n(float *a_x, float *a_y,
                                               float *a_z)
{
    if (a_x == nullptr || a_y == nullptr || a_z == nullptr)
    {
        return PYRO_ERROR;
    }
    *a_x = _acc_without_g_n[X];
    *a_y = _acc_without_g_n[Y];
    *a_z = _acc_without_g_n[Z];
    return PYRO_OK;
}

status_t ins_drv_t::__transform_b2n(float *v_b, float *v_n, float *b2n_q)
{
    if(v_b == nullptr || v_n == nullptr || b2n_q == nullptr)
    {
        return PYRO_ERROR;
    }
    /* QEKF_INS.q is b2n quaternion, R(q) = R_b2n, use q directly */
    float *q = b2n_q;
    v_n[0] = (1 - 2 * (q[2] * q[2] + q[3] * q[3])) * v_b[0] 
           + (2 * (q[1] * q[2] - q[0] * q[3]))     * v_b[1] 
           + (2 * (q[1] * q[3] + q[0] * q[2]))     * v_b[2];

    v_n[1] = (2 * (q[1] * q[2] + q[0] * q[3]))     * v_b[0]
           + (1 - 2 * (q[1] * q[1] + q[3] * q[3])) * v_b[1]
           + (2 * (q[2] * q[3] - q[0] * q[1]))     * v_b[2];

    v_n[2] = (2 * (q[1] * q[3] - q[0] * q[2]))     * v_b[0]
           + (2 * (q[2] * q[3] + q[0] * q[1]))     * v_b[1]
           + (1 - 2 * (q[1] * q[1] + q[2] * q[2])) * v_b[2];
    return PYRO_OK;
}

status_t ins_drv_t::__transform_n2b(float *v_n, float *v_b, float *b2n_q)
{
    if(v_n == nullptr || v_b == nullptr || b2n_q == nullptr)
    {
        return PYRO_ERROR;
    }
    /* QEKF_INS.q is b2n quaternion, R(q)^T = R(q*) = R_n2b, conjugate q */
    float q[4];
    q[0] = b2n_q[0];
    q[1] = -b2n_q[1];
    q[2] = -b2n_q[2];
    q[3] = -b2n_q[3];
    v_b[0] = (1 - 2 * (q[2] * q[2] + q[3] * q[3])) * v_n[0] 
           + (2 * (q[1] * q[2] - q[0] * q[3]))     * v_n[1] 
           + (2 * (q[1] * q[3] + q[0] * q[2]))     * v_n[2];

    v_b[1] = (2 * (q[1] * q[2] + q[0] * q[3]))     * v_n[0]
           + (1 - 2 * (q[1] * q[1] + q[3] * q[3])) * v_n[1]
           + (2 * (q[2] * q[3] - q[0] * q[1]))     * v_n[2];

    v_b[2] = (2 * (q[1] * q[3] - q[0] * q[2]))     * v_n[0]
           + (2 * (q[2] * q[3] + q[0] * q[1]))     * v_n[1]
           + (1 - 2 * (q[1] * q[1] + q[2] * q[2])) * v_n[2];
    return PYRO_OK;
}