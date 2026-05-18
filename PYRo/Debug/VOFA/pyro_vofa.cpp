#include "pyro_vofa.h"

#include "pyro_core_dma_heap.h"
#include "task.h"

#include "cstring"
#include "pyro_core_config.h"
#include "pyro_wl_chassis.h"
#include "pyro_powermeter.h"

namespace pyro
{
extern wl_chassis_t *infantry2_chassis_ptr;
powermeter_drv_t powermeter(0x212, can_hub_t::can2);
powermeter_data powermeter_data;
float power_bias[2];
float power_total;
float power_bias_total;

vofa_drv_t::vofa_drv_t(uint8_t max_length, uart_drv_t *uart)
{
    _data_pack = static_cast<float *>(pvPortDmaMalloc(4 * max_length));
    _length    = 0;
    _vofa_uart = uart;
}

vofa_drv_t::~vofa_drv_t()
{
    if (_data_pack)
    {
        delete[] _data_pack;
        _data_pack = nullptr;
    }
}

vofa_drv_t &vofa_drv_t::get_instance(uint8_t max_length)
{
    static vofa_drv_t instance(max_length, uart_drv_t::get_instance(static_cast<uart_drv_t::which_uart>(VOFA_DEBUG_PORT)));
    return instance;
}

void vofa_drv_t::init()
{
}

void vofa_drv_t::add_data(float *data)
{
    if (data)
    {
        data_node_t temp;
        temp.data = data;
        temp.size = 1;
        _length += temp.size;
        _data_nodes.push_back(temp);
    }
}

void vofa_drv_t::add_data(float *data, const uint8_t len)
{
    if (data)
    {
        data_node_t temp;
        temp.data = data;
        temp.size = len;
        _length += temp.size;
        _data_nodes.push_back(temp);
    }
}

void vofa_drv_t::remove_data(const float *data)
{
    for (auto it = _data_nodes.begin(); it != _data_nodes.end(); ++it)
    {
        if (it->data == data)
        {
            _length -= it->size;
            _data_nodes.erase(it);
            break;
        }
    }
}

void vofa_drv_t::update_data()
{
    static uint8_t frame_tail[4] = {0x00, 0x00, 0x80, 0x7F};
    uint8_t offset               = 0;
    for (const auto &[data, size] : _data_nodes)
    {
        for (uint8_t i = 0; i < size; ++i)
        {
            _data_pack[offset++] = data[i];
        }
    }
    _data_pack[offset] = *reinterpret_cast<float *>(frame_tail);
}

void vofa_drv_t::send()
{
    _vofa_uart->write(reinterpret_cast<uint8_t *>(_data_pack),
                      (_length + 1) * 4);
}


void vofa_drv_t::thread()
{
    powermeter.init();
    /* kalman filter */
    // add_data(&infantry2_chassis_ptr->_leg_data[0].kf_x);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].kf_x);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].x);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].x);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].kf_v);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].kf_v);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].dx);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].dx);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].kf_w);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].kf_w);
    // add_data(&infantry2_chassis_ptr->a_forward);
    // add_data(&infantry2_chassis_ptr->g_yaw);
    /* force pid */
    add_data(&infantry2_chassis_ptr->_leg_data[0].F[0]);
    add_data(&infantry2_chassis_ptr->_leg_data[0].F[1]);
    add_data(&infantry2_chassis_ptr->_leg_data[0].l);
    add_data(&infantry2_chassis_ptr->_leg_data[1].l);
    add_data(&infantry2_chassis_ptr->_leg_data[0].ref_l);
    add_data(&infantry2_chassis_ptr->_leg_data[1].ref_l);
    add_data(&infantry2_chassis_ptr->_leg_data[0].d_l);
    add_data(&infantry2_chassis_ptr->_leg_data[1].d_l);
    add_data(&infantry2_chassis_ptr->_leg_data[0].ref_d_l);
    add_data(&infantry2_chassis_ptr->_leg_data[1].ref_d_l);
    add_data(&infantry2_chassis_ptr->_leg_data[0].P);
    add_data(&infantry2_chassis_ptr->_leg_data[1].P);
    /* lqr data */
    // add_data(&infantry2_chassis_ptr->_leg_data[0].kf_x);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].kf_x);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].x);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].x);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].kf_v);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].kf_v);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].dx);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].dx);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].gamma);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].gamma);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].d_gamma);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].d_gamma);
    //  add_data(&infantry2_chassis_ptr->_leg_data[0].beta);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].beta);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].d_beta);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].d_beta);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].T_w);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].T_w);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].F[1]);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].F[1]);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].l);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].l);

    // vmc data
    // add_data(&infantry2_chassis_ptr->_leg_data[0].l);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].l);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].d_l);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].d_l);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].alpha);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].alpha);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].d_alpha);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].d_alpha);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].F[0]);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].F[0]);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].F[1]);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].F[1]);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].theta1);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].theta1);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].theta2);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].theta2);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].d_theta1);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].d_theta1);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].d_theta2);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].d_theta2);

    // power control
    // add_data(&powermeter_data.current);
    // add_data(&powermeter_data.voltage);
    // add_data(&powermeter_data.power);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].predict_power);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].predict_power);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].T_w);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].T_w);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].T_w_real);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].T_w_real);
    // add_data(&infantry2_chassis_ptr->_leg_data[0].w);
    // add_data(&infantry2_chassis_ptr->_leg_data[1].w);
    // add_data(&power_bias[0]);
    // add_data(&power_bias[1]);
    // add_data(&power_total);
    // add_data(&power_bias_total);
    // add_data(&infantry2_chassis_ptr->_power_ctrl.)
    while (true)
    {
        powermeter.get_data(powermeter_data);
        for(uint8_t i = 0; i < 2; i++)
        {
            power_bias[i] = powermeter_data.power - infantry2_chassis_ptr->_leg_data[i].predict_power;
        }
        // power_total = infantry2_chassis_ptr->_leg_data[0].predict_power + infantry2_chassis_ptr->_leg_data[1].predict_power;
        power_total = infantry2_chassis_ptr->_power_ctrl.get_cmd_power();
        power_bias_total = powermeter_data.power - power_total;
        
        update_data();
        send();
        vTaskDelay(10);
    }
}

} // namespace pyro

extern "C" void pyro_vofa_task(void *arg)
{
    pyro::vofa_drv_t &vofa = pyro::vofa_drv_t::get_instance(20);
    vofa.thread();
}