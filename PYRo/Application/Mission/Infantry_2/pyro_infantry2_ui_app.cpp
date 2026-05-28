#include "pyro_wl_chassis.h"
#include "pyro_supercap_drv.h"  
#include "pyro_uart_drv.h"      
#include "pyro_core_config.h"   
#include "ui-renderer-srvc.h"
#include "referee-hud-ui.h"
#include "pyro_infantry2_chassis_intf.h"
#include"usart.h"


using namespace pyro;

namespace pyro {
extern wl_chassis_t *infantry2_chassis_ptr;
extern GimbalToChassisComm gimbal_rx;
extern ChassisToGimbalComm gimbal_tx;
extern cmd_t cmd, last_cmd;
status_t ui_tread_init(void* argument);

__attribute__((section(".dma_heap"))) static uint8_t refereeUiTxBuffer[512]={0};


static bool transmitRefereeUiPacket(const uint8_t* data, uint16_t length, void* context) {
    (void)context;
    
    auto uart = uart_drv_t::get_instance(static_cast<pyro::uart_drv_t::which_uart>(REFEREE_UART));
    
    if (uart) {
// HAL_UART_Transmit(&huart10, data, length, 100);
        return uart->write(const_cast<uint8_t*>(data), length) == pyro::status_t::PYRO_OK;
    }
    return false;
}


static UiRendererSrvc renderer({
    transmitRefereeUiPacket, nullptr,3  , 35, refereeUiTxBuffer, sizeof(refereeUiTxBuffer)
});

// referee_drv_t::get_instance()->get_robot_id()
static RefereeHudUi hudUi;

void refereeUiRendererTask(void *argument) {
    if (!renderer.init()) {
        vTaskDelete(NULL);
    }
    for (;;) {
        renderer.run();
        vTaskDelay(pdMS_TO_TICKS(35));
    }
}


void hudProducerTask(void *argument) {
    hudUi.reset(renderer);

    for (;;) {
        RefereeHudInput input {};

        input.capVoltage = pyro::supercap_drv_t::get_instance()->get_feedback().vot_cap/ 100.0f; 
        input.capEnabled = true;
        input.feederEnabled = gimbal_rx.msg.shootEn;

       
        input.spinEnabled = (cmd.mode == cmd_t::SPIN);             
        input.stepClimbEnabled = (cmd.mode == cmd_t::STEP_CLIMB);  
        input.aimModeState = gimbal_rx.msg.aimMode;
    //  input.aimTargetState =gimbal_rx.msg.;
        static uint8_t last_reset = 0; 
        
        if (last_reset == 0 && gimbal_rx.msg.resetUI == 1) {
            hudUi.reset(renderer); 
        }
        
        last_reset = gimbal_rx.msg.resetUI;

 
        float r_angle, l_angle;
        float r_leg, l_leg;
        infantry2_chassis_ptr->get_cur_angle(&r_angle, &l_angle);
        infantry2_chassis_ptr->get_cur_length(&r_leg, &l_leg);
        
  
        input.leftLegThighAngleDeg =90.0f -l_angle * 180.0f / PI;
        input.rightLegThighAngleDeg =90.0f- r_angle * 180.0f / PI;
        
        input.leftLegHipWheelDistance = l_leg / 0.21f;
        input.rightLegHipWheelDistance = r_leg / 0.21f;

       
        hudUi.draw(renderer, input);
        
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}


status_t ui_tread_init(void* argument)
{
    BaseType_t ret;
    ret = xTaskCreate(refereeUiRendererTask, "ui_renderer", 1024, nullptr, 1, nullptr);
    CHECK_OS_RET(ret)
    ret = xTaskCreate(hudProducerTask,       "ui_producer", 1024, nullptr, 1, nullptr);
    CHECK_OS_RET(ret)
    return status_t::PYRO_OK;
}

}