#include "application.h"
#include "stm32f4xx_hal.h"

#define APP_ADDR 0x8008000U

int app_is_valid(void)
{
    uint32_t app_sp = *(uint32_t*)APP_ADDR;
    if (app_sp >= 0x20000000 && app_sp <= 0x20020000)
        return 1;
    else
        return 0;
}

void jump_to_app(void)
{
    uint32_t app_sp  = *(uint32_t*)APP_ADDR;
    uint32_t app_rst = *(uint32_t*)(APP_ADDR + 4);

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    HAL_RCC_DeInit();
    HAL_DeInit();

    SCB->VTOR = APP_ADDR;
    __set_MSP(app_sp);
    __enable_irq();
    void (*app_reset_handler)(void) = (void (*)(void))app_rst;
    app_reset_handler();
}
