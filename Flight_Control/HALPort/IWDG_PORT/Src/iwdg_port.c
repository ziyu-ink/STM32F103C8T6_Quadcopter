#include "iwdg_port.h"
#include "iwdg.h"

void IWDG_Port_Refresh(void)
{
    /* 喂狗：刷新独立看门狗计数器 */
    HAL_IWDG_Refresh(&hiwdg);
}
