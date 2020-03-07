#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"

void odrive_can_init(uint8_t axis)
{
    HAL_StatusTypeDef status = HAL_CAN_Start(&hcan1);

    if (status != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }
}


void odrive_can_send(uint8_t axis)
{
    CAN_TxHeaderTypeDef HeaderGetVBus;

    HeaderGetVBus.StdId = AXIS0_NODE_ID + 0x17; //MSG_GET_VBUS_VOLTAGE;
    HeaderGetVBus.IDE = CAN_ID_STD;
    HeaderGetVBus.RTR = CAN_RTR_REMOTE;
    HeaderGetVBus.DLC = 0;

    /* Request transmission */
    uint8_t empty[0]; //declare byte to be transmitted //declare a receive byte
    uint8_t rData[8];
    uint32_t TxMailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &HeaderGetVBus, empty, &TxMailbox);

    if (status != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }

    /* Wait transmission complete */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3)
    {
    }
}