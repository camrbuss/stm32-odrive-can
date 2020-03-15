#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "odrive_can.h"
#include "freertos_vars.h"
#include "cmsis_os.h"

uint8_t odrive_can_init(Axis_t axis)
{
    CAN_FilterTypeDef filter;
    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (status != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
        return 1;
    }

    status = HAL_CAN_Start(&hcan1);

    if (status != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
        return 1;
    }

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    return 0;
}

uint8_t odrive_handle_msg(CanMessage_t *msg)
{
    switch (msg->id)
    {
    case (MSG_GET_VBUS_VOLTAGE | AXIS0_NODE_ID):
        vbus_voltage.a = (msg->buf[3] << 24) | (msg->buf[2] << 16) | (msg->buf[1] << 8) | (msg->buf[0] << 0);
        break;
    
    default:
        return 1;
        break;
    }
    return 0;
}

uint8_t odrive_can_send(Axis_t axis, OdriveMsg_t msg)
{
    uint32_t can_error = HAL_CAN_GetError(&hcan1);
    if (can_error == HAL_CAN_ERROR_NONE) // HAL_CAN_GetError()
    {
        CAN_TxHeaderTypeDef header;
        if (axis == AXIS_0)
        {
            header.StdId = AXIS0_NODE_ID + msg;
        }
        else if (axis == AXIS_1)
        {
            header.StdId = AXIS1_NODE_ID + msg;
        }
        else
        {
            return -1;
        }

        header.IDE = CAN_ID_STD;
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;

        uint32_t retTxMailbox = 0;
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
        {
            uint8_t empty[0]; //declare byte to be transmitted //declare a receive byte
            HAL_CAN_AddTxMessage(&hcan1, &header, empty, &retTxMailbox);
        }

        // while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0);

        return 1;
    }
    else
    {
        return can_error;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    // osSemaphoreRelease(canInterruptBinarySemHandle);
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0))
    {
        CAN_RxHeaderTypeDef header;
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
        {
            CanMessage_t rxmsg;
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rxmsg.buf);

            rxmsg.id = header.StdId;
            rxmsg.len = header.DLC;
            rxmsg.rtr = header.RTR;
            osMessageQueuePut(canRxBufferHandle, &rxmsg, 0, 0);
        }
    }
}