/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_H
#define __FREERTOS_H

// List of semaphores
extern osTimerId_t canSendTimerHandle;
extern osThreadId_t controlTaskHandle;
extern osThreadId_t displayTaskHandle;
extern osThreadId_t canTaskHandle;

#endif /* __FREERTOS_H */