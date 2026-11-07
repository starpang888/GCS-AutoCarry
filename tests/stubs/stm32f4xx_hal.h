#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H

#include <stdint.h>
#define DMA_CIRCULAR 1U
#define DMA_NORMAL 0U
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { struct { uint32_t Mode; } Init; uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; DMA_HandleTypeDef *hdmatx; } UART_HandleTypeDef;
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
static inline void __DMB(void) { __asm volatile ("dmb" ::: "memory"); }
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *);
uint32_t HAL_GetTick(void);

#endif
