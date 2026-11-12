#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H

#include <stdint.h>
#define DMA_CIRCULAR 1U
#define DMA_NORMAL 0U
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { uint32_t compare; uint32_t compare2; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { struct { uint32_t Mode; } Init; uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; DMA_HandleTypeDef *hdmatx; } UART_HandleTypeDef;
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define __HAL_TIM_SET_COMPARE(timer, channel, value) do { if ((channel) == TIM_CHANNEL_2) (timer)->Instance->compare2 = (value); else (timer)->Instance->compare = (value); } while (0)
static inline void __DMB(void) { __asm volatile ("dmb" ::: "memory"); }
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);

#endif
