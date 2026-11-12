/* 板级组装：UART4资源只在board层出现，协议模块不引用huart4。 */
#ifndef BOARD_OPS_H
#define BOARD_OPS_H

#include "ops.h"

/* 本板统一定位新鲜度；供OPS接收与应用快照共同使用。 */
#define BOARD_OPS_FEEDBACK_TIMEOUT_MS 200U

typedef struct
{
    OPSFeedback feedback;   /* 展开后观察SI坐标、有效性和协议计数。 */
    uint32_t rx_bytes;      /* 已从UART环形区读出的字节数。 */
    uint32_t uart_errors;   /* HAL错误及软件接收环溢出的累计次数。 */
    bool initialized;      /* 板级注册成功，失败时原有电机任务仍可启动。 */
    bool uart_running;     /* 循环DMA已启动；不等于已经收到OPS数据。 */
} BoardOPSStatus;

/* Init在任务创建前调用；Start/Update/GetStatus由motorTask唯一调用。 */
bool BoardOPSInit(void);
bool BoardOPSStart(uint32_t now_ms);
void BoardOPSUpdate(uint32_t now_ms);
void BoardOPSGetStatus(BoardOPSStatus *status);

#endif
