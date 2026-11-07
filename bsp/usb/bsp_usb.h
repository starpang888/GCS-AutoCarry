/* USB FS CDC 静态服务：带接收时间的字节环形区与独占发送缓冲。 */
#ifndef BSP_USB_H
#define BSP_USB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_USB_RX_CAPACITY 512U
#define BSP_USB_TX_CAPACITY 240U
/* 每个字节携带 HAL 毫秒接收时刻，用于识别任务延迟造成的过期命令。 */
typedef struct { uint32_t arrival_ms; uint8_t byte; } BSPUSBByte;
typedef struct
{
    uint32_t epoch; /* 类初始化、反初始化、挂起或溢出时变化，恢复也递增。 */
    uint32_t overflows;
    bool connected;
} BSPUSBStatus;

/* 单静态 USB FS 实例，中断启用前由 C 静态零初始化建立初态。
 * Connected/Resume/Receive/TxComplete 对接 USB 回调；Receive 必须在
 * 重新提交 OUT 接收前复制数据，输入指针只在本次调用期间使用。
 * Read/Write/GetStatus 由唯一任务拥有，短 PRIMASK 临界区与中断同步，
 * 不调用 RTOS API。arrival_ms 和 ClockMs 均属于 HAL_GetTick 毫秒域。
 */
/** @brief 更新类配置/挂起状态并清空旧接收数据，递增连接代次。 */
void BSPUSBConnected(bool connected);
void BSPUSBResume(void); /* 恢复不能提前释放正在发送的 IN 缓冲。 */
/** @brief USB 中断复制收到的字节与时间；溢出则整包丢弃并清空缓存。 */
void BSPUSBReceive(const uint8_t *data, size_t length, uint32_t arrival_ms);
/** @brief USB IN 完成回调释放发送缓冲。 */
void BSPUSBTxComplete(void);
/** @brief 任务每次最多读取 32 字节，以限制关中断时间。 */
size_t BSPUSBRead(BSPUSBByte *data, size_t capacity);
/** @brief 复制一致的连接/溢出快照；status 必须指向有效存储。 */
void BSPUSBGetStatus(BSPUSBStatus *status);
/* 复制到独占发送区；忙或未连接时拒绝，不阻塞、不内部重试。 */
bool BSPUSBWrite(const uint8_t *data, size_t length);
/** @brief 提供与接收时间戳同域的 HAL 毫秒时钟。 */
uint32_t BSPUSBClockMs(void);

#endif
