/* USB CDC 缓冲实现：ISR 与任务通过短临界区同步，不在中断解释业务。 */
#include "bsp_usb.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include <string.h>

static BSPUSBByte ring[BSP_USB_RX_CAPACITY];
static uint32_t read_count, write_count;
static BSPUSBStatus status;
static uint8_t tx[BSP_USB_TX_CAPACITY];
static bool tx_busy;

/** @brief 读取 HAL 时钟，接收年龄计算不能混用 RTOS tick 原点。 */
uint32_t BSPUSBClockMs(void) { return HAL_GetTick(); }

/** @brief 配置变化后丢弃旧输入，使用 epoch 通知任务中止旧控制会话。 */
void BSPUSBConnected(bool connected)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    status.connected = connected;
    ++status.epoch;
    read_count = write_count = 0U;
    /* 挂起时不能释放 IN 缓冲；保持忙，直到发送完成或下一次类初始化。
     * 此处 connected=true 对应重新配置，可重置忙标志；恢复走 Resume。
     */
    if (connected) tx_busy = false;
    __set_PRIMASK(mask);
}

/** @brief 中断写入整包；空间不足时清空环形区并递增溢出和代次。 */
void BSPUSBReceive(const uint8_t *data, size_t length, uint32_t arrival_ms)
{
    if (!status.connected || data == NULL) return;
    if (length > BSP_USB_RX_CAPACITY - (uint32_t)(write_count - read_count))
    {
        ++status.overflows;
        ++status.epoch;
        read_count = write_count = 0U;
        return; /* 不覆盖未读字节，也不接收半个数据包。 */
    }
    for (size_t i = 0; i < length; ++i)
    {
        ring[write_count % BSP_USB_RX_CAPACITY] = (BSPUSBByte){arrival_ms, data[i]};
        ++write_count;
    }
}

/** @brief IN 完成后允许任务复用静态 TX 数据。 */
void BSPUSBTxComplete(void)
{
    tx_busy = false;
}

/** @brief 恢复连接并递增代次，保留尚未完成的 IN 缓冲占用。 */
void BSPUSBResume(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    status.connected = true;
    ++status.epoch;
    __set_PRIMASK(mask);
}

/** @brief 在保存/恢复 PRIMASK 的短临界区读取字节及其时间。 */
size_t BSPUSBRead(BSPUSBByte *data, size_t capacity)
{
    if (data == NULL) return 0U;
    if (capacity > 32U) capacity = 32U; /* 限制关中断时间。 */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    size_t count = 0U;
    while (count < capacity && read_count != write_count)
    {
        data[count++] = ring[read_count % BSP_USB_RX_CAPACITY];
        ++read_count;
    }
    __set_PRIMASK(mask);
    return count;
}

/** @brief 与中断同步取得一致快照。 */
void BSPUSBGetStatus(BSPUSBStatus *snapshot)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *snapshot = status;
    __set_PRIMASK(mask);
}

/** @brief 任务复制数据并尝试一次 CDC 发送，失败由调用者决定丢弃或后续处理。 */
bool BSPUSBWrite(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0U || length > sizeof(tx)) return false;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!status.connected || tx_busy)
    {
        __set_PRIMASK(mask);
        return false;
    }
    memcpy(tx, data, length);
    tx_busy = true;
    bool sent = CDC_Transmit_FS(tx, (uint16_t)length) == USBD_OK;
    if (!sent) tx_busy = false;
    __set_PRIMASK(mask);
    return sent;
}
