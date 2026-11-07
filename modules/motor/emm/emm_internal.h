/* EMM 模块内部接口：仅供总线与电机解析器协作，上层使用公开头文件。 */
#ifndef EMM_INTERNAL_H
#define EMM_INTERNAL_H

#include "emm_motor.h"

/** @brief 总线完成帧校验后同步解析回复，frame 仅在调用期间有效。 */
void EmmMotorResponse(EmmMotorInstance *motor, EmmTransactionKind kind,
                      const uint8_t *frame, uint32_t now_ms);
/** @brief 作废所有反馈有效位，保留原值供排错，不再当作实时观测。 */
void EmmMotorInvalidate(EmmMotorInstance *motor);
/** @brief 检查位置/使能/堵转反馈新鲜，且实际使能、无堵转和保护。 */
bool EmmMotorHealthy(const EmmMotorInstance *motor, uint32_t now_ms);

#endif
