/* 接收协议：$ODOM,seq,t_us,x_mm,y_mm,yaw_deg,valid,continuous,state,error*HH\r\n
 * XOR覆盖'$'之后、'*'之前的字节。按流寻找起点/换行，不假定DMA回调就是一帧。
 */
#include "ops.h"
#include <string.h>

/* 解析无符号十进制，显式检查uint32溢出，不接受空字段或隐含的正负号。 */
static bool ReadUnsigned(const char **cursor, char end, uint32_t *value)
{
    const char *p = *cursor;
    uint32_t result = 0U;
    if (*p < '0' || *p > '9') return false;
    do
    {
        uint32_t digit = (uint32_t)(*p++ - '0');
        if (result > (UINT32_MAX - digit) / 10U) return false;
        result = result * 10U + digit;
    } while (*p >= '0' && *p <= '9');
    if (*p != end) return false;
    *cursor = p + 1;
    *value = result;
    return true;
}

/* 发送端固定三位小数。本接收端严格匹配，不使用sscanf/浮点printf，
 * 也不接受NaN/Inf/科学计数法。最大整数部分999999999与OPS发送端一致。
 */
static bool ReadFixed(const char **cursor, float *value)
{
    const char *p = *cursor;
    bool negative = *p == '-';
    if (negative) ++p;
    uint32_t whole;
    if (!ReadUnsigned(&p, '.', &whole) || whole > 999999999U) return false;
    uint32_t fraction = 0U;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (*p < '0' || *p > '9') return false;
        fraction = fraction * 10U + (uint32_t)(*p++ - '0');
    }
    if (*p != ',') return false;
    *cursor = p + 1;
    *value = (float)whole + (float)fraction * 0.001f;
    if (negative) *value = -*value;
    return true;
}

static int HexDigit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* 连续性中断只记录一次：从有效变为无效时递增代次。
 * 后续帧即使在同一次OPSReceive中恢复有效，Robot也能发现连续性发生过变化。
 * 坐标保留供观察，不伪造新的采样或自动恢复旧位置目标。
 */
static void Invalidate(OPSInstance *ops)
{
    if (ops->feedback.valid) ++ops->feedback.continuity_epoch;
    ops->feedback.valid = false;
}

/* 只有完整CRLF结尾的报文才进入本函数，解析失败不会更新旧坐标。 */
static void ParseFrame(OPSInstance *ops, uint32_t now_ms)
{
    char *frame = ops->frame;
    size_t length = ops->length;
    if (length < 6U || memcmp(frame, "$ODOM,", 6U) != 0 ||
        length < 11U || frame[length - 2U] != '\r')
    {
        ++ops->feedback.format_errors;
        return;
    }
    size_t star = length - 5U;
    int high = HexDigit(frame[star + 1U]);
    int low = HexDigit(frame[star + 2U]);
    if (frame[star] != '*' || high < 0 || low < 0)
    {
        ++ops->feedback.format_errors;
        return;
    }
    uint8_t checksum = 0U;
    for (size_t i = 1U; i < star; ++i) checksum ^= (uint8_t)frame[i];
    if (checksum != (uint8_t)((high << 4) | low))
    {
        ++ops->feedback.checksum_errors;
        return;
    }
    /* 把校验分隔符换成NUL，字段解析被限制在已收齐的payload内。
     * 短路检查可保证遇到NUL后不会继续向后读；不使用strtok的全局状态。
     */
    frame[star] = '\0';
    const char *p = frame + 6U;
    uint32_t sequence, sample_us, valid, continuous, state, error;
    float x_mm, y_mm, yaw_deg;
    if (!ReadUnsigned(&p, ',', &sequence) || !ReadUnsigned(&p, ',', &sample_us) ||
        !ReadFixed(&p, &x_mm) || !ReadFixed(&p, &y_mm) || !ReadFixed(&p, &yaw_deg) ||
        !ReadUnsigned(&p, ',', &valid) || !ReadUnsigned(&p, ',', &continuous) ||
        !ReadUnsigned(&p, ',', &state) || !ReadUnsigned(&p, '\0', &error) ||
        valid > 1U || continuous > 1U || state > 3U || error > 6U)
    {
        ++ops->feedback.format_errors;
        return;
    }

    OPSFeedback *feedback = &ops->feedback;
    /* seq允许跳号和uint32自然回绕。倒退提示OPS重启/乱序，取消旧目标代次。
     * 不要求逐帧+1，也不把seq当上位机运动许可或命令期限。
     */
    bool sequence_restarted = feedback->frames != 0U &&
        (uint32_t)(sequence - feedback->sequence) >= 0x80000000U;
    if (sequence_restarted) Invalidate(ops);
    ++feedback->frames;
    feedback->sequence = sequence;
    feedback->sample_us = sample_us;
    feedback->received_ms = now_ms;
    feedback->source_valid = valid != 0U;
    feedback->continuous = continuous != 0U;
    feedback->state = (uint8_t)state;
    feedback->error = (uint8_t)error;
    bool usable = valid != 0U && continuous != 0U && state == 1U && error == 0U;
    if (!usable) Invalidate(ops);

    /* 不能只看seq：发送任务还在运行，不等于定位采样还在推进。
     * t_us自然回绕用无符号差值处理；两次采样间隔须小于2^31 us。
     * 倒退帧仅重建时间比较基准，本帧不接收位姿，下一帧推进后再恢复。
     * 连续性代次由Robot绑定到当前目标；恢复数据不代表恢复旧目标。
     */
    if (ops->have_sample)
    {
        uint32_t delta = sample_us - ops->previous_sample_us;
        if (delta == 0U)
        {
            ++feedback->repeated_samples;
            return;
        }
        if (delta >= 0x80000000U)
        {
            ++feedback->time_restarts;
            ops->previous_sample_us = sample_us;
            Invalidate(ops);
            return;
        }
    }
    ops->previous_sample_us = sample_us;
    ops->have_sample = true;
    if (usable && !sequence_restarted)
    {
        /* OPS已经积分得到位姿，这里只做单位换算，不能再次积分。
         * yaw_deg为度，乘pi/180得到rad，不是mrad。
         */
        feedback->x_m = x_mm * 0.001f;
        feedback->y_m = y_mm * 0.001f;
        feedback->yaw_rad = yaw_deg * 0.017453292519943295f;
        feedback->pose_ms = now_ms;
        feedback->valid = true;
        ops->have_pose = true;
    }
}

bool OPSInit(OPSInstance *ops, const OPS_Init_Config_s *config)
{
    if (ops == NULL || config == NULL || config->feedback_timeout_ms == 0U ||
        config->feedback_timeout_ms >= 0x80000000U) return false;
    memset(ops, 0, sizeof(*ops));
    ops->config = *config;
    ops->feedback.age_ms = UINT32_MAX;
    return true;
}

void OPSUpdate(OPSInstance *ops, uint32_t now_ms)
{
    if (ops == NULL) return;
    OPSFeedback *feedback = &ops->feedback;
    feedback->online = feedback->frames != 0U &&
        (uint32_t)(now_ms - feedback->received_ms) < ops->config.feedback_timeout_ms;
    feedback->age_ms = ops->have_pose ? now_ms - feedback->pose_ms : UINT32_MAX;
    /* 一旦失效，只有后续有效新采样可恢复；不能靠读取旧快照重新变为有效。 */
    if (!feedback->online || feedback->age_ms >= ops->config.feedback_timeout_ms)
        Invalidate(ops);
}

void OPSReceive(OPSInstance *ops, const uint8_t *data, size_t length, uint32_t now_ms)
{
    if (ops == NULL || data == NULL) return;
    OPSUpdate(ops, now_ms); /* 接受新帧前先兑现旧采样超时。 */
    for (size_t i = 0U; i < length; ++i)
    {
        char c = (char)data[i];
        if (c == '$')
        {
            /* 不完整报文中再次遇到$，直接重同步，不等到超时或缓冲填满。 */
            if (ops->collecting) ++ops->feedback.format_errors;
            ops->length = 0U;
            ops->collecting = true;
        }
        if (!ops->collecting) continue;
        if (ops->length >= OPS_FRAME_CAPACITY - 1U || c == '\0')
        {
            ++ops->feedback.format_errors;
            ops->collecting = false;
            ops->length = 0U;
            continue;
        }
        ops->frame[ops->length++] = c;
        if (c == '\n')
        {
            ops->frame[ops->length] = '\0';
            ParseFrame(ops, now_ms);
            ops->collecting = false;
            ops->length = 0U;
        }
    }
    OPSUpdate(ops, now_ms);
}

void OPSTransportFault(OPSInstance *ops)
{
    if (ops == NULL) return;
    ++ops->feedback.transport_errors;
    ops->collecting = false;
    ops->length = 0U;
    Invalidate(ops);
}
