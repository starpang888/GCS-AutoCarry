#include "host_service.h"
#include "vofa_protocol.h"
#include <math.h>
#include <string.h>

static void Command(void *context, const HostCommand *command)
{
    HostServiceInstance *service = context;
    if (service->stop_in_batch) return;
    RobotHostCommand(service->robot, command, service->now_ms);
    if (command->type == HOST_STOP) service->stop_in_batch = true;
}

void HostServiceInit(HostServiceInstance *service, RobotInstance *robot)
{
    /* Avoid an 800-byte aggregate temporary on the pre-scheduler MSP stack
     * in Debug builds. Buffers belong to this long-lived static instance.
     */
    memset(service, 0, sizeof(*service));
    service->robot = robot;
    HostProtocolInit(&service->protocol, Command, service);
}

bool HostServiceSelectFeedback(HostServiceInstance *service, uint32_t mode)
{
    if (mode > HOST_FEEDBACK_FIREWATER) return false;
    if (mode != (uint32_t)service->status.feedback_mode &&
        ChassisControlIsBusy(service->robot->control)) return false;
    service->status.feedback_mode = (HostFeedbackMode)mode;
    return true;
}

static void LinkFault(HostServiceInstance *service, uint32_t now_ms)
{
    HostProtocolReset(&service->protocol);
    service->stop_in_batch = true;
    /* 取消应用层位置目标和底盘目标，恢复连接后须新的外部命令。 */
    if (service->robot->mode == ROBOT_POSITION ||
        ChassisControlIsBusy(service->robot->control))
        RobotStop(service->robot, CHASSIS_LINK_LOST, now_ms);
}

void HostServiceReceive(HostServiceInstance *service, uint32_t now_ms)
{
    service->now_ms = now_ms;
    service->stop_in_batch = false;
    BSPUSBStatus link;
    BSPUSBGetStatus(&link);
    if (link.epoch != service->status.epoch)
    {
        LinkFault(service, now_ms);
        service->status.epoch = link.epoch;
    }
    service->status.connected = link.connected;
    service->status.rx_overflows = link.overflows;
    /* Drain at most one ring capacity per call, even under continuous input.
     * Epoch changes discard the current batch and require a new command later.
     */
    for (size_t batch = 0; batch < BSP_USB_RX_CAPACITY / 32U; ++batch)
    {
        size_t count = BSPUSBRead(service->rx, 32U);
        if (!count) break;
        for (size_t i = 0; i < count; ++i)
        {
            uint32_t age = BSPUSBClockMs() - service->rx[i].arrival_ms;
            if (!service->stop_in_batch && link.connected)
                HostProtocolReceive(&service->protocol, service->rx[i].byte, now_ms - age);
        }
        BSPUSBStatus current;
        BSPUSBGetStatus(&current);
        if (current.epoch != link.epoch || !current.connected)
        {
            LinkFault(service, now_ms);
            /* Leave epoch unacknowledged until next cycle also discards its
             * batch, so reconnect/overflow bytes cannot arm immediately.
             */
        }
    }
    service->status.received_frames = service->protocol.valid_frames;
    service->status.crc_errors = service->protocol.crc_errors;
    service->status.format_errors = service->protocol.format_errors;
}

static void PutSigned(uint8_t *bytes, float value)
{
    /* Fixed milli-units, INT32_MIN means unavailable. Clamp before casting. */
    int32_t wire = INT32_MIN;
    if (isfinite(value) && fabsf(value) < 2147483.0f)
        wire = (int32_t)(value * 1000.0f);
    HostPutU32(bytes, (uint32_t)wire);
}

static void PutVelocity(uint8_t *bytes, const OmniBodyVelocity *velocity)
{
    PutSigned(bytes, velocity->vx_m_s);
    PutSigned(bytes + 4U, velocity->vy_m_s);
    PutSigned(bytes + 8U, velocity->wz_rad_s);
}

/* 0x82的64字节载荷。全部多字节字段为小端，位置mm、角度mrad。
 * 该函数只编码反馈；既不设置目标，也不改变使能状态。
 */
static size_t EncodePose(HostServiceInstance *service, const OmniBodyVelocity *measured,
                         bool measured_valid, uint32_t now_ms)
{
    RobotInstance *robot = service->robot;
    const ChassisOdometryInstance *odometry = robot->odometry;
    uint8_t *out = service->payload;
    memset(out, 0, HOST_POSE_PAYLOAD_SIZE);
    bool pose_valid = odometry != NULL && odometry->initialized && odometry->has_baseline &&
        odometry->pose_valid && !odometry->tracking_lost &&
        (uint32_t)(now_ms - odometry->last_update_ms) < odometry->config.feedback_timeout_ms;
    bool position_active = robot->mode == ROBOT_POSITION;
    HostPutU32(out, now_ms);                                  /* 0: 本帧编码时间ms。 */
    HostPutU32(out + 4U, odometry ? odometry->last_update_ms : 0U); /* 4: 位姿更新时间。 */
    HostPutU32(out + 8U, robot->position_sequence);             /* 8: 位置命令seq。 */
    HostPutU32(out + 12U, robot->goal_generation);              /* 12: 目标代次。 */
    out[16] = (uint8_t)robot->mode;
    out[17] = (uint8_t)robot->arrival.state;
    bool within = position_active && robot->position_control->result.valid &&
        robot->position_control->result.within_tolerance;
    out[18] = (uint8_t)(pose_valid | (uint32_t)measured_valid << 1U |
        (uint32_t)within << 2U | (uint32_t)position_active << 3U);
    /* 19保留为0。无效值用INT32_MIN，同时必须读取flags，不能当作坐标。 */
    PutSigned(out + 20U, pose_valid ? odometry->pose.x_m : NAN);
    PutSigned(out + 24U, pose_valid ? odometry->pose.y_m : NAN);
    PutSigned(out + 28U, pose_valid ? odometry->pose.yaw_rad : NAN);
    for (size_t i = 0; i < 3U; ++i)
        HostPutU32(out + 32U + 4U * i, (uint32_t)robot->position_target_milli[i]);
    PutVelocity(out + 44U, measured); /* 实测车体vx/vy/wz，mm/s、mm/s、mrad/s。 */
    HostPutU32(out + 56U, robot->arrival.stable_ms);
    HostPutU32(out + 60U, robot->arrival_config.hold_ms);
    return HostProtocolEncode(HOST_POSE, service->tx_sequence, out,
        HOST_POSE_PAYLOAD_SIZE, service->tx, sizeof(service->tx));
}

void HostServiceTransmit(HostServiceInstance *service, const BoardMotorStatus *motors, uint32_t now_ms)
{
    /* 二进制总计40帧/s，两类各20Hz；FireWater仍为原来的20Hz。
     * 一次只提交一帧，不增加240字节USB缓冲，不混入文本。
     */
    bool binary = service->status.feedback_mode == HOST_FEEDBACK_BINARY;
    uint32_t period_ms = binary ? 25U : 50U;
    if ((uint32_t)(now_ms - service->last_tx_ms) < period_ms) return;
    service->last_tx_ms = now_ms;
    ChassisControlInstance *control = service->robot->control;
    uint8_t *payload = service->payload;
    memset(payload, 0, sizeof(service->payload));
    HostPutU32(payload, now_ms);
    payload[4] = (uint8_t)control->status.state;
    payload[5] = (uint8_t)control->status.fault;
    payload[6] = service->status.connected;
    payload[7] = motors->motion_allowed;
    HostPutU32(payload + 8U, control->status.last_sequence);
    HostPutU32(payload + 12U, control->status.rejected_commands);
    HostPutU32(payload + 16U, service->status.received_frames);
    HostPutU32(payload + 20U, service->status.crc_errors);
    HostPutU32(payload + 24U, service->status.format_errors);
    HostPutU32(payload + 28U, service->status.rx_overflows);
    HostPutU32(payload + 32U, service->status.tx_skipped);
    HostPutU32(payload + 36U, service->status.epoch);
    PutVelocity(payload + 40U, &control->status.target.requested);
    PutVelocity(payload + 52U, &control->status.target.applied);
    float speeds[OMNI_WHEEL_COUNT];
    bool valid = true;
    uint32_t youngest_age = UINT32_MAX, oldest_age = 0U;
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        const EmmMotorFeedback *wheel = &motors->wheels[i];
        uint8_t *out = payload + 100U + i * 24U;
        uint64_t raw = (uint64_t)wheel->position_raw;
        HostPutU32(out, (uint32_t)raw);
        HostPutU32(out + 4U, (uint32_t)(raw >> 32U));
        PutSigned(out + 8U, wheel->speed_valid ? wheel->wheel_speed_rad_s : NAN);
        HostPutU32(out + 12U, wheel->position_ms);
        uint32_t flags = wheel->position_valid | (uint32_t)wheel->speed_valid << 1U |
            (uint32_t)wheel->enabled_valid << 2U | (uint32_t)wheel->enabled << 3U |
            (uint32_t)wheel->stall_valid << 4U | (uint32_t)wheel->stalled << 5U |
            (uint32_t)wheel->stall_protection << 6U;
        HostPutU32(out + 16U, flags);
        HostPutU32(out + 20U, wheel->enabled_ms);
        uint32_t age = now_ms - wheel->position_ms;
        if (age < youngest_age) youngest_age = age;
        if (age > oldest_age) oldest_age = age;
        valid &= wheel->speed_valid && wheel->position_valid && age < 200U;
        speeds[i] = wheel->wheel_speed_rad_s;
    }
    OmniBodyVelocity measured;
    valid = valid && oldest_age - youngest_age <= 80U &&
        OmniKinematicsForward(service->robot->kinematics, speeds, &measured);
    if (!valid) measured = (OmniBodyVelocity){NAN, NAN, NAN};
    PutVelocity(payload + 64U, &measured);
    HostPutU32(payload + 76U, valid);
    HostPutU32(payload + 80U, motors->transaction_timeouts);
    HostPutU32(payload + 84U, motors->protocol_errors);
    HostPutU32(payload + 88U, motors->rejected_commands);
    HostPutU32(payload + 92U, motors->transport_errors);
    HostPutU32(payload + 96U, motors->rx_errors);
    size_t length;
    if (service->status.feedback_mode == HOST_FEEDBACK_FIREWATER)
    {
        int64_t *channels = service->vofa_channels;
        memset(channels, 0, sizeof(service->vofa_channels));
        channels[0] = control->status.state;
        channels[1] = control->status.fault;
        channels[2] = motors->motion_allowed;
        for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
        {
            const EmmMotorFeedback *wheel = &motors->wheels[i];
            int32_t speed = (int32_t)HostGetU32(payload + 108U + i * 24U);
            /* Numeric placeholder for invalid speed; channel 17's mask is
             * authoritative. Never substitute commanded speed for feedback.
             */
            channels[3U + i] = speed == INT32_MIN ? 0 : speed;
            channels[7U + i] = wheel->position_raw;
            channels[16] |= (int64_t)wheel->position_valid << i;
            channels[17] |= (int64_t)(wheel->speed_valid && speed != INT32_MIN) << i;
            channels[18] |= (int64_t)wheel->enabled_valid << i;
            channels[19] |= (int64_t)(wheel->enabled_valid && wheel->enabled) << i;
            channels[20] |= (int64_t)(wheel->stall_valid && (wheel->stalled || wheel->stall_protection)) << i;
        }
        channels[11] = motors->transaction_timeouts;
        channels[12] = motors->protocol_errors;
        channels[13] = motors->rejected_commands;
        channels[14] = motors->transport_errors;
        channels[15] = motors->rx_errors;
        channels[21] = now_ms;
        length = VofaFireWaterEncode(channels, HOST_VOFA_CHANNEL_COUNT, 0x78U,
                                    service->tx, sizeof(service->tx));
    }
    else if (service->next_pose)
        length = EncodePose(service, &measured, valid, now_ms);
    else
        length = HostProtocolEncode(HOST_STATUS, service->tx_sequence, payload,
            sizeof(service->payload), service->tx, sizeof(service->tx));
    if (!BSPUSBWrite(service->tx, length))
        ++service->status.tx_skipped;
    else if (binary)
    {
        ++service->tx_sequence;
        /* USB忙时保留待发送类型，避免某一类反馈长期被饿死。 */
        service->next_pose = !service->next_pose;
    }
}
