#include "emm_motor.h"
#include <math.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#define NEAR(a, b) (fabsf((a) - (b)) < 0.002f)

typedef struct
{
    bool busy;
    bool fail;
    uint8_t frame[8];
    size_t length;
    uint32_t sends;
} TestTransport;

static bool Send(void *context, const uint8_t *data, size_t length, uint32_t now_ms)
{
    (void)now_ms;
    TestTransport *transport = context;
    if (transport->fail || transport->busy)
    {
        return false;
    }
    memcpy(transport->frame, data, length);
    transport->length = length;
    transport->busy = true;
    ++transport->sends;
    return true;
}

static bool Busy(void *context)
{
    return ((TestTransport *)context)->busy;
}

static bool Init(EmmBusInstance *bus, EmmMotorInstance motors[4],
                 TestTransport *transport, uint32_t now_ms)
{
    memset(transport, 0, sizeof(*transport));
    EmmBus_Init_Config_s config = {
        .send = Send, .tx_busy = Busy, .context = transport,
        .response_timeout_ms = 20U, .quiet_ms = 2U, .quarantine_ms = 40U,
        .command_timeout_ms = 200U, .feedback_timeout_ms = 500U,
    };
    if (!EmmBusInit(bus, &config, now_ms))
    {
        return false;
    }
    for (uint8_t i = 0; i < 4U; ++i)
    {
        EmmMotor_Init_Config_s motor_config = {
            .bus = bus, .address = i + 1U, .direction = 1,
            .gear_ratio = 1.0f, .wheel_rad_s_per_speed_unit = 0.0f,
        };
        if (!EmmMotorInit(&motors[i], &motor_config))
        {
            return false;
        }
    }
    return true;
}

/* Startup stop and disable use only motor 1's broadcast acknowledgement. */
static void Startup(EmmBusInstance *bus, TestTransport *transport, uint32_t start)
{
    uint8_t ack[] = {1U, 0xFEU, 2U, 0x6BU};
    EmmBusUpdate(bus, start);
    transport->busy = false;
    EmmBusReceive(bus, ack, sizeof(ack), start + 1U);
    EmmBusUpdate(bus, start + 1U);
    EmmBusUpdate(bus, start + 3U);
    transport->busy = false;
    ack[1] = 0xF3U;
    EmmBusReceive(bus, ack, sizeof(ack), start + 4U);
    EmmBusUpdate(bus, start + 4U);
}

/* Create fresh health by exercising real query transactions and response parser. */
static uint32_t ReadState(EmmBusInstance *bus, EmmMotorInstance motors[4],
                          TestTransport *transport, uint32_t now_ms, uint8_t status)
{
    for (uint8_t i = 0; i < 4U; ++i)
    {
        const EmmTransactionKind kinds[] = {EMM_QUERY_POSITION, EMM_QUERY_STATUS};
        for (uint8_t q = 0; q < 2U; ++q)
        {
            (void)EmmMotorQuery(&motors[i], kinds[q]);
            now_ms += 3U;
            EmmBusUpdate(bus, now_ms);
            transport->busy = false;
            uint8_t response[] = {(uint8_t)(i + 1U), 0x36U, 0U, 0U, 0U, 0U, 0U, EMM_CHECK_BYTE};
            if (q != 0U)
            {
                response[1] = 0x3AU;
                response[2] = status; /* Unknown high bits are allowed. */
                response[3] = EMM_CHECK_BYTE;
            }
            EmmBusReceive(bus, response, q == 0U ? 8U : 4U, now_ms + 1U);
            EmmBusUpdate(bus, now_ms + 1U);
        }
    }
    return now_ms + 1U;
}

static uint32_t ReadHealth(EmmBusInstance *bus, EmmMotorInstance motors[4],
                           TestTransport *transport, uint32_t now_ms)
{
    return ReadState(bus, motors, transport, now_ms, 0x83U);
}

int RunEmmTests(void)
{
    EmmBusInstance bus;
    EmmMotorInstance motors[4];
    TestTransport transport;
    CHECK(Init(&bus, motors, &transport, 0U));
    CHECK(!EmmBusArm(&bus, 0U));
    CHECK(!EmmMotorSetSpeedUnits(&motors[0], 50, 0, 0U));
    EmmMotorInstance extra;
    EmmMotor_Init_Config_s duplicate = motors[0].config;
    CHECK(!EmmMotorInit(&extra, &duplicate));
    CHECK(!EmmMotorInit(&motors[0], &duplicate));
    duplicate.address = 5U;
    CHECK(!EmmMotorInit(&extra, &duplicate));
    CHECK(motors[0].registered && motors[0].config.address == 1U);

    EmmBusUpdate(&bus, 0U);
    const uint8_t stop[] = {0U, 0xFEU, 0x98U, 0U, 0x6BU};
    CHECK(transport.length == 5U && memcmp(transport.frame, stop, 5U) == 0);
    const uint8_t wrong_ack[] = {2U, 0xFEU, 2U, 0x6BU};
    transport.busy = false;
    EmmBusReceive(&bus, wrong_ack, sizeof(wrong_ack), 1U);
    CHECK(!bus.response_received);
    uint8_t ack[] = {1U, 0xFEU, 2U, 0x6BU};
    EmmBusReceive(&bus, ack, 1U, 1U);
    EmmBusReceive(&bus, ack + 1, 3U, 1U);
    CHECK(bus.response_received);
    EmmBusUpdate(&bus, 1U);
    EmmBusUpdate(&bus, 3U);
    const uint8_t disable[] = {0U, 0xF3U, 0xABU, 0U, 0U, 0x6BU};
    CHECK(transport.length == 6U && memcmp(transport.frame, disable, 6U) == 0);
    transport.busy = false;
    ack[1] = 0xF3U;
    EmmBusReceive(&bus, ack, 4U, 4U);
    EmmBusUpdate(&bus, 4U);
    CHECK(!motors[0].feedback.position_valid); /* ACK is not measurement. */

    uint32_t now = ReadHealth(&bus, motors, &transport, 5U);
    CHECK(motors[3].feedback.position_valid && motors[3].feedback.enabled);
    CHECK(EmmBusArm(&bus, now));
    CHECK(!EmmMotorSetSpeed(&motors[0], 1.0f, 0, now)); /* Unknown conversion. */
    CHECK(!EmmMotorSetSpeed(&motors[0], NAN, 0, now));
    CHECK(!EmmMotorSetSpeedUnits(&motors[0], 5001, 0, now));
    CHECK(!EmmMotorSetSpeedUnits(&motors[0], -5001, 0, now));
    CHECK(EmmMotorSetSpeedUnits(&motors[0], 50, 0, now));
    CHECK(EmmMotorSetSpeedUnits(&motors[0], -5000, 0, now));
    CHECK(bus.queue_count == 1U); /* Latest target overwrites previous target. */
    EmmBusUpdate(&bus, now + 3U);
    const uint8_t maximum[] = {1U, 0xF6U, 1U, 0x13U, 0x88U, 0U, 0U, 0x6BU};
    CHECK(memcmp(transport.frame, maximum, 8U) == 0);
    CHECK(EmmMotorSetSpeedUnits(&motors[1], 100, 0, now + 3U));
    EmmBusStop(&bus, true, now + 4U);
    CHECK(bus.queue_count == 0U && !bus.motion_allowed);
    uint32_t sends = transport.sends;
    EmmBusUpdate(&bus, now + 4U);
    CHECK(transport.sends == sends); /* In-flight DMA bytes remain stable. */
    CHECK(memcmp(transport.frame, maximum, 8U) == 0);
    transport.busy = false;
    EmmBusUpdate(&bus, now + 5U);
    CHECK(memcmp(transport.frame, stop, 5U) == 0); /* Don't wait 20 ms for old ACK. */
    CHECK(!EmmBusArm(&bus, now + 5U));

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 6U);
    CHECK(transport.frame[1] == 0x36U);
    transport.busy = false;
    /* Embedded terminator byte must not truncate a fixed-length response. */
    const uint8_t negative[] = {1U, 0x36U, 1U, 0U, 0U, 0x6BU, 0U, 0x6BU};
    EmmBusReceive(&bus, negative, 2U, 7U);
    CHECK(!motors[0].feedback.position_valid);
    EmmBusReceive(&bus, negative + 2U, 6U, 7U);
    CHECK(motors[0].feedback.position_raw == -27392);
    CHECK(!motors[0].feedback.speed_valid);
    EmmBusUpdate(&bus, 7U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 9U);
    transport.busy = false;
    const uint8_t next[] = {1U, 0x36U, 1U, 0U, 0U, 0x6CU, 0U, 0x6BU};
    EmmBusReceive(&bus, next, sizeof(next), 107U);
    /* Bytes arriving after transaction timeout must not refresh feedback. */
    CHECK(!bus.response_received); /* Enforced by Receive as well as Update. */

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    now = ReadHealth(&bus, motors, &transport, 5U);
    CHECK(EmmBusArm(&bus, now));
    CHECK(EmmMotorSetSpeedUnits(&motors[0], 50, 0, now + 199U));
    EmmBusUpdate(&bus, now + 200U);
    CHECK(!bus.motion_allowed); /* Refreshing one wheel cannot keep others alive. */
    CHECK(transport.frame[0] == 0U && transport.frame[1] == 0xFEU);

    CHECK(Init(&bus, motors, &transport, 0xFFFFFFF0U));
    Startup(&bus, &transport, 0xFFFFFFF0U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 0xFFFFFFF6U);
    transport.busy = false;
    EmmBusUpdate(&bus, 10U); /* Exact 20 ms deadline across uint32 wrap. */
    CHECK(bus.timeout_count == 1U && !bus.motion_allowed);
    CHECK(!bus.active && bus.quarantine_active && !bus.stop_pending);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 6U);
    transport.busy = false;
    const uint8_t bad[] = {1U, 0x36U, 0U, 0U, 0U, 0U, 0U, 0x55U};
    EmmBusReceive(&bus, bad, sizeof(bad), 7U);
    CHECK(!motors[0].feedback.position_valid && bus.protocol_error_count == 1U);
    /* Actual captured position response, not a fabricated legacy packet. */
    const uint8_t valid[] = {1U, 0x36U, 0U, 0U, 0U, 0U, 0U, 0x6BU};
    EmmBusReceive(&bus, valid, sizeof(valid), 8U);
    CHECK(motors[0].feedback.position_valid); /* Resync after bad tail. */
    EmmBusUpdate(&bus, 8U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 10U);
    transport.busy = false;
    const uint8_t quarter_turn[] = {1U, 0x36U, 0U, 0U, 0U, 0x40U, 0U, 0x6BU};
    EmmBusReceive(&bus, quarter_turn, 8U, 18U);
    CHECK(motors[0].feedback.speed_valid);
    CHECK(NEAR(motors[0].feedback.wheel_position_rad, 1.5707963f));
    CHECK(NEAR(motors[0].feedback.wheel_speed_rad_s, 157.07963f));
    EmmMotorFeedback feedback;
    CHECK(EmmMotorGetFeedback(&motors[0], 518U, &feedback));
    CHECK(!feedback.position_valid && !feedback.speed_valid);
    EmmBusTransportFault(&bus, 519U);
    CHECK(!motors[0].feedback.position_valid && bus.stop_pending && bus.disable_pending);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 6U);
    transport.busy = false;
    EmmBusUpdate(&bus, 26U); /* A query timeout alone no longer requests stop. */
    CHECK(!bus.stop_pending && bus.quarantine_active);
    EmmBusStop(&bus, true, 26U);
    EmmBusUpdate(&bus, 26U); /* Explicit stop must still bypass quarantine. */
    CHECK(transport.frame[1] == 0xFEU);
    transport.busy = false;
    ack[1] = 0xFEU;
    EmmBusReceive(&bus, ack, 4U, 27U);
    EmmBusUpdate(&bus, 27U);
    EmmBusUpdate(&bus, 29U);
    CHECK(transport.frame[1] == 0xF3U);
    transport.busy = false;
    ack[1] = 0xF3U;
    EmmBusReceive(&bus, ack, 4U, 30U);
    EmmBusUpdate(&bus, 30U);
    CHECK(EmmMotorQuery(&motors[1], EMM_QUERY_POSITION));
    sends = transport.sends;
    EmmBusReceive(&bus, valid, sizeof(valid), 31U); /* Late old reply: discard. */
    EmmBusUpdate(&bus, 65U);
    CHECK(transport.sends == sends); /* Safety ACK must not shorten quarantine. */
    EmmBusUpdate(&bus, 66U);
    CHECK(transport.frame[0] == 2U && transport.frame[1] == 0x36U);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 6U);
    transport.busy = false;
    const uint8_t before_wrap[] = {1U, 0x36U, 1U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x6BU};
    EmmBusReceive(&bus, before_wrap, 8U, 7U);
    CHECK(motors[0].feedback.position_raw == -4294967295LL);
    EmmBusUpdate(&bus, 7U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 9U);
    transport.busy = false;
    const uint8_t after_wrap[] = {1U, 0x36U, 0U, 0U, 0U, 0U, 0U, 0x6BU};
    EmmBusReceive(&bus, after_wrap, 8U, 17U);
    CHECK(motors[0].feedback.position_raw == 0);
    CHECK(!motors[0].feedback.speed_valid); /* Unknown counter wrap/reset is not speed. */

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    now = ReadHealth(&bus, motors, &transport, 5U);
    CHECK(EmmBusArm(&bus, now));
    CHECK(EmmMotorSetEnabled(&motors[0], true, now));
    EmmBusUpdate(&bus, now + 3U);
    const uint8_t enable[] = {1U, 0xF3U, 0xABU, 1U, 0U, 0x6BU};
    CHECK(transport.length == 6U && memcmp(transport.frame, enable, 6U) == 0);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_STATUS));
    EmmBusUpdate(&bus, 6U);
    transport.busy = false;
    const uint8_t error[] = {1U, 0U, 0xEEU, 0x6BU};
    EmmBusReceive(&bus, error, 4U, 7U);
    CHECK(!motors[0].feedback.enabled_valid);
    CHECK(bus.response_failed && bus.rejected_command_count == 1U);
    CHECK(bus.last_rejected_address == 1U && bus.last_rejected_function == 0x3AU &&
          bus.last_rejected_status == 0xEEU);
    EmmBusUpdate(&bus, 7U); /* Rejection fails immediately, without a timeout. */
    CHECK(bus.timeout_count == 0U);
    CHECK(motors[0].command_error_count == 1U && !bus.motion_allowed);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 6U);
    transport.busy = false;
    /* A four-byte rejection must terminate an eight-byte query immediately. */
    EmmBusReceive(&bus, error, 2U, 7U);
    CHECK(!bus.response_failed);
    EmmBusReceive(&bus, error + 2U, 2U, 7U);
    CHECK(bus.response_failed && !motors[0].feedback.position_valid);
    EmmBusUpdate(&bus, 7U);
    CHECK(bus.timeout_count == 0U && bus.rejected_command_count == 1U);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_POSITION));
    EmmBusUpdate(&bus, 6U);
    transport.busy = false;
    const uint8_t status[] = {1U, 0x3AU, 0x82U, 0x6BU}; /* Captured on target. */
    EmmBusReceive(&bus, status, sizeof(status), 7U);
    CHECK(!bus.response_received && !motors[0].feedback.position_valid);
    const uint8_t invalid_sign[] = {1U, 0x36U, 2U, 0U, 0U, 0U, 0U, 0x6BU};
    EmmBusReceive(&bus, invalid_sign, sizeof(invalid_sign), 7U);
    CHECK(!bus.response_received);
    const uint8_t legacy[] = {1U, 0U, 0U, 0U, 0U, 0x6BU};
    EmmBusReceive(&bus, legacy, sizeof(legacy), 7U);
    CHECK(!bus.response_received);
    EmmBusReceive(&bus, valid, sizeof(valid), 8U);
    CHECK(bus.response_received && motors[0].feedback.position_raw == 0);
    EmmBusUpdate(&bus, 8U);
    CHECK(EmmMotorQuery(&motors[0], EMM_QUERY_STATUS));
    EmmBusUpdate(&bus, 10U);
    transport.busy = false;
    EmmBusReceive(&bus, status, sizeof(status), 11U);
    CHECK(motors[0].feedback.enabled_valid && motors[0].feedback.stall_valid);
    CHECK(motors[0].feedback.status_raw == 0x82U && !motors[0].feedback.enabled &&
          !motors[0].feedback.stalled && !motors[0].feedback.stall_protection);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    now = ReadHealth(&bus, motors, &transport, 5U);
    CHECK(EmmBusArm(&bus, now));
    CHECK(EmmMotorQuery(&motors[3], EMM_QUERY_STATUS));
    EmmBusUpdate(&bus, now + 3U);
    transport.busy = false;
    const uint8_t protection[] = {4U, 0x3AU, 0x09U, 0x6BU};
    EmmBusReceive(&bus, protection, sizeof(protection), now + 4U);
    CHECK(motors[3].feedback.stall_protection && !motors[3].feedback.stalled);
    EmmBusUpdate(&bus, now + 4U);
    CHECK(!bus.motion_allowed && bus.disable_pending);
    CHECK(transport.length == sizeof(stop) && memcmp(transport.frame, stop, sizeof(stop)) == 0);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    now = ReadState(&bus, motors, &transport, 5U, 0x82U);
    CHECK(!bus.motion_allowed && !bus.active);
    sends = transport.sends;
    EmmBusUpdate(&bus, 250U);
    CHECK(transport.sends == sends); /* All four freshly confirmed disabled. */
    EmmBusUpdate(&bus, 500U);
    CHECK(transport.sends == sends); /* Still within every motor's 500 ms lease. */
    EmmBusUpdate(&bus, 750U);
    CHECK(transport.sends == sends + 1U); /* Stale status cannot suppress stop. */
    CHECK(memcmp(transport.frame, stop, sizeof(stop)) == 0);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    now = ReadHealth(&bus, motors, &transport, 5U);
    sends = transport.sends;
    EmmBusUpdate(&bus, 250U); /* No motion permit, but drivers still enabled. */
    CHECK(transport.sends == sends + 1U && memcmp(transport.frame, stop, sizeof(stop)) == 0);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    now = ReadState(&bus, motors, &transport, 5U, 0x82U);
    CHECK(EmmMotorQuery(&motors[3], EMM_QUERY_STATUS));
    EmmBusUpdate(&bus, now + 3U);
    transport.busy = false;
    const uint8_t last_enabled[] = {4U, 0x3AU, 0x83U, 0x6BU};
    EmmBusReceive(&bus, last_enabled, sizeof(last_enabled), now + 4U);
    EmmBusUpdate(&bus, now + 4U);
    sends = transport.sends;
    EmmBusUpdate(&bus, 250U);
    CHECK(transport.sends == sends + 1U); /* One enabled motor is enough. */
    CHECK(memcmp(transport.frame, stop, sizeof(stop)) == 0);

    CHECK(Init(&bus, motors, &transport, 0U));
    Startup(&bus, &transport, 0U);
    now = ReadState(&bus, motors, &transport, 5U, 0x82U);
    sends = transport.sends;
    EmmBusStop(&bus, true, now + 3U); /* Explicit stop is never suppressed. */
    EmmBusUpdate(&bus, now + 3U);
    CHECK(transport.sends == sends + 1U && bus.disable_pending);
    CHECK(memcmp(transport.frame, stop, sizeof(stop)) == 0);

    CHECK(Init(&bus, motors, &transport, 0U));
    EmmBusUpdate(&bus, 0U);
    transport.busy = false;
    const uint8_t disabled_stop[] = {1U, 0xFEU, 0xE2U, 0x6BU};
    EmmBusReceive(&bus, disabled_stop, sizeof(disabled_stop), 1U);
    CHECK(bus.response_failed && bus.rejected_command_count == 1U);
    CHECK(bus.last_rejected_function == 0xFEU && bus.last_rejected_status == 0xE2U);
    EmmBusUpdate(&bus, 1U);
    EmmBusUpdate(&bus, 41U); /* Failed startup stop must still be followed by disable. */
    CHECK(transport.length == sizeof(disable) && memcmp(transport.frame, disable, sizeof(disable)) == 0);
    transport.busy = false;
    ack[1] = 0xF3U;
    EmmBusReceive(&bus, ack, sizeof(ack), 42U);
    EmmBusUpdate(&bus, 42U);
    now = ReadState(&bus, motors, &transport, 45U, 0x82U);
    sends = transport.sends;
    EmmBusUpdate(&bus, 250U);
    CHECK(transport.sends == sends && bus.rejected_command_count == 1U);
    CHECK(bus.timeout_count == 0U); /* Historical rejection is preserved, not hidden. */
    return 0;
}
