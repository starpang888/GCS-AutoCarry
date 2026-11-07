#include "host_protocol.h"
#include "bsp_usb.h"
#include "vofa_protocol.h"
#include <string.h>
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
static HostCommand last;
static unsigned calls;
static const uint8_t *owned;
static uint16_t owned_length;
static uint8_t usb_result;
uint32_t test_usb_clock;
uint32_t HAL_GetTick(void) { return test_usb_clock; }
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length)
{
    owned = data; owned_length = length; return usb_result;
}
static void Command(void *context, const HostCommand *command)
{
    (void)context; last = *command; ++calls;
}
int RunHostLinkTests(void)
{
    const int64_t channels[] = {INT64_MIN, -1, 1234567, UINT32_MAX};
    const char expected[] = "-9223372036854775808,-0.001,1234.567,4294967295\r\n";
    uint8_t ascii[128];
    size_t ascii_length = VofaFireWaterEncode(channels, 4, 6, ascii, sizeof(ascii));
    CHECK(ascii_length == sizeof(expected) - 1 && !memcmp(ascii, expected, ascii_length));
    memset(ascii, 0xCC, sizeof(ascii));
    CHECK(!VofaFireWaterEncode(channels, 4, 6, ascii, 5) && ascii[5] == 0xCC);
    CHECK(!VofaFireWaterEncode(channels, 33, 0, ascii, 128));
    CHECK(HostProtocolCRC((const uint8_t *)"123456789", 9) == 0x29B1);
    HostProtocolInstance parser;
    HostProtocolInit(&parser, Command, NULL);
    uint8_t payload[12], frame[24];
    HostPutU32(payload, (uint32_t)-50);
    HostPutU32(payload + 4, 25);
    HostPutU32(payload + 8, (uint32_t)-300);
    CHECK(HostProtocolEncode(HOST_VELOCITY, 0xFFFFFFFF, payload, 12, frame, 24) == 24);
    const uint8_t golden[] = {0xA5,0x5A,1,1,12,0,255,255,255,255,206,255,
        255,255,25,0,0,0,212,254,255,255,0x47,0x91};
    CHECK(!memcmp(frame, golden, sizeof(golden))); /* Shared Python/C wire vector. */
    for (unsigned i = 0; i < 3; ++i) HostProtocolReceive(&parser, 0xA5, 0xFFFFFFFA);
    for (unsigned i = 0; i < 24; ++i) HostProtocolReceive(&parser, frame[i], (uint32_t)(0xFFFFFFFA + i));
    CHECK(calls == 1 && last.sequence == 0xFFFFFFFF && last.vx_mm_s == -50 &&
          last.vy_mm_s == 25 && last.wz_mrad_s == -300 && last.received_ms == 17);
    frame[10] ^= 1;
    for (unsigned i = 0; i < 24; ++i) HostProtocolReceive(&parser, frame[i], 30);
    CHECK(calls == 1 && parser.crc_errors == 1);
    /* Unbounded hostile length/noise cannot overrun the fixed 24-byte parser. */
    const uint8_t bad[] = {0xA5, 0x5A, 1, 1, 0xFF, 0xFF};
    for (unsigned n = 0; n < 100; ++n)
        for (unsigned i = 0; i < sizeof(bad); ++i) HostProtocolReceive(&parser, bad[i], 31);
    size_t length = HostProtocolEncode(HOST_STOP, 7, NULL, 0, frame, 24);
    HostProtocolReceive(&parser, 0xA5, 32);
    for (unsigned i = 0; i < length; ++i) HostProtocolReceive(&parser, frame[i], 132);
    CHECK(calls == 2 && last.type == HOST_STOP && last.sequence == 7 && parser.count == 0);
    CHECK(!HostProtocolEncode(HOST_STOP, 0, NULL, 0, frame, 11));

    BSPUSBConnected(true);
    BSPUSBStatus before, after;
    BSPUSBGetStatus(&before);
    uint8_t bytes[512];
    for (unsigned i = 0; i < sizeof(bytes); ++i) bytes[i] = (uint8_t)i;
    BSPUSBReceive(bytes, 512, 123);
    BSPUSBByte read[32];
    for (unsigned batch = 0; batch < 16; ++batch)
    {
        CHECK(BSPUSBRead(read, 32) == 32);
        for (unsigned i = 0; i < 32; ++i)
            CHECK(read[i].byte == (uint8_t)(batch * 32 + i) && read[i].arrival_ms == 123);
    }
    CHECK(BSPUSBRead(read, 32) == 0);
    BSPUSBReceive(bytes, 510, 124);
    BSPUSBReceive(bytes, 3, 125);
    BSPUSBGetStatus(&after);
    CHECK(after.overflows == before.overflows + 1 && after.epoch == before.epoch + 1);
    CHECK(BSPUSBRead(read, 32) == 0);
    CHECK(BSPUSBWrite(bytes, 12) && owned != bytes && owned_length == 12);
    bytes[0] = 99;
    CHECK(owned[0] == 0 && !BSPUSBWrite(bytes, 12));
    BSPUSBConnected(false);
    CHECK(!BSPUSBWrite(bytes, 12));
    BSPUSBResume();
    CHECK(!BSPUSBWrite(bytes, 12)); /* Resume must retain pending IN storage. */
    BSPUSBTxComplete();
    CHECK(BSPUSBWrite(bytes, 12) && owned[0] == 99);
    BSPUSBConnected(false);
    BSPUSBConnected(true); /* Reconfiguration cancels old ownership. */
    usb_result = 1;
    CHECK(!BSPUSBWrite(bytes, 12));
    usb_result = 0;
    CHECK(BSPUSBWrite(bytes, 12));
    BSPUSBTxComplete();
    return 0;
}
