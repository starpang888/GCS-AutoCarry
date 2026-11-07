#include "omni_kinematics.h"
#include <float.h>
#include <math.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static bool Near(float a, float b)
{
    return isfinite(a) && isfinite(b) && fabsf(a - b) < 0.00002f;
}

int RunOmniKinematicsTests(void)
{
    OmniKinematicsInstance model;
    OmniKinematics_Init_Config_s config = {
        .wheel_radius_m = 0.0375f, .wheelbase_m = 0.240f,
        .track_width_m = 0.240f,
    };
    CHECK(OmniKinematicsInit(&model, &config));
    OmniKinematicsTarget target;
    OmniBodyVelocity velocity = {.vx_m_s = 0.1f};
    CHECK(OmniKinematicsInverse(&model, &velocity, &target));
    CHECK(target.valid);
    CHECK(Near(target.wheel_rad_s[0], -1.885618083f));
    CHECK(Near(target.wheel_rad_s[1], 1.885618083f));
    CHECK(Near(target.wheel_rad_s[2], 1.885618083f));
    CHECK(Near(target.wheel_rad_s[3], -1.885618083f));
    velocity = (OmniBodyVelocity){.vy_m_s = 0.1f};
    CHECK(OmniKinematicsInverse(&model, &velocity, &target));
    CHECK(Near(target.wheel_rad_s[0], 1.885618083f));
    CHECK(Near(target.wheel_rad_s[1], 1.885618083f));
    CHECK(Near(target.wheel_rad_s[2], -1.885618083f));
    CHECK(Near(target.wheel_rad_s[3], -1.885618083f));
    velocity = (OmniBodyVelocity){.wz_rad_s = 0.5f};
    CHECK(OmniKinematicsInverse(&model, &velocity, &target));
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        CHECK(Near(target.wheel_rad_s[i], 2.2627417f));
    }

    /* Independent wheel-center velocity projection using geometric angles;
     * unequal dimensions catch accidental use of full rather than half spans.
     */
    config.wheelbase_m = 0.30f;
    config.track_width_m = 0.20f;
    CHECK(OmniKinematicsInit(&model, &config));
    const float angle[] = {2.35619449f, 0.785398163f, -0.785398163f, -2.35619449f};
    const float x[] = {0.15f, 0.15f, -0.15f, -0.15f};
    const float y[] = {0.10f, -0.10f, -0.10f, 0.10f};
    for (int n = -4; n <= 4; ++n)
    {
        velocity = (OmniBodyVelocity){.vx_m_s = n * 0.031f,
            .vy_m_s = (n + 1) * -0.021f, .wz_rad_s = (n - 1) * 0.15f};
        CHECK(OmniKinematicsInverse(&model, &velocity, &target));
        for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
        {
            float local_x = velocity.vx_m_s - velocity.wz_rad_s * y[i];
            float local_y = velocity.vy_m_s + velocity.wz_rad_s * x[i];
            float projected = local_x * cosf(angle[i]) + local_y * sinf(angle[i]);
            CHECK(Near(target.wheel_rad_s[i] * config.wheel_radius_m, projected));
        }
        OmniBodyVelocity recovered;
        CHECK(OmniKinematicsForward(&model, target.wheel_rad_s, &recovered));
        CHECK(Near(recovered.vx_m_s, velocity.vx_m_s));
        CHECK(Near(recovered.vy_m_s, velocity.vy_m_s));
        CHECK(Near(recovered.wz_rad_s, velocity.wz_rad_s));
    }
    /* Forward basis from independent wheel observations, no target clipping. */
    const float wheels[] = {-1.0f, 1.0f, 1.0f, -1.0f};
    OmniBodyVelocity recovered;
    CHECK(OmniKinematicsForward(&model, wheels, &recovered));
    CHECK(Near(recovered.vx_m_s, 0.0530330086f));
    CHECK(Near(recovered.vy_m_s, 0.0f) && Near(recovered.wz_rad_s, 0.0f));

    /* Mixed targets above the old 20 RPM ceiling must retain all three axes. */
    velocity = (OmniBodyVelocity){.vx_m_s = 0.5f, .vy_m_s = -0.2f, .wz_rad_s = 1.0f};
    CHECK(OmniKinematicsInverse(&model, &velocity, &target));
    float peak = 0.0f;
    for (size_t i = 0; i < OMNI_WHEEL_COUNT; ++i)
    {
        peak = fmaxf(peak, fabsf(target.wheel_rad_s[i]));
    }
    CHECK(peak > 2.09439510239f);
    CHECK(OmniKinematicsForward(&model, target.wheel_rad_s, &recovered));
    CHECK(Near(recovered.vx_m_s, velocity.vx_m_s));
    CHECK(Near(recovered.vy_m_s, velocity.vy_m_s));
    CHECK(Near(recovered.wz_rad_s, velocity.wz_rad_s));
    CHECK(Near(target.applied.vx_m_s, velocity.vx_m_s));
    CHECK(Near(target.applied.vy_m_s, velocity.vy_m_s));
    CHECK(Near(target.applied.wz_rad_s, recovered.wz_rad_s));

    velocity = (OmniBodyVelocity){0};
    CHECK(OmniKinematicsInverse(&model, &velocity, &target));
    CHECK(target.wheel_rad_s[0] == 0.0f);
    velocity.vx_m_s = NAN;
    CHECK(!OmniKinematicsInverse(&model, &velocity, &target));
    CHECK(!target.valid && target.wheel_rad_s[0] == 0.0f);
    velocity = (OmniBodyVelocity){.vx_m_s = FLT_MAX, .vy_m_s = FLT_MAX};
    CHECK(!OmniKinematicsInverse(&model, &velocity, &target));
    CHECK(!target.valid);
    float invalid_wheels[] = {1.0f, 2.0f, INFINITY, 4.0f};
    CHECK(!OmniKinematicsForward(&model, invalid_wheels, &recovered));
    CHECK(recovered.vx_m_s == 0.0f && recovered.wz_rad_s == 0.0f);
    CHECK(!OmniKinematicsInverse(NULL, &velocity, &target));
    CHECK(!OmniKinematicsForward(&model, NULL, &recovered));
    config.wheel_radius_m = 0.0f;
    CHECK(!OmniKinematicsInit(&model, &config));
    CHECK(!model.initialized);
    config.wheel_radius_m = NAN;
    CHECK(!OmniKinematicsInit(&model, &config));
    config.wheel_radius_m = 0.0375f;
    config.track_width_m = -1.0f;
    CHECK(!OmniKinematicsInit(&model, &config));
    return 0;
}
