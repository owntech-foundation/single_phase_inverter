/* Host tests of singlePhaseInverter: modulation duties and PI parameters */
#include <stdio.h>
#include <math.h>
#include "singlePhaseInverter.h"

static int failures = 0;
#define CHECK(cond, ...) do {                       \
        bool ok_ = (cond);                          \
        printf(ok_ ? "  ok   " : "  FAIL ");        \
        if (!ok_) failures++;                       \
        printf(__VA_ARGS__);                        \
        printf("\n");                               \
    } while (0)

static const float32_t V_BUS = 400.0F;
static const float32_t V_PK = 325.0F;
static const float32_t W0 = 2.0F * PI * 50.0F;
static const float32_t TS = 100e-6F;

/* Runs FORMING open on a resistive-like measurement for n steps and checks
 * the leg 2 duty against leg 1 at every step */
static void check_modulation(modulation_scheme mod, const char *name)
{
    singlePhaseInverter inv;
    inv.init(FORMING, mod, V_BUS, V_PK, W0, TS);
    dqo_t vref = {200.0F, 0.0F, 0.0F};
    inv.setVdqRef(vref);

    float32_t worst_rel = 0.0F, worst_avg = 0.0F, dmin = 1.0F, dmax = 0.0F;
    for (int k = 0; k < 2000; k++)
    {
        float32_t v = 150.0F * sinf(W0 * k * TS);
        float32_t d1 = inv.calculateDuty(v, v / 50.0F);
        float32_t d2 = inv.getDutyLeg2();
        /* high-side on-time fraction of leg 2: buck = d2, boost = 1 - d2 */
        float32_t f2 = (mod == UNIPOLAR) ? d2 : 1.0F - d2;
        float32_t rel = (mod == UNIPOLAR) ? fabsf(d1 + d2 - 1.0F) : fabsf(d1 - d2);
        float32_t avg = fabsf((d1 - f2) - (2.0F * d1 - 1.0F));
        if (rel > worst_rel) worst_rel = rel;
        if (avg > worst_avg) worst_avg = avg;
        if (d1 < dmin) dmin = d1;
        if (d1 > dmax) dmax = d1;
    }
    printf("  %s: d1 in [%.3f, %.3f]\n", name, (double)dmin, (double)dmax);
    CHECK(inv.getModulation() == mod, "%s: getModulation()", name);
    CHECK(worst_rel < 1e-5F, "%s: %s (max error %.2e)", name,
          mod == UNIPOLAR ? "d1 + d2 = 1" : "d1 = d2", (double)worst_rel);
    CHECK(worst_avg < 1e-5F, "%s: leg average (f1 - f2) = 2 d1 - 1 (max error %.2e)",
          name, (double)worst_avg);
    CHECK(dmax - dmin > 0.2F, "%s: duty actually varies", name);
}

/* Saturates both loops (voltage measured 0, reference 200 V) and returns the
 * largest current-loop correction |Vd_out - Vd_ref| */
static float32_t saturated_correction(singlePhaseInverter &inv)
{
    dqo_t vref = {200.0F, 0.0F, 0.0F};
    inv.setVdqRef(vref);
    float32_t worst = 0.0F;
    for (int k = 0; k < 5000; k++)
    {
        inv.calculateDuty(0.0F, 0.0F);
        float32_t c = fabsf(inv.getVdqOut().d - vref.d);
        if (c > worst) worst = c;
    }
    return worst;
}

int main()
{
    printf("== modulation\n");
    check_modulation(UNIPOLAR, "UNIPOLAR");
    check_modulation(BIPOLAR, "BIPOLAR");

    printf("== PI parameters\n");
    singlePhaseInverter inv;
    inv.init(FORMING, UNIPOLAR, V_BUS, V_PK, W0, TS);
    float32_t c_default = saturated_correction(inv);
    CHECK(fabsf(c_default - 30.0F) < 1e-3F, "default current PI bound 30 V (got %.3f)",
          (double)c_default);

    inv.setCurrentPi(0.001F, 0.0003F, 5.0F);
    float32_t c_custom = saturated_correction(inv);
    CHECK(fabsf(c_custom - 5.0F) < 1e-3F, "setCurrentPi bound 5 V (got %.3f)",
          (double)c_custom);

    inv.setMode(FOLLOWING);
    inv.setMode(FORMING);
    float32_t c_after = saturated_correction(inv);
    CHECK(fabsf(c_after - 5.0F) < 1e-3F, "bound kept after setMode() (got %.3f)",
          (double)c_after);

    printf("%d failure(s)\n", failures);
    return failures != 0;
}
