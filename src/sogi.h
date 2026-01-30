#ifndef SOGI_H
#define SOGI_H

#include "pid.h"
#ifndef __arm__
#include "my_types.h"
#else
#include "arm_math.h"
#endif

#include "transform.h"

/**
 * @brief Class implementing the SOGI-PLL (Second-Order Generalized Integrator Phase-Locked Loop) system.
 */
class Sogi {
public:
    /**
     * @brief Constructor for the SogiPLL class.
     */
    Sogi();

    /**
     * @brief Initializes the SOGI parameters.
     *
     * @param Kr Resonance gain.
     * @param Ts Sampling time.
     */
    void init(float32_t Kr, float32_t Ts);

    /**
     * @brief Performs SOGI calculation.
     *
     * @param input Input signal.
     * @param w0 Angular frequency.
     * @return Clarke transformation result of the input signal.
     */
    clarke_t calc(float32_t input, float32_t w0);


private:

    float32_t _num[2]; ///< Numerator coefficients of the SOGI filter.
    float32_t _den[3]; ///< Denominator coefficients of the SOGI filter.
    float32_t _Ts;     ///< Sampling time.
    float32_t _Kr;     ///< Resonance gain.

};

#endif // SOGI_H