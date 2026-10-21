#pragma once
#include "PidTuningParameters.h"

/** @brief A base class for %PID implementations
 * 
 * Provides a shared %PID algorithm. @see compute.
 */
class PidCore
{
public:
    PidCore(void) = default;

    /** @brief Set the output limits */
    void setOutputLimits(int32_t min, int32_t max) {
        if(min >= max) return;
        _min = min;
        _max = max;
    }

    /**
     * @brief Set the PID parameters
     *
     * @param params The new parameters
     * @param bumpless Rescale the integral so the integral term (Ki * integral) is unchanged.
     * Otherwise a new Ki makes the output jump.
     */
    void setTunings(const PidTuningParameters& params, bool bumpless = false) {
        if (bumpless) { rescaleIntegral(_pidParams.Ki, params.Ki); }
        _pidParams = params;
    }

    /** @brief (Optional) Reset the integral term */
    void resetIntegral(void) {
        _integralTerm = 0;
    }

    /**
     * @brief Preset the integral, so the next compute() returns the desired output (before clamping)
     *
     * Assumes the next compute() gets the same error and a zero derivative (I.e. an unchanged input).
     * With a zero Ki the integral is just reset, as it has no effect on the output.
     */
    void presetIntegral(int32_t desiredOutput, int32_t feedForwardTerm, int32_t error);

    /** @brief Compute the next correction. */
    int32_t compute(int32_t feedForwardTerm, int32_t error, int32_t derivative);

#if !defined(UNIT_TEST)
private:
#endif

    void rescaleIntegral(int16_t oldKi, int16_t newKi);

    PidTuningParameters _pidParams;

    int32_t _integralTerm = 0;
    // TODO: shrink these to int16_t if possible
    int32_t _min = 0;
    int32_t _max = 0;
};