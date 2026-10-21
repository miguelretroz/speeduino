#pragma once

#include <stdint.h>
#include "PidCore.h"

class PID
{
public:

  /** @brief Default construction */
  PID(void);

  /** @name Configuration methods */
  ///@{

  /** @brief Set the output limits */
  void setOutputLimits(int32_t min, int32_t max);

  /**
   * @brief Set the PID parameters
   *
   * @param params P, I & D terms
   * @param bumpless Keep the integral term unchanged, so a new I gain doesn't make the output jump
   */
  void setTunings(PidTuningParameters params, bool bumpless = false);

  /** @brief Set the controller set point */
  void setSetPoint(uint16_t setpoint) { _setpoint = setpoint; }
  ///@}

  /**
   * @brief Initialize/reset the controller
   * 
   * @param input Current input value (same value as passed into compute())
   */
	void resetIntegral(int32_t input);

  /**
   * @brief Compute the next correction.
   * 
   * @param input The input value
   * @return A correction to be applied to the input.
   */
  int32_t compute(int32_t input);

  /**
   * @brief Make the next compute() return the desired correction, for an unchanged input
   *
   * Hands control back to the controller without a jump, after something else has driven the output.
   *
   * @param desiredOutput The correction the next compute() should return
   * @param input The current input value
   */
  void presetOutput(int32_t desiredOutput, int32_t input);

private:
  int16_t _setpoint = 0;
  int32_t _integralTerm;
  int32_t _lastInput;
  PidCore _pidCore;
};

