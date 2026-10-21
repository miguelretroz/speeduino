#ifndef IDLE_H
#define IDLE_H

#include <stdint.h>

void initialiseIdle(bool forcehoming);
void idleControl(void);
void disableIdle(void);
void idleInterrupt(void);

/** @brief Fork specific: the closed loop idle state the PID auto tune works from */
struct idleTuneInfo_t
{
  int32_t output;     ///< Current PID output (steps x4, or PWM timer counts x4)
  int32_t outputMin;  ///< PID output limits
  int32_t outputMax;
  int32_t fullScale;  ///< PWM valves: the PID output at 100% duty
  bool isReady;       ///< Closed loop idle is running: engine running, initialised, homed and done tapering
  bool isStepper;
  bool isVelocityForm; ///< Stepper closed loop: the PID works on the change of the output
};

/** @brief Fork specific: the closed loop idle state the PID auto tune works from */
idleTuneInfo_t getIdleTuneInfo(void);

/** @brief Fork specific: hand the valve back to the idle PID after the auto tune, without a jump */
void idleTuneHandBack(int32_t output);

#endif
