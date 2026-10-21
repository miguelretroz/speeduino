#include "PID.h"

PID::PID(void)
{
	setOutputLimits(0, 255);
}

int32_t PID::compute(int32_t input)
{
   // We are using "Derivative on Measurement" as described [here](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/)
   int32_t output = _pidCore.compute(0L, _setpoint - input, _lastInput - input);

   /*Remember some variables for next time*/
   _lastInput = input;

   return output/1000;
}

void PID::setTunings(PidTuningParameters pidParams, bool bumpless)
{
  pidParams.Kd = pidParams.Kd * 10;
  _pidCore.setTunings(pidParams, bumpless);
}

void PID::setOutputLimits(int32_t Min, int32_t Max)
{
   _pidCore.setOutputLimits(Min*1000, Max*1000);
}

void PID::resetIntegral(int32_t input)
{
   _pidCore.resetIntegral();
   _lastInput = input;
}

void PID::presetOutput(int32_t desiredOutput, int32_t input)
{
   _lastInput = input;
   // compute() truncates towards zero, so aim for the middle of the output step
   const int32_t halfStep = (desiredOutput < 0) ? -500L : ((desiredOutput > 0) ? 500L : 0L);
   _pidCore.presetIntegral((desiredOutput * 1000L) + halfStep, 0L, _setpoint - input);
}