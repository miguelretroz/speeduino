#include "PidCore.h"
#include "../../maths.h"

void PidCore::presetIntegral(int32_t desiredOutput, int32_t feedForwardTerm, int32_t error)
{
   if (_pidParams.Ki == 0)
   {
      _integralTerm = 0;
   }
   else
   {
      // compute() adds the error to the integral before using it
      int32_t integralTerm = desiredOutput - feedForwardTerm - ((int32_t)_pidParams.Kp * error);
      _integralTerm = DIV_ROUND_CLOSEST(integralTerm, (int32_t)_pidParams.Ki, int32_t) - error;
   }
}

void PidCore::rescaleIntegral(int16_t oldKi, int16_t newKi)
{
   if (oldKi == newKi)
   {
      // Nothing to do
   }
   else if ((oldKi == 0) || (newKi == 0))
   {
      // Either the integral had no effect on the output, or it won't have any
      _integralTerm = 0;
   }
   else
   {
      // Limit the integral so the product (plus the rounding) can't overflow. The output clamping keeps it well inside this anyway.
      const int32_t absOldKi = (oldKi < 0) ? -(int32_t)oldKi : (int32_t)oldKi;
      const int32_t limit = (INT32_MAX / 2) / absOldKi;
      const int32_t integral = (_integralTerm > limit) ? limit : ((_integralTerm < -limit) ? -limit : _integralTerm);
      _integralTerm = DIV_ROUND_CLOSEST(integral * oldKi, (int32_t)newKi, int32_t);
   }
}

int32_t PidCore::compute(int32_t feedForwardTerm, int32_t error, int32_t derivative)
{
   _integralTerm += error;

   /*Compute PID Output*/
   int32_t output = feedForwardTerm + _pidParams.compute(error, _integralTerm, derivative);

   /* Clamp output and back-calculate integral if necessary */
   if (output > _max)
   {
       output = _max;
       _integralTerm -= error;
   }
   else if (output < _min)
   {
       output = _min;
       _integralTerm -= error;
   }   
   else
   {
        // Nothing to do, but keep the static checker happy
   }

   return output;
}