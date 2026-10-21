/*
All functions in the gamma file return

*/
#ifndef CORRECTIONS_H
#define CORRECTIONS_H

void initialiseCorrections(void);
uint16_t correctionsFuel(void);
uint8_t calculateAfrTarget(table3d16RpmLoad &afrLookUpTable, const statuses &current, const config2 &page2, const config6 &page6);

int8_t correctionsIgn(int8_t advance);
int8_t correctionFixedTiming(int8_t advance);
int8_t correctionCrankingFixedTiming(int8_t advance);

uint16_t correctionsDwell(uint16_t dwell);

/** @brief Fork specific: the AFR closed loop is enabled and in its operating window (temperature, RPM, TPS, MAP...) */
bool isAfrClosedLoopActive(void);

/** @brief Fork specific: hand the AFR correction back to the PID after the auto tune, without a jump */
void egoTuneHandBack(uint8_t correction);


#endif // CORRECTIONS_H
