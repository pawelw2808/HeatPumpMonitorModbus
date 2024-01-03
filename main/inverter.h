#pragma once

#include "main.h"

#define INVERTER_INIT_SUCCESS 0
#define INVERTER_INIT_FAILED 1


int INV_InverterInit(void);
int INV_TurnOffMotor(void);
int INV_TurnOnMotor(int direction);
int INV_SetFreqValue(uint8_t new_freq_value);
int INV_ChangeDirection(void);
int INV_ChangeDirectionDuringPause(void);
int INV_RpmUp(uint8_t rpm_to_increase);
int INV_RpmDown(uint8_t rpm_to_decrease);
int INV_GetDirection(void);
void delay_between_commands(void);

