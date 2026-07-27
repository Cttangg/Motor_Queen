#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "ti_msp_dl_config.h"

void motor_init(uint8_t motorID);
void motor_set_duty(uint8_t motorID, uint32_t duty);
void motor_set_direction(uint8_t motorID, uint8_t dir);  /* 0=正向 1=反向 */

#endif
