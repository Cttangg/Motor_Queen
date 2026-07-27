#ifndef GRAYSCALE_H_
#define GRAYSCALE_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

void Grayscale_Init(void);
uint8_t Grayscale_Read(void);
void Grayscale_PrintBinary8(uint8_t value);

#endif /* GRAYSCALE_H_ */
