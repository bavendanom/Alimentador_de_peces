#ifndef DS18B20_H
#define DS18B20_H

#include <stdint.h>
#include "main.h"
#include "stm32f0xx_hal.h"

extern TIM_HandleTypeDef htim1;


// Prototipos de funciones públicas
//void DS18B20_Init();
uint8_t DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);
float DS18B20_ReadTemperature();

#endif
