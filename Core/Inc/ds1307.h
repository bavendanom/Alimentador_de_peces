#ifndef DS1307_H
#define DS1307_H

#include <stdint.h>
#include "main.h"
#include "stm32f0xx_hal.h"

#define DS1307_ADDRESS 0xD0

// Conversión decimal a BCD
uint8_t decToBcd(int val);

// Conversión BCD a decimal
int bcdToDec(uint8_t val);

// Estructura para manejar el tiempo
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofweek;
    uint8_t dayofmonth;
    uint8_t month;
    uint8_t year;
} TIME;

extern TIME time;  // Variable global para acceder a los datos del tiempo

// Funciones para establecer/obtener el tiempo
void Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);
void Get_Time(void);

#endif
