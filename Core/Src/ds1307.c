#include "ds1307.h"

extern I2C_HandleTypeDef hi2c2;  // Declaración externa del manejador I2C (debe estar definido en tu proyecto)

TIME time;  // Definición de la variable global

// Implementación de conversión decimal a BCD
uint8_t decToBcd(int val) {
    return (uint8_t)((val / 10 * 16) + (val % 10));
}

// Implementación de conversión BCD a decimal
int bcdToDec(uint8_t val) {
    return (int)((val / 16 * 10) + (val % 16));
}

// Función para configurar el tiempo
void Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year) {
    uint8_t set_time[7] = {
        decToBcd(sec),
        decToBcd(min),
        decToBcd(hour),
        decToBcd(dow),
        decToBcd(dom),
        decToBcd(month),
        decToBcd(year)
    };
    HAL_I2C_Mem_Write(&hi2c2, DS1307_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

// Función para leer el tiempo
void Get_Time(void) {
    uint8_t get_time[7];
    HAL_I2C_Mem_Read(&hi2c2, DS1307_ADDRESS, 0x00, 1, get_time, 7, 1000);
    time.seconds = bcdToDec(get_time[0]);
    time.minutes = bcdToDec(get_time[1]);
    time.hour = bcdToDec(get_time[2]);
    time.dayofweek = bcdToDec(get_time[3]);
    time.dayofmonth = bcdToDec(get_time[4]);
    time.month = bcdToDec(get_time[5]);
    time.year = bcdToDec(get_time[6]);
}
