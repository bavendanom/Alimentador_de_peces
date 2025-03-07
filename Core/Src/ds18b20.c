
#include "ds18b20.h"
#include <string.h>


uint8_t Presence;
uint8_t Temp_byte1;
uint8_t Temp_byte2;
uint8_t size_to_send;

void Set_Pin_Input (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	//HAL_GPIO_DeInit(GPIOx, GPIO_Pin);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Output (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	//HAL_GPIO_DeInit(GPIOx, GPIO_Pin);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


void delay (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);   // set the pin as output
	HAL_GPIO_WritePin (Temperature_sensor_GPIO_Port, Temperature_sensor_Pin, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (Temperature_sensor_GPIO_Port, Temperature_sensor_Pin))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = 0;

	delay (400); // 480 us delay totally.

	return Response;
}
void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);  // set as output
			HAL_GPIO_WritePin (Temperature_sensor_GPIO_Port, Temperature_sensor_Pin, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);  // set as input
			delay (60);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);
			HAL_GPIO_WritePin (Temperature_sensor_GPIO_Port, Temperature_sensor_Pin, 0);  // pull the pin LOW
			delay (60);  // wait for 60 us

			Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);
		}
	}
}
float convert_temperature(uint8_t byte_1, uint8_t byte_2 ){
	uint16_t tempval = byte_2 << 8 | byte_1;
	float result_temp = (128.0 / 2048)*tempval;

	return  result_temp;

}
uint8_t DS18B20_Read (void)
{
	uint8_t value=0;
	Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);;   // set as output

		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, 0);  // pull the data pin LOW
		delay (2);  // wait for 2 us

		Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);  // set as input
		delay (5);  // wait for 2 us
		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_3))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us
	}
	return value;
}

float DS18B20_ReadTemperature() {
	Presence = DS18B20_Start();
	HAL_Delay (1);
	DS18B20_Write (0xCC);  // skip ROM
	DS18B20_Write (0x44);  // convert t
	HAL_Delay (800);

	Presence = DS18B20_Start ();
	HAL_Delay(1);
	DS18B20_Write (0xCC);  // skip ROM
	DS18B20_Write (0xBE);  // Read Scratch-pad

	Temp_byte1 = DS18B20_Read();
	Temp_byte2 = DS18B20_Read();

	//HAL_UART_Transmit(&huart1,(uint8_t *) "read \r\n", 9, 1000);
	//size_to_send = sprintf( transmit_text, "presence %d el primer %d y el segundo %d \r\n", Presence,Temp_byte1,Temp_byte2);
	//transmit_text[size_to_send] = '\0';
	//HAL_UART_Transmit(&huart1, (uint8_t *)transmit_text, size_to_send, 1000);
	float temp_dec = convert_temperature(Temp_byte1 , Temp_byte2);
	//size_to_send = sprintf( transmit_text, "result temperature %.2f\r\n", temp_dec );
	//transmit_text[size_to_send] = '\0';
	//HAL_UART_Transmit(&huart1, (uint8_t *)transmit_text, size_to_send, 1000);
	return temp_dec;
}
