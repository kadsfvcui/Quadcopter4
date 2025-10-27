#include "stm32f4xx.h"                  // Device header
#include "Service.h"

extern int8_t data[100];

int main()
{
	ALL_Init();
//	Ano_SendData(data, 7, 0x03);
	
	while(1)
	{
//		test();
		Madgwick_Test();             

	}
}			
