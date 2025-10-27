#ifndef __USART_H__
#define __USART_H__

extern uint8_t RxData;

void USART_GPIO_Init(void);
void USART_Config(uint32_t bps);
void USART_SendByte(uint8_t Byte);
void USART_SendArray(int8_t *array, uint16_t length);
void Ano_SendData(int8_t *data, uint8_t len, uint8_t id);
void USART_SendString(char *str);
//uint8_t USART_ReceiveByte(void);

#endif
