
#include "main.h"
#include "ibus.h"
#include "filters.h"
#include "math.h"


uint8_t ibus_ptr;
pt1Filter_t roll_flt1,pitch_flt1,yaw_flt1;

void configure_ibus(UART_HandleTypeDef *huart)
{
	init_pt1Filter(&roll_flt1,23,0.007);
	init_pt1Filter(&pitch_flt1,23,0.007);
	init_pt1Filter(&yaw_flt1,23,0.007);
while (ibus[0] != ((0x20) | (0x40 << 8)))
{
	HAL_UART_Receive(huart,&ibus, 3,100);
	//printf("%i 	%i\r\n",ibus[0], (int16_t)((0x20 << 8) | 0x40));
	//HAL_Delay(1);
}
//HAL_UART_Receive(huart,&ibus, 32,1000);
//HAL_Delay(1);
HAL_UART_Receive(huart,&ibus, (IBUS_BUFFER_LEN-3),100);
HAL_UART_Receive_DMA(huart,&ibus, IBUS_BUFFER_LEN);
}

void ibus_callback(void)
{
InputR = pt1FilterApply(&roll_flt1,(((float)ibus[2] - 1500)*(M_PI/2500.0f)));
InputP = pt1FilterApply(&pitch_flt1,(((float)ibus[1] - 1500)*(M_PI/2500.0f)));
InputY = pt1FilterApply(&yaw_flt1,(((float)ibus[4] - 1500))*8);
InputT = ((float)ibus[3] - 1000);
}
