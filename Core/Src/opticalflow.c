#include "main.h"
#include "opticalflow.h"
#include "stm32f7xx_hal_def.h"


UART_HandleTypeDef *optiflow_huart;



void enable_opticalflow (UART_HandleTypeDef *huart)
{
	optiflow_huart = huart;

HAL_UART_Receive_DMA(optiflow_huart, &opticalflow.message.start,7);
}

void optical_flow_callback(UART_HandleTypeDef *huart)
{

		//if(((huart->pRxBuffPtr)-7) != &opticalflow.message.start)
		//{
		//	HAL_UART_Receive_DMA(optiflow_huart, &opticalflow.message.start,7);
		//}
		//else
		//{
			switch(opticalflow.message.type)
			{
			case 0x01:
			HAL_UART_Receive_DMA(optiflow_huart,&opticalflow.lidar.start,7);
			opticalflow.message.type = 0;
			break;

			case 0x02:
			HAL_UART_Receive_DMA(optiflow_huart,&opticalflow.flow.start,11);
			opticalflow.message.type = 0;
			break;

			default:
			HAL_UART_Receive_DMA(optiflow_huart, &opticalflow.message.start,7);

			}

		//}

}

