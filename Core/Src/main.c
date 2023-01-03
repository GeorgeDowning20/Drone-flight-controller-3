/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "stdbool.h"
#include "filters.h"
#include "ibus.h"
#include "Baro.h"
#include "imu.h"
#include "AHRS.h"
#include "opticalflow.h"
#include "AERS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


uint16_t failsafe;

uint16_t i = 0;
uint16_t debug;
uint16_t debug2;



float InputfltR,InputfltP,dinputR,dinputP;


uint16_t in_pitch, in_roll, in_throttle, in_yaw;
uint8_t MSG[100] = {'\0'};



uint16_t motor1_out,motor2_out,motor3_out,motor4_out;

#define P 0
#define I 1
#define D 2
#define D2 3
#define R 1
#define Y 2
float ERR[3][4];
float err_last[2];
float K[3][4]= {
	{17000, 0, 2.5 ,12},	//pitch PIDD2  p:50 d:3000
	{17000, 0, 2.5, 12},	//roll PIDD2
	{0.8, 0, 0, 0},	//Yaw PIDD2
};
float roll_factor, pitch_factor, yaw_factor,input_throttle;

bool safety_flag;



biquadFilter_t alt_biquad,delta_alt_biquad;



uint8_t buffer;




//struct filterstruct filter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void init_filter (biquadFilter_t *filter);
float biquadFilterApply(biquadFilter_t *filter,float input);
float biquadFilterApplyDF1(float input);
void check_arm_satus (void);


#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);



  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //uint8_t UART4_rxBuffer[12] = {0};

 // UART_HandleTypeDef huart4;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  configure_ibus(&huart2);


 while (opticalflow.lidar.distance == 0)
 {
	 enable_opticalflow(&huart1);
	 HAL_Delay(10);
 }

  init_biquad_filter(&delta_alt_biquad,12.0f,4000.0f);

  HAL_UART_Receive_IT(&huart4, &buffer, 1);
//  HAL_UART_Receive_IT(&huart3, &opticalflow.message.start,7);

 // HAL_UART_Init(&huart3);
 // HAL_Delay(100);
  //enable_opticalflow(&huart3);



 configure_imu (&hspi1,GPIOB,GPIO_PIN_2);

 configure_baro(&hi2c1);

 // calabrate_acc_gyr(&IMU);

  //HAL_Delay(10000);
 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
HAL_TIM_Base_Start_IT(&htim2);

HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

HAL_TIM_Base_Start_IT(&htim4);



            //  HAL_UART_Receive_DMA(&huart2, ibus, BUFFER_LEN);

    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  //  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
   // reg_address = 0x75 | 0x80;
   // HAL_SPI_Transmit(&hspi1, &reg_address, 1, 100);
    //HAL_SPI_Receive(&hspi1, &data_rx, 1, 100);
   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
   // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
   // HAL_SPI_TransmitReceive(&hspi1, &reg_address, &data_rx, 2, 100);

   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
   // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_SET);
    //printf("%i,%i,%i\r\n",data_rx[0],data_rx[1],data_rx[2]);
//HAL_Delay(5000);
//TIM3->CCR1 = 900;
//TIM3->CCR2 = 900;





 //HAL_UART_DeInit(&huart3);
 //HAL_Delay(100);
//printf("%f	",c0);
//printf("%f	",c1);
//printf("%f	",c00);
//printf("%f	",c10);
//printf("%f	",c01);
//printf("%f	",c11);
//printf("%f	",c20);
//printf("%f	",c21);
//printf("%f	\r\n",c30);


 while (pressure == 0){
	 read_baro();
 }
	 // registerSetBits(dev, DPS310_REG_PRS_CFG, DPS310_PRS_CFG_BIT_PM_RATE_32HZ | DPS310_PRS_CFG_BIT_PM_PRC_16);

 InitKalmanZ (&NavZ);
float initalAlt = pressureToAltitude(pressure);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

	  //  data_tx[0] = 0x6A;
	  //  data_tx[1] = 0x10;
	  //  HAL_SPI_Transmit(&hspi1, data_tx, 2, 100);
	  //  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	   // data_tx[0] = 0x6A;
	  //  data_tx[1] = 0x10;
	  //  HAL_SPI_Transmit(&hspi1, data_tx, 2, 100);
	   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  //  HAL_Delay(1);
	 // reg_address = 0x75 | 0x80;
	    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

	    //HAL_SPI_Transmit(&hspi1, &reg_address, 1, 100);
	   //// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  //  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	 // HAL_SPI_Receive(&hspi1, &data_rx, 1, 100);
	 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	   // HAL_SPI_TransmitReceive(&hspi1, &reg_address, &data_rx, 2, 100);
	    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);


if (failsafe > 2000)
{
	safety_flag = 0;
}
	 // printf("hi\r\n");
	//  in_pitch = ibus[1];
	 // 	  in_roll = ibus[2];
	  //	  in_throttle = ibus[3];
	  	//  in_yaw = ibus[4];
	 // HAL_UART_Receive(&huart3, ibus, 1,100);

	  	 // for (i=0;i<8;i++)
	  	  //{
	  	//	  printf("%i	",ibus[i]);
	  	 // }
	  	// printf("\r\n");
	  	 // printf("%4d\t%4d\t%4d\t%4d\t", ibus[1], in_roll, in_throttle, in_yaw);
	 // printf("%i,	%i,	%i,	%i,	%i,	%i,	%i	",acc_data[7],acc_data[8],acc_data[9],acc_data[10],acc_data[11],acc_data[12],acc_data[13]);
	//  printf("%i,	%i,	%i,	%i,	%i,	%i,	%i,	%i,	%i,	%i,	%i\r\n",ibus[0],ibus[1],ibus[2],ibus[3],ibus[4],ibus[5],ibus[6],ibus[7],ibus[8],ibus[9],ibus[10]);
	// printf("%f,	%f,	%f	%f,\r\n",InputP,InputR,InputY,InputT);
	 // printf("%i	%i	%i",InputR,fgyry,debug);
//	 printf("	%f	%f	%f",gyrx,gyry,gyrz);
 //printf("	%f	%f	%f	%f	",roll_factor,pitch_factor,dinputR,dinputP);
//printf("%i	",safety_flag);

//printf("%f\r\n",motor1_out);
	  // read_baro();
	 // update_AERS();

	 // printf("%f 	%f\r\n",pressure,temperature);
///printf("%f	%f	%f	%f	%f	%f	%i	",gyrx,gyry,gyrz,accx,accy,accz,debug);

//printf("%i	%i	%i	%i\r\n",opticalflow.lidar.distance,opticalflow.flow.vx,opticalflow.flow.vy,debug);
	 // baro_alt = (-pressure + inital_preqssure)*11.1;
	 // printf("%f	\r\n",baro_alt);
	//  printf("%f	%f	%f\r\n",InputR,InputP,InputY);
	  //printf("%f	%f	%f\r\n",baro_alt,pressure,temperature);
	//  printf("%i	",DPS310_rx_buffer[2]);
	//  printf("%i\r\n",DPS310_rx_buffer[3]);
	    //HAL_Delay(100);
	  //  printf("%f,	%f,	%f\r\n", rMat[0][0], rMat[0][1], rMat[0][2]);
	   // printf("%f,	%f,	%f\r\n", rMat[1][0], rMat[1][1], rMat[1][2]);
	   // printf("%f,	%f,	%f\r\n", rMat[2][0], rMat[2][1], rMat[2][2]);
	   // printf("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n");

	  //  printf("%f,	%f,	%f	", roll, pitch, yaw);
//printf("%i	%i",debug,debug2);
//float newBaroAlt = pressureToAltitude(pressure);

//if (M_FLAG){
	//initalAlt = newBaroAlt;
//}

	//printf("%f	%f	%f	%f\r\n",(newBaroAlt-initalAlt),pressure,initalAlt,newBaroAlt);
	//printf("%i	 ",debug);
	//read_baro();
	   // printf("%f,	%f,	%f	", roll, pitch, yaw);
	   // printf("%f	%f	%f	%f	%f	%f	\r\n",IMU.accx,IMU.accy,IMU.accz,IMU.gyrx,IMU.gyry,IMU.gyrz);
	   // printf("hi %i,%i,%i\r\n",acc_data[0],acc_data[1],acc_data[2]);
	   // TIM3->CCR1 = ibus[4];
	   // TIM3->CCR2 = ibus[4];
	    //TIM3->CCR3 = ibus[4];
	 	//TIM3->CCR4 = ibus[4];

//printf("%f	%f	%f	%f",NavZ.Acc.Acc,NavZ.Est.Pos,NavZ.Est.Vel,NavZ.Acc.Bias);

//printf("%f	%f	%f	",NavZ.Corr.Pos,NavZ.Corr.Vel,baroAltResidual);

//printf("%f	%f	%f",NavZ.Rusult.Pos,NavZ.Rusult.Vel,NavZ.Baro.Pos);

//for (uint16_t i = 0; i<11;i++)
//{
	//printf("%f\t",*((float*)(&NavAlt)+i));
//}

//printf("%i\r\n",debug);
//printf("\r\n");
printf("%f	%f	%f	%f	%f	%f\r\n",NavAlt.BaroAlt*10,NavAlt.BaroVel,NavAlt.Pos*10,NavAlt.Vel,NavAlt.accint_delayed*400,NavAlt.VelCorr*400);
//printf ("%i		",BuffPtr);
//printf("%f	",NavAlt.Acc_feedback);
//printf("%f	%f\r\n",NavAlt.BaroVel,NavAlt.Vel);
  }

 // }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void PID (void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim == &htim2)
		{
		debug2++;
			failsafe++;
			IMU_read(&IMU);
			PID();
			updateIMU(&IMU);
			debug++;
		}

	if (htim == &htim4)
	{
		//debug++;
		read_baro();
	}

}

void update_motors (uint16_t M1,uint16_t M2,uint16_t M3,uint16_t M4)
{
TIM3->CCR1 = (M3/31)+600;
TIM3->CCR2 = (M1/31)+600;
TIM3->CCR3 = (M4/31)+600;
TIM3->CCR4 = (M2/31)+600;
}

float constrainf(float input,float max, float min);

void motor_mix (void);
void PID (void)
{
	dinputR = InputfltR;
	dinputP = InputfltP;

	InputfltR = 0.958*(InputfltR) + 0.015*(InputR*938.719727597f);
	InputfltP = 0.958*(InputfltP) + 0.015*(InputP*938.719727597f);

	dinputR = 4000*(InputfltR - dinputR);
	dinputP = 4000*(InputfltP - dinputP);

	//0xFFFF/4000=16ishperdegree
	err_last[P] = ERR[P][D];
	ERR[P][P] = pitch - InputP;
	ERR[P][D] = -IMU.gyry - (dinputP);
	ERR[P][D2] = ERR[P][D] - err_last[P];

	err_last[R] = ERR[R][D];
	ERR[R][D] = IMU.gyrx - (dinputR);
	ERR[R][P] = -roll - InputR;
	ERR[R][D2] = ERR[R][D] - err_last[R];

	ERR[Y][P] = -IMU.gyrz - InputY;

	pitch_factor = (K[P][P]*ERR[P][P]) + (K[P][I]*ERR[P][I]) + (K[P][D]*ERR[P][D]) + (K[P][D2]*ERR[P][D2]);
	roll_factor = (K[R][P]*ERR[R][P]) + (K[R][I]*ERR[R][I]) + (K[R][D]*ERR[R][D]) + (K[R][D2]*ERR[R][D2]);
	yaw_factor = (K[Y][P]*ERR[Y][P]) + (K[Y][I]*ERR[Y][I]);


	//alt = alt_est;
	//d_alt = (biquadFilterApply(&delta_alt_biquad,(alt - alt1))); //   bandpass 2 lp 1 hp
//	alt_setpoint = 300;
	//alt_err  = alt - alt_setpoint;

	input_throttle = InputT*60 - NavAlt.Vel*alt_Kp;
	//input_throttle = InputT*60;
	//input_throttle = ( InputT*60 - d_alt*alt_Kp - alt_err * 0)/constrainf(costilt,1,0.6);
	motor_mix();
	//ERR[R][D2] = ERR[R][D];
	//ERR[P][D2] = ERR[P][D];
	//alt1 = alt;
}

float constrainf(float input,float max, float min)
{
	if (input < min)
	{
	return min;
	}
	else if (input > max)
	{
		return max;
	}
	else
	{
		return input;
	}
}


void motor_mix (void)
{
	motor1_out = +roll_factor - pitch_factor - yaw_factor + input_throttle + 0x2000;
	if (motor1_out < 0x1900)
	{
		motor1_out = 0x1900;
	}
	motor2_out = +roll_factor + pitch_factor + yaw_factor + input_throttle + 0x2000 ;
	if (motor2_out < 0x1900)
	{
		motor2_out = 0x1900;
	}
	motor3_out = -roll_factor - pitch_factor + yaw_factor + input_throttle + 0x2000;
	if (motor3_out < 0x1900)
	{
		motor3_out = 0x1900;
	}
	motor4_out = -roll_factor + pitch_factor - yaw_factor + input_throttle + 0x2000;
	if (motor4_out < 0x1900)
	{
		motor4_out = 0x1900;
	}

	if (safety_flag == 1)
	{
		update_motors(motor1_out,motor2_out,motor3_out,motor4_out);
	}
	else
	{
		update_motors(0,0,0,0);
	}
	//debug++;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart == &huart1)
	{
		//debug++;
		optical_flow_callback(huart);
		//HAL_UART_Receive_DMA(&huart1, &opticalflow.message.start,60);
	}

	if (huart == &huart2)
	{
		failsafe = 0;
	ibus_callback();
	}

	if (huart == &huart4)
		{
		 switch(buffer)
		 	{
		 	case 'q':
		 	safety_flag = !safety_flag;
		 	inital_pressure = pressure;
		 	alt_est = 0;
		 	baro_alt = 0;
		 	break;

		 	case 'p':
		 	K[P][P]+=1000;
		 	K[R][P]+=1000;
		 	break;

		 	case 'l':
		 	K[P][P]-=1000;
		 	K[R][P]-=1000;
		 	break;

		 	case 'd':
		 	K[P][D]+=1;
		 	K[R][D]+=1;
		 	break;

		 	case 'x':
		 	K[P][D]-=1;
		 	K[R][D]-=1;
		 	break;

		 	case 'f':
			K[P][D2]+=1;
			K[R][D2]+=1;
			break;

			case 'c':
			K[P][D2]-=1;
			K[R][D2]-=1;
			break;


		 	case 'y':
			K[Y][P]+=0.05;
			break;

			case 'g':
			K[Y][P]-=0.05;
			break;

			case 'a':
			alt_Kp+=1;//0.5
			break;

			case 'z':
			alt_Kp-=1;
			break;

			case 'm':
			M_FLAG = !M_FLAG;
			break;


			case'e':
				printf("\r\n\r\n\r\n");
		printf("%f	%f	%f	%f\r\n",K[P][P],K[P][I],K[P][D],K[P][D2]);
		printf("%f	%f	%f	%f\r\n",K[R][P],K[R][I],K[R][D],K[R][D2]);
		printf("%f	%f	%f	%f\r\n",K[Y][P],K[Y][I],K[Y][D],K[Y][D2]);
		printf("\r\n\r\n\r\n");


		break;
		 	}
		 HAL_UART_Receive_IT(&huart4, &buffer, 1);
		}



}










/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
