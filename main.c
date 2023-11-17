/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"

// USART function
void send_num(int value);
void send_num_with_new_line(int value);
void send_string(const char* str);
void send_string_with_new_line(const char* str);
void send_float(float value);
void send_float_with_new_line(float value);

//------------------//
uint16_t count = 0;
float ts = 0.1;

volatile int nua_giay = 0;
float vr = 500, wr = 1, k1 = 0.2, k2 = 2000, k3 = 4;
float b = 195;  // khoang cach 2 banh xe
float r = 40;   // ban kinh banh xe

float ds = 0.1;
float e1 = 0, e2 = 0, e3 = 0, e2_pre = 0;

float velocity, omega;

//---------------------------------------------------------------------//
// ADC 
uint32_t adc_value[5], sensor_value[5];
uint32_t sum_sensor_value = 0;
uint32_t data_calib_min[5] = {655, 185, 198, 242, 1098};
uint32_t data_calib_max[5] = {3775, 3649, 3665, 3709, 3704};
uint32_t y_min = 1000, y_max = 1000;
float trung_binh_trong_so = 0;
float coefficient[5];

void read_sensor_and_calculate_e2_e3(){
	y_min = data_calib_min[0];
	y_max = data_calib_max[0];
	for (int i = 0; i < 5; i++) 
	{ 
    if(adc_value[i]> 100 && adc_value[i]<3950)
		{
			// update array data_calib_min and data_calib_max
			if(data_calib_min[i] > adc_value[i]) data_calib_min[i] = adc_value[i];
			if(data_calib_max[i] < adc_value[i]) data_calib_max[i] = adc_value[i];
		}
  }
	for (int i = 0; i < 5; i++) 
	{
		if(y_min > data_calib_min[i])	y_min = data_calib_min[i]; // find y_min in data_calib_min
		if(y_max < data_calib_max[i])	y_max = data_calib_max[i]; // find y_max in data_calib_max
	}
	for(int i = 0; i<5; i++){  // calculate coef
		coefficient[i]= (float)(y_max - y_min) / (float)(data_calib_max[i] - data_calib_min[i]);
	}
	for (int i = 0; i < 5; i++) { // calib adc_value to sensor_value
		sensor_value[i] = y_min + coefficient[i] * (adc_value[i] - data_calib_min[i]);
	}
	// Calculate y, e2, e3 
	sum_sensor_value = 0;
	for(int i = 0; i<5 ; i++){
		sum_sensor_value += sensor_value[i];
	}
	trung_binh_trong_so = 2*((float)sensor_value[0] - (float)sensor_value[4]) + ((float)sensor_value[1] - (float)sensor_value[3]);
	trung_binh_trong_so = 14 * trung_binh_trong_so / sum_sensor_value;
	if(abs((int)trung_binh_trong_so) > 4) e2 = (trung_binh_trong_so-0.2)/0.8;
	else e2 = (trung_binh_trong_so-0.2)/1.3;
	//ds = velocity * ts
	e3 = atan(e2 - e2_pre)/ds;
}
/*
uint32_t min_value[5] = {3775, 3645, 3714, 3569, 3704};
uint32_t max_value[5] = {430, 255, 460, 281, 1404};
void send_adc_value(){
	for (int i=0; i<5; i++)
	{
		if(adc_value[i]>0 && min_value[i] > adc_value[i]) min_value[i] = adc_value[i];
		if(adc_value[i]>0 && max_value[i] < adc_value[i]) max_value[i] = adc_value[i];
		send_string("Sensor: "); send_num(i+1); send_string(" Value: "); send_num(adc_value[i]); send_string(" min: "); send_num(min_value[i]); send_string(" max: "); send_num_with_new_line(max_value[i]); 
	}
}
*/
void send_sensor_value(){
	for (int i=0; i<5; i++)
	{
		send_string("Sensor: "); send_num(i+1); send_string(" Value: "); send_num_with_new_line(sensor_value[i]); 
	}
	send_string("e2: "); send_float(e2); send_string(" e3: "); send_float_with_new_line(e3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// Timer 4 ngat moi 1ms
  if (htim->Instance == TIM4)
	{
		count++;
		if(count == 200){
			count = 0;
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			read_sensor_and_calculate_e2_e3();
			send_sensor_value();
		}
	}
}
///////////////////////////////////////////////////////////////////////////
//-------------------------------------------//
// Color sensor
/*
Cach dung: 
	start_color_sensor();
	sau do goi ham: get_color();
	- red: color = 1
	- blue: color = 2
	- else: color = 0
	nen goi ham get_color nhieu lan
	khi ket thuc, goi ham stop_color_sensor();
*/
volatile uint32_t red_value = 0, blue_value = 0, green_value = 0;
int color = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
       __HAL_TIM_SetCounter(&htim1, 0);
    }
}
void start_color_sensor(){
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
}
void stop_color_sensor(){
	HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_3);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	// start pwm
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
void get_color(){
		for (int i = 0; i < 3; i++)
    {
        switch (i)
        {
        case 0:
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
            HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
						red_value = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
            break;
        case 1:
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
            HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
            blue_value = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
            break;
        case 2:
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
            HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
            green_value = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
        }
    }
    // code to read red_value, green_value, and blue_value 
    if (red_value > 700 && red_value < 1000 &&
        green_value > 600 && green_value < 800 &&
        blue_value > 200 && blue_value < 400)
    {
      color = 1;
			send_string_with_new_line("Red");
    }
    else if (red_value > 500 && red_value < 700 &&
             green_value > 700 && green_value < 900 &&
             blue_value > 900 && blue_value < 1100)
    {
      color = 2;
			send_string_with_new_line("Blue");
    }
		else 
		{
			color = 0;
			send_string_with_new_line("No detect");
		}
		//send_num(red_value);
		//send_num(green_value);
		//send_num(blue_value);
}

//-----------------------------------------------------------------//
// ENCODER
unsigned int pulse1 = 0, pulse2 = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
 if (GPIO_Pin == ENCODER_1A_Pin || GPIO_Pin == ENCODER_1B_Pin ) //neu la chan PA0 hoac PA1
 {
   pulse1++;
 }
 if (GPIO_Pin == ENCODER_2A_Pin || GPIO_Pin == ENCODER_2B_Pin ) //neu la chan PA2 hoac PA3
 {
   pulse2++;
 }
}
//------------------------------------------------------------------//
// MOTOR
float error_1 = 0, prev_error_1 = 0, integral_1 = 0, derivative_1 = 0;
float error_2 = 0, prev_error_2 = 0, integral_2 = 0, derivative_2 = 0;

float ppr = 11*35.5;
float rpm1 = 0, rpm2 = 0;
float pwm1 = 0, pwm2 = 0;
unsigned int duty1 = 0, duty2 = 0;

const float Kp1 = 0.78, Ki1 = 1.8, Kd1 = 0.061;
const float Kp2 = 0.62, Ki2 = 2.12, Kd2 = 0.052;
//float Kp1 = 0.72, Ki1 = 2.32, Kd1 = 0.05;
float v1 = 50, v2 = 100;

void calculate_rpm1(){
	rpm1 = (float)pulse1/4.0/ppr * 60.0 / ts;
	pulse1 = 0;
}
void calculate_rpm2(){
	rpm2 = (float)pulse2/4.0/ppr * 60.0 / ts;
	pulse2 = 0;
}
void PID_motor1(){
	calculate_rpm1();
	error_1 = v1 - rpm1;
	integral_1 = integral_1 + (error_1 * ts);
	derivative_1 = (error_1 - prev_error_1) / ts;
	prev_error_1 = error_1;
	pwm1 = ( Kp1 * error_1 + Ki1 * integral_1 + Kd1 * derivative_1);
	if(pwm1<0) pwm1 = 0;
	else if( pwm1 > 170) pwm1 = 170;
	duty1 = (pwm1+2.4)/0.178;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duty1);
}

void PID_motor2(){
	calculate_rpm2();
	error_2 = v2 - rpm2;
	integral_2 = integral_2 + (error_2 * ts);
	derivative_2 = (error_2 - prev_error_2) / ts;
	prev_error_2 = error_2;
	pwm2 = ( Kp2 * error_2 + Ki2 * integral_2 + Kd2 * derivative_2);
	if(pwm2<0) pwm2 = 0;
	else if( pwm2 > 170) pwm2 = 170;
	duty2 = (pwm2+2.4)/0.178;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, duty2);
}
/*void send_pulse(){
	send_string("pulse1: ");
	send_num_with_new_line(pulse1);
	send_string("pulse2: ");
	send_num_with_new_line(pulse2);
}*/
void calculate_v1_v2(){
	velocity = vr * cos(e3) + k1 * e1;
	omega = k2 * e2 * vr + wr + k3 * sin(e3);
	// Tính van toc cua hai bánh xe
	v1 = (1 / r) * (velocity + b * omega);
	v2 = (1 / r) * (velocity - b * omega);
	ds = velocity * ts;
	integral_1 = integral_2 = 0;
}
//volatile int dem = 0;
int speed = 0;
int flag_100ms = 0;
int flag_1s = 0;
int count2 = 0;
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM4)
	{
		count++;
		if(count == 100){ //0.1
			count = 0;
			
			PID_motor1();
			//calculate_rpm1();
			//send_string("rpm1: ");
			send_float_with_new_line(rpm1);
			//calculate_rpm2();
			//send_string("rpm2: ");
			//send_float_with_new_line(rpm2);
		}
	}
}
*/
/*
//-------------------------------------------------------//
// GIAI THUAT DIEU KHIEN
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM4)
	{
		count++;
		count2++;
		if(count == 100){    // 100ms = 0.1s
			count = 0;
			flag_100ms = 1;
		}
		if (count2 == 1000){ //1000ms = 1s
			count2 = 0;
			flag_1s = 1;
		}
	}
}
*/
void brake(){
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
}
int reached_the_first_intersection(){
	if( sensor_value[0] < 2000 && 
			sensor_value[1] > 3000 && 
			sensor_value[2] > 3000 && 
			sensor_value[3] > 3000 && 
			sensor_value[4] < 2000 )
	{
		return true;
	}	
	else return false;
}
int reached_the_stop_position_to_wait_for_the_goods(){
	if( sensor_value[0] >800 && 
			sensor_value[1] >3000 && 
			sensor_value[2] >3000 && 
			sensor_value[3] >3000 && 
			sensor_value[4] >750 )
	{
		return true;
	}
	else return false;
}

int reached_the_turn_position(){
	if( sensor_value[0] > 9000 && 
			sensor_value[1] > 3000 && 
			sensor_value[2] > 3000 && 
			sensor_value[3] > 3000 && 
			sensor_value[4] > 700 )
	{
		return true;
	}
	else return false;
}
void bam_line(){
	if(flag_100ms == 1)
		{
			PID_motor1();
			PID_motor2();
		}
		if (flag_1s == 1)
		{
			read_sensor_and_calculate_e2_e3();
			calculate_v1_v2();
		}
}
void go_straight(){
	v1 = velocity;
	v2 = velocity;
	while(flag_1s == 0){
		if(flag_100ms == 1)
		{
			PID_motor1();
			PID_motor2();
		}
	}
}
void turn_left(){
	
}
void turn_right(){
	
}
void turn_left_2(){
	
}
int reached_the_intersection_2(){
	if( sensor_value[0] > 3000 && 
			sensor_value[1] > 3000 && 
			sensor_value[2] > 3000 && 
			sensor_value[3] > 3000 && 
			sensor_value[4] > 3000 )
	{
		return true;
	}
	else return false;
	
}
int reached_the_end_position(){
	if( sensor_value[0] < 1000 && 
			sensor_value[1] < 1000 && 
			sensor_value[2] < 1000 && 
			sensor_value[3] < 1000 && 
			sensor_value[4] < 1000 )
	{
		return true;
	}
	else return false;
}
void run(){
	while(1){
		bam_line();
		if(reached_the_first_intersection()){
			go_straight(); //cho v1 = v2 = velocity, PID trong 1s
			break;
		}
	}
	// chay duong thang
	while(1){
		bam_line();
		if(reached_the_stop_position_to_wait_for_the_goods())
		{
			brake();
			break;
		}
	}
	// cho dat goi hang len
	start_color_sensor();
	while(1)
	{
		// kiem tra mau goi hang 100 lan
		for(int i = 0; i < 100; i++)
		{
			get_color();
		}
		if(color != 0)  // 1:red  2:blue  else: 0
		{
			stop_color_sensor();
			HAL_Delay(3000); // chò 3 giây 
			break;
		}
		//kiem tra duoc? break
	}
	
	while(1){
		bam_line();
		if(reached_the_turn_position()) break;
	}
	if(color == 1){
		turn_left();
		while(1){
			bam_line();
			if(reached_the_end_position()){
				brake();
				break;
				//end
			}
		}
	}	
	else{
		turn_right();
		/*while(1){
			bam_line();
			if(reached_the_intersection_2()){
				turn_left_2();
				break;
			}				
		}*/
		while(1){
			bam_line();
			if(reached_the_end_position()){
				brake();
				break;
				//end
			}
		}
	}
}

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADC_Start_DMA(&hadc1, adc_value, 5);
	HAL_TIM_Base_Start_IT(&htim4);
	
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 300);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 300);
	send_string_with_new_line("START-------------------------");
	
	
	//HAL_UART_Receive_IT(&huart3, buff, 10);
	//HAL_Delay(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, S3_Pin|S2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S3_Pin S2_Pin */
  GPIO_InitStruct.Pin = S3_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_1A_Pin ENCODER_1B_Pin ENCODER_2A_Pin ENCODER_2B_Pin */
  GPIO_InitStruct.Pin = ENCODER_1A_Pin|ENCODER_1B_Pin|ENCODER_2A_Pin|ENCODER_2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void send_num(int value){
	char buffer[10]; 
	sprintf(buffer, "%d", value);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
void send_num_with_new_line(int value){
	char buffer[10]; 
	sprintf(buffer, "%d\r\n", value);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
void send_float(float value){
    char buffer[20]; 
    sprintf(buffer, "%.4f", value); 
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
void send_float_with_new_line(float value){
    char buffer[20]; 
    sprintf(buffer, "%.4f\r\n", value); 
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
void send_string(const char* str){
  HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}
void send_string_with_new_line(const char* str){
  char buffer[50]; 
  sprintf(buffer, "%s\r\n", str);
  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
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
