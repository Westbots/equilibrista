/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L0X.h"
#include "sd_hal_mpu6050.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/******************************************************************************
VL53L0X DEFINITIONS:
		LONG_RANGE: increases the sensitivity of the sensor and extends its 
								potential range, but increases the likelihood of getting an 
								inaccurate reading because of reflections from objects other 
								than the intended target. It works best in dark conditions.
		HIGH_SPEED:	higher speed at the cost of lower accuracy. OR
 HIGH_ACCURACY:	higher accuracy at the cost of lower speed. 
 ******************************************************************************/

//#define LONG_RANGE
#define HIGH_SPEED
//#define HIGH_ACCURACY

#define PI  3.14159265359
#define n   5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


// Estruturas MPU6050
SD_MPU6050 mpu1;						// Variáveis da leitura
SD_MPU6050_Result result;		// Resultado da inicialização

// Variáveis de leitura do MPU6050
int16_t g_x = 0;       
int16_t g_y = 0;
int16_t g_z = 0;
int16_t a_x = 0;
int16_t a_y = 0;
int16_t a_z = 0;

// Contagem de tempo
uint32_t timer = 0;
double delta_t = 0.0;

// Medição do ângulo
double roll = 0.0;

//Variaveis da biblioteca Kalman
double Q_angle = 0.001;
double Q_bias = 0.003;
double R_measure = 0.03;

double angle = 0.0; 	// Reset the angle
double rate = 0.0;
double bias = 0.0; 		// Reset bias
double gyroXrate = 0.0;
double gyroXangle = 0.0;
double compAngleX = 0.0;
double kalAngleX = 0.0;

float P[2][2];		// Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical

// Estrutura de parâmetros iniciais do VL53L0X
struct VL53L0X myTOFsensor = {.hi2c = &hi2c1, .io_2v8 = true, .address = 0x29, .io_timeout = 800, .did_timeout = false};

// Variáveis de medição para o VL53L0X
uint16_t distance = 65535;		
uint16_t testValue = 65535;

// Checagem de funcionamento dos sensores
bool VL53L0X_OK = false;
bool MPU6050_OK = false;

// Leitura dos botões
bool bt1 = false;
bool bt2 = false;

// Leitura de velocidade
double velocidade = 0.0;
double sentido = 0.0;

// Variáveis de início e parada
bool parar = true;
bool equilibrio = false;
bool distancia = false;

// Mínimos e máximos de PWM
double PWM_max_eq = 0.0;
double PWM_max_dt = 0.0;
double PWM_min_hr = 0.0;
double PWM_min_ahr = 0.0;
double PWM = 0.0;

// Variáveis de controle dos motores (controle de ângulo/equilibrio)
double erro_mt = 0.0;
double erro_mt_a = 0.0;

double PID_mt = 0.0;
double P_mt = 0.0;
double D_mt = 0.0;
double I_mt = 0.0;
double I_mt_a = 0.0;

double kp_mt = 0.0;
double ki_mt = 0.0;
double kd_mt = 0.0;

double setpoint_angle = 0.0;

// Variáveis de controle do setpoint do ângulo (controle de velocidade)
double erro_vl = 0.0;
double erro_vl_a = 0.0;

double PID_vl = 0.0;
double P_vl = 0.0;
double D_vl = 0.0;
double I_vl = 0.0;
double I_vl_a = 0.0;

double kp_vl = 0.0;
double ki_vl = 0.0;
double kd_vl = 0.0;

double setpoint_angle_min = 0.0;
double setpoint_angle_max = 0.0;
double setpoint_angle_PID = 0.0;

double setpoint_velocidade_eq = 0.0;

// Váriáveis de controle de distância
double erro_dt = 0.0;
double erro_dt_a = 0.0;

double PID_dt = 0.0;
double P_dt= 0.0;
double D_dt= 0.0;
double I_dt = 0.0;
double I_dt_a = 0.0;

double kp_dt = 0.0;
double ki_dt = 0.0;
double kd_dt = 0.0;

double setpoint_velocidade_min = 0.0;
double setpoint_velocidade_max = 0.0;
double setpoint_velocidade_dt = 0.0;

double setpoint_distancia = 0.0;

double distance_mm[n];
double distance_mm_a[n];

uint32_t ADT[2];							// Vetor que armazena as leituras do ADC1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool Reset_MPU6050()
{
	bool working = false;
	
	HAL_GPIO_WritePin(XSHUT_MPU_GPIO_Port, XSHUT_MPU_Pin, GPIO_PIN_RESET);	// Tira a energia do CI
	HAL_Delay(50);																													
	HAL_GPIO_WritePin(XSHUT_MPU_GPIO_Port, XSHUT_MPU_Pin, GPIO_PIN_SET);		// Retorna a energia do CI
	
	HAL_Delay(1000);
	
	result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
	if (result == SD_MPU6050_Result_Ok)	working = true;			// Checa se o sensor está funcionando
	
	return working;
}

bool Reset_VL53L0X()
{
	bool working = false;
	
	HAL_GPIO_WritePin(XSHUT_TOF_GPIO_Port, XSHUT_TOF_Pin, GPIO_PIN_RESET);	// Tira a energis do CI
	HAL_Delay(50);
	HAL_GPIO_WritePin(XSHUT_TOF_GPIO_Port, XSHUT_TOF_Pin, GPIO_PIN_SET);		// Retorna a energia do CI
	
	if (VL53L0X_init(&myTOFsensor))			
	{ // Se o sensor iniciou...
		VL53L0X_startContinuous(&myTOFsensor, 0);		// Habilita a medição
		for (int k = 0; k < 2; k++)			// Realiza 3 medições
		{
			testValue = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor);
			
			if (testValue > 8500 || VL53L0X_timeoutOccurred(&myTOFsensor))		
			{ // Se a medição saiu do range ou ocorreu um timeout...
				VL53L0X_stopContinuous(&myTOFsensor);		// Desabilita a medição
				working = false;
				break;
			}
			else 	// Se a medição ocorreu conforme esperado...
			{
				working = true;
			}
		}
	}
	
	return working;
}


void Motor_Horario(int velocidade)
{	
		HAL_GPIO_WritePin(L_EN_GPIO_Port,L_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(R_EN_GPIO_Port,R_EN_Pin,GPIO_PIN_SET);
		TIM1 -> CCR1 = velocidade;
		TIM1 -> CCR2 = 0;
}
	
void Motor_AntiHorario(int velocidade)
{	
		HAL_GPIO_WritePin(L_EN_GPIO_Port,L_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(R_EN_GPIO_Port,R_EN_Pin,GPIO_PIN_SET);
		TIM1 -> CCR1 = 0;
		TIM1 -> CCR2 = velocidade;
}

void Motor_Parar()
{	
		HAL_GPIO_WritePin(L_EN_GPIO_Port,L_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(R_EN_GPIO_Port,R_EN_Pin,GPIO_PIN_RESET);
		TIM1 -> CCR1 = 0;
		TIM1 -> CCR2 = 0;
}

void LedG_Liga()
{
	HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, GPIO_PIN_SET);	
}

void LedG_Desliga()
{
	HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, GPIO_PIN_RESET);	
}

void LedR_Liga()
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, GPIO_PIN_SET);	
}

void LedR_Desliga()
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, GPIO_PIN_RESET);	
}

void Leitura_Botoes()
{
	if(!bt2 && !bt1)
	{
		if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin) == GPIO_PIN_SET) 
		{
			HAL_Delay(30);
			if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin) == GPIO_PIN_SET)
			{
				while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin) == GPIO_PIN_SET);
				bt1 = true;
			}
		}
	}
	else if(!bt2 && bt1)
	{
		if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin) == GPIO_PIN_SET) 
		{
			HAL_Delay(30);
			if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin) == GPIO_PIN_SET)
			{
				while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin) == GPIO_PIN_SET);
				bt1 = false;
			}
		}
	}
	
	if(!bt1 && !bt2)
	{
		if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin) == GPIO_PIN_SET) 
		{
			HAL_Delay(30);
			if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin) == GPIO_PIN_SET)
			{
				while(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin) == GPIO_PIN_SET);
				bt2 = true;
			}
		}
	}
	else if(!bt1 && bt2)
	{
		if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin) == GPIO_PIN_SET) 
		{
			HAL_Delay(30);
			if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin) == GPIO_PIN_SET)
			{
				while(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin) == GPIO_PIN_SET);
				bt2 = false;
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM2)
	{
		timer++;
	}
}


float Kalman(float newAngle, float newRate, float dt) 
{
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	rate = newRate - bias;
	angle += dt * rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = P[0][0] + R_measure; // Estimate error
	/* Step 5 */
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - angle; // Angle difference
	/* Step 6 */
	angle += K[0] * y;
	bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];

	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;

	return angle;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	SD_MPU6050_Result result ;

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	// Inicialização do ADC1
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	return 0;
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADT, 2) != HAL_OK)
	return 0;
	
	// Inicialização do TIM1 para PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); 
	TIM1 -> CCR1 = 0;		// Zero na saída do PWM1
	TIM1 -> CCR2 = 0;  	// Zero na saída do PWM2
	
	// Inicialização do TIM2
	HAL_TIM_Base_Start_IT(&htim2);

	// Reset inicial dos sensores (por segurança)
	MPU6050_OK = Reset_MPU6050();
	VL53L0X_OK = Reset_VL53L0X();
	
	// Definições de medição do VL53L0X
	#ifdef LONG_RANGE
		// lower the return signal rate limit (default is 0.25 MCPS)
		VL53L0X_setSignalRateLimit(&myTOFsensor, 0.1);
		// increase laser pulse periods (defaults are 14 and 10 PCLKs)
		VL53L0X_setVcselPulsePeriod(&myTOFsensor, VcselPeriodPreRange, 18);
		VL53L0X_setVcselPulsePeriod(&myTOFsensor, VcselPeriodFinalRange, 14);
	#endif
	#ifdef HIGH_SPEED
		// reduce timing budget to 20 ms (default is about 33 ms)
		VL53L0X_setMeasurementTimingBudget(&myTOFsensor, 20000);
	#else //HIGH_ACCURACY
		// increase timing budget to 200 ms
		VL53L0X_setMeasurementTimingBudget(&myTOFsensor, 200000);
	#endif
	
	// Setpoint de distância
	setpoint_distancia = 100.0;
	
	// Setpoint de velocidade de equilíbrio do robô
	setpoint_velocidade_eq = 0.0;
	
	// Setpoint de ângulo
	setpoint_angle = 9.0;
	
	// Definição de velocidade mínima e máxima
	//setpoint_velocidade_min = setpoint_velocidade - 300.0;
	//setpoint_velocidade_max = setpoint_velocidade + 300.0;
	
	// Definição de`ângulo mínimo e máximo
	setpoint_angle_min = setpoint_angle - 3.0;
	setpoint_angle_max = setpoint_angle + 3.0;

	// Definição de PWM mínimo e máximo
	PWM_min_hr = 550.0;
	PWM_min_ahr = 350.0;
	PWM_max_eq = 7000.0;
	PWM_max_dt = 2000.0;
	
	// Definição de ganhos do controle de velocidade
	kp_vl = 0.0025; //0.001     	                                                                                ;
	kd_vl = 0.0;			
	ki_vl = 0.0000025;		
	
	// Definição de ganhos do controle do ângulo
	kp_mt = 550.0;     	//ganho proporcional 600                                                                                    ;
	kd_mt = 100.0;			//ganho derivativo 150
	ki_mt = 70.0;			//ganho integral 100	
	
	// Definição de ganhos do controle de distância
	kp_dt = 12.0;     	                                                                                ;
	kd_dt = 0.1;			
	ki_dt = 0.001;		
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (!VL53L0X_OK)
		{	// Se o VL53L0X não estiver funcionando adequadamente...
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			VL53L0X_OK = Reset_VL53L0X();
		}
		else if (!MPU6050_OK)
		{ // Se o MPU6050 não estiver funcionando adequadamente...
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			MPU6050_OK = Reset_MPU6050();
		}
		else
		{ // Se ambos os sensores estiverem funcionando...
			
			// Acende o led da placa para sinalizar o funcionamento
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			
			// Verifica estado dos botões
			Leitura_Botoes();
			
			// Mede distância do sensor TOF
			distance = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor);
		
			// Se parar de funcionar/timeout ocorrer, sinaliza que parou de funcionar
			if (distance > 10000 || VL53L0X_timeoutOccurred(&myTOFsensor))
			{
				VL53L0X_OK = false;
			}
			
			// Realiza a leitura do MPU
			SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);		
			g_x = mpu1.Gyroscope_X;
			g_y = mpu1.Gyroscope_Y;
			g_z = mpu1.Gyroscope_Z;
			
			SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
			a_x = mpu1.Accelerometer_X;
			a_y = mpu1.Accelerometer_Y;
			a_z = mpu1.Accelerometer_Z;
			
			// Define o dt
			delta_t = timer*0.00005;
			timer = 0;
			
			// Ajusta a leitura do acelerômetro e do giroscópio
			roll = atan2(a_x,(sqrt(a_y*a_y + a_z*a_z))) * 180/PI;
			
			gyroXrate = -g_y / 131.0;
			
			// Aplica filtro de Kalman no roll
			kalAngleX = Kalman(roll,gyroXrate,delta_t);
			
			// Filtro complementar (?)
			gyroXangle += gyroXrate * delta_t;
		
			compAngleX = 0.96 * (compAngleX + gyroXrate * delta_t) + 0.04 * kalAngleX;

			if (gyroXangle < -90 || gyroXangle > 90)
				gyroXangle = kalAngleX;
			
			// Verifica velocidade dos motores e faz a média entre eles
			velocidade = sentido*(ADT[0] + ADT[1])/2.0;
			
			// Verifica critério de início e parada
			if (bt1)
			{
				equilibrio = false;
				distancia = true;
				parar = false;
			}
			else if (bt2)
			{
				equilibrio = true;
				distancia = false;
				parar = false;
			}
			else 
			{
				equilibrio = false;
				distancia = false;
				parar = true;
				
				// Zerando os termos de controle
				I_vl_a = 0.0; 		I_mt_a = 0.0; 		I_dt_a = 0.0;
				erro_vl_a = 0.0;	erro_mt_a = 0.0;	erro_dt_a = 0.0;
				
			}
			
			if (parar)	// Antes de apertar qualquer botão e após apertar BT2
			{
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
				Motor_Parar();
			}
			else if	(equilibrio)	// Após apertar BT2 para iniciar o funcionamento do equilíbrio
			{
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
				
				//------- Rotina do controle de velocidade --------//
				erro_vl = setpoint_velocidade_eq - velocidade;
				
				P_vl = kp_vl * erro_vl;
				I_vl = ki_vl * erro_vl + I_vl_a;
				D_vl = kd_vl * (erro_vl - erro_vl_a);
				
				PID_vl = P_vl + I_vl + D_vl;
				
				I_vl_a = I_vl;
				erro_vl_a = erro_vl;
				
				// Joga o sinal do controle para o setpoint do ângulo
				setpoint_angle_PID = setpoint_angle - PID_vl;
				
				if (fabs(setpoint_angle_PID) > setpoint_angle_max)
				{
					setpoint_angle_PID = setpoint_angle_max;
					//I_vl_a = setpoint_angle_PID;
				}
				else if (fabs(setpoint_angle_PID) < setpoint_angle_min)
				{
					setpoint_angle_PID = setpoint_angle_min;
					//I_vl_a = setpoint_angle_PID;
				}
				
				//------- Rotina do controle de ângulo --------//
				erro_mt = setpoint_angle_PID - compAngleX;
				
				P_mt = kp_mt * erro_mt;
				I_mt = (ki_mt * erro_mt) + I_mt_a;
				D_mt = kd_mt * (erro_mt - erro_mt_a);
								
				PID_mt = P_mt + I_mt + D_mt;
				
				I_mt_a = I_mt;
				erro_mt_a = erro_mt;
				
				// Joga o sinal do controle para o PWM dos motores
				PWM = PID_mt;
				
				if (PWM > 0)
				{
						sentido = 1.0;
					// Se o PWM resultante for maior que o PWM máximo, limita o sinal ao máximo (mantendo o módulo)
					if ((fabs(PWM) + PWM_min_hr) > PWM_max_eq)
					{
						PWM = PWM_max_eq * PWM/fabs(PWM);
						if (I_mt_a > PWM)
							I_mt_a = PWM;	
					}
					Motor_Horario(fabs(PWM) + PWM_min_hr);
				}
				else if (PWM < 0)
				{
					sentido = -1.0;
					// Se o PWM resultante for maior que o PWM máximo, limita o sinal ao máximo (mantendo o módulo)
					if ((fabs(PWM) + PWM_min_ahr) > PWM_max_eq)
					{
						PWM = PWM_max_eq * PWM/fabs(PWM);
						if (I_mt_a < PWM)
							I_mt_a = PWM;	
					}
					Motor_AntiHorario(fabs(PWM) + PWM_min_ahr);
				}
			}
			else if (distancia)
			{
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				
				// Média móvel do erro
				for (int k = 0; k < n; k++)
					distance_mm_a[k] = distance_mm[k];
				
				distance_mm[0] = distance;
				
				for (int k = 1; k < n; k++)
					distance_mm[k] = distance_mm_a[k-1];
				
				distance = 0.0;
				for (int k = 0; k < n; k++)
					distance += distance_mm[k];
				
				distance = distance/n;
				
				if (distance > 200.0) distance = 200.0;
				
				//------- Rotina do controle de distância --------//
				erro_dt = setpoint_distancia - distance;
				
				P_dt = kp_dt * erro_dt;
				I_dt = ki_dt * erro_dt + I_dt_a;
				D_dt = kd_dt * (erro_dt - erro_dt_a);
				
				PID_dt = P_dt + I_dt + D_dt;
				
				I_dt_a = I_dt;
				erro_dt_a = erro_dt;
				
				// Joga o sinal do controle para o setpoint da velocidade
				PWM = PID_dt;
				
				if (PWM > 0)
				{
					sentido = 1.0;
					// Se o PWM resultante for maior que o PWM máximo, limita o sinal ao máximo (mantendo o módulo)
					if ((fabs(PWM) + PWM_min_hr) > PWM_max_dt)
					{
						PWM = PWM_max_dt * PWM/fabs(PWM);
						if (I_mt_a > PWM)
							I_mt_a = PWM;	
					}
					Motor_Horario(fabs(PWM) + PWM_min_hr);
				}
				else if (PWM < 0)
				{
					sentido = -1.0;
					// Se o PWM resultante for maior que o PWM máximo, limita o sinal ao máximo (mantendo o módulo)
					if ((fabs(PWM) + PWM_min_ahr) > PWM_max_dt)
					{
						PWM = PWM_max_dt * PWM/fabs(PWM);
						if (I_mt_a < PWM)
							I_mt_a = PWM;	
					}
					Motor_AntiHorario(fabs(PWM) + PWM_min_ahr);
				}
			}
			
			
//			// Teste dos motores
//			if(bt1)
//			{
//				Motor_Horario(1500 + PWM_min_hr);
//				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
//			}
//			else if(bt2)
//			{
//				Motor_AntiHorario(1500 + PWM_min_ahr);
//				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
//			}
//			else
//			{
//				Motor_Parar();
//				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
//			}
		}
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7999;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|LED_R_Pin|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_EN_Pin|R_EN_Pin|XSHUT_MPU_Pin|XSHUT_TOF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin LED_R_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LED_R_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BT1_Pin BT2_Pin */
  GPIO_InitStruct.Pin = BT1_Pin|BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L_EN_Pin R_EN_Pin XSHUT_MPU_Pin XSHUT_TOF_Pin */
  GPIO_InitStruct.Pin = L_EN_Pin|R_EN_Pin|XSHUT_MPU_Pin|XSHUT_TOF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
