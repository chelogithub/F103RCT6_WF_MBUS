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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESP8266_Chelo.h"
#include "ModBUS_Chelo.h"
#include "STR_Chelo.h"
#include "ETH_W5100.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TOK 1
#define FIND 0
#define SERVIDOR 1
#define CLIENTE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;
DMA_HandleTypeDef hdma_dac_ch2;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
uint32_t ms_ticks=0,
		 min_ticks=0;

uint32_t ADC_VAL[4];
uint32_t DAC_VAL[2];
uint32_t DAC_VAL2[2];

struct WIFI wf;
struct MBUS mb_eth;			// Instancia Ethernet
struct MBUS mb_wf;		// Instancia Wi-Fi

uint8_t WF_DBG_EN=1;
uint8_t ETH_DBG_EN=0;
uint8_t WF_SND_FLAG=0;
int wf_snd_flag_ticks=0;

uint32_t REG[254];		//Registros para ver ModBUS

struct W5100_SPI ETH; // Instancia de la comunicación Ethernet
// ****** Begin Firmware Registers ****** //

uint16_t W5100_socket0_STATUS ;

// ****** End Firmware Registers ****** //

// ****** Begin Socket Memory assignment ****** //
uint16_t 	S0_get_size = 0,
			S0_get_offset = 0,
			S0_RX_RD = 0,
			S0_get_start_address = 0,
			tx_mem_pointer=0,
			rx_mem_pointer=0,
			destination_addr=0,
			left_size=0,
			upper_size=0,
			S0_READ_RX_RD=0,
			S0_bf_rcv_offset=0,

			Sn_TX_WR=0,
			get_offset=0,
			get_free_size=0,
			get_start_address=0,
			source_addr=0,
			send_size=0;

// ****** End Socket Memory assignment ****** //
uint8_t spi_Data[64],
		spi_no_debug=0;

uint8_t ESP_REinit=0,			//Conteo de intentos de incializacion
		ESP_InitF=0,			//Flag de error por no encontrar la sentencia
		ESP_HW_Init=0,
		EN_UART2_TMR=0,
		EN_UART1_TMR=0,
		FLAG_TIMEOUT=0,
		FLAG_UART2=0,
		resultado=0,
		error_rxdata=0,
		debug_ser=1,
		esp_restart=0,
		conexion,
		asc=0,
		//--------debug----//
		CP_ready=0,
		CP_ai=0;
		SPI_READ_EN=0;

char	UART_RX_vect[384],
		datarx_uart1[384],
		AT_debug[384],
		datarx1[2],
		UART_RX_vect_hld[384],
		UART_RX_vect[384],
		RXdata_uart1[384],
		RXdata_DEBUG[384],
		WIFI_NET[]="PLC_DEV",//WIFI_NET[]="Fibertel WiFi967 2.4GHz",//WIFI_NET[]="PLC_DEV",//
		WIFI_PASS[]="12345678",//WIFI_PASS[]="0042880756",//WIFI_PASS[]="12345678",//
		TCP_SERVER[]="192.168.0.65",//TCP_SERVER[]="192.168.0.102",//TCP_SERVER[]="192.168.0.47",
		TCP_PORT[]="502",
		TCP_SERVER_LOCAL[]="192.168.0.33",//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_GWY[]="192.168.0.100",//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_MSK[]="255.255.255.0",//TCP_SERVER[]="192.168.0.47",
		TCP_PORT_LOCAL[]="502",
		RX2[]="RX.",
		RX[384],
		CMP_VECT[]="\0",
	    TESTA[32],
		TESTB[32],
		TESTC[32],
		UART_RX_byte[2];

int UART_RX_items=0,
    ESP_ticks=0,
	MBUS_ticks=0,
	ETH_ticks=0,
	items_rx_debug=0,
    ticks=0,
	ntesta=17,
	ntestb=4,
	ntestc=0,
	postesta=0,
	funcion=0,
	uart1pass=0,
	UART1_ticks=0,
	FLAG_UART1=0,
	chr_pos=0,
	items_rx=0,

	UART_RX_pos=0;

long dbgn=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ESP8266_HW_Init(UART_HandleTypeDef *);
void ESP8266_HW_Reset(void);
void Actualizar_RXdata(int );
void BorrarVect(void);
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

	//----------------------- ETHERNET W5100 Environment-------------------------//

	//	GATEWAY ADDRESS
		ETH.GAR[0]=192;
		ETH.GAR[1]=168;
		ETH.GAR[2]=3;
		ETH.GAR[3]=1;
	//	SUBNET MASK
		ETH.SUBR[0]=255;
		ETH.SUBR[1]=255;
		ETH.SUBR[2]=255;
		ETH.SUBR[3]=0;
	//	MAC ADDRESS
		ETH.SHAR[0]=0x00;
		ETH.SHAR[1]=0x08;
		ETH.SHAR[2]=0xDC;
		ETH.SHAR[3]=0x00;
		ETH.SHAR[4]=0x00;
		ETH.SHAR[5]=0x01;

	//	IP ADDRESS
		ETH.SIPR[0]=192;
		ETH.SIPR[1]=168;
		ETH.SIPR[2]=3;
		ETH.SIPR[3]=34,
	//  Socket RX memory
		ETH.RMSR=0x55;
	//  Socket TX memory
		ETH.TMSR=0x55;
	//  S0 Port Number
		ETH.S0_PORT[0]=0x01;
		ETH.S0_PORT[1]=0xF6;
	//	S0 Client IP ADDRESS
		ETH.S0_DIPR[0]=192;
		ETH.S0_DIPR[1]=168;
		ETH.S0_DIPR[2]=3;
		ETH.S0_DIPR[3]=3;
	//	S0 Client IP ADDRESS
		ETH.S0_DPORT[0]=0x01;
		ETH.S0_DPORT[1]=0xF6;

		ETH.gS0_RX_BASE = 0x6000;
		ETH.gS0_RX_MASK = 0x07FF;
		ETH.gS1_RX_BASE = 0x6800;
		ETH.gS1_RX_MASK = 0x07FF;
		ETH.gS2_RX_BASE = 0x7000;
		ETH.gS2_RX_MASK = 0x07FF;
		ETH.gS3_RX_BASE = 0x7800;
		ETH.gS3_RX_MASK = 0x07FF;
		ETH.gS0_TX_BASE = 0x4000;
		ETH.gS0_TX_MASK = 0x07FF;
		ETH.gS1_TX_BASE = 0x4800;
		ETH.gS1_TX_MASK = 0x07FF;
		ETH.gS2_TX_BASE = 0x5000;
		ETH.gS2_TX_MASK = 0x07FF;
		ETH.gS3_TX_BASE = 0x5800;
		ETH.gS3_TX_MASK = 0x07FF;

		ETH.S0_ENserver = 0;			//Actúa como servidor S0_ENserver=1 o cliente S0_ENserver=0

		//----------------------- ETHERNET W5100 Environment-------------------------//


	  //----------------------- WIFI ------------------------//
 	  	Inicializar(&wf); 									//Borra todos los registros de la estructura
		strcpy(wf._WF_Net, WIFI_NET);						//Nombre de la red WIFI  a conectar Fibertel WiFi967 2.4GHz
		strcpy(wf._WF_Pass, WIFI_PASS);						//Password de la red WIFI
		strcpy(wf._TCP_Remote_Server_IP, TCP_SERVER);		//char _TCP_Remote_Server_IP[16];		//IP del Servidor TCP
		strcpy(wf._TCP_Remote_Server_Port, TCP_PORT);		//char _TCP_Remote_Server_Port[16];			//Puerto del Servidor TCP
		strcpy(wf._TCP_Local_Server_IP, TCP_SERVER_LOCAL);
		strcpy(wf._TCP_Local_Server_GWY, TCP_SERVER_LOCAL_GWY);
		strcpy(wf._TCP_Local_Server_MSK, TCP_SERVER_LOCAL_MSK);
		strcpy(wf._TCP_Local_Server_Port, TCP_PORT_LOCAL);
		wf._TCP_Local_Server_EN=0;							//Habilito el Servidor Local
		wf._data2SND[0]=0x00;//strcpy(wf._data2SND,"01;03;00;00;00;0A;C5;CD");//strcpy(wf._data2SND,"20;352;52#");
		wf._data2SND[1]=0x00;
		wf._data2SND[2]=0x00;
		wf._data2SND[3]=0x00;
		wf._data2SND[4]=0x00;
		wf._data2SND[5]=0x06;
		wf._data2SND[6]=0x01;
		wf._data2SND[7]=0x03;
		wf._data2SND[8]=0x00;//strcpy(wf._data2SND,"01;03;00;00;00;0A;C5;CD");//strcpy(wf._data2SND,"20;352;52#");
		wf._data2SND[9]=0x00;
		wf._data2SND[10]=0x00;
		wf._data2SND[11]=0x0A;
		wf._data2SND[12]=0x00;
		wf._data2SND[13]=0x33;
		wf._data2SND[14]=0x34;
		wf._data2SND[15]=0x35;
		wf._n_D2SND=12;
		wf._estado_conexion=100;//Si no se define no arranca	//wf._estado_conexion=1;					//Arranco en WiFi Desconectado
		wf._automatizacion=WF_CONNECT_TCP;//wf._automatizacion=WF_SEND;
		//wf._send_data=1;
		// ----------- INICIO - Seteo de módulo Ethernet W5100 ----------- //
	    // Conectado a SPI2
		// PIN NSS - PortB 12
		spi_no_debug=1;
		ETH.NSS_PORT=GPIOB;
		ETH.NSS_PIN=GPIO_PIN_12;
		ETH.SPI= &hspi2;

		// ----------- FIN - Seteo de módulo Ethernet W5100 ----------- //


	 //----------------------- WIFI ------------------------//

	 //---------------------- ModBUS -----------------------//

		ModBUS_Config(&mb_eth);		//ETHERNET como cliente TCP envía  ModBUS
		mb_eth._mode = CLIENTE;
		ModBUS_Config(&mb_wf);	//WIFI como servidor TCP, recibe comadno ModBUS
		mb_wf._mode = CLIENTE;
		ModBUS_F03_Assign(&mb_wf,3,0xAA55);


	 //---------------------- ModBUS -----------------------//

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Config(SystemCoreClock/1000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  ITM0_Write("\r\n INICIO OK\r\n",strlen("\r\n INICIO OK\r\n"));
  ESP8266_HW_Reset();	//WRNNG Hardcoded	  //Reseteo el modulo desde el pin de RESET
  if (WF_DBG_EN) ITM0_Write("\r\n RESET ESP8266 \r\n",strlen("\r\n RESET ESP8266 \r\n"));
  HAL_TIM_Base_Start(&htim6); //Timer como base de tiempo
  HAL_UART_Receive_IT(&huart4,(uint8_t *)UART_RX_byte,1);
  //HAL_UART_Receive_IT(&huart5,(uint8_t *)datarx1,1); //Habilito recepción por puerto serie 5
  if (ETH_DBG_EN)ITM0_Write("\r\n SET-UP W5100 \r\n",strlen("\r\n SET-UP W5100 \r\n"));
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	 ETH.operacion=SPI_WRITE;
	 ETH.TX[1]= 0;
	 ETH.TX[2]= 1;
	 ETH.TX[3]= 192;

	 eth_init(&ETH);

	 eth_socket_init(&ETH,0);


SPI_READ_EN=1;
ETH.operacion=SPI_READ;
ETH.TX[1]= 0;
ETH.TX[2]= 1;
ETH.TX[3]= 0;



  if(ESP8266_HW_Init(&huart4)==1)
  {
	  ESP_HW_Init=1;
	  //HAL_UART_Transmit(&huart5, "\r\n ESP HW Init OK\r\n",strlen("\r\n ESP HW Init OK\r\n"),100);
	  if (WF_DBG_EN) ITM0_Write("\r\n ESP HW Init OK\r\n",strlen("\r\n ESP HW Init OK\r\n"));
  }
  else
  {
	  ESP8266_HW_Reset(); //WRNNG Hardcoded
	  if(ESP8266_HW_Init(&huart4)==1)
	  {
		  ESP_HW_Init=1;
		  //HAL_UART_Transmit(&huart5, "\r\n ESP HW Init OK\r\n",strlen("\r\n ESP HW Init OK\r\n"),100);
		  if (WF_DBG_EN) ITM0_Write("\r\n ESP HW Init OK\r\n",strlen("\r\n ESP HW Init OK\r\n"));
	  }
	  else
	  {
		  ESP_HW_Init=0;
		  //HAL_UART_Transmit(&huart5, "\r\n ESP HW Init Fail\r\n",strlen("\r\n ESP HW Init Fail\r\n"),100);
		  if (WF_DBG_EN)  ITM0_Write("\r\n ESP HW Init Fail\r\n",strlen("\r\n ESP HW Init Fail\r\n"));
	  }
  }

  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {//2
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//----------------INSTRUCCIONS POR PUERTO SERIE---------------------

//----------------INSTRUCCIONS POR PUERTO SERIE---------------------

/**************[ INICIO PIDO ENVIAR DATOS ]**************/


	  if (ESP_HW_Init==1)
	  {
			if((WF_SND_FLAG==1)&&(wf._TCP_Local_Server_EN==0)&&(wf._estado_conexion>=609))
			{	wf_snd_flag_ticks=0;
				WF_SND_FLAG=0;
				ModBUS_F03_Request(&mb_wf, 0 , 10);
				ModBUS(&mb_wf);							// Create ModBUS info to be sent
				CopiaVector(wf._data2SND,mb_wf._MBUS_2SND,mb_wf._n_MBUS_2SND,0,'A');
				wf._n_D2SND=mb_wf._n_MBUS_2SND;

				if(wf._automatizacion < WF_SEND)		// Send only with automation sent diasabled
				{
					EnviarDatos(&wf);
					wf._estado_conexion=TCP_SND_EN_CURSO;
				}
			}
	  }
/**************[ FIN PIDO ENVIAR DATOS ]**************/

		if ((FLAG_UART2==1)||(FLAG_TIMEOUT==1))  //Si recibí datos o me fui por TimeOUT
		{
			if(FLAG_UART2==1)
				{
					CopiaVector(wf._uartRCVD,UART_RX_vect_hld,UART_RX_items,1,CMP_VECT);
					FLAG_UART2=0;

						if (error_rxdata==3)
						{
							error_rxdata=0;
						}
						if (error_rxdata==1)
						{
							error_rxdata=5;
							error_rxdata=0;
						}
				}
			if(FLAG_TIMEOUT==1)
					{
						FLAG_TIMEOUT=0;
					}

			if (ESP_HW_Init==1) //Si el módulo se inició correctamente
				{
					/*************** Copio y proceso info recibida ***************/
					wf._n_orig=UART_RX_items;
					CopiaVector(wf._uartRCVD,UART_RX_vect_hld,UART_RX_items,1,CMP_VECT);
					resultado=AT_ESP8266_ND(&wf);

					/*************** Si recibo datos y estan correctos me fijo que son ***************/

					if ((wf._new_data_rcv==1)&&(wf._estado_rcv_data==99))
					{

						CopiaVector(mb_wf._MBUS_RCVD,wf._dataRCV,wf._n_dataRCV,0,'A');
						mb_wf._n_MBUS_RCVD=wf._n_dataRCV;

						ModBUS(&mb_wf);

						CopiaVector(wf._data2SND,mb_wf._MBUS_2SND,mb_wf._n_MBUS_2SND,0,'A');
						wf._n_D2SND=mb_wf._n_MBUS_2SND;
						wf._new_data_rcv=0;//
						wf._send_data=1;
					}else
						{
							// DATA ERRONEA NO SE PROCESA
						}
					}

		}

		if (ESP_HW_Init==1) //Si el módulo se inició correctamente
			{
				conexion=WiFi_Conn_ND(&wf,&huart4,1);	//Tiene que ir en el main el chequeo es constante
			}
		if (esp_restart==1) //WRNNG Hardcoded RESET WIFI
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_Delay(2000);//210419
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
				HAL_Delay(5000);//210419
				esp_restart=0;
			}

  }//2
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x19;
  sTime.Minutes = 0x30;
  sTime.Seconds = 0x31;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_THURSDAY;
  DateToUpdate.Month = RTC_MONTH_MARCH;
  DateToUpdate.Date = 0x25;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 90;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7200-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_OSC2_GPIO_Port, DEBUG_OSC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_OSC1_GPIO_Port, DEBUG_OSC1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|SPI2_NSS_Pin|DBG_ESP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_WIFI_GPIO_Port, RST_WIFI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DEBUG_OSC2_Pin */
  GPIO_InitStruct.Pin = DEBUG_OSC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_OSC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_OSC1_Pin RST_WIFI_Pin */
  GPIO_InitStruct.Pin = DEBUG_OSC1_Pin|RST_WIFI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_NSS_Pin DBG_ESP_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin|DBG_ESP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int ITM0_Write( char *ptr, int len)
{
 int DataIdx;

  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	ms_ticks++;	//100 ms

	ESP_ticks++;
	if(mb_eth._w_answer) ETH_ticks++;
	if ( mb_eth._w_answer && (mb_eth._timeout < ETH_ticks))
		{
			mb_eth._w_answer=0;
			ETH_ticks=0;
		}

// ENVIO DATOS WF ---------------------------------------------------------------//

	if((wf._estado_conexion==609 || wf._estado_conexion==700)&&(wf._TCP_Local_Server_EN==0))  wf_snd_flag_ticks++;

	if(wf_snd_flag_ticks>= 2000 && wf._ejecucion!=1 && wf._TCP_Local_Server_EN==0)		 	  WF_SND_FLAG=1;

// ENVIO DATOS WF ----------------------------------- ---------------------------//

if (ms_ticks==100)//(ms_ticks==250)//(ms_ticks==50)
  {
	  dbgn++;
	  ms_ticks=0;
	  min_ticks++;


	  	if(MBUS_ticks==360) MBUS_ticks=0;

	  	if (asc==0)  MBUS_ticks++;
	  	if (MBUS_ticks==100) asc=1;
	  	if (asc==1) MBUS_ticks--;
	  	if (MBUS_ticks==0) asc=0;


	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
	  if(spi_no_debug)
	  	  {
	  if(SPI_READ_EN)
	  {
	     switch(eth_rd_SOCKET_STAT(&ETH,0))	//Check Socket status
	     {
			 case SOCK_CLOSED :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_CLOSED \r\n",strlen("\r\nS0_SOCK_CLOSED \r\n"));
					eth_wr_SOCKET_CMD(&ETH, 0 ,OPEN );
				 }
			 break;
			 case  SOCK_INIT :
				 {
					 if(ETH.S0_ENserver == 1)
					 {
						 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_INIT \r\n",strlen("\r\nS0_SOCK_INIT \r\n"));
							eth_wr_SOCKET_CMD(&ETH, 0, LISTEN );
					 }
					 else
					 {
						 	eth_wr_SOCKET_CMD(&ETH,0, CONNECT);																				//only for server
						 	if (ETH_DBG_EN)ITM0_Write("\r\nETH-W5100-CONNECT\r\n",strlen("\r\nETH-W5100-CONNECT\r\n"));
					 }

				 }
			 break;
			 case SOCK_LISTEN :
				 {
					 if (ETH_DBG_EN)ITM0_Write("\r\nS0_SOCK_LISTEN \r\n",strlen("\r\nS0_SOCK_LISTEN \r\n"));
				 }
			 break;
			 case SOCK_SYNSENT :
				 {
					 if (ETH_DBG_EN)ITM0_Write("\r\nS0_SOCK_SYNSENT \r\n",strlen("\r\nS0_SOCK_SYNSENT \r\n"));
				 }
			 break;
			 case SOCK_SYNRECV :
				 {
					 if (ETH_DBG_EN)ITM0_Write("\r\nS0_SOCK_SYNRECV \r\n",strlen("\r\nS0_SOCK_SYNRECV \r\n"));
				 }
			 break;
			 case SOCK_ESTABLISHED :
				 {
					 if (ETH_DBG_EN)ITM0_Write("\r\nS0_SOCK_ESTABLISHED \r\n",strlen("\r\nS0_SOCK_ESTABLISHED \r\n"));

					if (ETH.S0_ENserver == 1)  // Si el puerto Ethernet actúa como server (Recibe datos conexión mas pedido mbus
					{

							S0_get_size = SPI_ETH_REG(&ETH, S0_RX_SZ_ADDR_BASEHH,S0_RX_SZ_ADDR_BASEHL ,SPI_READ, spi_Data,2);
							if(S0_get_size != 0x00)
							{
								eth_rd_SOCKET_DATA(&ETH,0,&rx_mem_pointer,S0_get_size); // read socket data
								SPI_ETH_WR_REG_16(&ETH,S0_RX_RD0,rx_mem_pointer );		// write rx memory pointer
								eth_wr_SOCKET_CMD(&ETH,0,RECV);							// write command to execute
								while(eth_rd_SOCKET_CMD(&ETH,0))						// wait until end of command execution
								{}

								CopiaVector(mb_eth._MBUS_RCVD, ETH.data, S0_get_size, 0, ADC_VAL );
								mb_eth._n_MBUS_RCVD=S0_get_size;
								if(ModBUS_Check(mb_eth._MBUS_RCVD, mb_eth._n_MBUS_RCVD))		//Ckecks ModBUS type data
								{
									ModBUS(&mb_eth);										//ModBUS protocol execution
									CopiaVector(ETH.data, mb_eth._MBUS_2SND, mb_eth._n_MBUS_2SND, 0, ADC_VAL);
								}
								else
								{
									if (ETH_DBG_EN) ITM0_Write("\r\n NO MBUS \r\n",strlen("\r\n\r\n NO MBUS \r\n\r\n"));
								}

								send_size=mb_eth._n_MBUS_2SND;  //ModBUS data qty

								eth_wr_SOCKET_DATA(&ETH,0, &tx_mem_pointer, send_size);	// write socket data
								SPI_ETH_WR_REG_16(&ETH,0x424,tx_mem_pointer);			// write tx memory pointer
								eth_wr_SOCKET_CMD(&ETH,0,SEND);							// write command to execute
								while(eth_rd_SOCKET_CMD(&ETH,0))						// wait until end of command execution
								{}

							}
					}
					else	// Puerto ethernet labura como esclavo, se conecta al server para pedir datos
					{

						if (mb_eth._w_answer==0)
						{
							//Si ya envié vuelvo a enviar

							ETH.data[0]=0x00;
							ETH.data[1]=0x00;
							ETH.data[2]=0x00;
							ETH.data[3]=0x00;
							ETH.data[4]=0x00;
							ETH.data[5]=0x06;
							ETH.data[6]=0x01;
							ETH.data[7]=0x03;
							ETH.data[8]=0x00;
							ETH.data[9]=0x00;
							ETH.data[10]=0x00;
							ETH.data[11]=0x0A;
							send_size=12;

							ModBUS_F03_Request(&mb_eth,0,15);
							CopiaVector(ETH.data, mb_eth._MBUS_2SND, 12, 0, ADC_VAL );

							eth_wr_SOCKET_DATA(&ETH,0, &tx_mem_pointer, send_size);	// write socket data
							SPI_ETH_WR_REG_16(&ETH,0x424,tx_mem_pointer);			// write tx memory pointer
							eth_wr_SOCKET_CMD(&ETH,0,SEND);							// write command to execute
							while(eth_rd_SOCKET_CMD(&ETH,0))						// wait until end of command execution
							{}
							mb_eth._w_answer=1;	// Waiting answer flag
							ETH_ticks=0;	// restart counting
							if (ETH_DBG_EN) ITM0_Write("\r\n SENT MBUS REQ \r\n",strlen("\r\n\r\n SENT MBUS REQ \r\n\r\n"));
						}
						else
						{
						S0_get_size = SPI_ETH_REG(&ETH, S0_RX_SZ_ADDR_BASEHH,S0_RX_SZ_ADDR_BASEHL ,SPI_READ, spi_Data,2);
							if(S0_get_size != 0x00)
							{
								eth_rd_SOCKET_DATA(&ETH,0,&rx_mem_pointer,S0_get_size); // read socket data
								SPI_ETH_WR_REG_16(&ETH,S0_RX_RD0,rx_mem_pointer );		// write rx memory pointer
								eth_wr_SOCKET_CMD(&ETH,0,RECV);							// write command to execute
								while(eth_rd_SOCKET_CMD(&ETH,0))						// wait until end of command execution
								{}

								CopiaVector(mb_eth._MBUS_RCVD, ETH.data, S0_get_size, 0, ADC_VAL );
								mb_eth._n_MBUS_RCVD=S0_get_size;
								if(ModBUS_Check(mb_eth._MBUS_RCVD, mb_eth._n_MBUS_RCVD))		//Ckecks ModBUS type data
									{
										mb_eth._w_answer=0;  									//Si el mensaje recibido ya es modbus digo que ya recibi
										ETH_ticks=0;
										ModBUS(&mb_eth);										//ModBUS protocol execution
										//CopiaVector(ETH.data, mb_eth._MBUS_2SND, mb_eth._n_MBUS_2SND, 0, ADC_VAL);
										CopiaVector(ETH.swap, mb_eth._MBUS_RCVD, mb_eth._n_MBUS_RCVD, 0, ADC_VAL);
										CopiaVector(mb_wf._Holding_Registers, mb_eth._Holding_Registers, 64, 0, ADC_VAL);
										if (ETH_DBG_EN) ITM0_Write("\r\n RCVD MBUS REQ \r\n",strlen("\r\n\r\n RCVD MBUS REQ \r\n\r\n"));
									}
									else
										{
										if (ETH_DBG_EN) ITM0_Write("\r\n NO MBUS \r\n",strlen("\r\n\r\n NO MBUS \r\n\r\n"));
										}


							}
						}
					}
				 }
			 break;
			 case SOCK_FIN_WAIT :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_FIN_WAIT \r\n",strlen("\r\nS0_SOCK_FIN_WAIT \r\n"));
				 }
			 break;
			 case SOCK_CLOSING :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_CLOSING \r\n",strlen("\r\nS0_SOCK_CLOSING \r\n"));
				 }
			 break;
			 case  SOCK_TIME_WAIT :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_TIME_WAIT \r\n",strlen("\r\nS0_SOCK_TIME_WAIT \r\n"));
					eth_wr_SOCKET_CMD(&ETH,0, DISCON );
					while( SPI_ETH_REG(&ETH, S0_CR_ADDR_BASEH,S0_CR_ADDR_BASEL ,SPI_READ, spi_Data,1))
					{}
				 }
			 break;
			 case SOCK_CLOSE_WAIT :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_CLOSE_WAIT \r\n",strlen("\r\nS0_SOCK_CLOSE_WAIT \r\n"));
					eth_wr_SOCKET_CMD(&ETH,0,DISCON );
					while( SPI_ETH_REG(&ETH, S0_CR_ADDR_BASEH,S0_CR_ADDR_BASEL ,SPI_READ, spi_Data,1))
					{}
				 }
			 break;
			 case SOCK_LAST_ACK :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_LAST_ACK \r\n",strlen("\r\nS0_SOCK_LAST_ACK \r\n"));
				 }
			 break;
			 case SOCK_UDP :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_UDP \r\n",strlen("\r\nS0_SOCK_UDP \r\n"));
				 }
			 break;
			 case  SOCK_IPRAW :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_IPRAW \r\n",strlen("\r\nS0_SOCK_IPRAW \r\n"));
				 }
			 break;
			 case  SOCK_MACRAW :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_MACRAW \r\n",strlen("\r\nS0_SOCK_MACRAW \r\n"));
				 }
			 break;
			 case SOCK_PPOE :
				 {
					 if (ETH_DBG_EN) ITM0_Write("\r\nS0_SOCK_PPOE \r\n",strlen("\r\nS0_SOCK_PPOE \r\n"));
				 }
			 break;

			 default:
				 {

				 }
	     }
	  }
	  }else
	  	  {
		  //ETH.operacion=SPI_READ;
		  //ETH.TX[3]=0x00;
		  SPI_ETH(&ETH);
	  	  }
	  if(min_ticks==2)//if(min_ticks==10)
		  {
		  	  min_ticks=0;  /* SETEO CADA 2 min*/
		  }
  }

	if(EN_UART1_TMR==1) UART1_ticks++;

	if(UART1_ticks>=2)//if(UART1_ticks>=10)
	{
		UART1_ticks=0;
		FLAG_UART1=1;
		EN_UART1_TMR=0;
		items_rx=uart1pass;
		uart1pass=0;
	}

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	if(wf._estado_conexion==4)//if((wf._estado_conexion!=1)&&(wf._estado_conexion!=2)&&(resultado!=20)&&(resultado!=24)) //Solo cuento cuando no estahaciendo otra cosa
	{
		ticks++;
	}
	else
	{
		ticks=0;
	}

if(wf._ejecucion==1)
	{
		if (FLAG_TIMEOUT!=1)
		{
			if(wf._instruccion!=2) wf._ticks++;//-----------------------Solo cuento una vez reconcido el timeout, cuando entro al timeout no cuento
			if(wf._instruccion==2) wf._ticks2++;
		}


		if ((wf._instruccion!=2)&&(wf._ticks > 5500)) //if (wf._ticks > 5000)
		{
			FLAG_TIMEOUT=1;
			if(huart4.Instance->CR1 == 0x200C)  //--------------------Evito error UART colgado
			{
				HAL_UART_Receive_IT(&huart4,(uint8_t *)UART_RX_byte,1);
				EN_UART2_TMR=0; //OBS-VER Para que me vuelva a habilitar el timer
			}
			//wf._ticks=0;
		}
		if ((wf._instruccion==2)&&(wf._ticks2 > 20500)) //if (wf._ticks > 5000)
		{
			FLAG_TIMEOUT=1;
			if(huart4.Instance->CR1 == 0x200C)  //--------------------Evito error UART colgado
			{
				HAL_UART_Receive_IT(&huart4,(uint8_t *)UART_RX_byte,1);
				EN_UART2_TMR=0; //OBS-VER Para que me vuelva a habilitar el timer
			}
			//wf._ticks=0;
		}

	}
	else
	{
		wf._ticks=0;
	}
  /* USER CODE END SysTick_IRQn 1 */
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart4)
{
	 volatile int aore=0;
	 volatile int bore=0;

	// if ( UART_FLAG_ORE == HAL_UART_GetError(huart4))
	//{
	//Al leer los registros de esta forma SR y luego DR se resetean los errores de Framing Noise y Overrun FE NE ORE
	//}
	 	 wf._debug_count9++;
		aore=huart4->Instance->SR;
		bore=huart4->Instance->DR;


	//HAL_UART_Transmit_IT(&huart5,"U4",strlen("U4"));
	 HAL_UART_DeInit(huart4);
	 MX_UART4_Init();
	 HAL_UART_Receive_IT(huart4,(uint8_t *)UART_RX_byte,1);
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim2)
{

		wf._debug_count10++;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *INTSERIE)
{

	if(INTSERIE->Instance==UART5)
	 {
		EN_UART1_TMR=1;									//Habilito Timeout de software
		UART1_ticks=0;										//Reseteo Timer
		datarx_uart1[uart1pass]=datarx1[0];
		uart1pass++;
		if(uart1pass>=384) uart1pass=384;									//limito el vector para evitar que cuelgue el micro
		HAL_UART_Receive_IT(INTSERIE,(uint8_t *)datarx1,1);
	 }

	if(INTSERIE->Instance==UART4)
		 {
			UART_RX_vect[UART_RX_pos]=UART_RX_byte[0];
			UART_RX_pos++;
			if(UART_RX_pos>=384) UART_RX_pos=384;
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);//HAL_TIM_Base_Start_IT(&htim7);	//Habilito el timer
			TIM2->CNT=1;
			EN_UART2_TMR=1;	//Habilito Timeout de software
			HAL_UART_Receive_IT(INTSERIE,(uint8_t *)UART_RX_byte,1);
		 }

 }

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim2)
	{
		 HAL_TIM_OC_Stop_IT(htim2, TIM_CHANNEL_1); //Paro el timer
		 FLAG_UART2=1;
		 EN_UART2_TMR=0;
		 UART_RX_items=UART_RX_pos;
		 UART_RX_pos=0;
		 UART_RX_vect[384]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
		 CopiaVector(UART_RX_vect_hld,UART_RX_vect,UART_RX_items,1,CMP_VECT);
		 HAL_UART_Receive_IT(&huart4,(uint8_t *)UART_RX_byte,1); //Habilito le recepcón de puerto serie al terminar

		 if (WF_DBG_EN==1)
		 {
			 ITM0_Write((uint8_t *)UART_RX_vect_hld,UART_RX_items);
		 }


}
void Actualizar_RXdata(int x)
 {

	 if (x==1)
	 {
		int h=0;													//Borro el vector vectrx
		/*while(h<384)
				{
						RXdata_uart1[h]=0;
						h++;
				}*/
		h=0;
		while(h<items_rx)								//Copio el vector recibido en vectrx
			{
						RXdata_uart1[h]=datarx_uart1[h];
						h++;
			}
		}
	 if (x==2)
	 {
		int h2=0;													//Borro el vector vectrx
		/*while(h2<384)
				{
						UART_RX_vect[h2]=0;
						h2++;
				}*/
		h2=0;
		while(h2<UART_RX_items)								//Copio el vector recibido en vectrx
			{
						UART_RX_vect[h2]=UART_RX_vect[h2];
						h2++;
			}
		}
 }

void ESP8266_HW_Reset(void)
{
	  ESP_REinit=0;
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	  HAL_Delay(2000);											//Tiempo de reset del módulo
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);		//Habilito módulo
}
uint8_t ESP8266_HW_Init(UART_HandleTypeDef *SerialPort) //Devuelve 1 si reinició OK, y 0 si no
{
	  do{
		  HAL_UART_Transmit(SerialPort, "AT+RESTORE\r\n",strlen("AT+RESTORE\r\n"),100);
		  HAL_Delay(500);

		  wf._n_fcomp=strlen("ready");
		  wf._n_orig=UART_RX_items;

		  while(FT_String_ND(UART_RX_vect_hld,&wf._n_orig,"ready",&wf._n_fcomp,wf._uartRCVD_tok,&wf._n_tok,&ntestc,&wf._id_conn,FIND)!=1)
		  {
			  	  wf._n_orig=UART_RX_items;
			  	  if (ESP_ticks>=5000)
			  		 {
			  		 ESP_InitF=1;
			  		 break;
			  		 }
		  }

		  if(ESP_InitF==0)	//Si encontró la sentencia anterior analizo la siguiente
		  {
			  wf._n_fcomp=strlen("ready");
			  wf._n_orig=UART_RX_items;
			  while(FT_String_ND(UART_RX_vect_hld,&wf._n_orig,"ready",&wf._n_fcomp,wf._uartRCVD_tok,&wf._n_tok,&ntestc,&wf._id_conn,FIND)!=1)
			  {
				  wf._n_orig=UART_RX_items;
				  if (ESP_ticks>=5000)
					 {
					 break;
					 }
			  }
		  }

		  if (ESP_ticks<5000)
		  {
			  ESP_REinit=10;
			  ESP_ticks=0;
		  }
		  else
		  {
			  ESP_REinit++;
			  ESP_ticks=0;
		  }

	  } while (ESP_REinit<=5);

	  if(ESP_REinit==10)
	  {
		  return(1);
	  }
	  else
	  {
		  return(0);
	  }
}
void BorrarVect(void)
{
	wf._uartRCVD[0]='\0';
	wf._uartRCVD[1]='\0';
	wf._n_orig=2;
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
