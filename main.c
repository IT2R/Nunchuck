/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   STM32F4xx HAL API Template project 
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX

#include "Driver_I2C.h"                 // ::CMSIS Driver:I2C

#include "Board_LED.h"                  // ::Board Support:LED

#define SLAVE_I2C	_ADDR       0x00		// Adresse esclave sur 7 bits

#define nunchuk_ID 0xA4 >> 1   //L.A 

#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS2                  // when RTE component CMSIS RTOS2 is used
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#endif

//_______________________INIT_______________________//

#include "stdio.h " 
#include "Driver_SPI.h"                 // ::CMSIS Driver:SPI


extern ARM_DRIVER_I2C Driver_I2C1;



 unsigned char joy_x_axis;
 unsigned char joy_y_axis;
 int accel_x_axis;
 int accel_y_axis;
 int accel_z_axis;
 unsigned char z_button;
 unsigned char c_button;
	
unsigned char buffer[6];// array to store arduino output //L.A
int cnt = 0;      //L.A



//void write1byte (unsigned char composant, unsigned char registre, unsigned char valeur);
//unsigned char read1byte (unsigned char composant, unsigned char registre);

// INIT I2C1  




void Init_I2C1(void){
	Driver_I2C1.Initialize(NULL);
	Driver_I2C1.PowerControl(ARM_POWER_FULL);
	Driver_I2C1.Control(	ARM_I2C_BUS_SPEED,				// 2nd argument = débit
							ARM_I2C_BUS_SPEED_STANDARD  );		// 100 kHz 
//	Driver_I2C1.Control(	ARM_I2C_BUS_CLEAR,	0 );
}
 
 //_______________________FIN INIT_______________________//


void write1byte (unsigned char composant, unsigned char valeur1, unsigned char valeur2)
{
	uint8_t tab[2];
	tab[0]=valeur1;
	tab[1]=valeur2;
	
	
	Driver_I2C1.MasterTransmit(composant, tab, 2, false); 
	
	while(Driver_I2C1.GetStatus().busy==1); 
	
}




unsigned char read1byte (unsigned char composant){
	unsigned int valeur;
	unsigned char lecture;
	uint8_t tab[8]; // (non-signé) sur 8 bytes 
	tab[0]=valeur;
	

//Envoyer les données
	
	Driver_I2C1.MasterTransmit(composant, tab, 1, true);
	while(Driver_I2C1.GetStatus().busy==1);

//Recevoir les données
	
	Driver_I2C1.MasterReceive(composant, &lecture, 1, false);
	while(Driver_I2C1.GetStatus().busy==1);
	return lecture;
}



//_______________________Fin 2 _______________________





//_______________________LA _______________________//
/*
void setup ()
{
 // I2C_begin (9600);
 Wire.begin (); // join i2c bus with address 0x52
 nunchuck_init (); // send the initilization handshake
 // delay (100);  // a remplacer avec timer / boucle for 
}

*/

void nunchuck_init()
{
	write1byte(0x52 , 0x40 , 0x00) ; 
}

/* 
void send_zero ()
{
 Wire.beginTransmission (nunchuk_ID); // transmit to device 0x52
 Wire.send (0x00); // sends one byte
 Wire.endTransmission (); // stop transmitting
}
*/


// Print the input data we have recieved
// accel data is 10 bits long
// so we read 8 bits, then we have to add
// on the last 2 bits.


void joystick ()
{
	
 joy_x_axis = buffer[0];
 joy_y_axis = buffer[1];
 accel_x_axis = (buffer[2]) << 2;
 accel_y_axis = (buffer[3]) << 2;
 accel_z_axis = (buffer[4]) << 2;
	
	
 // byte outbuf[5] contains bits for z and c buttons
 // it also contains the least significant bits for the accelerometer data
 // so we have to check each bit of byte outbuf[5]
	
	
 if ((buffer[5] & 0x01)!=0)
 { z_button = 1; }
 else
 { z_button = 0; }
 if ((buffer[5] & 0x02)!=0)

 { c_button = 1; }
 else
 { c_button = 0; }
 accel_x_axis += ((buffer[5]) >> 2) & 0x03;
 accel_y_axis += ((buffer[5]) >> 4) & 0x03;
 accel_z_axis += ((buffer[5]) >> 6) & 0x03;
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
char nunchuk_decode_byte (char x)
{
 x = (x ^ 0x17) + 0x17;
 return x;
}


void loop ()
{

 //Wire.requestFrom (nunchuk_ID, 6); // request data from nunchuck

	
	 // If we recieved the 6 bytes, then go print them
 if (cnt >= 5)
 {
 joystick ();
 }
 
 
		read1byte(0x52) ; 
	// remise à 0 ? 
  //delay (100); // remplacer avec timer / boucle for 
} 


//_______________________Fin LA _______________________//






/* I2C Driver */
static ARM_DRIVER_I2C *I2Cdrv = &Driver_I2C1;
#ifdef RTE_CMSIS_RTOS2_RTX5
/**
  * Override default HAL_GetTick function
  */
uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  if (osKernelGetState () == osKernelRunning) {
    return ((uint32_t)osKernelGetTickCount ());
  }

  /* If Kernel is not running wait approximately 1 ms then increment 
     and return auxiliary tick counter value */
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
}

#endif

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */







int main(void)
{

	char composant, addX; 
	int Lecture ; 
	char ETAT = 0 ; 

	
	
	
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  SystemCoreClockUpdate();

  /* Add your application code here
     */
	//#ifdef RTE_CMSIS_RTOS2
  /* Initialize CMSIS-RTOS2 */
  osKernelInitialize ();
	
	//NVIC_SetPriority(USART2_IRQn,2);		// nécessaire ? (si LCD ?)
	
	LED_Initialize();
	Init_I2C1() ; 
	
	nunchuck_init();  //L.A
	joystick() ; 


  /* Create thread functions that start executing, 
  Example: osThreadNew(app_main, NULL, NULL); */
	
  /* Start thread execution */
  osKernelStart();
	//LED_On (3);
//#endif
	//osDelay(osWaitForever);
	
  /* Infinite loop */
 








while (1)
  {
		write1byte(0x52 , 0x00 , 0x00) ;
		read1byte(0x53) ;  
	/*
			
	
	while(1)
  {
	switch(ETAT) 
		{ 
		case 0 : 
		
		if(((joy_x_axis) && (joy_y_axis)) > 0) 
		{ 
				ETAT = 1 ;   // avancer 
		}
		if(((joy_x_axis) && (joy_y_axis)) < 0)
		{
 			ETAT=2 ;     // reculer
		}
	
    
		break ; 

// ________________ Avancer ___________________  // 

		
		case 1 : 	//code pour avancer 
		{
				
			
			
			
		
		if( (z_button == 1) && (c_button ==0) ) 
		{ 
		            // [foncion à modifier librement] 
		}
		
		if ((c_button == 1) && (z_button == 0) ) 
		{
               // [foncion à modifier librement] 
		}
		
		if ((c_button == 1) && (z_button == 1) ) 
		{
			         // [foncion à modifier librement] 
		}
		if(((joy_x_axis) && (joy_y_axis)) == 0)
		{
			ETAT=0;  //arrêt
		}
		
		break;
		}
		
// ________________ Recul ___________________ //
		
		case 2 : 	//code pour avancer 
		{
			
			
			
			
		if( (z_button == 1) && (c_button ==0) ) 
		{ 
		            // [foncion à modifier librement] 
		}
		
		if ((c_button == 1) && (z_button == 0) ) 
		{
               // [foncion à modifier librement] 
		}
		
		if ((c_button == 1) && (z_button == 1) ) 
		{
			         // [foncion à modifier librement] 
		}
		
		if(((joy_x_axis) && (joy_y_axis)) == 0)
		{
			ETAT=0;   // arrêt
		}
		
		
		
		break;
		}		
		
		
		
		
		
  }
}

  */
		
  }
	}


	
	
	
	
	
	
	
	
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
