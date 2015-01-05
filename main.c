/**
  ******************************************************************************
  * @file    main.c 
  * @author  MMY Application Team
  * @version V1.0.0
  * @date    08/21/2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MMY-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include "hw_config.h"

//#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_flash.h"
//#include "stm32f10x_exti.h"

#include "drv_lcdspi_ili9341.h"
#include "drv_LED.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "stm32f10x_pwr.h"

//#include "lib_ISO7816_Password.h"
//#include "lib_STProprietary_feature.h"

//#include "menu.h"

/* -------------------------------------------------------------------------- 
* Delay TIMER configuration (ms)
* --------------------------------------------------------------------------
* 72 MHz / 72 = 1MHz (1µs)
* 1µs * 1000 + 1µs ~= 1ms	
* -------------------------------------------------------------------------- */ 
#define TIMER_DELAY												TIM2
#define TIMER_DELAY_PERIOD								1000
#define TIMER_DELAY_PRESCALER							72
#define TIMER_DELAY_CLOCK									RCC_APB1Periph_TIM2


/** 
 * @brief  NVIC definitions 
 */
#define TIMER_DELAY_PREEMPTION_PRIORITY						2
#define TIMER_DELAY_SUB_PRIORITY									2
#define TIMER_DELAY_IRQ_CHANNEL										TIM2_IRQn



/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//void Demo_Init(void);
void InterruptConfig(void);
void SysTick_Configuration(void);
void IntExtOnOffConfig(FunctionalState NewState);
void GPIO_Config(void);
void LedShow_Init(void);
void LedShow(FunctionalState NewState);
uint32_t Get_LedShowStatus(void);
void CheckBitmapFilesStatus(void);

void InitJoystick(void);
static uint8_t ReadKey(void);

void min_sec(char* a);
void itoa(char* a, int i, int n);
void adjust_time(void);

int min = 0;
int sec = 0;
// joystick stuff

/* Private types ------------------------------------------------------------*/
/* Private constants --------------------------------------------------------*/
#define J_SEL_PIN				GPIO_Pin_6
#define J_SEL_PORT			GPIOB
#define J_SEL_CLOCK			RCC_APB2Periph_GPIOB

#define J_DOWN_PIN			GPIO_Pin_7
#define J_DOWN_PORT			GPIOB
#define J_DOWN_CLOCK		RCC_APB2Periph_GPIOB

#define J_LEFT_PIN			GPIO_Pin_5
#define J_LEFT_PORT			GPIOB
#define J_LEFT_CLOCK		RCC_APB2Periph_GPIOB

#define J_RIGHT_PIN			GPIO_Pin_8
#define J_RIGHT_PORT		GPIOB
#define J_RIGHT_CLOCK		RCC_APB2Periph_GPIOB

#define J_UP_PIN				GPIO_Pin_9
#define J_UP_PORT				GPIOB
#define J_UP_CLOCK			RCC_APB2Periph_GPIOB

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define  MAX_MENU_LEVELS 4
#define  NOKEY  0
#define  SEL    1
#define  RIGHT  2
#define  LEFT   3
#define  UP     4
#define  DOWN   5
#define  KEY    6

#define NO_EVENT   0
#define RF_EVENT 	 1
#define KEY_EVENT  2



void RTC_Configuration(void);
void NVIC_Configuration(void);
void RTC_IRQHandler(void);
void RTC_Timer(void);


void bounce(int* dir, int* v, int min, int max, int speed);
uint16_t next_color();
uint16_t next_position(void);

static uint8_t ReadKey(void)
{
  if(!GPIO_ReadInputDataBit( J_RIGHT_PORT, J_RIGHT_PIN ))
  {
    while(GPIO_ReadInputDataBit( J_RIGHT_PORT, J_RIGHT_PIN ) == Bit_RESET) { } 
    return RIGHT; 
  }	
  if(!GPIO_ReadInputDataBit( J_LEFT_PORT, J_LEFT_PIN ))
  {
    while(GPIO_ReadInputDataBit( J_LEFT_PORT, J_LEFT_PIN ) == Bit_RESET) { }
    return LEFT; 
  }
  if(!GPIO_ReadInputDataBit( J_UP_PORT, J_UP_PIN ))
  {
    while(GPIO_ReadInputDataBit( J_UP_PORT, J_UP_PIN ) == Bit_RESET) { }
    return UP; 
  }
  if(!GPIO_ReadInputDataBit( J_DOWN_PORT, J_DOWN_PIN ))
  {
    while(GPIO_ReadInputDataBit( J_DOWN_PORT, J_DOWN_PIN ) == Bit_RESET) { } 
    return DOWN; 
  }
  if(!GPIO_ReadInputDataBit( J_SEL_PORT, J_SEL_PIN ))
  {
    while(GPIO_ReadInputDataBit( J_SEL_PORT, J_SEL_PIN ) == Bit_RESET) { } 
    return SEL; 
  }
  else 
  {
    return NOKEY;
  }
}


/** @addtogroup User_Appli
 * 	@{
 *  @brief      <b>This folder contains the application files</b> 
 */
 
/** @addtogroup Main
 * 	@{
 *  @brief      This file contains the entry point of this demonstration firmeware 
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t SELStatus = 0;

//static __IO uint16_t					counter_delay_ms;

uint8_t I2C_Cmd_Flag = 0;
uint8_t NFCT4_SS_Flag = 0;
uint8_t NFCT4_CC_Flag = 0;
uint8_t NFCT4_NDEF_Flag = 0;
uint8_t ISO7816_Flag = 0;
uint8_t ST_Flag = 0;


static void LED_Init(void);
static void LED_AllOff(void);
static void LED_AllOn(void);
static void LED_Bump(void);

/** @addtogroup Main_Private_Functions
 * 	@{
 */

/**
  * @brief  initialize Led state.
  * @param  None
  * @retval None
  */
static void LED_Init(void)
{
	Led_TypeDef ledx;
	/* configure the LED  */
	for( ledx=LED1; ledx<LEDn; ledx++)
	{
		LED_Config(ledx);	
	}
}



int main(void)
{			
  ErrorStatus HSEStartUpStatus = ERROR;
	
	/* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 	
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

  /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC 
         | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG 
         | RCC_APB2Periph_AFIO, ENABLE);
  
  /* TIM1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

/*------------------- Resources Initialization -----------------------------*/	
	/* init the interuption */ 
	Interrupts_Config();
	
	Timer_Config();
	
	/* Configure the systick */    
  SysTick_Configuration();

/*------------------- Drivers Initialization -------------------------------*/	
	
	/* Initialize the Joystick */
	InitJoystick();
	
	/* configure the LED  */
	LED_Init();	
	//LED_Show();

	/* configure the LCD  */
	LCD_Init();
		
	/* Initialize the I2C interface */
	// I2C is connected to the NFC chip.
	//TT4_Init();
	

	// Enable joystick interrupts (???)
	IntExtOnOffConfig(ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	while(GPIO_ReadInputDataBit(J_SEL_PORT, J_SEL_PIN) == RESET) {  }


	RTC_Timer();

	char v[5];

	enum states { USE_THIS, USE_OTHER, BREAK, INVALID };

	enum states current_state = INVALID;
	enum states last_state = INVALID;

	int color_change_speed;
	int color_change_next;
	
	int bounce_speed;
	int bounce_speed_next;

	int led_bump_speed = 10;
	int led_bump_next = led_bump_speed;
	
	int x = 0;
	int y = 0;
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(Yellow);
	LED_AllOff();		    

	while (true) {
	  if (min >=0 && min <= 26) {
	    current_state = USE_THIS;
	    if (last_state != current_state) {
	      last_state = current_state;
	      LED_AllOff();		    
	      LCD_Clear(Black);
	      LCD_SetBackColor(Black);
	      LCD_SetTextColor(Blue);
	      bounce_speed = 100;
	      bounce_speed_next = bounce_speed;
	    }
	  }
	  else if (min >=30 && min <= 56) {
	    current_state = USE_OTHER;
	    if (last_state != current_state) {
	      last_state = current_state;
	      LCD_SetTextColor(Black);
	      bounce_speed = 1;
	      bounce_speed_next = bounce_speed;
	      color_change_speed = 5;
	      color_change_next = color_change_speed;
	    }
	    if (!--color_change_next) {
	      color_change_next = color_change_speed;
	      LCD_SetBackColor(next_color());
	    }
	    if (!--led_bump_next) {
	      led_bump_next = led_bump_speed;
	      LED_Bump();  
	    }
	  }
	  else {
	    current_state = BREAK;
	    if (last_state != current_state) {
	      last_state = current_state;
	      LED_AllOn();
	      LCD_Clear(Black);
	      LCD_SetBackColor(Black);
	      LCD_SetTextColor(Yellow);
	      bounce_speed = 50;
	      bounce_speed_next = bounce_speed;
	    }
	  }

	  if (!--bounce_speed_next) {
	    bounce_speed_next = bounce_speed;
	    uint16_t xy = next_position();
	    x = xy >> 8;
	    y = xy & 0xff;	    
	  }

	  min_sec(v);
	  for (int i = 0; i < 5; ++i) {	  
	    LCD_DrawCharXY(x + i * FONT_WIDTH, y, v[i]);
	  }

      adjust_time();
    }
}


void bounce(int* dir, int* v, int min, int max, int step) {
  if (*dir) {
    *v += step;
    if (*v >= max) {
      *v = max;
      *dir = 0;
    }
  }
  else {
    *v -= step;
    if (*v <= min) {
      *v = min;
      *dir = 1;
    }
  }
}

uint16_t next_position(void)
{
  static int x = 0;
  static int y = 0;
  static int xdir = 1;
  static int ydir = 1;
  bounce(&xdir, &x, 0, 320 - FONT_WIDTH * 5 - 2, 1); // 2 to make bounce irregular
  bounce(&ydir, &y, 0, 240 - FONT_HEIGHT - 1, 1);
  return x << 8 | y;
}

uint16_t next_color(void)
{
  static int r = 0;
  static int g = 0;
  static int b = 0;
  static int rdir = 0;
  static int gdir = 0;
  static int bdir = 0;
  bounce(&rdir, &r, 0, 31, 1);
  bounce(&gdir, &g, 0, 63, 2);
  bounce(&bdir, &b, 0, 31, 3);
  return r << 11 | g << 5 | b;
}

void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        /* Clear the RTC Second interrupt */
        RTC_ClearITPendingBit(RTC_IT_SEC);
	
	++sec;
	if (sec == 60) {
		sec = 0;
		++min;
		if (min == 60) {
			min = 0;
		}
	}

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }
}


void adjust_time(void) {
	  /* Check which key is pressed */
	  uint32_t MyKey = ReadKey();
      
	  if(MyKey == UP)
	  {
	      --min;
	      if (min <= 0) {
		  min = 59;
	      }
	  }
	  if(MyKey == DOWN)
	  {
	      ++min;
	      if (min >= 60) {
		  min = 0;
	      }
	  }
	  if(MyKey == LEFT)
	  {
		  --sec;
		  if (sec <= 0) {
			  sec = 59;
		  }
	  }
	  if(MyKey == RIGHT)
	  {
		  ++sec;
		  if (sec >= 60) {
			  sec = 0;
		  }
	  }
	  if(MyKey == SEL)
	  {
	  }		
  }

void min_sec(char* a) {
	itoa(a, min, 2);
	a[2] = ':';
	itoa(a + 3, sec, 2);
}

void itoa(char* a, int i, int n) {
	for (int j = n - 1; j >= 0; --j) {
		if (i) {
			a[j] = (i % 10) + '0';
			i /= 10;
		}
		else {
			a[j] = '0';
		}
	}
}

/**
  * @brief  Configure a SysTick Base time to 10 ms.
  * @param  None
  * @retval None
	*/
void SysTick_Configuration(void)
{
  /* Setup SysTick Timer for 10 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 100))
  { 
    /* Capture error */ 
    while (1);
  }
  
 /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}

/**
  * @brief  Enables or disables EXTI for the menu navigation keys :
  * @brief  EXTI lines 7, 6 and 9 which correpond respectively
  * @brief  to "DOWN", "SEL" and "UP".
  * @param  New State
  * @retval None
	*/
void IntExtOnOffConfig(FunctionalState NewState)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Initializes the EXTI_InitStructure */
  EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line 6, 7 and 9 on falling edge */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = EXTI_Line6 | EXTI_Line7 | EXTI_Line9;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
  /* Enable the EXTI line 6, 7 and 9 on falling edge */
  else
  {
    /* Clear the the EXTI line 6, 7 and 9 interrupt pending bit */
    EXTI_ClearITPendingBit(EXTI_Line6 | EXTI_Line7 | EXTI_Line9);

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = EXTI_Line6 | EXTI_Line7 | EXTI_Line9;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
}

/**
  * @brief  Initialize GPIO mapping of joystick
  * @param  None
  * @retval None
  */
void InitJoystick(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
       
   /* Enable BUTTON GPIO clock */
   RCC_APB2PeriphClockCmd( J_SEL_CLOCK 	|
                           J_DOWN_CLOCK	|
			J_LEFT_CLOCK	|
			J_RIGHT_CLOCK|
			J_UP_CLOCK		, 
			ENABLE );

  /* Configure JOYSTICK BUTTON pins as input */
 
   /* JOYSTICK SEL Button */
   GPIO_InitStructure.GPIO_Pin   = J_SEL_PIN;
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
   GPIO_Init( J_SEL_PORT, &GPIO_InitStructure );

   /* JOYSTICK DOWN Button */
   GPIO_InitStructure.GPIO_Pin   = J_DOWN_PIN ;
   GPIO_Init( J_DOWN_PORT, &GPIO_InitStructure );

   /* JOYSTICK LEFT Button */
   GPIO_InitStructure.GPIO_Pin   = J_LEFT_PIN;
   GPIO_Init( J_LEFT_PORT, &GPIO_InitStructure );  

   /* JOYSTICK RIGHT Button */
   GPIO_InitStructure.GPIO_Pin   = J_RIGHT_PIN;
   GPIO_Init( J_RIGHT_PORT, &GPIO_InitStructure );
    
   /* JOYSTICK UP Button */
   GPIO_InitStructure.GPIO_Pin   = J_UP_PIN;
   GPIO_Init( J_UP_PORT, &GPIO_InitStructure );  
	 
	 /* for EXTI config */
	 
	 /* RIGHT Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

  /* LEFT Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

  /* DOWN Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);

  /* UP Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

  /* SEL Button */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
}




// This example demonstrates and explains how to use the RTC peripheral. As an
// application example, it demonstrates how to setup the RTC peripheral, in
// terms of prescaler and interrupts, to be used to keep time and to generate
// Second interrupt. After downloading, you need configure the RTC. then the RTC
// Timer will display on the UART. the LED which connected to PB8 will toggles
// every 1 s. Please retarget The C library printf function to the USART1 at
// PrintChar function in printf.c. This example has been tested on KEIL MCBSTM32
// board, STM32F103RBT6 device. Before downloading executable file into
// Evaluation Board, make sure that an RS-232 cable has been connected between
// PC and development board, then run it.



/***************************************************************************//**
 * Global variables, private define and typedef
 ******************************************************************************/
#define RTCClockOutput_Enable  /* RTC Clock/64 is output on tamper pin(PC.13) */
#define COM1 0
#define COM2 1
__IO uint32_t  TimeDisplay = 0;


/**
  * @brief  The Low Speed External (LSE) clock is used as RTC clock source.
  * The RTC clock can be output on the Tamper pin (PC.13). To enable this functionality,
  * uncomment the corresponding line: #define RTCClockOutput_Enable.
  * 
  * The RTC is in the backup (BKP) domain, still powered by VBAT when VDD is switched off,
  * so the RTC configuration is not lost if a battery is connected to the VBAT pin.
  * A key value is written in backup data register1 (BKP_DR1) to indicate if the RTC
  * is already configured.
  * 
  * @param  None
  * @retval None
  */
void RTC_Timer(void)
{
    /* Initialize LED which connected on PB8, Enable the Clock*/
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Configure the GPIO_LED pin */
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
    */
    //USART_InitStructure.USART_BaudRate = 115200;
    //USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    //USART_InitStructure.USART_StopBits = USART_StopBits_1;
    //USART_InitStructure.USART_Parity = USART_Parity_No;
    //USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    //STM_COMInit(COM1, &USART_InitStructure);


    /* NVIC configuration */
    NVIC_Configuration();
    //if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
    //{
    //    /* Backup data register value is not correct or not yet programmed (when
    //       the first time the program is executed) */
    //
    //    // printf("\r\n\n RTC not yet configured....");
    //
    //    /* RTC Configuration */
    //    RTC_Configuration();
    //
    //    // printf("\r\n RTC configured....");
    //
    //    /* Adjust time by values entred by the user on the hyperterminal */
    //    //Time_Adjust();
    //
    //    BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
    //}
    //else
    //{
    //    /* Check if the Power On Reset flag is set */
    //    if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
    //    {
    //        // printf("\r\n\n Power On Reset occurred....");
    //    }
    //    /* Check if the Pin Reset flag is set */
    //    else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
    //    {
    //        // printf("\r\n\n External Reset occurred....");
    //    }
    //
        RTC_Configuration();
        // printf("\r\n No need to configure RTC....");
        /* Wait for RTC registers synchronization */
    //    RTC_WaitForSynchro();
        /* Enable the RTC Second */
        RTC_ITConfig(RTC_IT_SEC, ENABLE);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
 ////}

    
    
#ifdef RTCClockOutput_Enable
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Disable the Tamper Pin */
    BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
                                 functionality must be disabled */

    /* Enable RTC Clock Output on Tamper Pin */
    BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
#endif

    /* Clear reset flags */
    RCC_ClearFlag();

    /* Display time in infinite loop */
    //Time_Show();
}


///**
//  * @brief  Configures COM port.
//  * @param  COM: Specifies the COM port to be configured.
//  *   This parameter can be one of following parameters:
//  *     @arg 0, COM1
//  *     @arg 1, COM2
//  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
//  *   contains the configuration information for the specified USART peripheral.
//  * @retval None
//  */
//void STM_COMInit(uint16_t COM, USART_InitTypeDef* USART_InitStruct)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_TypeDef* COM_USART[2] = {USART1, USART2};
//    GPIO_TypeDef*  COM_TX_PORT[2] = {GPIOA, GPIOD};
//    GPIO_TypeDef*  COM_RX_PORT[2] = {GPIOA, GPIOD};
//    const uint32_t COM_USART_CLK[2] = {RCC_APB2Periph_USART1, RCC_APB1Periph_USART2};
//    const uint32_t COM_TX_PORT_CLK[2] = {RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOD};
//    const uint32_t COM_RX_PORT_CLK[2] = {RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOD};
//    const uint16_t COM_TX_PIN[2] = {GPIO_Pin_9, GPIO_Pin_5};
//    const uint16_t COM_RX_PIN[2] = {GPIO_Pin_10, GPIO_Pin_6};
//
//    /* Enable GPIO clock */
//    RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);
//
//
//    /* Enable UART clock */
//    if (COM == COM1)
//    {
//        RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
//    }
//    else
//    {
//        /* Enable the USART2 Pins Software Remapping */
//        GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
//        RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
//    }
//
//    /* Configure USART Tx as alternate function push-pull */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//    GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);
//
//    /* Configure USART Rx as input floating */
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
//    GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);
//
//    /* USART configuration */
//    USART_Init(COM_USART[COM], USART_InitStruct);
//
//    /* Enable USART */
//    USART_Cmd(COM_USART[COM], ENABLE);
//}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures the RTC.
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    
    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

    // Select HSE (High Speed External clock) as RTC Clock Source.
    // The board doesn't have a
	//LSE (32.768KHz clock), which is normally used for the RTC. So we use the HSE instead.
    RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div128);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    // We're running off of the High Speed External clock (HSE) which runs at
    // 8MHz. When the HSE is used as clock source, it's automatically divided by
    // 128 (see RCC_RTCCLKSource_HSE_Div128 above).
    RTC_SetPrescaler(8000000 / 128);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

///**
//  * @brief  Adjusts time.
//  * @param  None
//  * @retval None
//  */
//void Time_Adjust(void)
//{
//    /* Wait until last write operation on RTC registers has finished */
//    RTC_WaitForLastTask();
//    /* Change the current time */
//    RTC_SetCounter(Time_Regulate());
//    /* Wait until last write operation on RTC registers has finished */
//    RTC_WaitForLastTask();
//}

//    /* Reset RTC Counter when Time is 23:59:59 */
//    if (RTC_GetCounter() == 0x0001517F)
//    {
//        RTC_SetCounter(0x0);
//        /* Wait until last write operation on RTC registers has finished */
//        RTC_WaitForLastTask();
//    }
//
//    /* Compute  hours */
//    THH = TimeVar / 3600;
//    /* Compute minutes */
//    TMM = (TimeVar % 3600) / 60;
//    /* Compute seconds */
//    TSS = (TimeVar % 3600) % 60;
//
//    // printf("Time: %d:%d:%2d\r", THH, TMM, TSS);
//}

static void LED_AllOff(void)
{
	Led_TypeDef ledx;
	for( ledx=LED1; ledx < LEDn; ledx++)
	{
		LED_Off(ledx);
	}
}

static void LED_AllOn(void)
{
	Led_TypeDef ledx;
	for( ledx=LED1; ledx < LEDn; ledx++)
	{
		LED_On(ledx);
	}
}

static void LED_Bump(void)
{
	static int i = 0;
	++i;
	if (i == 4) {
		i = 0;
	}
	int p;
	if (i == 0) {
		p = 3;
	}
	else {
		p = i - 1;
	}
	LED_On(i);
	LED_Off(p);
}
