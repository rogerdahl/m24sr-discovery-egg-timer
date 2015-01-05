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


/* Includes ------------------------------------------------------------------*/
#include "main.h"

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

static __IO uint16_t					counter_delay_ms;

uint8_t I2C_Cmd_Flag = 0;
uint8_t NFCT4_SS_Flag = 0;
uint8_t NFCT4_CC_Flag = 0;
uint8_t NFCT4_NDEF_Flag = 0;
uint8_t ISO7816_Flag = 0;
uint8_t ST_Flag = 0;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void LED_Init(void);
static void LED_Show(void);
static void M24SR_DemoInit( void );

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

/**
  * @brief  all the Leds will switch on/off
  * @param  None
  * @retval None
  */
static void LED_Show(void)
{
	Led_TypeDef ledx;
	/* configure the LED  */
	for( ledx=LED1; ledx < LEDn; ledx++)
	{
		LED_On(ledx);
		delay_ms(100);
		LED_Off(ledx);
	}
}

/**
  * @brief  Initializes the M24SR for demonstration application.
  * @param  None
  * @retval None
  */
static void M24SR_DemoInit ( void )
{
	uint8_t ResetI2CPassword[16]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	ISO7816_DisableAllPassword(ResetI2CPassword);
}

/**
  * @}
  */

/** @addtogroup Main_Public_Functions
 * 	@{
 */
	

/**
  * @brief  This is the main program function and entry point of application.
  * @param  None
  * @retval None
  */
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
    FLASH_PrefetchBufferCmd(ENABLE);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_5);
 	
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div2); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div4);

    /* PLLCLK = 8MHz /8 * 336 / 2 = 168 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7); // 7 is for an USB clock of 8/8*336/7=48MHz

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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG, ENABLE);
  
  /* TIM1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/*------------------- Resources Initialization -----------------------------*/	
	/* init the interuption */ 
	Interrupts_Config();
	
	Timer_Config();
	
	/* Configure the systick */    
  SysTick_Configuration();
	
	RCC_ClocksTypeDef toto;
	RCC_GetClocksFreq(&toto);

/*------------------- Drivers Initialization -------------------------------*/	
	
	/* Initialize the Joystick */
	InitJoystick();
	
	/* configure the LED  */
	LED_Init();	
	LED_Show();

	/* configure the LCD  */
	LCD_Init();
	
	/* ST intro */
	STIntro();
	
	/* Initialize the I2C interface */
	TT4_Init();

	/* Set to be sure M24SR is not locked by password */
	M24SR_DemoInit();
	
	/* Enable RF */
	M24SR_RFConfig(1);
	LED_On(LED2);

	/* Launch a timer to be able to generate random key */
	StartPerfMeasurement();

	/* Bluetooth module init */
	/* It's long about 10 sec because of power ON/OFF we can remove it */
	if (BT_Init() == SUCCESS)
	{
		LED_On(LED3);
	}
	/* else we are not on premium edition BT module not available */
	
	/*------------------- Launch application -------------------------------*/	
	/* Display the main menu icons */
  ShowMenuIcons();
	
	while (1)
  {		
    /* If SEL pushbutton is pressed */
    if(SELStatus == 1)
    {
      /* External Interrupt Disable */
      IntExtOnOffConfig(DISABLE);

      /* Execute Sel Function */
      SelFunc();

      /* External Interrupt Enable */
      IntExtOnOffConfig(ENABLE);
      /* Reset SELStatus value */
      SELStatus = 0;
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

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
