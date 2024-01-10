/**
 ******************************************************************************
 * @file main.c
 * @brief This file contains the main function for: retarget the C library printf
 * scanf functions to the UART1 example.
 * @author  MCD Application Team, Nicolas Gutierrez
 * @version V1
 * @date   9-December-2023
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
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
#include "stm8s.h"
#include "stm8s_it.h" /* SDCC patch: required by SDCC for interrupts */
#include "stdio.h"
#include "delay.h"
#include "mirf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* SDCC patch: ensure same types as stdio.h */
#if SDCC_VERSION >= 30605 // declaration changed in sdcc 3.6.5 (officially with 3.7.0)
#define PUTCHAR_PROTOTYPE int putchar(int c)
#define GETCHAR_PROTOTYPE int getchar(void)
#else
#define PUTCHAR_PROTOTYPE void putchar(char c)
#define GETCHAR_PROTOTYPE char getchar(void)
#endif

/* Private variables ---------------------------------------------------------*/
unsigned int bIntFlag;
/* Private function prototypes -----------------------------------------------*/

void secondary()
{
  // Debuging LEDs
  GPIO_Init(
      GPIOB,
      GPIO_PIN_1,
      GPIO_MODE_OUT_OD_LOW_FAST); // Open Drain Mode, since it has to sink current
  GPIO_Init(
      GPIOB,
      GPIO_PIN_2,
      GPIO_MODE_OUT_OD_LOW_FAST);

  // Button
  GPIO_Init(
      GPIOB,
      GPIO_PIN_0,
      GPIO_MODE_IN_PU_IT);

  // Interrupt for button
  EXTI_DeInit();
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);
  EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_FALL_ONLY);

  enableInterrupts();

  // Turn off LEDs
  GPIO_WriteHigh(GPIOB, GPIO_PIN_1);
  GPIO_WriteHigh(GPIOB, GPIO_PIN_2);

  // NRF initialization
  Nrf24_init();
  bool PTX = 0;

  Nrf24_config(&PTX);

  // Set own address using 5 characters
  int ret = Nrf24_setRADDR((uint8_t *)"ABCDE");
  if (ret != SUCCESS)
  {
    while (1)
    {
      delay_ms(1);
    }
  }

  // Set the receiver address using 5 characters
  ret = Nrf24_setTADDR((uint8_t *)"FGHIJ");
  if (ret != SUCCESS)
  {
    while (1)
    {
      delay_ms(1);
    }
  }

  // This has to be set to 1MBps and 150us delay, if using 250kbps, use at least 1000us delay
  Nrf24_SetSpeedDataRates(RF24_1MBPS);
  Nrf24_setRetransmitDelay(0);

  // Output/Input Buffer
  uint8_t buf[32];

  // Clear RX FiFo
  while (1)
  {
    if (Nrf24_dataReady() == FALSE)
    {
      break;
    };
    Nrf24_getData(buf);
  }

  while (1)
  {
    // When the program is received, the received data is output from the serial port
    if (Nrf24_dataReady())
    {
      // Turns the led on when a packet is recived if the button was clicked
      if (bIntFlag == 1)
      {
        bIntFlag = 0;
        GPIO_WriteLow(GPIOB, GPIO_PIN_1);
      }
      else
      {
        GPIO_WriteHigh(GPIOB, GPIO_PIN_1);
      }

      Nrf24_getData(buf);
      // The ESP is printing stuff so it takes a while to switch to RX mode.
      delay_ms(10);

      Nrf24_send(buf, &PTX);

      // Same here, delay a bit.
      delay_ms(10);

      while (!Nrf24_isSend(50, &PTX))
      {
        // If no ACK after the preconfigured retries, send again.
        Nrf24_send(buf, &PTX);
        delay_ms(10);
      }
    }
    // Wait a bit to check if the NRF recieved data.
    delay_ms(1);
  }
}

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
void main(void)
{
  /*High speed internal clock prescaler: 1*/
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

  secondary();
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  (void)file;
  (void)line;
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/