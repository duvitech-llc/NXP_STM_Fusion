/*
  ******************************************************************************
  * @file    BSP_MemsShield.c
  * @author  George Vigelette
  * @version V1.0.0
  * @date    11 December 2015
  * @brief   
  ******************************************************************************
  * @attention
  *
  * (C) COPYRIGHT 2015 Duvitech
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
#include "BSP_MemsShield.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
	 
/* Private define ------------------------------------------------------------*/
#define YELLOW_PORT 		GPIOA
#define YELLOW_PIN 			GPIO_PIN_9
#define BLUE_PORT 			GPIOC
#define BLUE_PIN 				GPIO_PIN_7
#define GREEN_PORT 			GPIOA
#define GREEN_PIN 			GPIO_PIN_5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void LED_BLUE_ClrVal(){
	HAL_GPIO_WritePin(BLUE_PORT, BLUE_PIN, GPIO_PIN_RESET);
}

void LED_BLUE_SetVal(){
	HAL_GPIO_WritePin(BLUE_PORT, BLUE_PIN, GPIO_PIN_SET);	
}

void LED_BLUE_NegVal(){
	HAL_GPIO_TogglePin(BLUE_PORT, BLUE_PIN);	
}
	 
void LED_GREEN_ClrVal(){
	HAL_GPIO_WritePin(GREEN_PORT, GREEN_PIN, GPIO_PIN_RESET);	
}

void LED_GREEN_SetVal(){
	HAL_GPIO_WritePin(GREEN_PORT, GREEN_PIN, GPIO_PIN_SET);	
}

void LED_GREEN_NegVal(){
	HAL_GPIO_TogglePin(GREEN_PORT, GREEN_PIN);		
}


void LED_YELLOW_ClrVal(){
	HAL_GPIO_WritePin(YELLOW_PORT, YELLOW_PIN, GPIO_PIN_RESET);	
}

void LED_YELLOW_SetVal(){
	HAL_GPIO_WritePin(YELLOW_PORT, YELLOW_PIN, GPIO_PIN_SET);		
}

void LED_YELLOW_NegVal(){
	HAL_GPIO_TogglePin(YELLOW_PORT, YELLOW_PIN);		
}

/************************ (C) COPYRIGHT Duvitech *****END OF FILE****/