/*
  ******************************************************************************
  * @file    BSP_MemsShield.h
  * @author  George Vigelette
  * @version V1.0.0
  * @date    11 December 2015
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 Duvitech</center></h2>
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_MEMS_SHIELD_H
#define __BSP_MEMS_SHIELD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
	 
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 
void LED_BLUE_ClrVal(void);
void LED_BLUE_SetVal(void);
void LED_BLUE_NegVal(void);
	 
void LED_GREEN_ClrVal(void);
void LED_GREEN_SetVal(void);
void LED_GREEN_NegVal(void);
	 
void LED_YELLOW_ClrVal(void);
void LED_YELLOW_SetVal(void);
void LED_YELLOW_NegVal(void);
	 
#ifdef __cplusplus
}
#endif

#endif /*__BSP_MEMS_SHIELD_H */
