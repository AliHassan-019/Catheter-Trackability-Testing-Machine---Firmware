#ifndef STM_TB6600_H_
#define STM_TB6600_H_


/*
  Author:     Muhammad Tayyab
  WebSite:    https://github.com/MuhammadTayyab1002   
  
  Version:    0.2
  * Added Movement  with millimeter function
  * Added Manual Speed Function
  * Fix Speed Option increased
  
*/

#ifdef __cplusplus
extern "C" {
#endif
  
#include "main.h"
/* --------------- Check Mainstream series --------------- */

	#ifdef STM32F0
		#include "stm32f0xx_hal.h"       /* Import HAL library */
		#include "stm32f0xx_hal_tim.h" 
	#elif defined(STM32F1)
		#include "stm32f1xx_hal.h"       /* Import HAL library */
		#include "stm32f1xx_hal_tim.h"
	#elif defined(STM32F2)
		#include "stm32f2xx_hal.h"       /* Import HAL library */
		#include "stm32f2xx_hal_tim.h"
	#elif defined(STM32F3)
		#include "stm32f3xx_hal.h"       /* Import HAL library */
		#include "stm32f3xx_hal_tim.h"
	#elif defined(STM32F4)
		#include "stm32f4xx_hal.h"       /* Import HAL library */
		#include "stm32f4xx_hal_tim.h"
	#elif defined(STM32F7)
		#include "stm32f7xx_hal.h"       /* Import HAL library */
		#include "stm32f7xx_hal_tim.h"
	#elif defined(STM32G0)
		#include "stm32g0xx_hal.h"       /* Import HAL library */
		#include "stm32g0xx_hal_tim.h"
	#elif defined(STM32G4)
		#include "stm32g4xx_hal.h"       /* Import HAL library */
		#include "stm32g4xx_hal_tim.h"

	/* ------------ Check High Performance series ------------ */

	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
		#include "stm32h7xx_hal_tim.h"

	/* ------------ Check Ultra low power series ------------- */

	#elif defined(STM32L0)
		#include "stm32l0xx_hal.h"       /* Import HAL library */
		#include "stm32l0xx_hal_tim.h"
	#elif defined(STM32L1)
		#include "stm32l1xx_hal.h"       /* Import HAL library */0
		#include "stm32l1xx_hal_tim.h"
	#elif defined(STM32L5)
		#include "stm32l5xx_hal.h"       /* Import HAL library */
		#include "stm32l5xx_hal_tim.h"
	#elif defined(STM32L4)
		#include "stm32l4xx_hal.h"       /* Import HAL library */
		#include "stm32l4xx_hal_tim.h"
	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
		#include "stm32h7xx_hal_tim.h"
	#else
	#endif /* STM32F1 */
//####################################################################################################################

typedef struct
{
  GPIO_TypeDef  *enable_gpio;
  GPIO_TypeDef  *direction_gpio;
	GPIO_TypeDef  *pulse_gpio;
	TIM_HandleTypeDef timer;
  uint16_t      enable_pin;
  uint16_t      direction_pin;
	uint16_t      pulse_pin;
	uint16_t			PulsePerRev;
	uint16_t			Speed;
	uint32_t      Channel;
	uint32_t		Count;
}tb6600_t;


typedef enum
    {
				CCW = 1,  ///< Counter-Clockwise
        CW  = 0  ///< Clockwise
    } Direction;

typedef enum
    {
				VFast 	= 50,  //12mm/sec (Dia=8mm)
				Fast 		= 75,	 //8mm/sec (Dia=8mm)
        Medium  = 150, //4mm/sec (Dia=8mm)   
				Slow		= 300, //2mm/sec (Dia=8mm)
				VSlow		= 700  //1mm/sec (Dia=8mm)
		} Speed;
		
//####################################################################################################################
		
void tb6600_init(tb6600_t *tb6600, GPIO_TypeDef  *enable_gpio, uint16_t enable_pin, GPIO_TypeDef  *direction_gpio, uint16_t direction_pin,GPIO_TypeDef  *pulse_gpio,uint16_t pulse_pin,TIM_HandleTypeDef htimx);
void tb6600_init_tim(tb6600_t *tb6600, GPIO_TypeDef  *enable_gpio, uint16_t enable_pin, GPIO_TypeDef  *direction_gpio, uint16_t direction_pin,TIM_HandleTypeDef timer,uint32_t Channel);
void tb6600_Set(tb6600_t *tb6600,Direction direction,Speed speed,uint16_t PulsePerRev,uint16_t Clock);
void tb6600_AngleMove(tb6600_t *tb6600,float Degree);
void tb6600_AngleMove_tim(tb6600_t *tb6600,uint16_t Degree);
void tb6600_Enable(tb6600_t *tb6600);
void tb6600_Disable(tb6600_t *tb6600);
void tb6600_ChangeDirection(tb6600_t *tb6600,Direction direction);
void tb6600_ToggleDirection(tb6600_t *tb6600);
void tb6600_ChangeSpeed(tb6600_t *tb6600,Speed speed);
void tb6600_ChangePulsePerRev(tb6600_t *tb6600,uint16_t PulsePerRev);
void tb6600_cont(tb6600_t *tb6600);
void tb6600_cont_tim(tb6600_t *tb6600);
void tb6600_Movemm(tb6600_t *tb6600,float Millimeters);
void tb6600_Movemm_tim(tb6600_t *tb6600,uint16_t Millimeters);
void tb6600_ManualSpeed(tb6600_t *tb6600,uint16_t mmpersec);
void tb6600_Stop(tb6600_t *tb6600);
void tb6600_Stop_tim(tb6600_t *tb6600);
uint32_t GETVALUE(tb6600_t *tb6600);
TIM_HandleTypeDef gettimer(tb6600_t *tb6600);


//####################################################################################################################
		
#endif
