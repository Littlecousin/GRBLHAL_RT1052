/*

  driver.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2022 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "main.h"
#include "grbl/driver_opts.h"

#define BITBAND_PERI(x, b) (*((__IO uint8_t *) (PERIPH_BB_BASE + (((uint32_t)(volatile const uint32_t *)&(x)) - PERIPH_BASE)*32 + (b)*4)))

//#define DIGITAL_IN(port, pin) BITBAND_PERI(port->IDR, pin)
//#define DIGITAL_OUT(port, pin, on) { BITBAND_PERI((port)->ODR, pin) = on; }

//引脚输入
#define DIGITAL_IN(port, pin) GPIO_PinRead(port,pin)
//引脚输出
#define DIGITAL_OUT(port, pin, on) GPIO_PinWrite(port,pin,on)

#define GPIO_GET_INDEX(__GPIOx__)  GPIO_GetInstance(__GPIOx__)

#define CONTROL_INPUT_IRQ	GPIO1_Combined_16_31_IRQn
#define LIMIT_INPUT_IRQ		GPIO3_Combined_0_15_IRQn

/*
TMR3
#define QTMR_IRQ_ID      TMR3_IRQn
#define QTMR_IRQ_HANDLER TMR3_IRQHandler
*/
#define timer(t) timerN(t)
#define timerN(t) TMR ## t
#define timerCH(t) timerch(t)
#define timerch(t) kQTMR_Channel_ ## t
#define timerINT(t) timerint(t)
#define timerint(t) TMR ## t ## _IRQn
#define timerHANDLER(t) timerhandler(t)
#define timerhandler(t) TMR ## t ## _IRQHandler
#define timerCCEN(c, n) timerccen(c, n)
#define timerccen(c, n) TIM_CCER_CC ## c ## n ## E
#define timerCCMR(p, c) timerccmr(p, c)
#define timerccmr(p, c) TMR ## p->CCMR ## c
#define timerOCM(p, c) timerocm(p, c)
#define timerocm(p, c) TIM_CCMR ## p ##_OC ## c ## M_1|TIM_CCMR ## p ##_OC ## c ## M_2
#define timerOCMC(p, c) timerocmc(p, c)
#define timerocmc(p, c) (TIM_CCMR ## p ##_OC ## c ## M|TIM_CCMR ## p ##_CC ## c ## S)
#define timerCCR(t, c) timerccr(t, c)
#define timerccr(t, c) TMR ## t->CCR ## c
#define timerCCP(c, n) timerccp(c, n)
#define timerccp(c, n) TIM_CCER_CC ## c ## n ## P
#define timerCR2OIS(c, n) timercr2ois(c, n)
#define timercr2ois(c, n) TIM_CR2_OIS ## c ## n
#define timerAF(t, f) timeraf(t, f)
#define timeraf(t, f) GPIO_AF ## f ## _TIM ## t
#define timerCLKENA(t) timercken(t)
#define timercken(t) __HAL_RCC_TIM ## t ## _CLK_ENABLE

#define usart(t) usartN(t)
#define usartN(t) LPUART ## t
#define usartINT(t) usartint(t)
#define usartint(t) LPUART ## t ## _IRQn
#define usartHANDLER(t) usarthandler(t)
#define usarthandler(t) LPUART ## t ## _IRQHandler

// Configuration, do not change here

#define CNC_BOOSTERPACK     0

// Define GPIO output mode options
#define GPIO_SHIFT0   0
#define GPIO_SHIFT1   1
#define GPIO_SHIFT2   2
#define GPIO_SHIFT3   3
#define GPIO_SHIFT4   4
#define GPIO_SHIFT5   5
#define GPIO_SHIFT6   6
#define GPIO_SHIFT7   7
#define GPIO_SHIFT8   8
#define GPIO_SHIFT9   9
#define GPIO_SHIFT10 10
#define GPIO_SHIFT11 11
#define GPIO_SHIFT12 12
#define GPIO_SHIFT13 13
#define GPIO_MAP     14
#define GPIO_BITBAND 15

#include "generic_map.h"

#if IS_NUCLEO_DEVKIT == 1 && !defined(IS_NUCLEO_BOB)
#warning "Board map is not for Nucleo based boards and firmware may not work!"
#endif

#if defined(IS_NUCLEO_BOB) && USB_SERIAL_CDC
#error "Nucleo based boards does not support USB CDC communication!"
#endif

// Define timer allocations.
extern __IO uint32_t g_debounce_int_count;
extern __IO uint32_t g_pulse_int_count;
extern __IO uint32_t g_stepper_int_count;

//使用pit 24Mhz，一分频，中断时间为counts/24000000（秒）
//STEPPER定时器启动后就一直在进行定时
#define STEPPER_TIMER_N				1
#define STEPPER_TIMER_CH_N			0
#define STEPPER_TIMER				PIT//timer(STEPPER_TIMER_N)
#define STEPPER_TIMER_CH			kPIT_Chnl_0//timerCH(STEPPER_TIMER_CH_N)
#define STEPPER_TIMER_IRQn          PIT_IRQn//timerINT(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQHandler    PIT_IRQHandler//timerHANDLER(STEPPER_TIMER_N)//timerHANDLER(STEPPER_TIMER_N)//
//#define STEPPER_TIMER_CLOCK_ENA     timerCLKENA(STEPPER_TIMER_N)

//150Mhz，1分频，一次中断时间为counts/150000000（秒）
#define PULSE_TIMER_N               1
#define PULSE_TIMER_CH_N			      0
#define PULSE_TIMER                 timer(PULSE_TIMER_N)
#define PULSE_TIMER_CH				      timerCH(PULSE_TIMER_CH_N)
#define PULSE_TIMER_IRQn            timerINT(PULSE_TIMER_N)
#define PULSE_TIMER_IRQHandler      timerHANDLER(PULSE_TIMER_N)
#define PULSE_TIMER_CLOCK_ENA       timerCLKENA(PULSE_TIMER_N)

//150Mhz，128分频，一次中断时间为counts*128/150000000（秒）
#define DEBOUNCE_TIMER_N            2
#define DEBOUNCE_TIMER_CH_N			    0
#define DEBOUNCE_TIMER              timer(DEBOUNCE_TIMER_N)
#define DEBOUNCE_TIMER_CH			      timerCH(DEBOUNCE_TIMER_CH_N)
#define DEBOUNCE_TIMER_IRQn         timerINT(DEBOUNCE_TIMER_N)
#define DEBOUNCE_TIMER_IRQHandler   timerHANDLER(DEBOUNCE_TIMER_N)
#define DEBOUNCE_TIMER_CLOCK_ENA    timerCLKENA(DEBOUNCE_TIMER_N)

//USEC_TO_COUNT(pulse_length, (QTMR_SOURCE_CLOCK / 1))
//USEC_TO_COUNT(pulse_delay, (QTMR_SOURCE_CLOCK / 1))

#ifdef SPINDLE_PWM_PORT_BASE
#define SPINDLE_PWM_TIMER_N     3
#define SPINDLE_PWM_TIMER_CH    3
#define SPINDLE_PWM_TIMER_INV   0
#define SPINDLE_PWM_TIMER_AF    1

#if SPINDLE_PWM_TIMER_CH == 1 || SPINDLE_PWM_TIMER_CH == 2
#define SPINDLE_PWM_CCR 1
#else
#define SPINDLE_PWM_CCR 2
#endif
#define SPINDLE_PWM_TIMER           timer(SPINDLE_PWM_TIMER_N)
#define SPINDLE_PWM_TIMER_CCR       timerCCR(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_CH)
#define SPINDLE_PWM_TIMER_CCMR      timerCCMR(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_CCR)
#define SPINDLE_PWM_CCMR_OCM_SET    timerOCM(SPINDLE_PWM_CCR, SPINDLE_PWM_TIMER_CH)
#define SPINDLE_PWM_CCMR_OCM_CLR    timerOCMC(SPINDLE_PWM_CCR, SPINDLE_PWM_TIMER_CH)
#if SPINDLE_PWM_TIMER_INV
#define SPINDLE_PWM_CCER_EN         timerCCEN(SPINDLE_PWM_TIMER_CH, N)
#define SPINDLE_PWM_CCER_POL        timerCCP(SPINDLE_PWM_TIMER_CH, N)
#define SPINDLE_PWM_CR2_OIS         timerCR2OIS(SPINDLE_PWM_TIMER_CH, N)
#else
#define SPINDLE_PWM_CCER_EN         timerCCEN(SPINDLE_PWM_TIMER_CH, )
#define SPINDLE_PWM_CCER_POL        timerCCP(SPINDLE_PWM_TIMER_CH, )
#define SPINDLE_PWM_CR2_OIS         timerCR2OIS(SPINDLE_PWM_TIMER_CH, )
#endif

#define SPINDLE_PWM_PORT            ((GPIO_Type *)SPINDLE_PWM_PORT_BASE)// GPIO_TypeDef
#define SPINDLE_PWM_AF              timerAF(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_AF)
#define SPINDLE_PWM_CLOCK_ENA       timerCLKENA(SPINDLE_PWM_TIMER_N)

#endif // SPINDLE_PWM_PORT_BASE

#if defined(SPINDLE_PWM_PIN) && !defined(SPINDLE_PWM_TIMER_N)
#ifdef SPINDLE_PWM_PORT
#error Map spindle port by defining SPINDLE_PWM_PORT_BASE in the map file!
#else
#error Spindle PWM not supported on mapped pin!
#endif
#endif

#if SPINDLE_SYNC_ENABLE

#if SPINDLE_PWM_TIMER_N == 2 || SPINDLE_PWM_TIMER_N == 3
#error Timer conflict: spindle sync and spindle PWM!
#endif
#ifndef RPM_COUNTER_N
#define RPM_COUNTER_N               3
#endif
#define RPM_COUNTER                 timer(RPM_COUNTER_N)
#define RPM_COUNTER_IRQn            timerINT(RPM_COUNTER_N)
#define RPM_COUNTER_IRQHandler      timerHANDLER(RPM_COUNTER_N)
#define RPM_COUNTER_CLOCK_ENA       timerCLKENA(RPM_COUNTER_N)

#ifndef RPM_TIMER_N
#define RPM_TIMER_N                 2
#endif
#define RPM_TIMER                   timer(RPM_TIMER_N)
#define RPM_TIMER_IRQn              timerINT(RPM_TIMER_N)
#define RPM_TIMER_IRQHandler        timerHANDLER(RPM_TIMER_N)
#define RPM_TIMER_CLOCK_ENA         timerCLKENA(RPM_TIMER_N)

#elif PPI_ENABLE

#if SPINDLE_PWM_TIMER_N == 2
#error Timer conflict: laser PPI and spindle PWM!
#endif

#define PPI_TIMER_N                 2
#define PPI_TIMER                   timer(PPI_TIMER_N)
#define PPI_TIMER_IRQn              timerINT(PPI_TIMER_N)
#define PPI_TIMER_IRQHandler        timerHANDLER(PPI_TIMER_N)
#define PPI_TIMER_CLOCK_ENA         timerCLKENA(PPI_TIMER_N)

#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.0f // microseconds
#endif

// End configuration

#if EEPROM_ENABLE == 0
#define FLASH_ENABLE 1
#else
#define FLASH_ENABLE 0
#endif

#if MODBUS_ENABLE
#include "spindle/modbus.h"
#endif

#if MODBUS_ENABLE
#define MODBUS_TEST 1
#else
#define MODBUS_TEST 0
#endif

#if TRINAMIC_UART_ENABLE && !defined(MOTOR_UARTX_PORT)
#define TRINAMIC_TEST 1
#else
#define TRINAMIC_TEST 0
#endif

#if MPG_ENABLE
#define MPG_TEST 1
#else
#define MPG_TEST 0
#endif

#if KEYPAD_ENABLE == 2 && MPG_ENABLE == 0
#define KEYPAD_TEST 1
#else
#define KEYPAD_TEST 0
#endif

#if MODBUS_TEST + KEYPAD_TEST + MPG_TEST + TRINAMIC_TEST + BLUETOOTH_ENABLE > 1
#error "Only one option that uses the serial port can be enabled!"
#endif

#if MODBUS_TEST || KEYPAD_TEST || MPG_TEST || TRINAMIC_TEST || BLUETOOTH_ENABLE
#if IS_NUCLEO_DEVKIT
#define SERIAL2_MOD 1
#else
#define SERIAL2_MOD 2
#endif
#endif

#undef MODBUS_TEST
#undef KEYPAD_TEST
#undef MPG_TEST
#undef TRINAMIC_TEST

#if MPG_MODE == 1 && !defined(MPG_MODE_PIN)
#error "MPG_MODE_PIN must be defined!"
#endif

#if TRINAMIC_ENABLE
  #include "motors/trinamic.h"
  #ifndef TRINAMIC_MIXED_DRIVERS
    #define TRINAMIC_MIXED_DRIVERS 1
  #endif
#endif

// End configuration

#if KEYPAD_ENABLE == 1 && !defined(I2C_STROBE_PORT)
#error Keypad plugin not supported!
#elif I2C_STROBE_ENABLE && !defined(I2C_STROBE_PORT)
#error I2C strobe not supported!
#endif

#if SDCARD_ENABLE && !defined(SD_CS_PORT)
#error SD card plugin not supported!
#endif

#if I2C_ENABLE && !defined(I2C_PORT)
#define I2C_PORT 0
#endif

#if SPI_ENABLE && !defined(SPI_PORT)
#define SPI_PORT 1
#endif

#ifndef STEP_PINMODE
#define STEP_PINMODE PINMODE_OUTPUT
#endif

#ifndef DIRECTION_PINMODE
#define DIRECTION_PINMODE PINMODE_OUTPUT
#endif

#ifndef STEPPERS_ENABLE_PINMODE
#define STEPPERS_ENABLE_PINMODE PINMODE_OUTPUT
#endif

typedef struct {
    pin_function_t id;
    GPIO_Type *port;
    uint8_t pin;
    uint32_t bit;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_irq_mode_t irq_mode;
    pin_mode_t cap;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    GPIO_Type *port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;


/** 
  * @brief GPIO Init structure definition  
  */ 
typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins. 
                            This parameter can be a value of @ref GPIO_Alternate_function_selection */
}GPIO_InitTypeDef;


/** @defgroup GPIO_mode_define GPIO mode define
  * @brief GPIO Configuration Mode 
  *        Elements values convention: 0xX0yz00YZ
  *           - X  : GPIO mode or EXTI Mode
  *           - y  : External IT or Event trigger detection 
  *           - z  : IO configuration on External IT or Event
  *           - Y  : Output type (Push Pull or Open Drain)
  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
  * @{
  */ 
#define  GPIO_MODE_INPUT                        0x00000000U   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    0x00000001U   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    0x00000011U   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        0x00000002U   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        0x00000012U   /*!< Alternate Function Open Drain Mode    */

#define  GPIO_MODE_ANALOG                       0x00000003U   /*!< Analog Mode  */
    
#define  GPIO_MODE_IT_RISING                    0x10110000U   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   0x10210000U   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            0x10310000U   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
 
#define  GPIO_MODE_EVT_RISING                   0x10120000U   /*!< External Event Mode with Rising edge trigger detection               */
#define  GPIO_MODE_EVT_FALLING                  0x10220000U   /*!< External Event Mode with Falling edge trigger detection              */
#define  GPIO_MODE_EVT_RISING_FALLING           0x10320000U   /*!< External Event Mode with Rising/Falling edge trigger detection       */
/**
  * @}
  */

/** @defgroup GPIO_speed_define  GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @{
  */
#define  GPIO_SPEED_FREQ_LOW         0x00000000U  /*!< IO works at 2 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_MEDIUM      0x00000001U  /*!< range 12,5 MHz to 50 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_HIGH        0x00000002U  /*!< range 25 MHz to 100 MHz, please refer to the product datasheet  */
#define  GPIO_SPEED_FREQ_VERY_HIGH   0x00000003U  /*!< range 50 MHz to 200 MHz, please refer to the product datasheet  */
/**
  * @}
  */

 /** @defgroup GPIO_pull_define GPIO pull define
   * @brief GPIO Pull-Up or Pull-Down Activation
   * @{
   */  
#define  GPIO_NOPULL        0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        0x00000001U   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      0x00000002U   /*!< Pull-down activation                */

bool driver_init (void);
void Driver_IncTick (void);
#ifdef HAS_BOARD_INIT
void board_init (void);
#endif
#ifdef HAS_IOPORTS
void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (uint32_t bit);
#endif

#endif // __DRIVER_H__
