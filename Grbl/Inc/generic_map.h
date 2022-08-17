/*
  generic_map.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2022 Terje Io

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

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if TRINAMIC_ENABLE
#error "Trinamic plugin not supported!"
#endif

// Define step pulse output pins.
#define STEP_PORT               GPIO1
#define X_STEP_PIN              3
#define Y_STEP_PIN              2
#define Z_STEP_PIN              17
#define STEP_OUTMODE            GPIO_BITBAND
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define DIRECTION_PORT          GPIO1
#define X_DIRECTION_PIN         16
#define Y_DIRECTION_PIN         23
#define Z_DIRECTION_PIN         22
#define DIRECTION_OUTMODE       GPIO_BITBAND
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIO1
#define STEPPERS_ENABLE_PIN     11
//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIO3
#define X_LIMIT_PIN             13
#define Y_LIMIT_PIN             14
#define Z_LIMIT_PIN             15
#define LIMIT_INMODE            GPIO_BITBAND//GPIO_SHIFT12

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            STEP_PORT
#define M3_STEP_PIN             3
#define M3_DIRECTION_PORT       DIRECTION_PORT
#define M3_DIRECTION_PIN        7
#if N_AUTO_SQUARED
#define M3_LIMIT_PORT           LIMIT_PORT
#define M3_LIMIT_PIN            15
#endif
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIO1
#define SPINDLE_ENABLE_PIN      24
#define SPINDLE_DIRECTION_PORT  GPIO1
#define SPINDLE_DIRECTION_PIN   9

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT_BASE   GPIO1_BASE
#define SPINDLE_PWM_PIN         10

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIO1
#define COOLANT_FLOOD_PIN       18
#define COOLANT_MIST_PORT       GPIO1
#define COOLANT_MIST_PIN        19

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIO1
#define RESET_PIN               26
#define FEED_HOLD_PIN           27
#define CYCLE_START_PIN         20
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         8
#endif
#define CONTROL_INMODE GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PORT              GPIO3
#define PROBE_PIN               12

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         GPIO2
#define I2C_STROBE_PIN          7
#endif
