/*

  my_machine.h - configuration for STM32G0xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef __MY_MACHINE_H__
#define __MY_MACHINE_H__

// #define BOARD_BTT_SKR_MINI_E3_V30
// Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used.

#if defined(BOARD_BTT_SKR_MINI_E3_V30)
#include "boards/btt_skr_mini_e3_3.0_map.h"
#else
#include "boards/generic_map.h"
#endif

/*EOF*/