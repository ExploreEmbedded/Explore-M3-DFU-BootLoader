/***************************************************************************************************
                                   ExploreEmbedded	
****************************************************************************************************
 * File:   gpio.h
 * Version: 15.0
 * Author: ExploreEmbedded
 * Website: http://www.exploreembedded.com/wiki
 * Description: File contains the gpio pin number configuration and function prototypes for the gpio routines
 
The libraries have been tested on ExploreEmbedded development boards. We strongly believe that the 
library works on any of development boards for respective controllers. However, ExploreEmbedded 
disclaims any kind of hardware failure resulting out of usage of libraries, directly or indirectly.
Files may be subject to change without prior notice. The revision history contains the information 
related to updates. 
 
 
GNU GENERAL PUBLIC LICENSE: 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 
Errors and omissions should be reported to codelibraries@exploreembedded.com
 **************************************************************************************************/
 
 
 /***************************************************************************************************
                             Revision History
****************************************************************************************************				   
15.0: Initial version 
***************************************************************************************************/
 
#ifndef _EXP_GPIO_H
#define _EXP_GPIO_H

#include"stdutils.h"


typedef enum
{
    p0_0, p0_1, p0_2, p0_3, p0_4, p0_5, p0_6, p0_7, p0_8, p0_9, p0_10,p0_11,p0_12,p0_13,p0_14,p0_15,
    p0_16,p0_17,p0_18,p0_19,p0_20,p0_21,p0_22,p0_23,p0_24,p0_25,p0_26,p0_27,p0_28,p0_29,p0_30,p0_31,

    p1_0, p1_1, p1_2, p1_3, p1_4, p1_5, p1_6, p1_7, p1_8, p1_9, p1_10,p1_11,p1_12,p1_13,p1_14,p1_15,
    p1_16,p1_17,p1_18,p1_19,p1_20,p1_21,p1_22,p1_23,p1_24,p1_25,p1_26,p1_27,p1_28,p1_29,p1_30,p1_31,

    p2_0, p2_1, p2_2, p2_3, p2_4, p2_5, p2_6, p2_7, p2_8, p2_9, p2_10,p2_11,p2_12,p2_13,p2_14,p2_15,
    p2_16,p2_17,p2_18,p2_19,p2_20,p2_21,p2_22,p2_23,p2_24,p2_25,p2_26,p2_27,p2_28,p2_29,p2_30,p2_31,

    p3_0, p3_1, p3_2, p3_3, p3_4, p3_5, p3_6, p3_7, p3_8, p3_9, p3_10,p3_11,p3_12,p3_13,p3_14,p3_15,
    p3_16,p3_17,p3_18,p3_19,p3_20,p3_21,p3_22,p3_23,p3_24,p3_25,p3_26,p3_27,p3_28,p3_29,p3_30,p3_31,

    p4_0, p4_1, p4_2, p4_3, p4_4, p4_5, p4_6, p4_7, p4_8, p4_9, p4_10,p4_11,p4_12,p4_13,p4_14,p4_15,
    p4_16,p4_17,p4_18,p4_19,p4_20,p4_21,p4_22,p4_23,p4_24,p4_25,p4_26,p4_27,p4_28,p4_29,p4_30,p4_31,

    P_MAX,
    P_NC = 0xff
}gpioPins_et; 



/*************************************************************************************************
                           Constants for PIN Function Selection
*************************************************************************************************/
#define PINSEL_FUNC_0 0x00u   
#define PINSEL_FUNC_1 0x01u
#define PINSEL_FUNC_2 0x02u
#define PINSEL_FUNC_3 0x03u
/*************************************************************************************************/




/***************************************************************************************************
                             Function prototypes
***************************************************************************************************/
void GPIO_PinFunction(uint8_t v_pinNumber_u8, uint8_t v_pinFunction_u8);
void GPIO_PinDirection(uint8_t v_pinNumber_u8, uint8_t v_pinDirn_u8);
void GPIO_PinWrite(uint8_t v_pinNumber_u8, uint8_t v_pinValue_u8);
void GPIO_PinToggle(uint8_t v_pinNumber_u8);
uint8_t GPIO_PinRead(uint8_t v_pinNumber_u8);
/**************************************************************************************************/
#endif
