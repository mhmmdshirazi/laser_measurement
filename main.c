/*
 laser measurement
 
  @Company:
    Paolo Carlo Bernardi

  @File Name:
    main.c

  @Summary:
    Main section of the program laser_measurement
    http://www.paolocarlobernardi.it/index.php/en/embedded-en

  @Description:
    This is an example on how to use vl53l0x library.
    PIC settings prerequisite:
    - I2C enabled in fast speed mode (400KHz).
    - Timer0 enabled with IRQ and a period of 1msec. The only thing to do
      with TMR0_ISR is to increment the millisec variable every time interrupt
      occurred.
*/

/*   Copyright 2017 Paolo Carlo Bernardi

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include "mcc_generated_files/mcc.h"
#include "drivers/vl53l0x.h"

void ClearLeds() {
    LED_1_SetHigh();
    LED_2_SetHigh();
    LED_3_SetHigh();
    LED_4_SetHigh();
    LED_5_SetHigh();
    LED_6_SetHigh();
    LED_7_SetHigh();
}

void main(void)
{
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    VL53L0X_InitDevices();
    
    VL53L0X_ContinuousReading(&vl53l0xDev[LASER0], 0);
    VL53L0X_ContinuousReading(&vl53l0xDev[LASER1], 0);
        
    while(1) {
        uint16_t tmp0 = VL53L0X_ReadRange(&vl53l0xDev[LASER0]);
        uint16_t tmp1 = VL53L0X_ReadRange(&vl53l0xDev[LASER1]);
        uint16_t tmp;
        
        //60cm of maximum distance
        if (tmp0 < 600 && tmp1 < 600) {
            if (tmp1 > tmp0) {
                tmp  = tmp1 - tmp0;
                if (tmp > 5 && tmp < 15) {
                    ClearLeds();
                    LED_4_SetLow();
                } else if (tmp > 16 && tmp < 35) {
                    ClearLeds();
                    LED_3_SetLow();
                } else if (tmp > 36 && tmp < 55) {
                    ClearLeds();
                    LED_2_SetLow();
                } else if (tmp > 56) {
                    ClearLeds();
                    LED_1_SetLow();
                }
            }
            
            else {
                tmp  = tmp0 - tmp1;
                if (tmp > 5 && tmp < 15) {
                    ClearLeds();
                    LED_4_SetLow();
                } else if (tmp > 16 && tmp < 35) {
                    ClearLeds();
                    LED_5_SetLow();
                } else if (tmp > 36 && tmp < 55) {
                    ClearLeds();
                    LED_6_SetLow();
                } else if (tmp > 56) {
                    ClearLeds();
                    LED_7_SetLow();
                }
            }
        }
        
        else ClearLeds();
    }
}
