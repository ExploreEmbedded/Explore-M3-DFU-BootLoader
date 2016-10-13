/*****************************************************************************
 *                                                                            *
 * DFU/SD/SDHC Bootloader for LPC17xx                                         *
 *                                                                            *
 * by Triffid Hunter                                                          *
 *                                                                            *
 *                                                                            *
 * This firmware is Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the GNU General Public License as published by       *
 * the Free Software Foundation; either version 2 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * GNU General Public License for more details.                               *
 *                                                                            *
 * You should have received a copy of the GNU General Public License          *
 * along with this program; if not, write to the Free Software                *
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA *
 *                                                                            *
 *****************************************************************************/

#include "usbhw.h"
#include "usbcore.h"





#include "gpio.h"
#include "rgb.h"
#include "sbl_iap.h"
#include "sbl_config.h"
#include "timer.h"



#include "dfu.h"



#include "lpc17xx_wdt.h"
#include "LPC17xx.h"
#define ISP_BTN	P2_11
#define USB_CONNECT P2_9

//#if !(defined DEBUG)
//#define //printf(...) do {} while (0)
//#endif
#include "uart.h"

int dfu=0;
const char *firmware_file = "firmware.bin";
const char *firmware_old  = "firmware.cur";
volatile int TimeoutDetected=0;
#define DTU_TIMEOUT_VALUE 10000000
void myTimerIsr_0 (void);
static uint32_t delay_loop(uint32_t count);
#define SBIT_TIMER0  1
#define SBIT_TIMER1  2

#define SBIT_MR0I    0
#define SBIT_MR0R    1
#define SBIT_MR0S    2

#define SBIT_CNTEN   0

#define PCLK_TIMER0  2
#define PCLK_TIMER1  4    

#define MiliToMicroSec(x)  (x*1000)  /* ms is multiplied by 1000 to get us*/
#define START_TIMER() LPC_TIM0->TCR  = (1 <<SBIT_CNTEN)
#define STOP_TIMER()  LPC_TIM0->TCR  = 0x00

//extern unsigned int SystemCoreClock;
unsigned int getPrescalarForMicroSec(uint8_t timerPclkBit);

void setleds()
{
    
}

void usb_Reset(void)
{
NVIC_DisableIRQ(USBActivity_IRQn);
NVIC_DisableIRQ(USB_IRQn);

LPC_SC->PCONP = 0x042887DE;
LPC_USB->USBClkCtrl  = 0x00;    
LPC_SC->USBIntSt     = 0x80000000; 
 
LPC_USB->USBDevIntEn  = 0x00; 
LPC_USB->USBDevIntClr = 0x00; 
LPC_USB->USBDevIntSet = 0x00; 
LPC_USB->USBDevIntPri = 0x00; 



LPC_USB->USBEpIntEn     = 0x00; 
LPC_USB->USBEpIntClr    = 0x00; 
LPC_USB->USBEpIntSet    = 0x00; 
LPC_USB->USBEpIntPri    = 0x00; 

LPC_USB->USBReEp        = 0x03; 
LPC_USB->USBEpInd       = 0x00; 
LPC_USB->USBMaxPSize    = 0x08; 



LPC_USB->USBTxData = 0x00; 
LPC_USB->USBTxPLen = 0x00; 
LPC_USB->USBCtrl   = 0x00; 

LPC_USB->USBCmdCode      = 0x00;



LPC_USB->USBDMARClr       = 0x00; 
LPC_USB->USBDMARSet       = 0x00; 
LPC_USB->USBUDCAH         = 0x00; 
 
LPC_USB->USBEpDMAEn       = 0x00; 
LPC_USB->USBEpDMADis      = 0x00; 

LPC_USB->USBDMAIntEn      = 0x00; 
 
LPC_USB->USBEoTIntClr     = 0x00; 
LPC_USB->USBEoTIntSet     = 0x00; 
 
LPC_USB->USBNDDRIntClr    = 0x00; 
LPC_USB->USBNDDRIntSet    = 0x00; 
 
LPC_USB->USBSysErrIntClr  = 0x00; 
LPC_USB->USBSysErrIntSet  = 0x00; 
}



int isp_btn_pressed()
{
	return GPIO_get(ISP_BTN);
}

void start_dfu()
{
    char toggle=0;
    rgbInit(); 
	DFU_init();
	usb_init();
	usb_connect();
    START_TIMER();

	while (DFU_complete() == 0)
    {
		usb_task();
        UART0_Printf("\n\r%8U",TIMER_GetTime(0));
        if(DFU_DownloadStartStatus()==1)
        {
          // TIMER_Stop(0); 
         STOP_TIMER();         
        }
        else if(TIMER_GetTime(0) >= DTU_TIMEOUT_VALUE)
        {
            break;
        }
        rgbWrite(RGB_OFF,RGB_OFF,toggle);
        toggle=!toggle;
    }
    
    rgbWrite(RGB_OFF,RGB_OFF,RGB_OFF);
	
}



// this seems to fix an issue with handoff after poweroff
// found here http://knowledgebase.nxp.trimm.net/showthread.php?t=2869
static void boot(uint32_t a)
{
	asm("LDR SP, [%0]" : : "r"(a));
	asm("LDR PC, [%0, #4]" : : "r"(a));
	// never returns
}

static uint32_t delay_loop(uint32_t count)
{
	volatile uint32_t j, del;
	for(j=0; j<count; ++j){
		del=j; // volatiles, so the compiler will not optimize the loop
	}
	return del;
}

static void new_execute_user_code(void)
{
	uint32_t addr=(uint32_t)USER_FLASH_START;
	// delay
	delay_loop(3000);
	// relocate vector table
	SCB->VTOR = (addr & 0x1FFFFF80);
//	// switch to RC generator
//	LPC_SC->PLL0CON = 0x1; // disconnect PLL0
//	LPC_SC->PLL0FEED = 0xAA;
//	LPC_SC->PLL0FEED = 0x55;
//	while (LPC_SC->PLL0STAT&(1<<25));
//	LPC_SC->PLL0CON = 0x0;    // power down
//	LPC_SC->PLL0FEED = 0xAA;
//	LPC_SC->PLL0FEED = 0x55;
//	while (LPC_SC->PLL0STAT&(1<<24));
//	LPC_SC->FLASHCFG &= 0x0fff;  // This is the default flash read/write setting for IRC
//	LPC_SC->FLASHCFG |= 0x5000;
//	LPC_SC->CCLKCFG = 0x0;     //  Select the IRC as clk
//	LPC_SC->CLKSRCSEL = 0x00;
//	LPC_SC->SCS = 0x00;		    // not using XTAL anymore
	delay_loop(1000);
	// reset pipeline, sync bus and memory access
	__asm (
		   "dmb\n"
		   "dsb\n"
		   "isb\n"
		  );
	boot(addr);
}

/* int delay(void)
{
	int i=0;
	for(i=0; i<200; i++)
	{}
return 0;
} */

void jump_to_appln()
{
    Printf("Coming out of DFU");
    usb_disconnect();
            GPIO_init(USB_CONNECT);
        GPIO_output(USB_CONNECT);
        
        GPIO_write(USB_CONNECT,1);
        delay_loop(10000);
        
       // usb_Reset();
        GPIO_write(USB_CONNECT,0);
        delay_loop(10000);
        Printf("Entering Application");
        new_execute_user_code();
       // NVIC_SystemReset();
}

__attribute__ ((interrupt)) void TIMER0_IRQHandler(void)
{
  //  unsigned int isrMask;
    util_BitSet(LPC_TIM0->IR, SBIT_MR0I);
    UART0_Printf("\n\rTimer ISR");
  //  isrMask = LPC_TIM0->IR; 
  //  LPC_TIM0->IR |= 1<<isrMask;         /* Clear the Interrupt Bit */

		TimeoutDetected = 1;
        Printf("In Timer ISR");
        jump_to_appln();
}

void myTimerIsr_0 (void)
{
		TimeoutDetected = 1;
        Printf("In Timer ISR");
        jump_to_appln();
}



int main()
{
	
	
	//WDT_Feed();
    SystemInit();

	GPIO_init(ISP_BTN); 
    GPIO_input(ISP_BTN);

    UART0_Init(9600);
    Printf("In Main Function");


    LPC_SC->PCONP |= (1<<SBIT_TIMER0) | (1<<SBIT_TIMER1); /* Power ON Timer0,1 */

        /* Clear TC on MR0 match and Generate Interrupt*/
    LPC_TIM0->PR   = getPrescalarForMicroSec(PCLK_TIMER0);      /* Prescalar for 1us */
    LPC_TIM0->MR0  = DTU_TIMEOUT_VALUE;                 /* Load timer value to generate 100ms delay*/
    LPC_TIM0->MCR  = (1<<SBIT_MR0S);// | (1<<SBIT_MR0R); 
                       /* Start timer by setting the Counter Enable*/
 

	//UART_init(UART_RX, UART_TX, 2000000);
	//printf("Bootloader Start\n");

	// give SD card time to wake up
	/* for (volatile int i = (1UL<<12); i; i--);

	SDCard_init(P0_9, P0_8, P0_7, P0_6);
	if (SDCard_disk_initialize() == 0)
		check_sd_firmware(); */

	dfu = 1;
	if (isp_btn_pressed() == 0)
	
	{
		//printf("ISP button pressed, entering DFU mode\n");
		dfu =1 ;
	}
	
	
	else if (WDT_ReadTimeOutFlag()) {
		WDT_ClrTimeOutFlag();
		//printf("WATCHDOG reset, entering DFU mode\n");
		dfu =1 ;
	}

	if (dfu)
	{	
            
	   start_dfu();
       
       jump_to_appln();
       

 
	}
	
	 
    #ifdef WATCHDOG
	WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
	WDT_Start(1<<22);
	#endif




	new_execute_user_code();


	for (volatile int i = (1<<18);i;i--);

	NVIC_SystemReset();

}


void __aeabi_unwind_cpp_pr0(void){}
void __libc_init_array(void){}


/*
int _write(int fd, const char *buf, int buflen)
{
	if (fd < 3)
	{
		while (UART_cansend() < buflen);
		return UART_send((const uint8_t *)buf, buflen);
	}
	return buflen;
}*/

void NMI_Handler() {
	//printf("NMI\n");
	for (;;);
}
void HardFault_Handler() {
	//printf("HardFault\n");
	for (;;);
}
void MemManage_Handler() {
	//printf("MemManage\n");
	for (;;);
}
void BusFault_Handler() {
	//printf("BusFault\n");
	for (;;);
}
void UsageFault_Handler() {
	//printf("UsageFault\n");
	for (;;);
}

unsigned int getPrescalarForMicroSec(uint8_t timerPclkBit)
{
    unsigned int pclk,prescalarForUs;
    pclk = (LPC_SC->PCLKSEL0 >> timerPclkBit) & 0x03;  /* get the pclk info for required timer */

    switch ( pclk )                                    /* Decode the bits to determine the pclk*/
    {
    case 0x00:
        pclk = SystemCoreClock/4;
        break;

    case 0x01:
        pclk = SystemCoreClock;
        break; 

    case 0x02:
        pclk = SystemCoreClock/2;
        break; 

    case 0x03:
        pclk = SystemCoreClock/8;
        break;

    default:
        pclk = SystemCoreClock/4;
        break;  
    }

    prescalarForUs =pclk/1000000 - 1;                    /* Prescalar for 1us (1000000Counts/sec) */

    return prescalarForUs;
}

