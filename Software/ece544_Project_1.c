/**
*
* @file ece544_Project_1.c
*
* @author Mohak Patel (mohak@pdx.edu)
*
* @copyright Portland State University, 2018-2020
*
* This file implements a project program for the Nexys4IO and Digilent Pmod peripherals.
* The peripherals provides access to the Nexys4 pushbuttons
* and slide switches, the LEDs, the RGB LEDs, and the Seven Segment display
* on the Digilent Nexys4 DDR board and the PmodOLEDrgb (94 x 64 RGB graphics display) 
* and the PmodENC (rotary encoder + slide switch + pushbutton).

* @note
* The minimal hardware configuration for this test is a Microblaze-based system with at least 32KB of memory,
* an instance of Nexys4IO, an instance of the pmodOLEDrgb AXI slave peripheral, and instance of the pmodENC AXI
* slave peripheral, an instance of AXI GPIO, an instance of AXI timer and an instance of the AXI UARTLite 
* (used for xil_printf() console output)
*
* @note
* The driver code and test application(s) for the pmodOLDrgb and pmodENC are
* based on code provided by Digilent, Inc.
******************************************************************************/
#include <math.h>
#include <mb_interface.h>
#include <nexys4IO.h>
#include <PmodENC.h>
#include <PmodOLEDrgb.h>
#include <sleep.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/_stdint.h>
#include <xgpio.h>
#include <xil_types.h>
#include <xintc.h>
#include <xstatus.h>
#include <xtmrctr.h>
#include <xtmrctr_l.h>
#include "platform.h"

/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0


// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL		1
#define GPIO_0_OUTPUT_0_CHANNEL		2

// GPIO parameters for RED
#define GPIO_1_DEVICE_ID			XPAR_AXI_GPIO_1_DEVICE_ID
#define GPIO_1_INPUT_0_CHANNEL		1
#define GPIO_1_INPUT_1_CHANNEL		2

// GPIO parameters for GREEN
#define GPIO_2_DEVICE_ID			XPAR_AXI_GPIO_2_DEVICE_ID
#define GPIO_2_INPUT_0_CHANNEL		1
#define GPIO_2_INPUT_1_CHANNEL		2

//// GPIO parameters for BLUE
#define GPIO_3_DEVICE_ID			XPAR_AXI_GPIO_3_DEVICE_ID
#define GPIO_3_INPUT_0_CHANNEL		1
#define GPIO_3_INPUT_1_CHANNEL		2

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
// Microblaze peripheral instances
uint64_t 	timestamp = 0L;
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;
XGpio		GPIOInst0;					// GPIO instance
XGpio		GPIOInst1;					// GPIO instance for RED
XGpio		GPIOInst2;					// GPIO instance for GREEN
XGpio		GPIOInst3;					// GPIO instance for BLUE
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance


// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler


volatile u32			gpio_in;				// GPIO input port
volatile u32            gpio_red_plus;			// GPIO input port
volatile u32 			gpio_red_min;			// GPIO input port
volatile u32 			gpio_blue_plus;			// GPIO input port
volatile u32			gpio_blue_min;			// GPIO input port
volatile u32			gpio_green_plus;		// GPIO input port
volatile u32			gpio_green_min;			// GPIO input port

//The following variables are for calculating the duty cycle
// and for hardware or software PWM detection.
//Initializing all variable to Zero.

//variable for Software PWM detection
int 		red_pluscounter=0;					// Counter for high pulse of RED signal
int			blue_pluscounter=0;					// Counter for high pulse of BLUE signal
int			green_pluscounter=0;				// Counter for high pulse of Green Signal
int         red_mincounter=0;					// Counter for low pulse of RED signal
int			green_mincounter=0;					// Counter for low pulse of BLUE signal
int			blue_mincounter=0;					// Counter for low pulse of Green Signal
int         lastred=0;							// 	Store the previous PWM pulse for High or low pulse detection
int			lastblue=0;							// 	Store the previous PWM pulse for High or low pulse detection
int			lastgreen=0;						// 	Store the previous PWM pulse for High or low pulse detection

// Variable for selecting Hardware or Software PWM detection
int			Hardware_detection=0;				// To select the Hardware PWM detection
int			Software_detection=0;				// To select the Software PWM detection

// Variable for calculating the duty cycle
int			red_duty_cycle;						//RED duty cycle
int			blue_duty_cycle;					//BLUE duty cycle
int			green_duty_cycle;					//GREEN duty cycle

// variable for updating OLED display
float 		lastsat=0;							// Store the previous Saturation value
float		lastval=0;							// Store the previous Value
int 		lasthue=1;							/// Store the previous HUE Value

/************************** Function Prototypes *****************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											// initialize system
void FIT_Handler(void);										// fixed interval timer interrupt handler
int AXI_Timer_initialize(void);

void display_red_dutycycle(int plus_count,int min_count);	//Display the  duty cycle of RED signal
void display_blue_dutycycle(int plus_count,int min_count);	//Display the duty cycle of BLUE signal
void display_green_dutycycle(int plus_count,int min_count);	//Display the duty cycle of GREEN signal
void PWM_detection(void);                                   // software or hardware PWM detection
void Update_OLEDrgb(int ticks1,float saturation1, float value1, int r1, int g1, int b1); // update the OLEDrgb display

void Color_wheel(void);

/************************** MAIN PROGRAM ************************************/
int main(void)
{
    init_platform();

	uint32_t sts;

	// initialize the all peripherals for project
	sts = do_init();
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	// Enable the interrupt
	microblaze_enable_interrupts();

	//Turn off all leds(LD15..LD0) on the Nexys 4
	NX4IO_setLEDs(0x00000000);

	xil_printf("ECE 544 Project- Color Wheel implementation and PWM detection \n\r");
	xil_printf("By Mohak Patel. 20-April-2018\n\n\r");

	usleep(1500*1000);

	// blank the display digits and turn off the decimal points
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);

	//Color Wheel implementation and PWM detection
	Color_wheel();
	
	// clear the displays and power down the pmodOLEDrbg
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_end(&pmodOLEDrgb_inst);
	
	// cleanup and exit
    cleanup_platform();
    exit(0);
}

/****************************************************************************/
/**
* Test 6 - color wheel implementation and PWM detection
*
*Color wheel will generate a variable duty-cycle PWM output signal for the
*on board RGB LED’s and it will detect the duty cycle of PWM output of RGB
*LED by pulse width detection hardware and by implementing a pulse-width
*detection algorithm in software. A color wheel lets a user choose any color
*by varying the Hue, Saturation and the Value in the HSV scale.
*
*Functionality of peripherals:
*	1. Slides Switch [0] selects between software(down) and hardware(up) pulse width detection.
*	2. LD0 to indicate software or hardware pulse width detection.
*	3. The btnR and btnL buttons are used to vary the Saturation value in the HSV scale.
*	4. The btnU and btnD buttons are used to vary the Value in the HSV scale.
*	5. The rotary encoder knob is used to alter the Hue value.
*	6. PmodOLEDrgb display is used for displaying the current Hue, Saturation and Value values on the
*	   left side of the display and Rectangle showing the color for the corresponding value of the
*	   Hue, Saturation and Value values on the right side of display.
*	7. RGB1 and RGB2 – The tri-color LEDs indicate the color selected from the color wheel.
*	8. Seven segment display is used for display the duty cycle of Red signal, Green signal,Blue signal
*	   and hardware (H)/ software (S) detection.
* @param	*NONE*
*
* @return	*NONE*

*****************************************************************************/
void Color_wheel()
{
	//Local variable
	u32 state, laststate;
	int ticks = 0 ,hue=0;
	float saturation=0, value=0,red,green,blue;
	char s[] = " End Test";

	xil_printf("Starting the project...\n");
	xil_printf("Turn PmodENC shaft.  HUE count is displayed\n");
	xil_printf("Press BTNUP and BTND is for changing the VALUE");
	xil_printf("Press BTNUL and BTNR is for changing the SATURATION");
	xil_printf("Press Rotary encoder shaft or BTNC to exit\n");

	// Set up the display output
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 0);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Hue:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Sat:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Val:");

	// get the previous state
	laststate = ENC_getState(&pmodENC_inst);

	while(1)
	{
		// get the PmodENC state
		state = ENC_getState(&pmodENC_inst);

		//check for hardware or software PWM detection
		PWM_detection();

		// check if the rotary encoder pushbutton or BTNC is pressed
		// exit the loop if either one is pressed.
		if (ENC_buttonPressed(state) && !ENC_buttonPressed(laststate))break;
		if (NX4IO_isPressed(BTNC))break;

		// check BTNR and BTNL count if it's pressed
		// update the saturation[0:1]
		// BTNR to increase the saturation and BTNL to decrease the saturation
		if (NX4IO_isPressed(BTNR) && (saturation >=0) && (saturation<=1) )
			{
				saturation =saturation+ 0.1;
			}
			else if (NX4IO_isPressed(BTNL)&& (saturation >=0.1) && (saturation<=1.1) )
				{
					saturation =saturation- 0.1;
				}

		//check BTND and BTNU count if it's pressed
		//update the value[0:1]
		//BTNU to increase the Value and BTND to decrease the Value
		if (NX4IO_isPressed(BTNU) &&(value >=0) && (value<=1))
			{
				value =value+ 0.1;
			}
			else if (NX4IO_isPressed(BTND)&&(value >=0.1) && (value<=1.1))
				{
					value =value- 0.1;
				}

		// Update the Hue value[0:360]
		// Making color wheel
		ticks += ENC_getRotation(state, laststate);
		if(ticks==360) ticks=0;
		else if (ticks==(-1)) ticks=359;

		// Convert HSV to RGB for display on OLEDrgb and to drive RGB LEDs
			hue=ticks;

			//saturation is zero then store the Value as RGB signal
			if(saturation==0)
			{
				red = value;
				green = value;
				blue =value;

			}

			//if not then convert the HSV to RGB
			//divide the hue in six sector
			//calculate the hue in degree and find out in which sector hue belongs to
			//calculate the bottom level of the triangle
			//calculate the slop of other two side of triangle
			//assign the value of all side of triangle according to the sector hue belongs to
			else
			{
				if (hue == 360)
					hue = 0;
				else
					hue = hue / 60;

				int i = (int)trunc(hue);
				float delta_hue = hue-i;
				float level_bottom = value * (1.0 - saturation);
				float slop_down = value * (1.0 - (saturation * delta_hue));
				float slop_up  = value * (1.0 - (saturation * (1.0 - delta_hue)));
				switch (i)
				{
					case 0:
						red = value;
						green = slop_up;
						blue = level_bottom;
						break;

					case 1:
						red = slop_down;
						green = value;
						blue = level_bottom;
						break;

					case 2:
						red = level_bottom;
						green = value;
						blue = slop_up;
						break;

					case 3:
						red = level_bottom;
						green = slop_down;
						blue = value;
						break;

					case 4:
						red = slop_up;
						green = level_bottom;
						blue = value;
						break;

					default:
						red = value;
						green = level_bottom;
						blue = slop_down;
						break;
				}
			}

			// convert float value of RGB to integer and multiply with 255 for limit the value between o to 255
			int r=(int)(255*red);
			int g=(int)(255*green);
			int b=(int)(255*blue);

		//update the display with the new count if the count has changed
		Update_OLEDrgb(ticks,saturation,value, r, g, b);

		laststate = state;

	}
	// Finish by turning the both LEDs off
	// We'll do this by disabling all of the channels without changing
	// the duty cycles
	NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 0);
	NX4IO_RGBLED_setDutyCycle(RGB2, 0, 0, 0);

	// Write one final string
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst, s);

	return;

}



/**************************** HELPER FUNCTIONS ******************************/

/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/

int	 do_init(void)
{
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO0 channel 1 is an 8-bit input port.
	// GPIO0 channel 2 is an 8-bit output port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);

	status = XGpio_Initialize(&GPIOInst1, GPIO_1_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO1 channel 1 is an 32-bit input port.
	// GPIO1 channel 2 is an 32-bit output port.
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL, 0xFFFFFFFF);

	status = XGpio_Initialize(&GPIOInst2, GPIO_2_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO2 channel 1 is an 32-bit input port.
	// GPIO2 channel 2 is an 32-bit output port.
	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_0_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_1_CHANNEL, 0xFFFFFFFF);

	status = XGpio_Initialize(&GPIOInst3, GPIO_3_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO3 channel 1 is an 32-bit input port.
	// GPIO3 channel 2 is an 32-bit output port.
	XGpio_SetDataDirection(&GPIOInst3, GPIO_3_INPUT_0_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst3, GPIO_3_INPUT_1_CHANNEL, 0xFFFFFFFF);


	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}
	
	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}
/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion,
*
* @return  *NONE*

*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*

*****************************************************************************/
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;

  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;

    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*

*****************************************************************************/ 
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];
  
  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}


/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO port which reads back the hardware generated PWM wave for the RGB Leds
*

 *****************************************************************************/

void FIT_Handler(void)
{
	//if Switch [0] is off then software PWM detection
	if(Software_detection==1)
		{
			// Read the GPIO port to read back the generated PWM signal for RGB led's
			gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);
		}

	//if Switch [0] is on then hardware PWM detection
	if(Hardware_detection==1)
		{
		// Read the GPIO port to read back the counter value of high and low pulse of RED PWM signal
			gpio_red_plus = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL);
			gpio_red_min= XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL);

			// Read the GPIO port to read back the counter value of high and low pulse of GREEN PWM signal
			gpio_green_plus= XGpio_DiscreteRead(&GPIOInst2, GPIO_2_INPUT_0_CHANNEL);
			gpio_green_min= XGpio_DiscreteRead(&GPIOInst2, GPIO_2_INPUT_1_CHANNEL);

			// Read the GPIO port to read back the counter value of high and low pulse of BLUE PWM signal
			gpio_blue_plus = XGpio_DiscreteRead(&GPIOInst3, GPIO_3_INPUT_0_CHANNEL);
			gpio_blue_min= XGpio_DiscreteRead(&GPIOInst3, GPIO_3_INPUT_1_CHANNEL);
		}

}

/**************************** Display function ******************************/

/****************************************************************************/
/*
* calculate the duty cycle and display it on the seven segment display.
*/
/****************************************************************************/

void display_red_dutycycle(int plus_count,int min_count)
{
	//calculating duty cycle of RED PWM signal
	red_duty_cycle=round(((float)(plus_count*100)/(float)(plus_count+ min_count)));

	//Display the duty cycle on seven segment
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT7, (red_duty_cycle/10));
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT6, (red_duty_cycle%10));

}

void display_green_dutycycle(int plus_count,int min_count)

{
	//calculating duty cycle of GREEN PWM signal
	green_duty_cycle=round(((float)(plus_count*100)/(float)(plus_count+ min_count)));

	//Display the duty cycle on seven segment
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, (green_duty_cycle / 10));
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, (green_duty_cycle % 10));
}

void display_blue_dutycycle(int plus_count,int min_count)

{
	//calculating duty cycle of BLUE PWM signal
	blue_duty_cycle=round(((float)(plus_count*100)/(float)(plus_count+ min_count)));

	//Display the duty cycle on seven segment
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, (blue_duty_cycle/10));
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, (blue_duty_cycle%10));
}

/******************** Software/Hardware PWM detection ***********************/
/*
 * Select the software or hardware PWM detection.
 * Software PWM detection algorithm.
 *
 */
/****************************************************************************/
void PWM_detection()
{
	//If switch is up - Hardware detection
	if((NX4IO_getSwitches() & 0x0001)== 1)
			{
				// if it is first time or shift from software to hardware detection
				// then display "H" on seven segment
				if(Hardware_detection==0)
				{
					//Turn on the LD0
					NX4IO_setLEDs(0x00000001);

					// blank the display digits and turn off the decimal points
					NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
					NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);

					//display "H" on seven segment
					NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, 24);
					usleep(1500*1000);

					// set the variable for software detection and indication for hardware detection
					Hardware_detection=1;
					Software_detection=0;

					//Set the seven segment display zero to display duty cycle
					NX4IO_SSEG_setDigit(SSEGHI, DIGIT7, 0);
					NX4IO_SSEG_setDigit(SSEGHI, DIGIT6, 0);
					NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, 0);
					NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, 0);
					NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, 0);
					NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, 0);

				}

				//calculate the duty cycle and display it on seven segment display
				display_red_dutycycle(gpio_red_plus,gpio_red_min);
				display_green_dutycycle(gpio_green_plus,gpio_green_min);
				display_blue_dutycycle(gpio_blue_plus,gpio_blue_min);

			}

			//switch is down - Software detection
			else
			{
				// if it is first time or shift from hardware or software detection
				// then display "S" on seven segment
				if(Software_detection==0)
				{
					//Turn off the LD0
					NX4IO_setLEDs(0x00000000);

					// blank the display digits and turn off the decimal points
					NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
					NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);

					//display "S" on seven segment
					NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, 5);
					usleep(1500*1000);

					// set the variable for hardware detection and indication for software detection
					Software_detection=1;
					Hardware_detection=0;

					//Set the seven segment display zero to display duty cycle
					NX4IO_SSEG_setDigit(SSEGHI, DIGIT7, 0);
					NX4IO_SSEG_setDigit(SSEGHI, DIGIT6, 0);
					NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, 0);
					NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, 0);
					NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, 0);
					NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, 0);
					}

					//Local variable for software PWM detection
					u8    redval,blueval,greenval;

					// separate the red, green and blue bit by doing logical And
					redval=(gpio_in & 0x04);
					blueval=(gpio_in & 0x02);
					greenval=(gpio_in & 0x01);

					//Calculating the counter value for high and low signal of red,green and blue signal

					//check for red signal is high
					// if last PWM signal of red is not equal to the current PWM then
					//calculate the duty cycle and display it on seven segment display
					// Reset the counters
					// otherwise increment the counter and store the current PWM signal to Previous PWM signal
					if(redval==0x04)
					{
						if(lastred==0)
						{
							display_red_dutycycle(red_pluscounter, red_mincounter);
							red_pluscounter=0;
							red_mincounter=0;
						}
						red_pluscounter+=1;
						lastred=1;
					}
					// if signal is low then increment the counter and store the current PWM signal to Previous PWM signal
					else if (redval==0x00)
					{
						red_mincounter+=1;
						lastred=0;
					}

					//check for blue signal is high
					// if last PWM signal of red is not equal to the current PWM then
					//calculate the duty cycle and display it on seven segment display
					// Reset the counters
					// otherwise increment the counter and store the current PWM signal to Previous PWM signal
					if(blueval==0x02)
						{
							if(lastblue==0)
							{
								display_blue_dutycycle(blue_pluscounter, blue_mincounter);
								blue_pluscounter=0;
								blue_mincounter=0;
							}
							blue_pluscounter+=1;
							lastblue=1;
						}
					// if signal is low then increment the counter and store the current PWM signal to Previous PWM signal
						else if (blueval==0x00)
						{
							blue_mincounter+=1;
							lastblue=0;
						}

					//check for green signal is high
					// if last PWM signal of red is not equal to the current PWM then
					//calculate the duty cycle and display it on seven segment display
					// Reset the counters
					// otherwise increment the counter and store the current PWM signal to Previous PWM signal
					if(greenval==0x01)
						{
							if(lastgreen==0)
							{
								display_green_dutycycle(green_pluscounter, green_mincounter);
								green_pluscounter=0;
								green_mincounter=0;
							}
							green_pluscounter+=1;
							lastgreen=1;
						}
					// if signal is low then increment the counter and store the current PWM signal to Previous PWM signal
						else if (greenval==0x00)
						{
							green_mincounter+=1;
							lastgreen=0;
						}

			}

}

/******************** Update_OLED display ***********************/
/*
 * update the OLED display when saturation,hue or value change.
 * According to HSV value drive the RGB Leds
 *
 */
/****************************************************************************/

void Update_OLEDrgb(int ticks1,float saturation1, float value1, int r1, int g1, int b1)
{	//Local variable
	char t[3];
				//if anyone of the value change then display it on OLEDrgb display
				if ((ticks1 != lasthue) || (saturation1 != lastsat) || (value1 != lastval))
				{
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 0);
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"     ");
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 0);
					PMDIO_putnum(&pmodOLEDrgb_inst,(ticks1) , 10);

					snprintf(t, 4, "%f", saturation1);
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"     ");
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
					OLEDrgb_PutString(&pmodOLEDrgb_inst,t);

					snprintf(t, 4, "%f", value1);
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 2);
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"     ");
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 2);
					OLEDrgb_PutString(&pmodOLEDrgb_inst,t);

					//Draw rectangle on OLEDrgb display
					//filled with combination of the red,green and blue color
					OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 40,30 ,90 ,60,OLEDrgb_BuildRGB(255,255, 255), TRUE,OLEDrgb_BuildRGB(r1, g1, b1));

					// enable all three PWM channels for both RGB LEDs
					// Set the duty cycle according to the HSV
					NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
					NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
					NX4IO_RGBLED_setDutyCycle(RGB1, r1, g1, b1);
					NX4IO_RGBLED_setDutyCycle(RGB2, r1, g1, b1);

				}
			//store the current value of HSV to the previous HSV
			lasthue = ticks1;
			lastsat=saturation1;
			lastval=value1;

	}




