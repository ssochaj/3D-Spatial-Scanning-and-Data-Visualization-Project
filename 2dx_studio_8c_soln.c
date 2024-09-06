/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE
							
						Modified April 10, 2023
						Updated by Sydney Sochaj (400446729, sochajs)
						  - Final Project for 2DX3

*/
#include "stdio.h"
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

// sensor uses PB2 and PB3
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortE_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTE_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTE_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTE_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTE_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
		}

// initialize port H for motor
void PortH_Init(void) {
    // Use PortM pins (PM0-PM3) for output
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;          // Activate clock for Port M
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0) {}  // Allow time for clock to stabilize
    GPIO_PORTH_DIR_R |= 0x0F;                         // Configure Port M pins (PM0-PM3) as output
    GPIO_PORTH_AFSEL_R &= ~0x0F;                      // Disable alt funct on Port M pins (PM0-PM3)
    GPIO_PORTH_DEN_R |= 0x0F;                         // Enable digital I/O on Port M pins (PM0-PM3)
                                                      // Configure Port M as GPIO
    GPIO_PORTH_AMSEL_R &= ~0x0F;                      // Disable analog functionality on Port M pins (PM0-PM3)
    return;
}
// for interrupt
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x02;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	Enable weak pull up resistor on PJ1
}

//Enable LED D3, D4. Remember D3 is connected to PF4 and D4 is connected to PF0
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;  // Activate the clock for Port F
      while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};                             // Allow time for clock to stabilize
            
      GPIO_PORTF_DIR_R=0b00010001;           // Enable PF0 and PF4 as outputs
      GPIO_PORTF_DEN_R=0b00010001;           // Enable PF0 and PF4 as digital pins
      return;
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}
// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
		GPIO_PORTJ_IS_R = 0;   						// (Step 1) PJ1 is Edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;  						//     			PJ1 is not triggered by both edges 
		GPIO_PORTJ_IEV_R = 0;  						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x02; 					// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;  					// 					Arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000; 					// (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000;				// (Step 4) Set interrupt priority to 5

		EnableInt();											// (Step 3) Enable Global Interrupt. lets go!
}

void GPIOJ_IRQHandler(void){
  FlashLED3(1);	
	GPIO_PORTJ_ICR_R = 0x02; 					// Acknowledge flag by setting proper bit in ICR register

}


void spin_motor(void) {
    uint32_t delay = 1;  // Adjust delay as needed

    // CLOCKWISE ROTATION
		// full phase rotation of motor
    GPIO_PORTH_DATA_R = 0b00000011;
    SysTick_Wait10ms(delay);
    GPIO_PORTH_DATA_R = 0b00000110;
    SysTick_Wait10ms(delay);
    GPIO_PORTH_DATA_R = 0b00001100;
    SysTick_Wait10ms(delay);
    GPIO_PORTH_DATA_R = 0b00001001;
    SysTick_Wait10ms(delay);
}

// motor turns counterclockwise to untangle wires
void return_home(void) {
    uint32_t delay = 1;  // Adjust delay as needed
		for(int i=0; i<512; i++){												
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);											
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(delay);
		}   
}


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t sensorState=0, i=0;
  uint16_t Distance;
  uint8_t dataReady;
	int counter = 0;
	int input = 0;

	
	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();

	// 1 Wait for device ToF booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);

  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

		// waits for the transmission code from pc. if 's' recieved then send data	
	
	while(1){		
		//waits for the correct transmition initiation code
		counter = 0;
		while(1){
			input = UART_InChar();
			if (input == 's')
				break;
		}
		
		while(counter <=512) {
		spin_motor();
		counter++;
		if(counter %16 == 0) {
		
      // wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
      FlashLED4(1);
      VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		//7 read the data values from ToF sensor
	    status = VL53L1X_GetDistance(dev, &Distance);					//7 The Measured Distance value
			FlashLED3(1); // measurement status

	  status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/
		
		// print the resulted readings to UART (Python)
		sprintf(printf_buffer,"%u\r\n", Distance);
		UART_printf(printf_buffer);
	  SysTick_Wait10ms(20);
  }
	}
	return_home();
		
		
		
}
	
  
	


}


