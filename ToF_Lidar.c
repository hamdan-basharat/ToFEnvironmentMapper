/* ************************************************************************************
2DX4 Final Project - Embedded Systems
Hamdan Basharat - basham1 - 400124515

This program allows from the collection of data from a ToF sensor that is rotating on
top of a stepper motor. This data is sent over I2C serial communication to the computer
COM port to by recieved and processed.

Last Updated: April 9th, 2020
************************************************************************************ */
//imported files
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

// The VL53L1X uses a slightly different way to define the default address of 0x29
// The I2C protocol defintion states that a 7-bit address is used for the device
// The 7-bit address is stored in bit 7:1 of the address register.  Bit 0 is a binary
// value that indicates if a write or read is to occur.  The manufacturer lists the 
// default address as 0x52 (0101 0010).  This is 0x29 (010 1001) with the read/write bit
// alread set to 0.
//uint16_t	dev = 0x29;
uint16_t	dev=0x52;

int status=0;

volatile int IntCount;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void PortF_Init(void);
void PortL_Init(void);
void PortM_Init(void);
void PortN_Init(void);
void VL53L1X_XSHUT(void);

//capture values from VL53L1X for inspection
uint16_t debugArray[8];

/*Motor Code*/
void AB(){GPIO_PORTM_DATA_R = 0b00000011;} //assigns a specific portM sequence to a motor position
void BC(){GPIO_PORTM_DATA_R = 0b00000110;}
void CD(){GPIO_PORTM_DATA_R = 0b00001100;}
void DA(){GPIO_PORTM_DATA_R = 0b00001001;}
	
int motorFlag; //flag used to determine if motor is on

void extLED(){ //turns the external LED on if the motor is not running
	if(motorFlag == 0){
		GPIO_PORTL_DATA_R = 0b00000100;
	}
	else{
		GPIO_PORTL_DATA_R = 0b00000000;
	}
}

void motor(int direction, int speed){ //runs the motor
	motorFlag = 1; //turns off the external LED
	extLED();
	if (direction==1){ //turns clockwise
		for (int i=1;i<=64;i++){ //turns 45deg
			if(i % 64 == 0){ //blinks onboard LED every 45 deg
				GPIO_PORTN_DATA_R = 0b00000010;//LED1 on
			}
			AB();
			SysTick_Wait10ms(speed); //delay to allow the motor to turn
			BC();
			SysTick_Wait10ms(speed);
			CD();
			SysTick_Wait10ms(speed);
			DA();
			SysTick_Wait10ms(speed);
			if(i % 64 == 0){
				GPIO_PORTN_DATA_R = 0b00000000;//LED1 off
			}
		}	
	}
	else if(direction==-1){ //turns counter clockwise
		for (int i=1;i<=512;i++){
			if(i % 64 == 0){ //blinks onboard LED every 45 deg
				GPIO_PORTN_DATA_R = 0b00000010;//LED1 on
			}
			DA();
			SysTick_Wait10ms(speed);
			CD();
			SysTick_Wait10ms(speed);
			BC();
			SysTick_Wait10ms(speed);
			AB();
			SysTick_Wait10ms(speed);
			if(i % 64 == 0){
				GPIO_PORTN_DATA_R = 0b00000000;//LED1 off
			}
		}
	}
	GPIO_PORTM_DATA_R = 0b00000000; //turns the motor off and external LED on
	motorFlag = 0; 
	extLED();
}


int main(void) {
	//variable declaration
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortF_Init();
	PortL_Init();
	PortM_Init();
	PortN_Init();
	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX4 Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
  myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n");
 	UART_printf("One moment...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	Status_Check("StartRanging", status);

	while(1){
			if((GPIO_PORTF_DATA_R & 0b00000100) == 0b00000100){ //only runs below code if button was pressed
				for(int i = 0; i < 8; i++){
					while (dataReady == 0){
						status = VL53L1X_CheckForDataReady(dev, &dataReady);
								FlashLED3(1);
								VL53L1_WaitMs(dev, 5);
					}
						dataReady = 0;
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
					status = VL53L1X_GetDistance(dev, &Distance);
						FlashLED4(1);
						debugArray[i] = Distance;     // Acquire sample
			//	  status = VL53L1X_GetSignalRate(dev, &SignalRate);
			//	  status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
			//	  status = VL53L1X_GetSpadNb(dev, &SpadNum);
					status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
						//sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
						sprintf(printf_buffer,"%u\n", Distance); //Sends distance measurement over serial comms
						UART_printf(printf_buffer);				
					
					SysTick_Wait10ms(50); //delay
					motor(1,1); //turns the motor 45 degress
			}
			motor(-1,1); //resets the motor back to original position
			VL53L1X_StopRanging(dev); //stops taking measurements from the ToF sensor
		}
	}
}


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
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        // 8) configure for 100 kbps clock
        
  // 20*(TPR+1)*20ns = 10us, with TPR=24
    // TED 100 KHz
    //     CLK_PRD = 8.3ns
    //    TIMER_PRD = 1
    //    SCL_LP = 6
    //    SCL_HP = 4
    //    10us = 2 * (1 + TIMER_PRD) * (SCL_LP + SCL_HP) * CLK_PRD
    //    10us = 2 * (1+TIMER+PRD) * 10 * 8.3ns
    //  TIMER_PRD = 59 (0x3B)
    //
    // TIMER_PRD is a 6-bit value.  This 0-127
    //    @0: 2 * (1+ 0) * 10 * 8.3ns --> .1667us or 6.0MHz
    //  @127: 2 * (1+ 127) * 10 * 8.3ns --> 47kHz
    
    
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

void PortF_Init(void){//Used for the button
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;               
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){}; 
	GPIO_PORTF_DIR_R = 0b00000000;       						
  GPIO_PORTF_DEN_R = 0b00000100;
	return;
}

void PortL_Init(void){//Used for the external LED
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;               
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){}; 
	GPIO_PORTL_DIR_R = 0b00000100;       						
  GPIO_PORTL_DEN_R = 0b00000100;
	return;
}

void PortM_Init(void){//Used for the motor	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				    
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	
	GPIO_PORTM_DIR_R |= 0b00001111;        							
  GPIO_PORTM_AFSEL_R &= ~0b00001111;     							
  GPIO_PORTM_DEN_R |= 0b00001111;        							
  GPIO_PORTM_AMSEL_R &= ~0b00001111;     									
	return;
}

void PortN_Init(void){//Used for the onboard LED
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;               
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){}; 
	GPIO_PORTN_DIR_R = 0b00000011;       						
  GPIO_PORTN_DEN_R = 0b00000011;
	return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}
