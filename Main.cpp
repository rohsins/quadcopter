#include <LPC17xx.h>
#include <cmsis_os2.h>
extern "C" {
	#include <GPIO_LPC17xx.h>
}
#include <UART_LPC17xx.h>
#include <I2C_LPC17xx.h>
#include <stdio.h>
//#include "ITM_ARM.h"
#include <string.h>

#define IER_RBR 1U << 0
#define IER_THRE 1U << 1
#define IER_RLS 1U << 2

#define	DutyCycle0 LPC_PWM1->MR1
#define DutyCycle1	LPC_PWM1->MR2
#define DutyCycle2 LPC_PWM1->MR3
#define DutyCycle3	LPC_PWM1->MR4
#define DutyCycle4 LPC_PWM1->MR5

#define BNOADDRESS 0x29
const uint8_t WHOAMI = 0x01;
const uint8_t OPRMODE[2] = {0x3D, 0x0C};
const uint8_t EULERANGLE = 0x1A;

extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_I2C Driver_I2C1;

ARM_USART_STATUS Driver_USART1_STATUS;
USART_TRANSFER_INFO Driver_USART1_INFO;

osSemaphoreId_t semaphoreTransmitId;
osSemaphoreId_t semaphoreReceiveId;
osSemaphoreId_t semaphoreDataReadyId;

osSemaphoreAttr_t semaphoreTransmitAttr;
osSemaphoreAttr_t semaphoreReceiveAttr;
osSemaphoreAttr_t semaphoreDataReadyAttr;

char tempwhat[32];
uint8_t eulerAngle[6];
int16_t eulerAngleX;
int16_t eulerAngleY;
int16_t eulerAngleZ;

int timeDiff = 10;

float errorPitch;
float errorRoll;
float errorPitchD;
float errorRollD;
float errorPitchI;
float errorRollI;
float errorPitchPrev;
float errorRollPrev;

float pidPitch;
float pidRoll;
const float pidPitchP = 0.5;
const float pidPitchI = 0;
const float pidPitchD = 0.1;
const float pidRollP = 0.5;
const float pidRollI = 0;
const float pidRollD = 0.1;

float throttle;

void USART_callback(uint32_t event)
{
//		static int i;
	
    switch (event)
    {
    case ARM_USART_EVENT_RECEIVE_COMPLETE: 
//				itmPrintln("receive complete");
//					ringBufferWrite(pData);
//					ringBufferRead(readout);
//					itmPrint("readout:"); itmPrintlnInt(*readout);
//					i++;
//				itmPrintln(temp);
				break;
    case ARM_USART_EVENT_TRANSFER_COMPLETE:
//			itmPrintln("transfer complete");
			break;
    case ARM_USART_EVENT_SEND_COMPLETE:
//			itmPrintln("send complete");
			break;
    case ARM_USART_EVENT_TX_COMPLETE:
//			itmPrintln("tx complete");
        /* Success: Wakeup Thread */
//        osSignalSet(tid_myUART_Thread, 0x01);
        break;
 
    case ARM_USART_EVENT_RX_TIMEOUT:
//			itmPrintln("rx timeout");
         __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
        break;
 
    case ARM_USART_EVENT_RX_OVERFLOW:
//			itmPrintln("rx overflow");
			break;
    case ARM_USART_EVENT_TX_UNDERFLOW:
//        __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
		break;		
	}
}

void I2C_callback(uint32_t event) {
	if (Driver_I2C1.GetStatus().direction == 0) {
		osSemaphoreRelease(semaphoreTransmitId);
	} else if (Driver_I2C1.GetStatus().direction == 1) {
		osSemaphoreRelease(semaphoreReceiveId);
	}
}

void uart0Initialize(void) {
	Driver_USART0.Initialize(USART_callback);
	Driver_USART0.PowerControl(ARM_POWER_FULL);
	Driver_USART0.Control(ARM_USART_MODE_ASYNCHRONOUS| ARM_USART_DATA_BITS_8| ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE,115200);
	Driver_USART0.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART0.Control(ARM_USART_CONTROL_RX,1);      

	NVIC_EnableIRQ(UART0_IRQn);
	LPC_UART0->IER = IER_RBR | IER_THRE |IER_RLS; //Enable Interrupt
}

void uart0UnInitialize(void) {
	Driver_USART0.Uninitialize();
}

void uart1Initialize(void) {
	Driver_USART1.Initialize(USART_callback);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS| ARM_USART_DATA_BITS_8| ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE,115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX,1);  

	NVIC_EnableIRQ(UART1_IRQn);
	LPC_UART1->IER |= IER_RBR | IER_THRE |IER_RLS; //Enable Interrupt
}

void i2c1Initialize(void) {
	Driver_I2C1.Initialize(I2C_callback);
	Driver_I2C1.PowerControl(ARM_POWER_FULL);
	Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
	Driver_I2C1.Control(ARM_I2C_BUS_CLEAR, 0);
}

void uart1UnInitialize(void) {
	Driver_USART1.Uninitialize();
}

void ledInitialize(void) {
	GPIO_SetDir(2, 7, GPIO_DIR_OUTPUT);
}                                                                                                                                      

void configPWM(void) {
	LPC_PINCON->PINSEL4 = (1<<0) | (1<<2) | (1<<4) | (1<<6) | (1<<8); //PWM ON PIN 0, 1, 2, 3, 4
	LPC_PWM1->TCR = (1<<0) | (1<<2);
	LPC_PWM1->PR = 0xF8; // 0xF8 100Hz
	LPC_PWM1->MCR = (1<<1);
	
	LPC_PWM1->MR0 = 1000; //set period to 100%
	
	DutyCycle0 = 100 + 20;
	DutyCycle1 = 100 + 20;
	DutyCycle2 = 100 + 20;
	DutyCycle3 = 100 + 20;
	DutyCycle4 = 100 + 20;
	
	LPC_PWM1->LER = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5); //PWM ON PIN 0, 1, 2, 3, 4
	
	LPC_PWM1->PCR = (1<<9) | (1<<10) | (1<<11) | (1<<12) | (1<<13); //PWM ON PIN 0, 1, 2, 3, 4
}
	
void heartBeatThread(void *arg) {
	while (1) {
		GPIO_PinWrite(2, 7, 0);
		osDelay(20);
		GPIO_PinWrite(2, 7, 1);
		osDelay(2000);
	}
}

void configI2C(void) {
	uint8_t buf = 0;
	Driver_I2C1.MasterTransmit (BNOADDRESS, &WHOAMI, 1, false);
	osSemaphoreAcquire(semaphoreTransmitId, 1000);
  Driver_I2C1.MasterReceive (BNOADDRESS, &buf, 1, false);
	osSemaphoreAcquire(semaphoreReceiveId, 1000);
	
	Driver_I2C1.MasterTransmit (BNOADDRESS, OPRMODE, 2, false);
	osSemaphoreAcquire(semaphoreTransmitId, 1000);
}

void imuUpdater(void * params) {
	configI2C();
	configPWM();
	
	osDelay(1000);
	
	while (1) {
		Driver_I2C1.MasterTransmit (BNOADDRESS, &EULERANGLE, 1, false);
		osSemaphoreAcquire(semaphoreTransmitId, 1000);
		Driver_I2C1.MasterReceive (BNOADDRESS, eulerAngle, 6, false);
		osSemaphoreAcquire(semaphoreReceiveId, 1000);
		
		eulerAngleZ = eulerAngle[1] << 8 | eulerAngle[0];
		eulerAngleY = eulerAngle[3] << 8 | eulerAngle[2];
		eulerAngleX = eulerAngle[5] << 8 | eulerAngle[4];
		
		eulerAngleX = eulerAngleX >> 4;
		eulerAngleY = eulerAngleY >> 4;
		eulerAngleZ = eulerAngleZ >> 4;
		
		osSemaphoreRelease(semaphoreDataReadyId);
	}
}

void computeEngine(void * params) {
	
	throttle = 10;
	
	while (1) {
		osSemaphoreAcquire(semaphoreDataReadyId, 1000);
		
		errorPitch = 0 - eulerAngleX;
		errorRoll = 0 - eulerAngleY;
		
		errorPitchD = (errorPitch - errorPitchPrev)/timeDiff;
		errorRollD = (errorRoll - errorRollPrev)/timeDiff;
		
		errorPitchI += errorPitch * timeDiff;
		errorRollI += errorRoll * timeDiff;
		
		errorPitchPrev = errorPitch;
		errorRollPrev = errorRoll;
		
		pidPitch = (pidPitchP * errorPitch) + (pidPitchI * errorPitchI) + (pidPitchD * errorPitchD); 
		pidRoll = (pidRollP * errorRoll) + (pidRollI * errorRollI) + (pidRollD * errorRollD);
		
		pidPitch = pidPitch*10;
		pidRoll = pidRoll*10;
		
		DutyCycle0 = 100 + throttle + pidPitch;
		DutyCycle1 = 100 + throttle + pidRoll;
		DutyCycle2 = 100 + throttle - pidPitch;
		DutyCycle3 = 100 + throttle - pidRoll;
		
		osDelay(10);
	}
}

int main(void) {
	
	SystemCoreClockUpdate ();
	SysTick_Config(SystemCoreClock/1000);
		
	osKernelInitialize();

//	uart0Initialize();
//	uart1Initialize();
//	osDelay(100);
	ledInitialize();
	i2c1Initialize();
	
	semaphoreTransmitAttr.name = "Transmit Semaphore";
	semaphoreReceiveAttr.name = "Receive Semaphore";
	semaphoreDataReadyAttr.name = "DataReady Semaphore";
	
	semaphoreTransmitId = osSemaphoreNew(1, 0, &semaphoreTransmitAttr);
	semaphoreReceiveId = osSemaphoreNew(1, 0, &semaphoreReceiveAttr);
	semaphoreDataReadyId = osSemaphoreNew(1, 0, &semaphoreDataReadyAttr);
	
//	osThreadNew(heartBeatThread, NULL, NULL);
	osThreadNew(imuUpdater, NULL, NULL);
	osThreadNew(computeEngine, NULL, NULL);
	
	osKernelStart();
	
	return 0;
}
