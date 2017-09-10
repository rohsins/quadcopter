#include <LPC17xx.h>
#include <cmsis_os2.h>
extern "C" {
	#include <GPIO_LPC17xx.h>
}
#include <UART_LPC17xx.h>
#include <I2C_LPC17xx.h>
#include <stdio.h>
#include <stdlib.h>
#include "ITM_ARM.h"
#include <string>
#include "RingBuffer.h"

#define IER_RBR 1U << 0
#define IER_THRE 1U << 1
#define IER_RLS 1U << 2

#define	DutyCycle0 LPC_PWM1->MR1
#define DutyCycle1	LPC_PWM1->MR2
#define DutyCycle4 LPC_PWM1->MR3
#define DutyCycle2	LPC_PWM1->MR4
#define DutyCycle3 LPC_PWM1->MR5

#define BNOADDRESS 0x29
const uint8_t WHOAMI = 0x01;
const uint8_t OPRMODE[2] = {0x3D, 0x0C};
const uint8_t EULERANGLE = 0x1A;
const uint8_t CALIBSTAT = 0x35;

extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_I2C Driver_I2C1;

ARM_USART_STATUS Driver_USART1_STATUS;
USART_TRANSFER_INFO Driver_USART1_INFO;

osSemaphoreId_t semaphoreTransmitId;
osSemaphoreId_t semaphoreReceiveId;
osSemaphoreId_t semaphoreDataReadyId;
osSemaphoreId_t semaphoreUartDataReadyId;
osSemaphoreId_t semaphoreShowDataReadyId;

osSemaphoreAttr_t semaphoreTransmitAttr;
osSemaphoreAttr_t semaphoreReceiveAttr;
osSemaphoreAttr_t semaphoreDataReadyAttr;
osSemaphoreAttr_t semaphoreUartDataReadyAttr;
osSemaphoreAttr_t semaphoreShowDataReadyAttr;

uint8_t eulerAngle[6];
int16_t eulerAngleX;
int16_t eulerAngleY;
int16_t eulerAngleZ;

int timeDiff = 10;

float errorPitch;
float errorRoll;
float errorYaw;
float errorPitchD;
float errorRollD;
float errorYawD;
float errorPitchI;
float errorRollI;
float errorYawI;
float errorPitchPrev;
float errorRollPrev;
float errorYawPrev;

int16_t pitchAngle;
int16_t rollAngle;
int16_t yawAngle = 180;

float pidPitch;
float pidRoll;
float pidYaw;
const float pidPitchP = 2;
const float pidPitchI = 0;
const float pidPitchD = 0.1;
const float pidRollP = 2;
const float pidRollI = 0;
const float pidRollD = 0.1;
const float pidYawP = 2;
const float pidYawI = 0;
const float pidYawD = 0.1;

int motor1;
int motor2;
int motor3;
int motor4;

int throttle;

char pData;
char readout[34];

static char readCheck;

RingBuffer ringBuffer;

using namespace std;

void USART_callback(uint32_t event)	{
    switch (event)	{
			case ARM_USART_EVENT_RECEIVE_COMPLETE: 
				readCheck = pData;
				ringBuffer.ringBufferWrite(pData);
				osSemaphoreRelease(semaphoreUartDataReadyId);
//			itmPrintln("receive complete");
				break;
			case ARM_USART_EVENT_TRANSFER_COMPLETE:
//				itmPrintln("transfer complete");
				break;
			case ARM_USART_EVENT_SEND_COMPLETE:
//				itmPrintln("send complete");
				break;
			case ARM_USART_EVENT_TX_COMPLETE:
//				itmPrintln("tx complete");
					/* Success: Wakeup Thread */
	//        osSignalSet(tid_myUART_Thread, 0x01);
					break;
	 
			case ARM_USART_EVENT_RX_TIMEOUT:
//				itmPrintln("rx timeout");
					 __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
					break;
	 
			case ARM_USART_EVENT_RX_OVERFLOW:
//				itmPrintln("rx overflow");
				break;
			case ARM_USART_EVENT_TX_UNDERFLOW:
	        __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
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
	GPIO_SetDir(2, 6, GPIO_DIR_OUTPUT);
}                                                                                                                                      

void configPWM(void) {
	LPC_PINCON->PINSEL4 = (1<<0) | (1<<2) | (1<<4) | (1<<6) | (1<<8); //PWM ON PIN 0, 1, 2, 3, 4
	LPC_PWM1->TCR = (1<<0) | (1<<2);
	LPC_PWM1->PR = 0x18; // 0xF8 100Hz
	LPC_PWM1->MCR = (1<<1);
	
	LPC_PWM1->MR0 = 10000; //set period to 100%
	
	DutyCycle0 = 0;
	DutyCycle1 = 0;
	DutyCycle2 = 0;
	DutyCycle3 = 0;
	DutyCycle4 = 0;
	
	LPC_PWM1->LER = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5); //PWM ON PIN 0, 1, 2, 3, 4
	
	LPC_PWM1->PCR = (1<<9) | (1<<10) | (1<<11) | (1<<12) | (1<<13); //PWM ON PIN 0, 1, 2, 3, 4
}

void calibrateMotor(void) {
	DutyCycle0 = 2000;
	DutyCycle1 = 2000;
	DutyCycle2 = 2000;
	DutyCycle3 = 2000;
	DutyCycle4 = 2000;

	osDelay(1000);

	DutyCycle0 = 1000;
	DutyCycle1 = 1000;
	DutyCycle2 = 1000;
	DutyCycle3 = 1000;
	DutyCycle4 = 1000;	
}
	
void heartBeatThread(void *arg) {
	while (1) {
		GPIO_PinWrite(2, 7, 0);
		osDelay(10);
		GPIO_PinWrite(2, 7, 1);
		osDelay(1000);
	}
}

void configBNO(void) {
	uint8_t buf = 0;
	uint8_t calibStat = 0;
	Driver_I2C1.MasterTransmit (BNOADDRESS, &WHOAMI, 1, false);
	osSemaphoreAcquire(semaphoreTransmitId, osWaitForever);
  Driver_I2C1.MasterReceive (BNOADDRESS, &buf, 1, false);
	osSemaphoreAcquire(semaphoreReceiveId, osWaitForever);
	
	
//	do {
//		Driver_I2C1.MasterTransmit (BNOADDRESS, &CALIBSTAT, 1, false);
//		osSemaphoreAcquire(semaphoreTransmitId, osWaitForever);
//		Driver_I2C1.MasterReceive (BNOADDRESS, &calibStat, 1, false);
//		osSemaphoreAcquire(semaphoreReceiveId, osWaitForever);
//		osDelay(100);
//	} while (calibStat != 0xFF);
	
	GPIO_PinWrite(2, 6, 1);
	
	Driver_I2C1.MasterTransmit (BNOADDRESS, OPRMODE, 2, false);
	osSemaphoreAcquire(semaphoreTransmitId, osWaitForever);
}

void imuUpdater(void * params) {
	osDelay(1000);
	
	configBNO();
	configPWM();
	calibrateMotor();
	
	osDelay(100);
	
	int16_t eulerAngleXTemp = 0;
	int16_t eulerAngleYTemp = 0;
	int16_t eulerAngleZTemp = 0;
	
	while (1) {
		Driver_I2C1.MasterTransmit (BNOADDRESS, &EULERANGLE, 1, false);
		osSemaphoreAcquire(semaphoreTransmitId, 100);
		Driver_I2C1.MasterReceive (BNOADDRESS, eulerAngle, 6, false);
		osSemaphoreAcquire(semaphoreReceiveId, 100);
		
		eulerAngleZTemp = eulerAngle[1] << 8 | eulerAngle[0];
		eulerAngleYTemp = eulerAngle[3] << 8 | eulerAngle[2];
		eulerAngleXTemp = eulerAngle[5] << 8 | eulerAngle[4];
		
		eulerAngleX = eulerAngleXTemp >> 4;
		eulerAngleY = eulerAngleYTemp >> 4;
		eulerAngleZ = eulerAngleZTemp >> 4;
		
		osSemaphoreRelease(semaphoreDataReadyId);	 
		osDelay(10);
	}
}

void computeEngine(void * params) {
	
	throttle = 0;
	
	while (1) {
		osSemaphoreAcquire(semaphoreDataReadyId, 10);
		
		errorPitch = pitchAngle - eulerAngleX;
		errorRoll = rollAngle - eulerAngleY;
		errorYaw = yawAngle - eulerAngleZ;
		
		errorPitchD = (errorPitch - errorPitchPrev)/timeDiff;
		errorRollD = (errorRoll - errorRollPrev)/timeDiff;
		errorYawD = (errorYaw - errorYawPrev)/timeDiff;
		
		errorPitchI += (errorPitch/10) * timeDiff;
		errorRollI += (errorRoll/10) * timeDiff;
		errorYawI += (errorYaw/10) * timeDiff;
		
		errorPitchPrev = errorPitch;
		errorRollPrev = errorRoll;
		errorYawPrev = errorYaw;
		
		pidPitch = (pidPitchP * errorPitch) + (pidPitchI * errorPitchI) + (pidPitchD * errorPitchD); 
		pidRoll = (pidRollP * errorRoll) + (pidRollI * errorRollI) + (pidRollD * errorRollD);
		pidYaw = (pidYawP * errorYaw) + (pidYawI * errorYawI) + (pidYawD * errorYawD);
		
		DutyCycle0 = motor1 = 1000 + throttle + pidPitch - pidRoll - pidYaw;
		DutyCycle1 = motor2 = 1000 + throttle + pidRoll + pidPitch + pidYaw;
		DutyCycle2 = motor3 = 1000 + throttle - pidPitch + pidRoll - pidYaw;
		DutyCycle3 = motor4 = 1000 + throttle - pidRoll - pidPitch + pidYaw;
		
//		DutyCycle0 = motor1 = 1000 + throttle + pidPitch - pidRoll;
//		DutyCycle1 = motor2 = 1000 + throttle + pidRoll + pidPitch;
//		DutyCycle2 = motor3 = 1000 + throttle - pidPitch + pidRoll;
//		DutyCycle3 = motor4 = 1000 + throttle - pidRoll - pidPitch;
		
		osSemaphoreRelease(semaphoreShowDataReadyId);
		
		osDelay(10);
	}
}

void uart0Thread(void * params) {
	static int checkIntegerValue = 0;
	char tempArray[34];
	while (1) {
		Driver_USART0.Receive(&pData, 1);
		osSemaphoreAcquire(semaphoreUartDataReadyId, osWaitForever);
		if (readCheck == 0x0A) {
			ringBuffer.ringBufferStringRead(readout);
//			itmPrint((char *)"readout:"); itmPrintln(readout);
			if (strncmp(readout, "throttle1", 9) == 0) {
				checkIntegerValue = (atoi(&readout[10]))*10;
				checkIntegerValue /= 10;
				DutyCycle0 = checkIntegerValue;
				Driver_USART0.Send("throttle1: ", 10);
				memset(tempArray, 0, 34);
				sprintf(tempArray, "%d", checkIntegerValue);
				Driver_USART0.Send(tempArray, 4);
				Driver_USART0.Send("\n", 1);
//				checkIntegerValue += atoi(&readout[10]);
			}else if (strncmp(readout, "throttle2", 9) == 0) {
				checkIntegerValue = (atoi(&readout[10]))*10;
				checkIntegerValue /= 10;
				DutyCycle1 = checkIntegerValue;
				Driver_USART0.Send("throttle2: ", 10);
				memset(tempArray, 0, 34);
				sprintf(tempArray, "%d", checkIntegerValue);
				Driver_USART0.Send(tempArray, 4);
				Driver_USART0.Send("\n", 1);
//				checkIntegerValue += atoi(&readout[10]);
			}else if (strncmp(readout, "throttle3", 9) == 0) {
				checkIntegerValue = (atoi(&readout[10]))*10;
				checkIntegerValue /= 10;
				DutyCycle2 = checkIntegerValue;
				Driver_USART0.Send("throttle3: ", 10);
				memset(tempArray, 0, 34);
				sprintf(tempArray, "%d", checkIntegerValue);
				Driver_USART0.Send(tempArray, 4);
				Driver_USART0.Send("\n", 1);
//				checkIntegerValue += atoi(&readout[10]);
			}else if (strncmp(readout, "throttle4", 9) == 0) {
				checkIntegerValue = (atoi(&readout[10]))*10;
				checkIntegerValue /= 10;
				DutyCycle3 = checkIntegerValue;
				Driver_USART0.Send("throttle4: ", 10);
				memset(tempArray, 0, 34);
				sprintf(tempArray, "%d", checkIntegerValue);
				Driver_USART0.Send(tempArray, 4);
				Driver_USART0.Send("\n", 1);
//				checkIntegerValue += atoi(&readout[10]);
			}else if (strncmp(readout, "throttle:", 9) == 0) {
				checkIntegerValue = (atoi(&readout[9]))*10;
				checkIntegerValue /= 10;
				throttle = checkIntegerValue;
				DutyCycle0 = throttle;
				DutyCycle1 = throttle;
				DutyCycle2 = throttle;
				DutyCycle3 = throttle;
				Driver_USART0.Send("throttle: ", 10);
				memset(tempArray, 0, 34);
				sprintf(tempArray, "%d", throttle);
				Driver_USART0.Send(tempArray, 4);
				Driver_USART0.Send("\n", 1);
//				checkIntegerValue += atoi(&readout[10]);
			}
			memset(readout, 0, 34);
		}
	}
}

void showReading(void * params) {
	char td[70] = {};
	while (1) {
		osSemaphoreAcquire(semaphoreShowDataReadyId, osWaitForever);
		memset(td, 0, 34);
		sprintf(td, "\n\n\neulerAngleX: %d \neulerAngleY: %d \neulerAngleZ: %d\n\nmotor1: %d \nmotor2: %d \nmotor3: %d \nmotor4: %d\n\n", eulerAngleX, eulerAngleY, eulerAngleZ, motor1, motor2, motor3, motor4);
		Driver_USART0.Send(td, 120);
		osDelay(1000);
	}
}

int main(void) {
	
	SystemCoreClockUpdate ();
	SysTick_Config(SystemCoreClock/1000);
		
	osKernelInitialize();

	uart0Initialize();
	
	Driver_USART0.Send("\nSystem Initialize\n", 19);
	
	ledInitialize();
	i2c1Initialize();
	
	semaphoreTransmitAttr.name = "Transmit Semaphore";
	semaphoreReceiveAttr.name = "Receive Semaphore";
	semaphoreDataReadyAttr.name = "DataReady Semaphore";
	semaphoreShowDataReadyAttr.name = "ShowData Semaphore";
	
	semaphoreTransmitId = osSemaphoreNew(1, 0, &semaphoreTransmitAttr);
	semaphoreReceiveId = osSemaphoreNew(1, 0, &semaphoreReceiveAttr);
	semaphoreDataReadyId = osSemaphoreNew(1, 0, &semaphoreDataReadyAttr);
	semaphoreUartDataReadyId = osSemaphoreNew(1, 0, &semaphoreUartDataReadyAttr);
	semaphoreShowDataReadyId = osSemaphoreNew(1, 0, &semaphoreShowDataReadyAttr);
	
	osThreadAttr_t osThreadUpdaterAttr;
	osThreadUpdaterAttr.name = "IMU Updater";
	osThreadUpdaterAttr.priority = osPriorityHigh;
	osThreadUpdaterAttr.stack_size = 200;
	osThreadUpdaterAttr.cb_mem = 0;
	osThreadUpdaterAttr.cb_size = 0;
	osThreadUpdaterAttr.stack_mem = 0;
	
	osThreadAttr_t osThreadHeartBeatAttr;
	osThreadHeartBeatAttr.name = "HeartBeat";
	osThreadHeartBeatAttr.priority = osPriorityNormal;
	osThreadHeartBeatAttr.stack_size = 0;
	osThreadHeartBeatAttr.cb_mem = 0;
	osThreadHeartBeatAttr.cb_size = 0;
	osThreadHeartBeatAttr.stack_mem = 0;
	
	osThreadAttr_t osThreadComputeEngineAttr;
	osThreadComputeEngineAttr.name = "Compute Engine";
	osThreadComputeEngineAttr.priority = osPriorityNormal;
	osThreadComputeEngineAttr.stack_size = 400;
	osThreadComputeEngineAttr.cb_mem = 0;
	osThreadComputeEngineAttr.cb_size = 0;
	osThreadComputeEngineAttr.stack_mem = 0;
	
	osThreadAttr_t osThreadUartReadAttr;
	osThreadUartReadAttr.name = "UartRead";
	osThreadUartReadAttr.priority = osPriorityNormal;
	osThreadUartReadAttr.stack_size = 200;
	osThreadUartReadAttr.cb_mem = 0;
	osThreadUartReadAttr.cb_size = 0;
	osThreadUartReadAttr.stack_mem = 0;
	
	osThreadAttr_t osThreadShowReadAttr;
	osThreadUartReadAttr.name = "Show Reading";
	osThreadUartReadAttr.priority = osPriorityNormal;
	osThreadUartReadAttr.stack_size = 200;
	osThreadUartReadAttr.cb_mem = 0;
	osThreadUartReadAttr.cb_size = 0;
	osThreadUartReadAttr.stack_mem = 0;
	
	osThreadNew(heartBeatThread, NULL, &osThreadHeartBeatAttr);
	osThreadNew(imuUpdater, NULL, &osThreadUpdaterAttr);
	osThreadNew(computeEngine, NULL, &osThreadComputeEngineAttr);
	osThreadNew(uart0Thread, NULL, &osThreadUartReadAttr);
	osThreadNew(showReading, NULL, &osThreadShowReadAttr);
	
	osKernelStart();
}
