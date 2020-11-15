#include "string.h"
#include <math.h>
#include "spi.h"


#include "task.h"
#include "os.h"

#if 1

#define SPI						SPI2
#define SPI_CLK					RCC_APB1Periph_SPI2
#define SPI_CLK_INIT			RCC_APB1PeriphClockCmd
#define SPI_IRQ_HANDLER			SPI2_IRQHandler
#define SPI_IRQn				SPI2_IRQn

#define SPI_DMA_IRQ_PRIO        (7)
#define SPI_DMA                 DMA1
#define SPI_DMA_CLK             RCC_AHB1Periph_DMA1
#define SPI_DMA_CLK_INIT        RCC_AHB1PeriphClockCmd

#define SPI_TX_DMA_STREAM       DMA1_Stream4
#define SPI_TX_DMA_IRQ          DMA1_Stream4_IRQn
#define SPI_TX_DMA_IRQHandler	DMA1_Stream4_IRQHandler
#define SPI_TX_DMA_CHANNEL      DMA_Channel_0
#define SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF4

#define SPI_RX_DMA_STREAM       DMA1_Stream3
#define SPI_RX_DMA_IRQ          DMA1_Stream3_IRQn
//#define spiRxDmaIsr   			DMA1_Stream3_IRQHandler
#define SPI_RX_DMA_CHANNEL      DMA_Channel_0
#define SPI_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF3

#define SPI_SCK_PIN           	GPIO_Pin_13
#define SPI_SCK_GPIO_PORT 		GPIOB
#define SPI_SCK_GPIO_CLK		RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE			GPIO_PinSource13
#define SPI_SCK_AF				GPIO_AF_SPI2

#define SPI_MISO_PIN			GPIO_Pin_14
#define SPI_MISO_GPIO_PORT		GPIOB
#define SPI_MISO_GPIO_CLK		RCC_AHB1Periph_GPIOB
#define SPI_MISO_SOURCE			GPIO_PinSource14
#define SPI_MISO_AF				GPIO_AF_SPI2

#define SPI_MOSI_PIN			GPIO_Pin_15
#define SPI_MOSI_GPIO_PORT		GPIOB
#define SPI_MOSI_GPIO_CLK		RCC_AHB1Periph_GPIOB
#define SPI_MOSI_SOURCE			GPIO_PinSource15
#define SPI_MOSI_AF				GPIO_AF_SPI2

#define DUMMY_BYTE         		0xA5

#endif

//static bool isInit = false;

//OS_SEM txComplete;
//OS_SEM rxComplete;
OS_MUTEX spiMutex;	//--会不会是这里要设置成Mutex？

//static void spiDMAInit(void);

void spi2Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* 使能GPIO时钟 */
	RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MISO_GPIO_CLK | SPI_MOSI_GPIO_CLK, ENABLE);

	/* 使能SPI时钟 */
	SPI_CLK_INIT(SPI_CLK, ENABLE);

	/* 使能DMA时钟*/
//	SPI_DMA_CLK_INIT(SPI_DMA_CLK, ENABLE);

	/*SPI 引脚配置*/

	/*复用功能设置 */
	GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);
	GPIO_PinAFConfig(SPI_MISO_GPIO_PORT, SPI_MISO_SOURCE, SPI_MISO_AF);
	GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	/*!< SPI SCK*/
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
	GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

	/*!< SPI MOSI*/
	GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_PIN;
	GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/*!< SPI MISO */
	GPIO_InitStructure.GPIO_Pin =  SPI_MISO_PIN;
	GPIO_Init(SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

	/*!< SPI DMA初始化 */
//	spiDMAInit();

	/*!< SPI 参数配置 */
	SPI_I2S_DeInit(SPI);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BAUDRATE_2MHZ;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 0; 

	SPI_Init(SPI, &SPI_InitStructure);
	SPI_Cmd(SPI, ENABLE);
//	isInit = true;
}

/*DMA 初始化*/
//void spiDMAInit()
//{
//	DMA_InitTypeDef  DMA_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	DMA_DeInit(DMA1_Stream4);
//	
//	/* DMA结构体配置 */
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI->DR)) ;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	DMA_InitStructure.DMA_BufferSize = 0;
//	DMA_InitStructure.DMA_Memory0BaseAddr = 0; 

//	// 配置 TX DMA
//	DMA_InitStructure.DMA_Channel = SPI_TX_DMA_CHANNEL;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	DMA_Cmd(SPI_TX_DMA_STREAM,DISABLE);
//	DMA_Init(SPI_TX_DMA_STREAM, &DMA_InitStructure);
//	
//	// 配置 RX DMA
//	DMA_InitStructure.DMA_Channel = SPI_RX_DMA_CHANNEL;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	DMA_Cmd(SPI_RX_DMA_STREAM,DISABLE);
//	DMA_Init(SPI_RX_DMA_STREAM, &DMA_InitStructure);

//	// 中断配置
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

//	NVIC_InitStructure.NVIC_IRQChannel = SPI_TX_DMA_IRQ;
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = SPI_RX_DMA_IRQ;
//	NVIC_Init(&NVIC_InitStructure);
//}


//bool spiTest(void)
//{
//	return isInit;
//}

bool spiExchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx)
{
//	OS_ERR err;
	uint8_t i=0,retry=0;
	// 设置内存地址 
//	SPI_TX_DMA_STREAM->M0AR = (uint32_t)data_tx;
//	SPI_TX_DMA_STREAM->NDTR = length;

//	SPI_RX_DMA_STREAM->M0AR = (uint32_t)data_rx;
//	SPI_RX_DMA_STREAM->NDTR = length;

	//使能SPI DMA 中断
//	DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
//	DMA_ITConfig(SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
//	SPI_ITConfig(SPI,SPI_I2S_IT_TXE,ENABLE);
//	SPI_ITConfig(SPI,SPI_I2S_IT_RXNE,ENABLE);
	for(i=0;i<length;i++){
		while(SPI_I2S_GetFlagStatus(SPI,SPI_I2S_FLAG_TXE) == RESET){
			retry++;
			if(retry>200){
				return false;		
			}
		}
		SPI_I2S_SendData(SPI,data_tx[i]);
		retry=0;
		while(SPI_I2S_GetFlagStatus(SPI,SPI_I2S_FLAG_RXNE) == RESET){
			retry++;
			if(retry>200){
				return false;		
			}		
		}
		retry=0;
		data_rx[i] = SPI_I2S_ReceiveData(SPI);
	}
	
//	// 清除中断标志位
//	DMA_ClearFlag(SPI_TX_DMA_STREAM, DMA_FLAG_FEIF4|DMA_FLAG_DMEIF4|DMA_FLAG_TEIF4|DMA_FLAG_HTIF4|DMA_FLAG_TCIF4);
//	DMA_ClearFlag(SPI_RX_DMA_STREAM, DMA_FLAG_FEIF3|DMA_FLAG_DMEIF3|DMA_FLAG_TEIF3|DMA_FLAG_HTIF3|DMA_FLAG_TCIF3);

//	// 使能 DMA 数据流
//	DMA_Cmd(SPI_TX_DMA_STREAM,ENABLE);
//	DMA_Cmd(SPI_RX_DMA_STREAM,ENABLE);

	//使能 SPI DMA 请求
//	SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx, ENABLE);
//	SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Rx, ENABLE);

	// 使能SPI
//	SPI_Cmd(SPI, ENABLE);

	// 等待传输完成
//	bool result = OSSemPend(&txComplete, 0, OS_OPT_PEND_BLOCKING, 0, &err);
//	result &= OSSemPend(&rxComplete, 0, OS_OPT_PEND_BLOCKING, 0, &err);
//	bool result = (xSemaphoreTake(txComplete, portMAX_DELAY) == pdTRUE)
//				&& (xSemaphoreTake(rxComplete, portMAX_DELAY) == pdTRUE);

	// 关闭SPI
//	SPI_Cmd(SPI, DISABLE);
	return true;
}

//void spiBeginTransaction(void)
//{
//	OS_ERR err;
//	OSMutexPend(&spiMutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
//	//xSemaphoreTake(spiMutex, portMAX_DELAY);
//}

//void spiEndTransaction()
//{
//	OS_ERR err;
//	OSMutexPost(&spiMutex, OS_OPT_POST_NONE, &err);
//	//xSemaphoreGive(spiMutex);
//}

/*DMA TX中断*/
//void  spiTxDmaIsr(void)
//{
//	OS_ERR err;
////	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

//	// 停止并清除 DMA 数据流
//	DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
//	DMA_ClearITPendingBit(SPI_TX_DMA_STREAM, SPI_TX_DMA_FLAG_TCIF);

//	// 清除标志位
//	DMA_ClearFlag(SPI_TX_DMA_STREAM,SPI_TX_DMA_FLAG_TCIF);

//	// 关闭 SPI DMA 请求
//	SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx, DISABLE);

//	// 关闭数据流
//	DMA_Cmd(SPI_TX_DMA_STREAM,DISABLE);

//	// 释放信号量
//	OSSemPost(&txComplete, OS_OPT_POST_1, &err);
////	xSemaphoreGiveFromISR(txComplete, &xHigherPriorityTaskWoken);

////	if (xHigherPriorityTaskWoken)
////	{
////		portYIELD();
////	}
//}

///*DMA RX中断*/
//void spiRxDmaIsr(void)
//{
//	OS_ERR err;
////	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

//	// 停止并清除 DMA 数据流
//	DMA_ITConfig(SPI_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
//	DMA_ClearITPendingBit(SPI_RX_DMA_STREAM, SPI_RX_DMA_FLAG_TCIF);

//	// 清除标志位
//	DMA_ClearFlag(SPI_RX_DMA_STREAM,SPI_RX_DMA_FLAG_TCIF);

//	// 关闭 SPI DMA 请求
//	SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Rx, DISABLE);

//	// 关闭数据流
//	DMA_Cmd(SPI_RX_DMA_STREAM,DISABLE);

//	// 释放信号量
//	OSSemPost(&rxComplete, OS_OPT_POST_1, &err);
////	xSemaphoreGiveFromISR(rxComplete, &xHigherPriorityTaskWoken);

////	if (xHigherPriorityTaskWoken)
////	{
////		portYIELD();
////	}
//}

//void  DMA1_Stream4_IRQHandler(void)
//{
//	OSIntEnter();
//	spiTxDmaIsr();
//	OSIntExit();
//}


////蓝牙串口中已定义
//void DMA1_Stream3_IRQHandler(void)
//{
//	OSIntEnter();
//	spiRxDmaIsr();
//	OSIntExit();
//}

