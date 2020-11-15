#include "my_platform.h"

//some array
uint8_t usart2_rx_buf[USART2_RX_LEN]={0};
uint8_t usart2_tx_buf[USART2_TX_LEN]={0};
int16_t angle_approximate[angle_bytes_num/2];
uint8_t angle_raw[angle_bytes_num];
//the angle raw data pointer;
uint16_t imu_dma_rx_count=0;
//platform initial
uint8_t imu_tx_busy_flag=0;
//imu busy flag
uint8_t plat_break_flag;
IMU imu={
	0,
	{
		imu_check,
		{
			imu_get_data,
			{
				angle_raw,
				angle_approximate
			}	
		}
	}
};
Platform_PID_OBJ plat_pid_roll={
	.kp=0,
	.ki=0,
	.kd=0,
	.erro=0,
	.last_erro=0,
	.pout=0,
	.iout=0,
	.dout=0,
	.out=0,
	.out_limit=0

};

Platform_PID_OBJ plat_pid_pitch={
	.kp=0,
	.ki=0,
	.kd=0,
	.erro=0,
	.last_erro=0,
	.pout=0,
	.iout=0,
	.dout=0,
	.out=0,
	.out_limit=0
};
Servo servo={
	.D1_value = 1500,
	.D2_value = 1500,
	.D3_value = 1500,	
	.D4_value = 1500,
	.breakout_flag=0
};
//the imu objective.
void platform_init(){
	imu_init();
//	Servo_init();
	platform_pid_init();
	
}

void imu_init(){
	usart2_init(115200);//default9600
//	imu_set_baudrate(baud_115200);
//	imu_set_rate(fre_100hz);
//	imu_keep_setting();
	imu.check.imu_check=imu_check;
	imu.check.imu_get_data.imu_get_data = imu_get_data;
	imu.check.imu_get_data.imu_data.angle_approximate_ptr = angle_approximate;
	imu.check.imu_get_data.imu_data.angle_raw_ptr = angle_raw;
	imu.imu_content = angle_pack_bit;
}

void Servo_init(){
	imu_set_D1_mode(PWM);
	delay_ms(100);
//	imu_keep_setting();
	imu_set_D2_mode(PWM);
	delay_ms(100);
//	imu_keep_setting();
	imu_set_D1_pwm_period(10000);
	delay_ms(100);
//	imu_keep_setting();
	imu_set_D2_pwm_period(10000);
	delay_ms(100);
//	imu_keep_setting();
	imu_set_D1_pulse_width(1500);
	delay_ms(100);
	imu_set_D2_pulse_width(1500);

	delay_ms(200);

}


void platform_pid_init(){
	plat_pid_roll.kp=0.007f;
	plat_pid_roll.ki=0;
	plat_pid_roll.kd=0.028f;
	plat_pid_roll.erro=0;
	plat_pid_roll.dout=0;
	plat_pid_roll.iout=0;
	plat_pid_roll.last_erro=0;
	plat_pid_roll.out=0;
	plat_pid_roll.out_limit=0;
	plat_pid_roll.pout=0;
	
	plat_pid_pitch.kp=0.007f;
	plat_pid_pitch.ki=0;
	plat_pid_pitch.kd=0.028f;
	plat_pid_pitch.erro=0;
	plat_pid_pitch.dout=0;
	plat_pid_pitch.iout=0;
	plat_pid_pitch.last_erro=0;
	plat_pid_pitch.out=0;
	plat_pid_pitch.out_limit=0;
	plat_pid_pitch.pout=0;	

	servo.D1_value=1500;
	servo.D2_value=1500;
	servo.D3_value=1500;
	servo.D4_value=1500;
	servo.breakout_flag=0;
}
//舵机D1 1000 - 2000  由负的pitch - 正的pitch
//舵机D2 1000 - 2000  由负的ROLL - 正的ROLL

void platform_control(int16_t target_angle_roll,int16_t target_angle_pitch){

	static uint8_t i=0;


	if(i){
		if((imu.check.imu_get_data.imu_data.angle_approximate_ptr[roll]<3500) && (imu.check.imu_get_data.imu_data.angle_approximate_ptr[roll]>-3500)){
			if((imu.check.imu_get_data.imu_data.angle_approximate_ptr[roll]>-100) && (imu.check.imu_get_data.imu_data.angle_approximate_ptr[roll]<100))imu.check.imu_get_data.imu_data.angle_approximate_ptr[roll]=0;	
			plat_pid_roll.erro = imu.check.imu_get_data.imu_data.angle_approximate_ptr[roll]-target_angle_roll ;
			plat_pid_roll.pout = plat_pid_roll.kp * plat_pid_roll.erro;
	//		plat_pid_roll.pout = Limits_data(plat_pid_roll.pout,100,-100);
			plat_pid_roll.dout = plat_pid_roll.kd * (plat_pid_roll.erro - plat_pid_roll.last_erro);
	//		plat_pid_roll.dout = Limits_data(plat_pid_roll.dout,80,-80);
			plat_pid_roll.out = plat_pid_roll.pout+plat_pid_roll.dout;
			plat_pid_roll.last_erro = plat_pid_roll.erro;
			servo.D1_value += plat_pid_roll.out;
		}
		else
		{
//			servo.D1_value = 1500;

		
		}			
		if(servo.D1_value>=2200)servo.D1_value=2200;
		else if(servo.D1_value<800)servo.D1_value=800;
		imu_set_D1_pulse_width(servo.D1_value);
		i=0;
	}else{
	if((imu.check.imu_get_data.imu_data.angle_approximate_ptr[pitch]<3500) && (imu.check.imu_get_data.imu_data.angle_approximate_ptr[pitch]>-3500)){	
		if((imu.check.imu_get_data.imu_data.angle_approximate_ptr[pitch]>-100) && (imu.check.imu_get_data.imu_data.angle_approximate_ptr[pitch]<100))imu.check.imu_get_data.imu_data.angle_approximate_ptr[pitch]=0;	
		plat_pid_pitch.erro = imu.check.imu_get_data.imu_data.angle_approximate_ptr[pitch] - target_angle_pitch ;
		plat_pid_pitch.pout = plat_pid_pitch.kp * plat_pid_pitch.erro;
//		plat_pid_pitch.pout = Limits_data(plat_pid_pitch.pout,50,-50);
		plat_pid_pitch.dout = plat_pid_pitch.kd * (plat_pid_pitch.erro - plat_pid_pitch.last_erro);
//		plat_pid_pitch.dout = Limits_data(plat_pid_pitch.dout,50,-50);
		plat_pid_pitch.out = plat_pid_pitch.pout+plat_pid_pitch.dout;
		plat_pid_pitch.last_erro = plat_pid_pitch.erro;	
		servo.D2_value -= plat_pid_pitch.out;
	
	}else{
//		servo.D2_value = 1500;
	
	}		
		if(servo.D2_value>=2200)servo.D2_value=2200;
		else if(servo.D2_value<800)servo.D2_value=800;
		imu_set_D2_pulse_width(servo.D2_value);
		i=1;
	}

}



/*************usart2 init*********************/
void usart2_init(uint32_t Bound){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;  
  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //使能UART5时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); //使能DMA1时钟
	

		//DMA发送中断设置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
	//DMA通道配置
	DMA_DeInit(DMA1_Stream6);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	//外设地址 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
	//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart2_tx_buf;  
	//dma传输方向
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
	//设置传输缓冲区的长度  
	DMA_InitStructure.DMA_BufferSize = USART2_TX_LEN;  
	//设置DMA的外设递增模式 一个外设  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	//设置DMA的内存递增模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	//外设数据字长 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	//内存数据字长  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
	//设置DMA传输模式  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	//设置优先级 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
	//指定如果FIFO模式或直接模式将用于指定的流：不使能FIFO模式    
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	//指定了FIFO的阈值水平 
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
	//指定的配置内存传输
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
	//指定的配置外围转移   
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
	//配置DMA1的通道           
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);    
	DMA_Cmd(DMA1_Stream6,DISABLE); 
	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);     	
	
	/* 串口收DMA配置 */
	//DMA通道配置  
	DMA_DeInit(DMA1_Stream5);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart2_rx_buf;   
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   
	DMA_InitStructure.DMA_BufferSize = USART2_RX_LEN;   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;   
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;    
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;       
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;           
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;           
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
		
	//配置DMA1 收通道          
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);    
	//使能通道
	DMA_Cmd(DMA1_Stream5,ENABLE); 

	//串口5对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);  //GPIOC12复用为UART5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); 	 //GPIOD2复用为UART5
	
	//UART5端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 				//GPIOC12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 				//GPIOD2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 			//上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	//UART5 初始化设置
	USART_InitStructure.USART_BaudRate = Bound;											//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;							//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); 												//初始化串口5
	
  // 中断配置	
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
  USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);  
  USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  
	//Uart5 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;				//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;	//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;				//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);													//根据指定的参数初始化VIC寄存器
	
	//采用DMA方式发送  
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	//采用DMA方式接收
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	 
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  
	
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);  
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);  
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  	
	
	
	USART_Cmd(USART2, ENABLE);  															//使能串口2 
}



void imu_usart_dma_tx_enable(uint16_t length){
	while(imu_tx_busy_flag);
	DMA_Cmd(DMA1_Stream6,DISABLE);
	DMA_SetCurrDataCounter(DMA1_Stream6,length);
	DMA_Cmd(DMA1_Stream6,ENABLE);
	imu_tx_busy_flag=1;
}	  

void imu_usart_dma_rx_enable(uint16_t length){
	DMA_Cmd(DMA1_Stream5,DISABLE);
	DMA_SetCurrDataCounter(DMA1_Stream5,length);
	DMA_Cmd(DMA1_Stream5,ENABLE);
}


uint16_t get_imu_usart_rx_cnt(){
	uint16_t temp_cnt=0;	
	DMA_Cmd(DMA1_Stream5,DISABLE);
	DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5); 
	temp_cnt = DMA_GetCurrDataCounter(DMA1_Stream5);
//	printf("tmp:%d\r\n",temp_cnt);
	return (USART2_RX_LEN-temp_cnt);
}
void DMA1_Stream6_IRQHandler(void)  
{  
		OSIntEnter();
		imu_tx_busy_flag=0;
		if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) != RESET)   
    {  
        DMA_ClearFlag(DMA1_Stream6,DMA_IT_TCIF6);  
        DMA_Cmd(DMA1_Stream6,DISABLE);    
				imu_tx_busy_flag=0;
    } 
		DMA_ClearFlag(DMA1_Stream6,DMA_IT_TCIF6);  
		OSIntExit();		
} 


uint8_t plat_update_flag;
void USART2_IRQHandler(void){
//	OS_ERR err;
	OSIntEnter();
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET){
		USART2->SR;		
		USART2->DR;
		//清除空闲中断位
		
		DMA_Cmd(DMA1_Stream5,DISABLE);    
		USART_ClearFlag(USART2,USART_FLAG_IDLE);
		imu_dma_rx_count = get_imu_usart_rx_cnt();
//		printf("rx length:%d\r\n",imu_dma_rx_count);
		imu_usart_dma_rx_enable(USART2_RX_LEN);
		//get the count of the rx data. 
		if(imu_dma_rx_count!=0){
			plat_update_flag=1;
		}
//		OSSemPost(&Platform_Semp,OS_OPT_POST_1,&err);
		
		
		//enable the dma rx and restart the receive.
	}
	OSIntExit();
}

/*************USART2 init*********************/







void imu_set_D1_mode(uint8_t mode){
	
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x0e;
	usart2_tx_buf[data_L] = mode;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);

}

void imu_set_D2_mode(uint8_t mode){
	
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x0f;
	usart2_tx_buf[data_L] = mode;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);

}

void imu_set_D3_mode(uint8_t mode){
	
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x10;
	usart2_tx_buf[data_L] = mode;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);

}

void imu_set_D4_mode(uint8_t mode){
	
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x11;
	usart2_tx_buf[data_L] = mode;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);

}

//us为单位
void imu_set_D1_pwm_period(uint16_t period){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x16;
	usart2_tx_buf[data_L] = (uint8_t)period&0x00ff;
	usart2_tx_buf[data_H] = (uint8_t)(period>>8)&0x00ff;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);

}
//us为单位
void imu_set_D2_pwm_period(uint16_t period){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x17;
	usart2_tx_buf[data_L] = (uint8_t)period&0x00ff;
	usart2_tx_buf[data_H] = (uint8_t)(period>>8)&0x00ff;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);

}
//us为单位
void imu_set_D3_pwm_period(uint16_t period){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x18;
	usart2_tx_buf[data_L] = (uint8_t)period&0x00ff;
	usart2_tx_buf[data_H] = (uint8_t)(period>>8)&0x00ff;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);

}
//us为单位
void imu_set_D4_pwm_period(uint16_t period){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x19;
	usart2_tx_buf[data_L] = (uint8_t)period&0x00ff;
	usart2_tx_buf[data_H] = (uint8_t)(period>>8)&0x00ff;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);

}


void imu_set_D1_pulse_width(uint16_t width){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x12;
	usart2_tx_buf[data_L] = (uint8_t)width&0x00ff;
	usart2_tx_buf[data_H] = (uint8_t)(width>>8)&0x00ff;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);


}
void imu_set_D2_pulse_width(uint16_t width){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x13;
	usart2_tx_buf[data_L] = (uint8_t)width&0x00ff;
	usart2_tx_buf[data_H] = (uint8_t)(width>>8)&0x00ff;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);


}
void imu_set_D3_pulse_width(uint16_t width){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x14;
	usart2_tx_buf[data_L] = (uint8_t)width&0x00ff;
	usart2_tx_buf[data_H] = (uint8_t)(width>>8)&0x00ff;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);


}
void imu_set_D4_pulse_width(uint16_t width){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x15;
	usart2_tx_buf[data_L] = (uint8_t)width&0x00ff;
	usart2_tx_buf[data_H] = (uint8_t)(width>>8)&0x00ff;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);


}
/*************servo part*********************/




/*************imu part*********************/



void imu_reset(){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = SAVE;
	usart2_tx_buf[data_L] = 1;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}


void imu_keep_setting(){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = SAVE;
	usart2_tx_buf[data_L] = 0;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}


void imu_set_calibration(enum imu_cali_mode mode){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = CALSW;
	usart2_tx_buf[data_L] = mode;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_set_install_dir(enum imu_install_dir dir){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x23;
	usart2_tx_buf[data_L] = dir;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_wake_reverse(){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x22;
	usart2_tx_buf[data_L] = 0x01;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_algorithm_change(enum imu_algo_set algo){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x24;
	usart2_tx_buf[data_L] = algo;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_gyro_auto_calibration(enum imu_gyro_cali_set cali_mode){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = 0x63;
	usart2_tx_buf[data_L] = cali_mode;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_set_return_content(uint16_t content_bit){
	imu.imu_content = content_bit;
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = RSW;
	usart2_tx_buf[data_L] = (uint8_t)content_bit;
	usart2_tx_buf[data_H] = (uint8_t)(content_bit>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);	
}


void imu_set_rate(enum imu_return_rate_set rate){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = RRATE;
	usart2_tx_buf[data_L] = rate;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}


void imu_set_baudrate(enum imu_baudrate_set baudrate){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = BAUD;
	usart2_tx_buf[data_L] = baudrate;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}


void imu_set_x_acce_offset(uint16_t offset){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = AXOFFSET;
	usart2_tx_buf[data_L] = (uint8_t)offset;
	usart2_tx_buf[data_H] = (uint8_t)(offset>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}


void imu_set_y_acce_offset(uint16_t offset){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = AYOFFSET;
	usart2_tx_buf[data_L] = (uint8_t)offset;
	usart2_tx_buf[data_H] = (uint8_t)(offset>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_set_z_acce_offset(uint16_t offset){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = AZOFFSET;
	usart2_tx_buf[data_L] = (uint8_t)offset;
	usart2_tx_buf[data_H] = (uint8_t)(offset>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_set_x_gyro_offset(uint16_t offset){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = GXOFFSET;
	usart2_tx_buf[data_L] = (uint8_t)offset;
	usart2_tx_buf[data_H] = (uint8_t)(offset>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_set_y_gyro_offset(uint16_t offset){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = GYOFFSET;
	usart2_tx_buf[data_L] = (uint8_t)offset;
	usart2_tx_buf[data_H] = (uint8_t)(offset>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_set_z_gyro_offset(uint16_t offset){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = GZOFFSET;
	usart2_tx_buf[data_L] = (uint8_t)offset;
	usart2_tx_buf[data_H] = (uint8_t)(offset>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}


void imu_set_x_mag_offset(uint16_t offset){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = HXOFFSET;
	usart2_tx_buf[data_L] = (uint8_t)offset;
	usart2_tx_buf[data_H] = (uint8_t)(offset>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_set_y_mag_offset(uint16_t offset){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = HYOFFSET;
	usart2_tx_buf[data_L] = (uint8_t)offset;
	usart2_tx_buf[data_H] = (uint8_t)(offset>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_set_z_mag_offset(uint16_t offset){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = HZOFFSET;
	usart2_tx_buf[data_L] = (uint8_t)offset;
	usart2_tx_buf[data_H] = (uint8_t)(offset>>8);
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}


void imu_set_dev_addr(uint8_t addr){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = IICADDR;
	usart2_tx_buf[data_L] = addr;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

void imu_set_led_on_off(enum imu_led_onoff onoff){
	usart2_tx_buf[head_1] = IMU_TX_CMD_HEAD_L;
	usart2_tx_buf[head_2] = IMU_TX_CMD_HEAD_H;	
	usart2_tx_buf[addr] = LEDOFF;
	usart2_tx_buf[data_L] = onoff;
	usart2_tx_buf[data_H] = 0;
	imu_usart_dma_tx_enable(imu_tx_cmd_length);
}

/*****************************IMU USART feature API********************************/

imu_res imu_check(IMU_DATA_GET imu_get_data){
	uint8_t get_sum=0,cal_sum=0;

	uint8_t pack_num=0,i=0,j=0;

	uint8_t right_num=0;
	if((imu_dma_rx_count%11)!=0){
	//the rx count is not the multiple of 11.
		return imu_erro_length;
	}
//	printf("rx length:%d\r\n",imu_dma_rx_count);
	pack_num=imu_dma_rx_count/11;
//	printf("pack num : %d\r\n",pack_num);
	for(i=0;i<pack_num;i++){
		get_sum = usart2_rx_buf[((i+1)*11)-1];
		cal_sum=0;
		for(j=0;j<10;j++){
			cal_sum+=usart2_rx_buf[j+(i*11)];		
		}
		if(cal_sum!=get_sum)continue;
		else
		{
			imu_get_data.imu_get_data(imu_get_data.imu_data,usart2_rx_buf[1+(i*11)],i);
			right_num=right_num+1 ;
//			printf("right num ++\r\n");
			
		}
	}
//	printf("right num:%d\r\n",right_num);
	if(right_num==0){	
//		printf("IMU_CRC\r\n");
//		printf("\r\n");
		return imu_erro_crc;
	}
	else if(right_num<pack_num){
		printf("IMU部分正常\r\n");
		printf("\r\n");
		return imu_success_part;
	}
	else if(right_num==pack_num){
//		printf("IMU正常\r\n");
//		printf("\r\n");
		return imu_success;
	}
	else {
		printf("IMU错误\r\n");
		printf("\r\n");
		return imu_erro_other;
	}
	
}


void imu_get_data(IMU_DATA imu_data,uint8_t pack_type,uint8_t seq){
	uint8_t i=0;
	switch(pack_type){
		case time:{
			
			break;
		}
		case acce:{
			
			break;
		}
		case gyro:{
			
			break;
		}
		case angle:{
			for(i=0;i<8;i++){
				imu_data.angle_raw_ptr[i]=usart2_rx_buf[(2+(seq*11)+i)];			
			}
			imu_data.angle_approximate_ptr[roll]=(int16_t)(((float)((int16_t)(((uint16_t)imu_data.angle_raw_ptr[1])<<8)|imu_data.angle_raw_ptr[0]))/32768*180*100);
			imu_data.angle_approximate_ptr[pitch]=(int16_t)(((float)((int16_t)(((uint16_t)imu_data.angle_raw_ptr[3])<<8)|imu_data.angle_raw_ptr[2]))/32768*180*100);
			imu_data.angle_approximate_ptr[yaw]=(int16_t)(((float)((int16_t)(((uint16_t)imu_data.angle_raw_ptr[5])<<8)|imu_data.angle_raw_ptr[4]))/32768*180*100);
			imu_data.angle_approximate_ptr[temp]=(int16_t)((((uint16_t)imu_data.angle_raw_ptr[7])<<8)|imu_data.angle_raw_ptr[6]);
//			printf("roll:%d\tpitch:%d\tyaw:%d\ttemp:%d\r\n",imu.check.imu_get_data.imu_data.angle_approximate_ptr[roll],\
//																									imu.check.imu_get_data.imu_data.angle_approximate_ptr[pitch],\
//																									imu.check.imu_get_data.imu_data.angle_approximate_ptr[yaw],\
//																									imu.check.imu_get_data.imu_data.angle_approximate_ptr[temp]);
			break;
		}
		case magn:{
			
			break;
		}
		case port:{
			
			break;
		}
		case height:{
			
			break;
		}
		case geographic:{
			
			break;
		}
		case speed_ground:{
			
			break;
		}
		case quaternion:{
			
			break;
		}		
		case gps:{
			
			break;
		}			
		default:break;
	}
	for(uint16_t i=0;i<imu_dma_rx_count;i++){
		usart2_rx_buf[i]=0;
	}
}

void imu_task(){
	imu.check.imu_check(imu.check.imu_get_data);
}
/*************imu part*********************/



