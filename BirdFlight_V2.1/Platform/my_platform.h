#ifndef MY_PLATFROM_H
#define MY_PLATFROM_H

#include "stm32f4xx.h"
#include "task.h"

#define USART2_TX_LEN 15	
#define USART2_RX_LEN 30




typedef enum{
	AIN=0x00,
	DIN=0x01,
	OUT_H=0x02,
	OUT_L=0x03,
	PWM=0x04,
	CLR=0x05
	
}DIO_MODE;

typedef enum{
	roll=0,
	pitch=1,
	yaw=2,
	temp=3,
	angle_bytes_num=8
}IMU_ANGLE_;
//the enum of the imu angle get by iic

enum{
	head_1=0,
	head_2=1,
	addr=2,
	data_L=3,
	data_H=4,
	imu_tx_cmd_length=5
};
//the imu tx command format
enum imu_cali_mode{
	exit_cali=0,
	enter_acces_cali=1,
	enter_mag_cali=2,
	set_high_zero=3
};
//the imu set command enum.
enum imu_install_dir{
	install_level=0,
	install_vertical=1
};
//the install direction.
enum imu_algo_set{
	nine_axes_algo=0,
	six_axes_algo=1
};
//the imu algorithm set.
enum imu_gyro_cali_set{
	gyro_auto_cali=0,
	gyro_no_auto_cali=1
};
//the imu gyro calibration set
enum imu_return_content_set{
	time_pack_bit=0x01,
	acce_pack_bit=0x02,
	gyro_pack_bit=0x04,
	angle_pack_bit=0x08,
	magn_pack_bit=0x10,
	port_stat_bit=0x20,
	height_pack_bit=0x40,
	gps_pack_bit=0x80,
		//Low byte.
	level_v_pack_bit=0x0100,
	quat_pack_bit=0x0200,
	sate_preci_bit=0x0400,
	//high byte.
};
//the return content set
enum imu_return_rate_set{
	fre_0_1hz=0x01,
	fre_0_5hz=0x02,
	fre_1hz=0x03,
	fre_2hz=0x04,
	fre_5hz=0x05,
	fre_10hz=0x06,
	fre_20hz=0x07,
	fre_50hz=0x08,
	fre_100hz=0x09,
	fre_125hz=0x0a,
	fre_200hz=0x0b,
	output_once=0x0c,
	output_none=0x0d		
};
//set the return rate.
enum imu_baudrate_set{
	baud_2400=0x01,	
	baud_4800=0x01,
	baud_9600=0x02,
	baud_19200=0x03,
	baud_38400=0x04,
	baud_57600=0x05,
	baud_115200=0x06,
	baud_230400=0x07,
	baud_460800=0x08,
	baud_921600=0x09,
};
//set the baudrate.
enum imu_led_onoff{
	led_on=0,
	led_off=1
};
//set imu led on or off.
typedef enum{
	imu_success=0x00,
	imu_success_part=0x04,
	imu_erro_length=0x01,
	imu_erro_crc=0x02,
	imu_erro_other=0x03
}imu_res;
//the result of the imu res.
enum{
	time=0x50,
	acce=0x51,
	gyro=0x52,
	angle=0x53,
	magn=0x54,
	port=0x55,
	height=0x56,
	geographic=0x57,
	speed_ground=0x58,
	quaternion=0x59,
	gps=0x5A
};
//the imu pack type
typedef struct{
	uint8_t *angle_raw_ptr;
	//the angle raw data pointer;
	int16_t *angle_approximate_ptr;
	//the approximate angle for convenient calculation.

}IMU_DATA;
//the imu data
typedef void (*imu_get_data_fn)(IMU_DATA imu_data,uint8_t pack_type,uint8_t seq);
//get data function.
typedef struct{
	imu_get_data_fn imu_get_data;
	//get data function.
	IMU_DATA imu_data;
	//the imu data
}IMU_DATA_GET;
//the imu data gathering function,
typedef imu_res (*imu_data_check_fn)(IMU_DATA_GET imu_get_data);
//check function.
typedef struct{
	uint16_t imu_content;
	//imu content.
	struct{
		imu_data_check_fn imu_check;
		//check data function.
		IMU_DATA_GET imu_get_data;
	}check;

}IMU;
//imu structure.


#define IMU_TX_CMD_HEAD_L 0xFF
#define IMU_TX_CMD_HEAD_H 0xAA
//imu tx command head
#define IMU_RX_CMD_HEAD 0x55
//imu rx command head
#define IMU_DEV_DEFAULT_ADDR	0x50
//device default address
#define SAVE 			0x00
#define CALSW 		0x01
#define RSW 			0x02
#define RRATE			0x03
#define BAUD 			0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define GPSBAUD		0x1c
#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS					0x33
#define AX					0x34
#define AY					0x35
#define AZ					0x36
#define GX					0x37
#define GY					0x38
#define GZ					0x39
#define HX					0x3a
#define HY					0x3b
#define HZ					0x3c			
//#define Roll				0x3d
//#define Pitch				0x3e
//#define Yaw					0x3f
#define TEMP				0x40
#define D0Status		0x41
#define D1Status		0x42
#define D2Status		0x43
#define D3Status		0x44
#define PressureL		0x45
#define PressureH		0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight   0x4d
#define GPSYAW      0x4e
#define GPSVL				0x4f
#define GPSVH				0x50
//register address define 


#define ACCURACY	7.4  //бу/us
typedef struct{
	float kp;
	float ki;
	float kd;
	int16_t erro;
	int16_t last_erro;
	int16_t pout;
	int16_t iout;
	int16_t dout;
	int16_t out;
	int16_t out_limit;
}Platform_PID_OBJ;


typedef struct{
	uint16_t D1_value;
	uint16_t D2_value;
	uint16_t D3_value;
	uint16_t D4_value;
	uint8_t breakout_flag;
}Servo;
void platform_init(void);
void Servo_init(void);
void imu_init(void);
void platform_pid_init(void);
void platform_control(int16_t target_angle_roll,int16_t target_angle_pitch);


void usart2_init(uint32_t Bound);
void imu_usart_dma_init(void);
void imu_usart_dma_tx_enable(uint16_t length);
void imu_usart_dma_rx_enable(uint16_t length);
uint16_t get_imu_usart_rx_cnt(void);
//imu usart initial function

void imu_set_D1_mode(uint8_t mode);
void imu_set_D2_mode(uint8_t mode);
void imu_set_D3_mode(uint8_t mode);
void imu_set_D4_mode(uint8_t mode);
void imu_set_D1_pwm_period(uint16_t period);
void imu_set_D2_pwm_period(uint16_t period);
void imu_set_D3_pwm_period(uint16_t period);
void imu_set_D4_pwm_period(uint16_t period);
void imu_set_D1_pulse_width(uint16_t width);
void imu_set_D2_pulse_width(uint16_t width);
void imu_set_D3_pulse_width(uint16_t width);
void imu_set_D4_pulse_width(uint16_t width);












void imu_reset(void);
void imu_keep_setting(void);
void imu_set_calibration(enum imu_cali_mode mode);
void imu_set_install_dir(enum imu_install_dir dir);
void imu_wake_reverse(void);
void imu_algorithm_change(enum imu_algo_set algo);
void imu_gyro_auto_calibration(enum imu_gyro_cali_set cali_mode);
void imu_set_return_content(uint16_t content_bit);
void imu_set_rate(enum imu_return_rate_set rate);
void imu_set_baudrate(enum imu_baudrate_set baudrate);
void imu_set_x_acce_offset(uint16_t offset);
void imu_set_y_acce_offset(uint16_t offset);
void imu_set_z_acce_offset(uint16_t offset);
void imu_set_x_gyro_offset(uint16_t offset);
void imu_set_y_gyro_offset(uint16_t offset);
void imu_set_z_gyro_offset(uint16_t offset);
void imu_set_x_mag_offset(uint16_t offset);
void imu_set_y_mag_offset(uint16_t offset);
void imu_set_z_mag_offset(uint16_t offset);
void imu_set_dev_addr(uint8_t addr);
void imu_set_led_on_off(enum imu_led_onoff onoff);
//feature function by USART

imu_res imu_check(IMU_DATA_GET imu_get_data);
void imu_get_data(IMU_DATA imu_data,uint8_t pack_type,uint8_t seq);
void imu_task(void);
//objective function
/********dividing line*********/
#endif



