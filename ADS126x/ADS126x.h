/**
 * @file : ADS126x.h
 * @date : May.24th.2025
 * @author : Scaxlibur
 * @brief : ADS1262 / ADS1263 的STM32HAL库驱动
 */

#ifndef ADS126X_H_
#define ADS126X_H_


#include "main.h"
#include "myspi.h"
//#include "mydelay.h"

/*********************************引脚声明*********************************************/
#define ads1263_cs_0      HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define ads1263_cs_1      HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)
#define ads1263_rst_0     HAL_GPIO_WritePin(ads1263_rst_GPIO_Port, ads1263_rst_Pin, GPIO_PIN_RESET)
#define ads1263_rst_1     HAL_GPIO_WritePin(ads1263_rst_GPIO_Port, ads1263_rst_Pin, GPIO_PIN_SET)
#define ads1263_start_0   HAL_GPIO_WritePin(ads1263_start_GPIO_Port, ads1263_start_Pin, GPIO_PIN_RESET)
#define ads1263_start_1   HAL_GPIO_WritePin(ads1263_start_GPIO_Port, ads1263_start_Pin, GPIO_PIN_SET)

#define ads1263_drdy      HAL_GPIO_ReadPin(ads1263_drdy_GPIO_Port, ads1263_drdy_Pin)


/******************************寄存器地址列表******************************************/
#define ADS1263_REG_ID       		0x00
#define ADS1263_REG_POWER       	0x01
#define ADS1263_REG_INTERFACE       0x02
#define ADS1263_REG_MODE0       	0x03
#define ADS1263_REG_MODE1       	0x04
#define ADS1263_REG_MODE2       	0x05
#define ADS1263_REG_INPMUX       	0x06
#define ADS1263_REG_OFCAL0       	0x07
#define ADS1263_REG_OFCAL1       	0x08
#define ADS1263_REG_OFCAL2       	0x09
#define ADS1263_REG_FSCAL0       	0x0A
#define ADS1263_REG_FSCAL1       	0x0B
#define ADS1263_REG_FSCAL2      	0x0C
#define ADS1263_REG_IDACMUX       	0x0D
#define ADS1263_REG_IDACMAG       	0x0E
#define ADS1263_REG_REFMUX       	0x0F
#define ADS1263_REG_TDACP       	0x10
#define ADS1263_REG_TDACN       	0x11
#define ADS1263_REG_GPIOCON       	0x12
#define ADS1263_REG_GPIODIR       	0x13
#define ADS1263_REG_GPIODAT       	0x14
#define ADS1263_REG_ADC2CFG       	0x15
#define ADS1263_REG_ADC2MUX       	0x16
#define ADS1263_REG_ADC2OFC0       	0x17
#define ADS1263_REG_ADC2OFC1       	0x18
#define ADS1263_REG_ADC2FSC0       	0x19
#define ADS1263_REG_ADC2FSC1       	0x1A

/******************************寄存器数据列表******************************************/
#define ADS1263_PGA_ENABLE 			0x00
#define ADS1263_PGA_BYPASS 			0x80

#define ADS1263_PGA_GAIN_1      	0x00
#define ADS1263_PGA_GAIN_2      	0x10
#define ADS1263_PGA_GAIN_4      	0x20
#define ADS1263_PGA_GAIN_8      	0x30
#define ADS1263_PGA_GAIN_16     	0x40
#define ADS1263_PGA_GAIN_32     	0x50

#define ADS1263_DATA_RATE_2_5_SPS   0x00
#define ADS1263_DATA_RATE_5_SPS     0x01
#define ADS1263_DATA_RATE_10_SPS    0x02
#define ADS1263_DATA_RATE_16_6_SPS  0x03
#define ADS1263_DATA_RATE_20_SPS    0x04
#define ADS1263_DATA_RATE_50_SPS    0x05
#define ADS1263_DATA_RATE_60_SPS    0x06
#define ADS1263_DATA_RATE_100_SPS   0x07
#define ADS1263_DATA_RATE_400_SPS   0x08
#define ADS1263_DATA_RATE_1200_SPS  0x09
#define ADS1263_DATA_RATE_2400_SPS  0x0A
#define ADS1263_DATA_RATE_4800_SPS  0x0B
#define ADS1263_DATA_RATE_7200_SPS  0x0C
#define ADS1263_DATA_RATE_14400_SPS 0x0D
#define ADS1263_DATA_RATE_19200_SPS 0x0E
#define ADS1263_DATA_RATE_38400_SPS 0x0F
 
#define ADS1263_PGA_GAIN_ADC2_1    	0x00
#define ADS1263_PGA_GAIN_ADC2_2    	0x01
#define ADS1263_PGA_GAIN_ADC2_4    	0x02
#define ADS1263_PGA_GAIN_ADC2_8    	0x03
#define ADS1263_PGA_GAIN_ADC2_16    0x04
#define ADS1263_PGA_GAIN_ADC2_32    0x05
#define ADS1263_PGA_GAIN_ADC2_64    0x06
#define ADS1263_PGA_GAIN_ADC2_128   0x07
 
#define ADS1263_DATA_RATE_ADC2_10_SPS     0x00
#define ADS1263_DATA_RATE_ADC2_100_SPS     0x40
#define ADS1263_DATA_RATE_ADC2_400_SPS     0x80
#define ADS1263_DATA_RATE_ADC2_800_SPS     0xC0

#define ADS1263_INTERNAL_REF     0x00
#define ADS1263_INTERNAL_AIN0_1     0x08
#define ADS1263_INTERNAL_AIN2_3     0x10
#define ADS1263_INTERNAL_AIN4_5     0x18
#define ADS1263_INTERNAL_VDD_VSS    0x20

/* 滤波类型 */
#define ADS1263_SINC1_MODE 0x00
#define ADS1263_SINC2_MODE 0x20
#define ADS1263_SINC3_MODE 0x40
#define ADS1263_SINC4_MODE 0x60
#define ADS1263_FIR_MODE   0x80
 
#define ADS1263_Chop_Mode_Enable 0x10
#define ADS1263_CONTINUOUS_CONVERSION 0x00
#define ADS1263_PULSE_CONVERSION 0x40
 
#define ADS1263_STAUS_BYTE_DISABLE 0x00
#define ADS1263_STAUS_BYTE_ENABLE 0x04
#define ADS1263_CHECK_SUM_BYTE_DISABLE 0x00
 
#define ADS1263_INTERNAL_2_5_REF_P 0x00
#define ADS1263_ENTERNAL_AIN0_P      0x08
#define ADS1263_ENTERNAL_AIN2_P     0x10
#define ADS1263_ENTERNAL_AIN4_P      0x18
#define ADS1263_INTERNAL_ANALOG_SUPPLY_P 0x20
#define ADS1263_INTERNAL_2_5_REF_N 0x00
#define ADS1263_ENTERNAL_AIN1_N     0x01
#define ADS1263_ENTERNAL_AIN3_N     0x02
#define ADS1263_ENTERNAL_AIN5_N      0x03
#define ADS1263_INTERNAL_ANALOG_SUPPLY_N 0x04
 
/* 通道选择 */
#define ADS1263_MUXP_AIN0   0x00
#define ADS1263_MUXP_AIN1   0x10
#define ADS1263_MUXP_AIN2   0x20
#define ADS1263_MUXP_AIN3   0x30
#define ADS1263_MUXP_AIN4   0x40
#define ADS1263_MUXP_AIN5   0x50
#define ADS1263_MUXP_AIN6   0x60
#define ADS1263_MUXP_AIN7   0x70
#define ADS1263_MUXP_AIN8   0x80
#define ADS1263_MUXP_AIN9   0x90
#define ADS1263_MUXP_AINCOM 0xA0
#define ADS1263_TEMPERATURE_SENSOR_MONITOR_P 0xB0
#define ADS1263_ANALOG_POWER_SUPPLY_MONITOR_P 0xC0
#define ADS1263_DIGITAL_POWER_SUPPLY_MONITOR_P 0xD0
#define ADS1263_TDAC_TEST_SIGNAL_P     0xE0
#define ADS1263_FLOAT_P 0xF0
 
#define ADS1263_MUXN_AIN0   0x00
#define ADS1263_MUXN_AIN1   0x01
#define ADS1263_MUXN_AIN2   0x02
#define ADS1263_MUXN_AIN3   0x03
#define ADS1263_MUXN_AIN4   0x04
#define ADS1263_MUXN_AIN5   0x05
#define ADS1263_MUXN_AIN6   0x06
#define ADS1263_MUXN_AIN7   0x07
#define ADS1263_MUXN_AIN8   0x08
#define ADS1263_MUXN_AIN9   0x09
#define ADS1263_MUXN_AINCOM 0x0A
#define ADS1263_TEMPERATURE_SENSOR_MONITOR_N 0x0B
#define ADS1263_ANALOG_POWER_SUPPLY_MONITOR_N 0x0C
#define ADS1263_DIGITAL_POWER_SUPPLY_MONITOR_N 0x0D
#define ADS1263_TDAC_TEST_SIGNAL_N     0x0E
#define ADS1263_FLOAT_N 0x0F
 
/******************************指令******************************************/
#define ADS1263_CMD_NOP         0x00
#define ADS1263_CMD_RESET       0x06
#define ADS1263_CMD_RDATA_ADC1  0x12
#define ADS1263_CMD_RDATA_ADC2  0x14
#define ADS1263_CMD_START_ADC1  0x08
#define ADS1263_CMD_START_ADC2  0x0C
#define ADS1263_CMD_STOP_ADC1   0x0A
#define ADS1263_CMD_STOP_ADC2   0x0E
#define ADS1263_CMD_SYOCAL_ADC1 0x16
#define ADS1263_CMD_SYGCAL_ADC1 0x17
#define ADS1263_CMD_SFOCAL_ADC1 0x19
#define ADS1263_CMD_SYOCAL_ADC2 0x1B
#define ADS1263_CMD_SYGCAL_ADC2 0x1C
#define ADS1263_CMD_SFOCAL_ADC2 0x1E
 
//量化单位//（请校准自己的电压基准）
#define MIN_Unit_ADC1     2.4985/2147483648//先使用别的校准好的高精度万用表测量ADS1263的基准，再把值填入这里

/*******************************外部函数声明*******************************************/
extern void ads1263_init(void);
extern void ads1263_write_reg(uint8_t addr, uint8_t data);
extern uint8_t ads1263_read_reg(uint8_t addr);
extern  void ads1263_sel_adc1_ch(uint8_t ch);
extern void ads1263_reset(void);
extern void ads1263_conveision_start_adc1(void);
extern void ads1263_conveision_stop_adc1(void);
extern uint32_t ads1263_read_data_adc1(void);
double single_read_ADC_voltage(uint8_t channel);
double differential_read_ADC_voltage(uint8_t channel);

#endif
