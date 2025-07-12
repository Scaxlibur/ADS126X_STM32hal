#include "ADS126x.h"
#include "usart.h"
#include "stdio.h"

void Delay_us(uint32_t delay);

/**
 * @brief       ads1263引脚初始化
 * @param       无
 * @retval      无
 */

void ads1263_init(void)
{
	MySPI_Init();
	
	ads1263_reset();
	ads1263_start_0;			/* 停止ADC转换，避免寄存器配置出错 */
	ads1263_cs_1;
	HAL_Delay(10);

	ads1263_cs_0;
	
	Delay_us(10);
	
//	ads1263_write_reg(ADS1263_REG_MODE0,ADS1263_Chop_Mode_Enable);/* 开启输入切割模式（牺牲转换时间来增加信噪比），进入连续转换模式 */
	// 使用SINC1滤波器,不同滤波器下的采样噪声会不同 
	ads1263_write_reg(ADS1263_REG_MODE1,ADS1263_SINC1_MODE);
	/* PGA被旁路（否则转换数据值会偏小），数据速率2.5SPS */
	ads1263_write_reg(ADS1263_REG_MODE2,ADS1263_PGA_BYPASS|ADS1263_DATA_RATE_14400_SPS);
	/* 打开状态字节，关闭冗余校验 */
	ads1263_write_reg(ADS1263_REG_INTERFACE,ADS1263_STAUS_BYTE_ENABLE|ADS1263_CHECK_SUM_BYTE_DISABLE);
	//ADC1配置
	ads1263_write_reg(ADS1263_REG_REFMUX,ADS1263_INTERNAL_2_5_REF_P|ADS1263_INTERNAL_2_5_REF_N);//使用内部基准
	ads1263_write_reg(ADS1263_REG_INPMUX,ADS1263_MUXP_AIN0|ADS1263_MUXN_AINCOM);//AIN0正极输入，AINCOM负极输入
	
	Delay_us(10);
	
	ads1263_cs_1;
}

/**
 * @brief       ads1263写寄存器
 * @param       地址为001r rrrr 其中r rrrr为开始的寄存器地址
								数量为000n nnnn 其中n nnnn为读取寄存器数量减一
 * @retval      无
 */
void ads1263_write_reg(uint8_t addr, uint8_t data)
{
	uint8_t reg = 0x40 | (addr & 0x1F);	
	MySPI_SwapByte(reg);
	MySPI_SwapByte(0x00);
	MySPI_SwapByte(data);
	
}

/**
 * @brief       ads1263读寄存器
* @param       	地址为001r rrrr 其中r rrrr为开始的寄存器地址
								数量为000n nnnn 其中n nnnn为读取寄存器数量减一
 * @retval      无
 */
uint8_t ads1263_read_reg(uint8_t addr)
{
	uint8_t rxdata;
	uint8_t reg = 0x20 | (addr & 0x1F);
	
	MySPI_SwapByte(reg);
	MySPI_SwapByte(0x00);//返回一个寄存器的数据
	rxdata = MySPI_SwapByte(0x00);//返回数据
	
	return rxdata;
}

/**
 * @brief       ads1263选择转换通道
 * @param       无
 * @retval      无
 */
void ads1263_sel_adc1_ch(uint8_t ch)
{
	ads1263_cs_0;
	
	Delay_us(1);
	
	ads1263_write_reg(ADS1263_REG_INPMUX,ch|ADS1263_MUXN_AINCOM);
	ads1263_cs_1;
}

/**
 * @brief       ads1263 differential选择转换通道
 * @param       无
 * @retval      无
 */
void ads1263_sel_adc1_diff_ch(uint8_t ch)
{
	ads1263_cs_0;
	Delay_us(1);
	ads1263_write_reg(ADS1263_REG_INPMUX,ch);
	ads1263_cs_1;
}

/**
 * @brief       ads1263复位
 * @param       无
 * @retval      无
 */
void ads1263_reset(void)
{

	HAL_Delay(100);
  ads1263_rst_0;
  HAL_Delay(100);
  ads1263_rst_1;
  HAL_Delay(100);
}

/**
 * @brief       ads1263启动ADC1转换
 * @param       无
 * @retval      无
 */
void ads1263_conveision_start_adc1(void)
{
	ads1263_cs_0;
	
	Delay_us(1);
	
	MySPI_SwapByte(ADS1263_CMD_START_ADC1);
	
	ads1263_cs_1;
}

/**
 * @brief       ads1263停止ADC1转换
 * @param       无
 * @retval      无
 */
void ads1263_conveision_stop_adc1(void)
{
	ads1263_cs_0;
	Delay_us(1);
		
	MySPI_SwapByte(ADS1263_CMD_STOP_ADC1);
	
	ads1263_cs_1;
}

/**
 * @brief       ads1263读ADC1数据
 * @param       无
 * @retval      无
 */
uint32_t ads1263_read_data_adc1(void)
{
  uint8_t SPI_Receive_Data[6]={0, 0, 0, 0, 0, 0};;
	uint32_t rxdata;
	uint8_t state = 0;
	while(1)
	{
		ads1263_cs_0;
		Delay_us(1);
		
		MySPI_SwapByte(ADS1263_CMD_RDATA_ADC1);
		SPI_Receive_Data[1] = MySPI_SwapByte(0xFF);
		state = SPI_Receive_Data[1]&0x40;
		if(state>>6==1)//确保ADC状态正确
		{
			
			SPI_Receive_Data[2]=MySPI_SwapByte(0xFF);/* 发送ADC1读取转换数据命令 */
			SPI_Receive_Data[3]=MySPI_SwapByte(0xFF);/* 发送ADC1读取转换数据命令 */
			SPI_Receive_Data[4]=MySPI_SwapByte(0xFF);/* 发送ADC1读取转换数据命令 */
			SPI_Receive_Data[5]=MySPI_SwapByte(0xFF);/* 发送ADC1读取转换数据命令 */

			/* 读取到的6字节数据的低4位拼凑成转换数据 */
			rxdata=((uint32_t)SPI_Receive_Data[2]<<24)|((uint32_t)SPI_Receive_Data[3]<<16)|((uint32_t)SPI_Receive_Data[4]<<8)|((uint32_t)SPI_Receive_Data[5]<<0);
			return rxdata;
			Delay_us(1);
		}
		ads1263_cs_1;
	}
}

double single_read_ADC_voltage(uint8_t channel)
{
    int32_t AdcReg;
    uint32_t AdcValue;
    double AdcVoltage;

    ads1263_sel_adc1_ch(channel << 4);         /* 选择通道进行转换 */

    ads1263_conveision_start_adc1();           /* ADC1开始转换 */
	
    while (!ads1263_drdy);                  /* 等待转换完成 */
    
    ads1263_conveision_stop_adc1();            /* 停止ADC转换 */
    AdcValue = ads1263_read_data_adc1();       /* 读取ADC数据 */
    AdcReg = AdcValue;
    /* 根据ADS1263的参考电压进行修改 */
    AdcVoltage = (double)AdcReg / 0x7fffffff * 2497.5; 

    return AdcVoltage;                         /* 返回计算得到的电压值 */
}

double differential_read_ADC_voltage(uint8_t channel)
{
    int32_t AdcReg;
    uint32_t AdcValue;
    double AdcVoltage;

		ads1263_sel_adc1_diff_ch((channel*2<<4)|(channel*2+1));			/* 选择通道转换 */
		ads1263_conveision_start_adc1(); 	/* ADC1开始转换 */
		while(ads1263_drdy);
		ads1263_conveision_stop_adc1();		/* ADC1停止转换 */
		AdcValue = ads1263_read_data_adc1();
		AdcReg = AdcValue;
		AdcVoltage = (double)AdcReg/0x7fffffff*2500;  /* 返回计算得到的电压值 */
			
		return AdcVoltage;                         
}