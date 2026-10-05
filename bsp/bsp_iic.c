/**
 * [H7-011] 迁移: 软件I2C驱动 (ALIENTEK风格位带操作)
 *   - 修复: GPIOF→GPIOH, PIN0/1→PIN4/5 (与bsp_iic.h中PH4/5一致)
 */
#include "bsp_iic.h"
#include "bsp_delay.h"
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOH_CLK_ENABLE();
    
    GPIO_Initure.Pin=GPIO_PIN_4|GPIO_PIN_5;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull=GPIO_PULLUP;
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
    
    IIC_SDA_H();
    IIC_SCL_H();  
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA_H();	  	  
	IIC_SCL_H();
	delay_us(4);
 	IIC_SDA_L();//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_L();//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL_L();
	IIC_SDA_L();//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_H(); 
	delay_us(4);			
	IIC_SDA_H();//发送I2C总线结束信号				   	
}
//等待应答信号到来
//返回值: 1-接收应答失败
//        0-接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA_H();delay_us(1);	   
	IIC_SCL_H();delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L();//时钟线拉低	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL_L();
	SDA_OUT();
	IIC_SDA_L();
	delay_us(2);
	IIC_SCL_H();
	delay_us(2);
	IIC_SCL_L();
}
//产生NACK应答		    
void IIC_NAck(void)
{
	IIC_SCL_L();
	SDA_OUT();
	IIC_SDA_H();
	delay_us(2);
	IIC_SCL_H();
	delay_us(2);
	IIC_SCL_L();
}					 				     
//IIC发送一个字节
//返回从机是否有应答
//1-有应答
//0-无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL_L();//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        if(txd & 0x80) IIC_SDA_H(); else IIC_SDA_L();
        txd<<=1; 	  
		delay_us(2);   //对TEA5767来说，这个延时是必须的
		IIC_SCL_H();
		delay_us(2); 
		IIC_SCL_L();	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时发送ACK，ack=0时发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_L(); 
        delay_us(2);
		IIC_SCL_H();
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

