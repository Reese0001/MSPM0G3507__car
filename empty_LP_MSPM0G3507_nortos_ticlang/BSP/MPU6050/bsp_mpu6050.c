#include "bsp_mpu6050.h"
#include "stdio.h"

/******************************************************************
 * 函数名称：I2C_Start
 * 功能说明：IIC启动时序
 * 输入参数：无
 * 返回值：无
 * 备注：无
 * Function name: I2C_Start
 * Function description: IIC start timing
 * Function parameter: None
 * Function return: None
 * Remarks: None
******************************************************************/
void I2C_Start(void)
{
	SDA_OUT();
	SDA(1);
	SCL(1);
	delay_us(1);
	SDA(0);
	delay_us(1);
	SCL(0);
}
/******************************************************************
 * 函数名称：I2C_Stop
 * 功能说明：IIC停止信号
 * 输入参数：无
 * 返回值：无
 * 备注：无
 * Function name: I2C_Stop
 * Function description: IIC stop signal
 * Function parameters: None
 * Function return: None
 * Notes: None
******************************************************************/
void I2C_Stop(void)
{
        SDA_OUT();
        SCL(0);
        SDA(0);

        SCL(1);
        delay_us(5);
        SDA(1);
        delay_us(5);

}

/******************************************************************
 * 函数名称：I2C_Send_Ack
 * 功能说明：主机发送应答或非应答信号
 * 输入参数：0发送应答  1发送非应答
 * 返回值：无
 * 备注：无
 * Function name: I2C_Send_Ack
 * Function description: The host sends a response or non-response signal
 * Function parameter: 0 sends a response 1 sends a non-response
 * Function return: None
 * Notes: None
******************************************************************/
void I2C_Send_Ack(unsigned char ack)
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        delay_us(5);
        if(!ack) SDA(0);
        else         SDA(1);
        SCL(1);
        delay_us(5);
        SCL(0);
        SDA(1);
}

/******************************************************************
 * 函数名称：I2C_WaitAck
 * 功能说明：等待从机应答
 * 输入参数：无
 * 返回值：0应答  1超时无应答
 * 备注：无
 * Function name: I2C_WaitAck
 * Function description: Wait for slave response
 * Function parameter: None
 * Function return: 0 for response 1 for timeout and no response
 * Note: None
******************************************************************/
unsigned char I2C_WaitAck(void)
{

        char ack = 0;
        unsigned char ack_flag = 10;
        SCL(0);
        SDA(1);
        SDA_IN();

        SCL(1);
        while( (SDA_GET()==1) && ( ack_flag ) )
        {
                ack_flag--;
                delay_us(5);
        }

        if( ack_flag <= 0 )
        {
                I2C_Stop();
                return 1;
        }
        else
        {
                SCL(0);
                SDA_OUT();
        }
        return ack;
}

/******************************************************************
 * 函数名称：Send_Byte
 * 功能说明：写入一个字节
 * 输入参数：dat要写入的数据
 * 返回值：无
 * 备注：无
 * Function name: Send_Byte
 * Function description: Write a byte
 * Function parameter: dat data to be written
 * Function return: None
 * Notes: None
******************************************************************/
void Send_Byte(uint8_t dat)
{
        int i = 0;
        SDA_OUT();
        SCL(0);//拉低时钟开始数据传输 Pull the clock low to start data transmission

        for( i = 0; i < 8; i++ )
        {
                SDA( (dat & 0x80) >> 7 );
                delay_us(1);
                SCL(1);
                delay_us(5);
                SCL(0);
                delay_us(5);
                dat<<=1;
        }
}

/******************************************************************
 * 函数名称：Read_Byte
 * 功能说明：IIC读时序
 * 输入参数：无
 * 返回值：读取到的数据
 * 备注：无
 * Function name: Read_Byte
 * Function description: IIC read timing
 * Function parameters: None
 * Function returns: Read data
 * Notes: None
******************************************************************/
unsigned char Read_Byte(void)
{
        unsigned char i,receive=0;
        SDA_IN();//SDA设置为输入 SDA is set as input
    for(i=0;i<8;i++ )
        {
        SCL(0);
        delay_us(5);
        SCL(1);
        delay_us(5);
        receive<<=1;
        if( SDA_GET() )
        {
            receive|=1;
        }
        delay_us(5);
    }
        SCL(0);
  return receive;
}

/******************************************************************
 * 函数名称：MPU6050_WriteReg
 * 功能说明：IIC连续写入数据
 * 输入参数：addr设备地址 regaddr寄存器地址 num要写的长度 regdata写入数据的地址
 * 返回值：0=读取成功   其他=读取失败
 * 备注：无
 * Function name: MPU6050_WriteReg
 * Function description: IIC writes data continuously
 * Function parameters: addr device address regaddr register address num length to be written regdata data address to be written
 * Function return: 0 = read successfully Other = read failed
 * Notes: None
******************************************************************/
char MPU6050_WriteReg(uint8_t addr,uint8_t regaddr,uint8_t num,uint8_t *regdata)
{
    uint16_t i = 0;
        I2C_Start();
        Send_Byte((addr<<1)|0);
        if( I2C_WaitAck() == 1 ) {I2C_Stop();return 1;}
        Send_Byte(regaddr);
        if( I2C_WaitAck() == 1 ) {I2C_Stop();return 2;}

        for(i=0;i<num;i++)
    {
        Send_Byte(regdata[i]);
        if( I2C_WaitAck() == 1 ) {I2C_Stop();return (3+i);}
    }
        I2C_Stop();
    return 0;
}


/******************************************************************
 * 函数名称：MPU6050_ReadData
 * 功能说明：IIC连续读取数据
 * 输入参数：addr设备地址 regaddr寄存器地址 num要读取的长度 Read读取到的数据要存储的地址
 * 返回值：0=读取成功   其他=读取失败
 * 备注：无
 * Function name: MPU6050_ReadData
 * Function description: IIC reads data continuously
 * Function parameters: addr device address regaddr register address num length to read Read address where the read data is to be stored
 * Function return: 0 = read successfully Other = read failed
 * Notes: None
******************************************************************/
char MPU6050_ReadData(uint8_t addr, uint8_t regaddr,uint8_t num,uint8_t* Read)
{
        uint8_t i;
        I2C_Start();
        Send_Byte((addr<<1)|0);
        if( I2C_WaitAck() == 1 ) {I2C_Stop();return 1;}
        Send_Byte(regaddr);
        if( I2C_WaitAck() == 1 ) {I2C_Stop();return 2;}

        I2C_Start();
        Send_Byte((addr<<1)|1);
        if( I2C_WaitAck() == 1 ) {I2C_Stop();return 3;}

        for(i=0;i<(num-1);i++){
                Read[i]=Read_Byte();
                I2C_Send_Ack(0);
        }
        Read[i]=Read_Byte();
        I2C_Send_Ack(1);
        I2C_Stop();
        return 0;
}


/******************************************************************
 * 函数名称：MPU_Set_Gyro_Fsr
 * 功能说明：设置MPU6050陀螺仪传感器的量程范围
 * 输入参数：fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
 * 返回值：0,设置成功  其他,设置失败
 * 备注：无
 * Function name: MPU_Set_Gyro_Fsr
 * Function description: Set the full scale range of the MPU6050 gyroscope sensor
 * Function parameters: fsr: 0, ±250dps; 1, ±500dps; 2, ±1000dps; 3, ±2000dps
 * Function return: 0, setting successful Others, setting failed
 * Notes: None
******************************************************************/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
        return MPU6050_WriteReg(0x68,MPU_GYRO_CFG_REG,1,(uint8_t*)(fsr<<3)); //设置陀螺仪的量程范围 Set the gyroscope full-scale range
}

/******************************************************************
 * 函数名称：MPU_Set_Accel_Fsr
 * 功能说明：设置MPU6050加速度传感器的量程范围
 * 输入参数：fsr:0,±2g;1,±4g;2,±8g;3,±16g
 * 返回值：0,设置成功  其他,设置失败
 * 备注：无
 * Function name: MPU_Set_Accel_Fsr
 * Function description: Set the full scale range of the MPU6050 acceleration sensor
 * Function parameters: fsr: 0, ±2g; 1, ±4g; 2, ±8g; 3, ±16g
 * Function return: 0, setting successful Others, setting failed
 * Notes: None
******************************************************************/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
        return MPU6050_WriteReg(0x68,MPU_ACCEL_CFG_REG,1,(uint8_t*)(fsr<<3)); //设置加速度传感器的量程范围   Set the full-scale range of the accelerometer
}

/******************************************************************
 * 函数名称：MPU_Set_LPF
 * 功能说明：设置MPU6050的数字低通滤波器
 * 输入参数：lpf:数字低通滤波频率(Hz)
 * 返回值：0,设置成功  其他,设置失败
 * 备注：无
 * Function name: MPU_Set_LPF
 * Function description: Set the digital low-pass filter of MPU6050
 * Function parameter: lpf: digital low-pass filter frequency (Hz)
 * Function return: 0, setting successful Others, setting failed
 * Note: None
******************************************************************/
uint8_t MPU_Set_LPF(uint16_t lpf)
{
        uint8_t data=0;

        if(lpf>=188)data=1;
        else if(lpf>=98)data=2;
        else if(lpf>=42)data=3;
        else if(lpf>=20)data=4;
        else if(lpf>=10)data=5;
        else data=6;
    return data=MPU6050_WriteReg(0x68,MPU_CFG_REG,1,&data);//设置数字低通滤波器   Setting the digital low-pass filter
}
/******************************************************************
 * 函数名称：MPU_Set_Rate
 * 功能说明：设置MPU6050的采样率(假定Fs=1KHz)
 * 输入参数：rate:4~1000(Hz)  初始化时rate取50
 * 返回值：0,设置成功  其他,设置失败
 * 备注：无
 * Function name: MPU_Set_Rate
 * Function description: Set the sampling rate of MPU6050 (assuming Fs=1KHz)
 * Function parameter: rate: 4~1000 (Hz) Initialization rate is 50
 * Function return: 0, setting successful Others, setting failed
 * Note: None
******************************************************************/
uint8_t MPU_Set_Rate(uint16_t rate)
{
        uint8_t data;
        if(rate>1000)rate=1000;
        if(rate<4)rate=4;
        data=1000/rate-1;
        data=MPU6050_WriteReg(0x68,MPU_SAMPLE_RATE_REG,1,&data);        //设置数字低通滤波器 Setting the digital low-pass filter
         return MPU_Set_LPF(rate/2);            //自动设置LPF为采样率的一半 Automatically set LPF to half the sampling rate
}


/******************************************************************
 * 函数名称：MPU6050ReadGyro
 * 功能说明：读取陀螺仪数据
 * 输入参数：陀螺仪数据存储地址
 * 返回值：无
 * 备注：无
 * Function name: MPU6050ReadGyro
 * Function description: Read gyroscope data
 * Function parameter: Gyroscope data storage address
 * Function return: None
 * Notes: None
******************************************************************/
void MPU6050ReadGyro(short *gyroData)
{
        uint8_t buf[6];
        uint8_t reg = 0;
        //MPU6050_GYRO_OUT = MPU6050陀螺仪数据寄存器地址
        //陀螺仪数据输出寄存器总共由6个寄存器组成，
        //输出X/Y/Z三个轴的陀螺仪传感器数据，高字节在前，低字节在后
        //每一个轴16位，顺序为xyz
		//MPU6050_GYRO_OUT = MPU6050 gyroscope data register address
		//The gyroscope data output register consists of 6 registers in total,
		//Output the gyroscope sensor data of the three axes X/Y/Z, with the high byte in front and the low byte in the back.
		//Each axis is 16 bits, in order of xyz
        reg = MPU6050_ReadData(0x68,MPU6050_GYRO_OUT,6,buf);
        if( reg == 0 )
        {
                gyroData[0] = (buf[0] << 8) | buf[1];
                gyroData[1] = (buf[2] << 8) | buf[3];
                gyroData[2] = (buf[4] << 8) | buf[5];
        }
}

/******************************************************************
 * 函数名称：MPU6050ReadAcc
 * 功能说明：读取加速度数据
 * 输入参数：加速度数据存储地址
 * 返回值：无
 * 备注：无
 * Function name: MPU6050ReadAcc
 * Function description: Read acceleration data
 * Function parameter: Acceleration data storage address
 * Function return: None
 * Notes: None
******************************************************************/
void MPU6050ReadAcc(short *accData)
{
        uint8_t buf[6];
        uint8_t reg = 0;
        //MPU6050_ACC_OUT = MPU6050加速度数据寄存器地址
        //加速度传感器数据输出寄存器总共由6个寄存器组成，
        //输出X/Y/Z三个轴的加速度传感器数值，高字节在前，低字节在后
		//MPU6050_ACC_OUT = MPU6050 acceleration data register address
		//The acceleration sensor data output register consists of 6 registers in total,
		//Output the acceleration sensor values of the three axes X/Y/Z, with the high byte in front and the low byte in the back.
        reg = MPU6050_ReadData(0x68, MPU6050_ACC_OUT, 6, buf);
        if( reg == 0)
        {
                accData[0] = (buf[0] << 8) | buf[1];
                accData[1] = (buf[2] << 8) | buf[3];
                accData[2] = (buf[4] << 8) | buf[5];
        }
}

/******************************************************************
 * 函数名称：MPU6050_GetTemp
 * 功能说明：读取MPU6050上的温度
 * 输入参数：无
 * 返回值：温度值，单位为℃
 * 备注：温度转换公式为：Temperature = 36.53 + regval/340
 * Function name: MPU6050_GetTemp
 * Function description: Read the temperature on MPU6050
 * Function parameters: None
 * Function return: Temperature value in ℃
 * Note: Temperature conversion formula: Temperature = 36.53 + regval/340
******************************************************************/
float MPU6050_GetTemp(void)
{
        short temp3;
        uint8_t buf[2];
        float Temperature = 0;
        MPU6050_ReadData(0x68,MPU6050_RA_TEMP_OUT_H,2,buf);
    temp3= (buf[0] << 8) | buf[1];
        Temperature=((double) temp3/340.0)+36.53;
    return Temperature;
}

/******************************************************************
 * 函数名称：MPU6050ReadID
 * 功能说明：读取MPU6050的器件地址
 * 输入参数：无
 * 返回值：0=检测不到MPU6050   1=能检测到MPU6050
 * 备注：无
 * Function name: MPU6050ReadID
 * Function description: Read the device address of MPU6050
 * Function parameter: None
 * Function return: 0 = MPU6050 cannot be detected 1 = MPU6050 can be detected
 * Note: None
******************************************************************/
uint8_t MPU6050ReadID(void)
{
        unsigned char Re[2] = {0};
        //器件ID寄存器 = 0x75 Device ID Register = 0x75
        printf("mpu=%d\r\n",MPU6050_ReadData(0x68,0X75,1,Re)); //读取器件地址 Read device address

        if (Re[0] != 0x68)
        {
                        printf("Not Found MPU6050 Model");
                        return 1;
         }
        else
        {
                        printf("MPU6050 ID = %x\r\n",Re[0]);
                        return 0;
        }
        return 0;
}

/******************************************************************
 * 函数名称：MPU6050_Init
 * 功能说明：MPU6050初始化
 * 输入参数：无
 * 返回值：0成功  1没检测到MPU6050
 * 备注：无
 * Function name: MPU6050_Init
 * Function description: MPU6050 initialization
 * Function parameters: None
 * Function return: 0 success 1 MPU6050 not detected
 * Notes: None
******************************************************************/
char MPU6050_Init(void)
{
        SDA_OUT();
    delay_ms(10);
    //复位6050 Reset 6050
    MPU6050_WriteReg(0x68,MPU6050_RA_PWR_MGMT_1, 1,(uint8_t*)(0x80));
    delay_ms(100);
    //电源管理寄存器 Power Management Registers
    //选择X轴陀螺仪作为参考PLL的时钟源，设置CLKSEL=001
		//Select the X-axis gyro as the clock source for the reference PLL, set CLKSEL=001
    MPU6050_WriteReg(0x68,MPU6050_RA_PWR_MGMT_1,1, (uint8_t*)(0x00));

    MPU_Set_Gyro_Fsr(3);    //陀螺仪传感器,±2000dps Gyroscope sensor, ±2000dps
    MPU_Set_Accel_Fsr(0);   //加速度传感器,±2g Accelerometer, ±2g
    MPU_Set_Rate(50);

    MPU6050_WriteReg(0x68,MPU_INT_EN_REG , 1,(uint8_t*)0x00);        //关闭所有中断 Disable all interrupts
    MPU6050_WriteReg(0x68,MPU_USER_CTRL_REG,1,(uint8_t*)0x00);        //I2C主模式关闭 I2C Master Mode Off
    MPU6050_WriteReg(0x68,MPU_FIFO_EN_REG,1,(uint8_t*)0x00);                //关闭FIFO Close FIFO
    MPU6050_WriteReg(0x68,MPU_INTBP_CFG_REG,1,(uint8_t*)0X80);        //INT引脚低电平有效 INT pin low level is effective

    if( MPU6050ReadID() == 0 )//检查是否有6050 Check if there is 6050
    {
            MPU6050_WriteReg(0x68,MPU6050_RA_PWR_MGMT_1, 1,(uint8_t*)0x01);//设置CLKSEL,PLL X轴为参考 Set CLKSEL, PLL X axis as reference
            MPU6050_WriteReg(0x68,MPU_PWR_MGMT2_REG, 1,(uint8_t*)0x00);//加速度与陀螺仪都工作 Both the accelerometer and the gyroscope work
            MPU_Set_Rate(50);
            return 1;
    }
    return 0;
}

