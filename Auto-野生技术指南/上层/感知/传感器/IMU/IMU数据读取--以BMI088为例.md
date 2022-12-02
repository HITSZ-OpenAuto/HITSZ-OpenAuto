# IMU数据读取

>   这里以博世传感器公司产出的BMI088型号的IMU为例，其里面有3轴高精度加速度计和3轴高精度陀螺仪，其他的特性不再介绍
>
>   BMI088的数据手册附在文件夹里面，可以自行阅读
>
>   同时这里的IMU是安装在大疆公司出产的RoboMaster开发板C型，单片机芯片是STM32F407IGH6，其外围电路已经设计好，只需要读取IMU数据即可。
>
>   本篇不会介绍SPI、I2C等嵌入式通信协议，需要有一定嵌入式开发基础的同学来看（可以去看底层中介绍的STM32开发视频）

## 零、数据手册分析

我们打开这款陀螺仪的手册，可以看到，手册的第一章讲述了BMI088的硬件特性，需要多少伏的电压、电流之类的，这一章只有在我们拿到IMU芯片，想把它设计装在自己的电路板上面的时候才会考虑到，其他时候这些硬件特性不是我们需要考虑的范围，也不是本篇要介绍的内容。

![image-20221202200503443](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/c2268926bc7315cc8d86c4b8802bb71c/202212022005539.png)

第二章讲述了BMI088的内部结构，抓一些重点，BMI088里面，在加速度计的部分有一个温度传感器可以读取；同时加速度计和陀螺仪均有中断口，可以高速输出数据；加速度计和陀螺仪共用一个输出总线，可以选择SPI总线或者I2C总线；陀螺仪部分有一个控制单元；

![image-20221202200510660](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/69215005e7b7990536e045001f9df61b/202212022005748.png)

第三章开始，讲述了怎样快速开始，告诉我们IMU是通过监测PS引脚的电平来决定采用SPI还是I2C通信协议，同时告诉了我们一个重要的信息，BMI088的加速度计上电的时候默认是I2C通信方式，直到它监测到了SPI的片选IO口电平上升变化，同时加速度计默认是挂起模式，即不更新加速度或者温度等数据，但内部的ID号之类的是可以读取的，所以我们初始化的时候，要先将加速度计的片选IO口电平拉起，之后将加速度计的模式从挂起模式设置为正常模式。同时给出了一种快速的初始化过程，先上电，等1ms，然后往ACC_PWR_CTRL寄存器写入4，之后等待50ms

![image-20221202200517665](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/cbd25427e402223c942115b47e4c20b6/202212022005739.png)

第四章开始，介绍了各个寄存器代表的意义和设置数值对应的结果，这一部分是读取的时候需要参考的，这里不再详细介绍

![image-20221202200525441](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/e09de0f7fed5c5014641587699def542/202212022005529.png)

第五章开始，列出了BMI088的加速度计和陀螺仪的寄存器表，这也是读取的时候需要参考的部分，这里不过多讲解

![image-20221202200534473](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/088bd18ea78b622ceaaf5c482338e2d7/202212022005572.png)

第六章开始，讲述了如何使用SPI或者I2C与BMI088进行数据通信，这一章比较重要，是我们读取的基础

![image-20221202200539952](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/16e1913668a5d678637b1d472a036a3a/202212022005027.png)

第七章是BMI088的引脚图

![image-20221202200545250](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/121335b3f33f6189d3e3805c5a7ded4c/202212022005318.png)

第八章是BMI088的设计参考图，以及需要参考的电路设计参数

![image-20221202200550477](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/c3b5e353dec03ddbeec28c55df2df602/202212022005581.png)

第九章是免责声明

![image-20221202200555116](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/d71b0f6e9f606834847ed4362d527a25/202212022005209.png)



## 一、CubeMX配置

由于这里是使用RoboMaster开发板C型（以下简称“C板”），我们需要看用户手册确定C板上面的IMU是怎么设计的

![image-20221202200601626](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/3535ba5c0966afc69ecdec97d5c610ec/202212022006709.png)

可以看到，在芯片中，BMI088的PS引脚是被接到了GND上面，也就是说，C板上面的BMI088默认使用SPI通信方式进行通信，同时使用的是C板的SPI1总线。现在我们需要确定Cube中SPI配置的几个参数，就是下面这几个（这个图是我配置好的图，可以直接抄）

![image-20221202200606100](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/57a43f5744cfb7991444f94283a93f7a/202212022006172.png)

首先是Mode，没有特别说明的话，都选择全双工SPI通信，就是图上这个Full-Duplex Master

然后是Hardware NSS Signal，这个是硬件片选的意思，我们这里采用软件写CS电平的方式，因为加速度计和陀螺仪共用一条SPI总线，我们需要通过片选来确定此时读取哪个模块，所以这个地方选择Disable

下方的Frame Format，没有特别说明都选择Motorola格式

再下方的Data Size，没有特别说明都选择8Bits

然后是First Bit，也就是大小端的问题，关于大小端是什么可以自行百度，这里不再赘述。同样查询数据手册发现

![image-20221202200612435](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/7781a1e8b0e7e4ee0f95a9449dfad1d1/202212022006500.png)

这里的值的第一位是bit7，也就是MSB端，所以可以断定我们的First Bit是MSB first

然后就到了时钟配置的预分频系数Prescaler这里，这里注意到是要跟你C板的时钟配置相关联的，放出我的C板时钟配置

![image-20221202200616657](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/d3c5e097f6cc66b4021f44af33804af9/202212022006768.png)

其中，C板（F406芯片）的SPI1总线是挂载在APB1时钟上的，也就是说这个时候SPI1的初始时钟是84MHz，我们再来看数据手册中关于SPI的时钟要求

![image-20221202200621848](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/9beb94c96c0024a4f591519deb40aace/202212022006929.png)

可以发现，BMI088的SPI时钟要求最大通信频率为10MHz，所以我们默认的Prescaler=2的是肯定不能用的，因为

`SPI时钟频率 = APB1时钟频率 / 预分频系数`

所以在APB1时钟频率为84MHz的情况下，我们需要起码预分频系数要大于8，所以我选择了Prescaler=16

下一个是CPOL和CPHA，这两个一般是同时配置的，关于这两个参数代表的含义这里也不再赘述，可以自行百度。我们继续查询数据手册：

![image-20221202200627021](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/4ee16a2694f4457a3d4a4ffa2bcb82cd/202212022006078.png)

手册上写的很清楚了，BMI088的SPI通信支持‘00’和‘11’两种模式，在Cube里面，CPOL=0和CPOL=Low是一个意思，CPHA=0和CPHA=1 edge也是一个意思，所以我们可以同时选择'CPOL=Low,CPHA=1 edge'或者‘CPOL=High,CPHA=2 edge'，效果是一样的

至于再下面的两个参数，CRC和NSS，按照默认就好了，即不需要CRC和软件片选

配置完了这些参数，还需要注意一点，就是C板上面SPI的端口有可能和Cube上面默认生成的端口不一致，我们需要额外确认一下

先观察C板的SPI端口，打开C板原理图：

![image-20221202200631582](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/ec09d7eee695b5832cef1657b5f25e06/202212022006743.png)

首先负责片选的CS口是不会被默认添加的，我们需要手动添加。观察得到，加速度计的CS口是PA4，陀螺仪的片选口是PB0，添加这两个GPIO口。同时观察BMI088数据手册得到，片选口拉低电平有效，所以把这两个GPIO口的电平选择默认高电平

其次观察SPI的部分，检查SPI1_MOSI端口是否是PA7，SPI1_CLK端口是否是PB3，SP1_MISO端口是否是PB4，正确的配置如下

![image-20221202200636739](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/eac7e6e93107b62a5507e4feeae28e39/202212022006814.png)

到此，我们的SPI部分就配置好了，用CubeMX生成代码即可

## 二、代码编写

代码编写的第一部分，先写一些比较基础的宏定义和变量类型，便于以后用

SPI端口定义：

```c
#define BMI088_SPI hspi1
#define BMI088_ACC_GPIOx GPIOA
#define BMI088_ACC_GPIOp GPIO_PIN_4
#define BMI088_GYRO_GPIOx GPIOB
#define BMI088_GYRO_GPIOp GPIO_PIN_0
```

BMI088用到的数据结构体定义

```c
typedef struct acc_raw_data_t {
    float x;
    float y;
    float z;
} acc_raw_data_t;

typedef struct gyro_raw_data_t {
    float roll;
    float pitch;
    float yaw;
} gyro_raw_data_t;

typedef struct acc_data_t {
    acc_raw_data_t acc_raw_data;
    float sensor_time;
    float temperature;
    bool enable_self_test;
} acc_data_t;

typedef struct gyro_data_t {
    gyro_raw_data_t gyro_raw_data;
    bool enable_self_test;
} gyro_data_t;

typedef enum bmi088_error_e {
    NO_ERROR = 0,
    ACC_CHIP_ID_ERR = 0x01,
    ACC_DATA_ERR = 0x02,
    GYRO_CHIP_ID_ERR = 0x04,
    GYRO_DATA_ERR = 0x08,
} bmi088_error_e;

typedef struct bmi088_data_t {
    acc_data_t acc_data;
    bmi088_error_e bmi088_error;
} bmi088_data_t;
```

BMI088的寄存器表

```C
/*-----加速度计寄存器表-----*/
#define ACC_CHIP_ID_ADDR 0x00
#define ACC_CHIP_ID_VAL 0x1E

#define ACC_ERR_REG_ADDR 0x02

#define ACC_STATUS_ADDR 0x03

#define ACC_X_LSB_ADDR 0x12
#define ACC_X_MSB_ADDR 0x13
#define ACC_Y_LSB_ADDR 0x14
#define ACC_Y_MSB_ADDR 0x15
#define ACC_Z_LSB_ADDR 0x16
#define ACC_Z_MSB_ADDR 0x17
#define ACC_XYZ_LEN 6

#define SENSORTIME_0_ADDR 0x18
#define SENSORTIME_0_UNIT (39.0625f / 1000000.0f)
#define SENSORTIME_1_ADDR 0x19
#define SENSORTIME_1_UNIT (10.0 / 1000.0f)
#define SENSORTIME_2_ADDR 0x1A
#define SENSORTIME_2_UNIT (2.56f)
#define SENSORTIME_LEN 3

#define ACC_INT_STAT_1_ADDR 0x1D

#define TEMP_MSB_ADDR 0x22
#define TEMP_LSB_ADDR 0x23
#define TEMP_LEN 2
#define TEMP_UNIT 0.125f
#define TEMP_BIAS 23.0f

#define ACC_CONF_ADDR 0x40
#define ACC_CONF_RESERVED 0x01
#define ACC_CONF_BWP_OSR4 0x00
#define ACC_CONF_BWP_OSR2 0x01
#define ACC_CONF_BWP_NORM 0x02
#define ACC_CONF_ODR_12_5_Hz 0x05
#define ACC_CONF_ODR_25_Hz 0x06
#define ACC_CONF_ODR_50_Hz 0x07
#define ACC_CONF_ODR_100_Hz 0x08
#define ACC_CONF_ODR_200_Hz 0x09
#define ACC_CONF_ODR_400_Hz 0x0A
#define ACC_CONF_ODR_800_Hz 0x0B
#define ACC_CONF_ODR_1600_Hz 0x0C

#define ACC_RANGE_ADDR 0x41
#define ACC_RANGE_3G 0x00
#define ACC_RANGE_6G 0x01
#define ACC_RANGE_12G 0x02
#define ACC_RANGE_24G 0x03

#define INT1_IO_CTRL_ADDR 0x53

#define INT2_IO_CTRL_ADDR 0x54

#define INT_MAP_DATA_ADDR 0x58

#define ACC_SELF_TEST_ADDR 0x6D
#define ACC_SELF_TEST_OFF 0x00
#define ACC_SELF_TEST_POS 0x0D
#define ACC_SELF_TEST_NEG 0x09

#define ACC_PWR_CONF_ADDR 0x7C
#define ACC_PWR_CONF_SUS 0x03
#define ACC_PWR_CONF_ACT 0x00

#define ACC_PWR_CTRL_ADDR 0x7D
#define ACC_PWR_CTRL_ON 0x04
#define ACC_PWR_CTRL_OFF 0x00

#define ACC_SOFTRESET_ADDR 0x7E
#define ACC_SOFTRESET_VAL 0xB6

/*-----陀螺仪寄存器表-----*/
#define GYRO_CHIP_ID_ADDR 0x00
#define GYRO_CHIP_ID_VAL 0x0F

#define GYRO_RATE_X_LSB_ADDR 0x02
#define GYRO_RATE_X_MSB_ADDR 0x03
#define GYRO_RATE_Y_LSB_ADDR 0x04
#define GYRO_RATE_Y_MSB_ADDR 0x05
#define GYRO_RATE_Z_LSB_ADDR 0x06
#define GYRO_RATE_Z_MSB_ADDR 0x07
#define GYRO_XYZ_LEN 6

#define GYRO_INT_STAT_1_ADDR 0x0A

#define GYRO_RANGE_ADDR 0x0F
#define GYRO_RANGE_2000_DEG_S 0x00
#define GYRO_RANGE_1000_DEG_S 0x01
#define GYRO_RANGE_500_DEG_S 0x02
#define GYRO_RANGE_250_DEG_S 0x03
#define GYRO_RANGE_125_DEG_S 0x04

#define GYRO_BANDWIDTH_ADDR 0x10
#define GYRO_ODR_2000Hz_BANDWIDTH_532Hz 0x00
#define GYRO_ODR_2000Hz_BANDWIDTH_230Hz 0x01
#define GYRO_ODR_1000Hz_BANDWIDTH_116Hz 0x02
#define GYRO_ODR_400Hz_BANDWIDTH_47Hz 0x03
#define GYRO_ODR_200Hz_BANDWIDTH_23Hz 0x04
#define GYRO_ODR_100Hz_BANDWIDTH_12Hz 0x05
#define GYRO_ODR_200Hz_BANDWIDTH_64Hz 0x06
#define GYRO_ODR_100Hz_BANDWIDTH_32Hz 0x07

#define GYRO_LPM1_ADDR 0x11
#define GYRO_LPM1_NOR 0x00
#define GYRO_LPM1_SUS 0x80
#define GYRO_LPM1_DEEP_SUS 0x20

#define GYRO_SOFTRESET_ADDR 0x14
#define GYRO_SOFTRESET_VAL 0xB6

#define GYRO_INT_CTRL_ADDR 0x15

#define GYRO_INT3_INT4_IO_CONF_ADDR 0x16

#define GYRO_INT3_INT4_IO_MAP_ADDR 0x18

#define GYRO_SELF_TEST_ADDR 0x3C
#define GYRO_SELF_TEST_ON 0x01
```

然后观察BMI088数据手册中的陀螺仪通信部分

![image-20221128162846818](pic/image-20221128162846818.png)

得知，想要往陀螺仪中写入数据，需要先片选陀螺仪，第一个发送字节的bit0为0，bit1-7为地址，然后第二个发送字节为值

所以我们可以得到以下代码

```C
void WriteDataToGyro(uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}
```

同时，如果想要读取陀螺仪的数据，如果是单个数据的话，就需要第一个发送字节为的bit0为1，bit1-7为地址，第二个字节就是我们读取的数据。如果是读取多个数据的话，可以使用burst-read模式，即发送一次地址，然后连续读取即可，地址会自动自增。所以我们又可以得到以下代码

```C
void ReadSingleDataFromGyro(uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromGyro(uint8_t addr, uint8_t len, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}
```

这样陀螺仪数据的基础读写函数就写完了

之后我们观察BMI088数据手册中的加速度计通信部分

![image-20221128163956858](pic/image-20221128163956858.png)

得知，加速度计的写部分跟陀螺仪没区别，但是加速度计的读的部分，其第二个字节是混乱的数据，所以需要用户读取两次才能得到正确的加速度计数据，综合得到以下代码

```C
void WriteDataToAcc(uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadSingleDataFromAcc(uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromAcc(uint8_t addr, uint8_t len, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}
```

这样子我们的基础读写函数就写好了

剩下的就是一些完善性的功能了，没有什么难度，赋上代码和效果图

bmi088.c

```
/**
 * @Author         : Minghang Li
 * @Date           : 2022-11-25 22:54
 * @LastEditTime   : 2022-11-28 16:32
 * @Note           :
 * @Copyright(c)   : Minghang Li Copyright
 */
#include "bmi088.h"

#include <math.h>

#include "bmi088reg.h"
#include "gpio.h"
#include "spi.h"

bmi088_error_e BMI088_INIT(void) {
    bmi088_error_e error = NO_ERROR;

    BMI088_CONF_INIT();

    error |= VerifyAccChipID();
    error |= VerifyGyroChipID();
    if (1) {  // 将来改成变量控制自检
        error |= VerifyAccSelfTest();
    }
    if (1) {  // 将来改成变量控制自检
        error |= VerifyGyroSelfTest();
    }
    return error;
}

void WriteDataToAcc(uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void WriteDataToGyro(uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void ReadSingleDataFromAcc(uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadSingleDataFromGyro(uint8_t addr, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromAcc(uint8_t addr, uint8_t len, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromGyro(uint8_t addr, uint8_t len, uint8_t *data) {
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;
    for (int i = 0; i < len; i++) {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void BMI088_CONF_INIT(void) {
    // 加速度计初始化
    // 先软重启，清空所有寄存器
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    // 打开加速度计电源
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    // 加速度计变成正常模式
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);

    // 陀螺仪初始化
    // 先软重启，清空所有寄存器
    WriteDataToGyro(GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    // 陀螺仪变成正常模式
    WriteDataToGyro(GYRO_LPM1_ADDR, GYRO_LPM1_NOR);

    // 加速度计配置写入
    // 写入范围，+-3g的测量范围
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    // 写入配置，正常带宽，1600hz输出频率
    WriteDataToAcc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));

    // 陀螺仪配置写入
    // 写入范围，+-500°/s的测量范围
    WriteDataToGyro(GYRO_RANGE_ADDR, GYRO_RANGE_500_DEG_S);
    // 写入带宽，2000Hz输出频率，532Hz滤波器带宽
    WriteDataToGyro(GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_532Hz);
}

bmi088_error_e VerifyAccChipID(void) {
    uint8_t chip_id;
    ReadSingleDataFromAcc(ACC_CHIP_ID_ADDR, &chip_id);
    if (chip_id != ACC_CHIP_ID_VAL) {
        return ACC_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

bmi088_error_e VerifyGyroChipID(void) {
    uint8_t chip_id;
    ReadSingleDataFromGyro(GYRO_CHIP_ID_ADDR, &chip_id);
    if (chip_id != GYRO_CHIP_ID_VAL) {
        return GYRO_CHIP_ID_ERR;
    }
    return NO_ERROR;
}

bmi088_error_e VerifyAccSelfTest(void) {
    acc_raw_data_t pos_data, neg_data;
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_24G);
    WriteDataToAcc(ACC_CONF_ADDR, 0xA7);
    HAL_Delay(10);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_POS);
    HAL_Delay(100);
    ReadAccData(&pos_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_NEG);
    HAL_Delay(100);
    ReadAccData(&neg_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_OFF);
    HAL_Delay(100);
    if ((fabs(pos_data.x - neg_data.x) > 0.1f) || (fabs(pos_data.y - neg_data.y) > 0.1f) || (fabs(pos_data.z - neg_data.z) > 0.1f)) {
        return ACC_DATA_ERR;
    }
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    WriteDataToAcc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    return NO_ERROR;
}

bmi088_error_e VerifyGyroSelfTest(void) {
    WriteDataToGyro(GYRO_SELF_TEST_ADDR, GYRO_SELF_TEST_ON);
    uint8_t bist_rdy = 0x00, bist_fail;
    while (bist_rdy == 0) {
        ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_rdy);
        bist_rdy = (bist_rdy & 0x02) >> 1;
    }
    ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_fail);
    bist_fail = (bist_fail & 0x04) >> 2;
    if (bist_fail == 0) {
        return NO_ERROR;
    } else {
        return GYRO_DATA_ERR;
    }
}

void ReadAccData(acc_raw_data_t *data) {
    uint8_t buf[ACC_XYZ_LEN], range;
    int16_t acc[3];
    ReadSingleDataFromAcc(ACC_RANGE_ADDR, &range);
    ReadMultiDataFromAcc(ACC_X_LSB_ADDR, ACC_XYZ_LEN, buf);
    acc[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    acc[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    acc[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    data->x = (float)acc[0] * BMI088_ACCEL_3G_SEN;
    data->y = (float)acc[1] * BMI088_ACCEL_3G_SEN;
    data->z = (float)acc[2] * BMI088_ACCEL_3G_SEN;
}

void ReadGyroData(gyro_raw_data_t *data) {
    uint8_t buf[GYRO_XYZ_LEN], range;
    int16_t gyro[3];
    float unit;
    ReadSingleDataFromGyro(GYRO_RANGE_ADDR, &range);
    switch (range) {
        case 0x00:
            unit = 16.384;
            break;
        case 0x01:
            unit = 32.768;
            break;
        case 0x02:
            unit = 65.536;
            break;
        case 0x03:
            unit = 131.072;
            break;
        case 0x04:
            unit = 262.144;
            break;
        default:
            unit = 16.384;
            break;
    }
    ReadMultiDataFromGyro(GYRO_RATE_X_LSB_ADDR, GYRO_XYZ_LEN, buf);
    gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    data->roll = (float)gyro[0] / unit * DEG2SEC;
    data->pitch = (float)gyro[1] / unit * DEG2SEC;
    data->yaw = (float)gyro[2] / unit * DEG2SEC;
}

void ReadAccSensorTime(float *time) {
    uint8_t buf[SENSORTIME_LEN];
    ReadMultiDataFromAcc(SENSORTIME_0_ADDR, SENSORTIME_LEN, buf);
    *time = buf[0] * SENSORTIME_0_UNIT + buf[1] * SENSORTIME_1_UNIT + buf[2] * SENSORTIME_2_UNIT;
}

void ReadAccTemperature(float *temp) {
    uint8_t buf[TEMP_LEN];
    ReadMultiDataFromAcc(TEMP_MSB_ADDR, TEMP_LEN, buf);
    uint16_t temp_uint11 = (buf[0] << 3) + (buf[1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023) {
        temp_int11 = (int16_t)temp_uint11 - 2048;
    } else {
        temp_int11 = (int16_t)temp_uint11;
    }
    *temp = temp_int11 * TEMP_UNIT + TEMP_BIAS;
}

```

bmi088.h

```
/**
 * @Author         : Minghang Li
 * @Date           : 2022-11-25 22:54
 * @LastEditTime   : 2022-11-28 16:09
 * @Note           :
 * @Copyright(c)   : Minghang Li Copyright
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bmi088reg.h"

#define BMI088_SPI hspi1
#define BMI088_ACC_GPIOx GPIOA
#define BMI088_ACC_GPIOp GPIO_PIN_4
#define BMI088_GYRO_GPIOx GPIOB
#define BMI088_GYRO_GPIOp GPIO_PIN_0

typedef struct acc_raw_data_t {
    float x;
    float y;
    float z;
} acc_raw_data_t;

typedef struct gyro_raw_data_t {
    float roll;
    float pitch;
    float yaw;
} gyro_raw_data_t;

typedef struct acc_data_t {
    acc_raw_data_t acc_raw_data;
    float sensor_time;
    float temperature;
    bool enable_self_test;
} acc_data_t;

typedef struct gyro_data_t {
    gyro_raw_data_t gyro_raw_data;
    bool enable_self_test;
} gyro_data_t;

typedef enum bmi088_error_e {
    NO_ERROR = 0,
    ACC_CHIP_ID_ERR = 0x01,
    ACC_DATA_ERR = 0x02,
    GYRO_CHIP_ID_ERR = 0x04,
    GYRO_DATA_ERR = 0x08,
} bmi088_error_e;

typedef struct bmi088_data_t {
    acc_data_t acc_data;
    bmi088_error_e bmi088_error;
} bmi088_data_t;

// 基础函数
void WriteDataToAcc(uint8_t addr, uint8_t data);
void WriteDataToGyro(uint8_t addr, uint8_t data);
void ReadSingleDataFromAcc(uint8_t addr, uint8_t *data);
void ReadSingleDataFromGyro(uint8_t addr, uint8_t *data);
void ReadMultiDataFromAcc(uint8_t addr, uint8_t len, uint8_t *data);
void ReadMultiDataFromGyro(uint8_t addr, uint8_t len, uint8_t *data);

// 初始化函数
bmi088_error_e BMI088_INIT(void);
void BMI088_CONF_INIT(void);

// 功能函数
void ReadAccData(acc_raw_data_t *data);
void ReadGyroData(gyro_raw_data_t *data);
void ReadAccSensorTime(float *time);
void ReadAccTemperature(float *temp);

// 校验函数
bmi088_error_e VerifyAccChipID(void);
bmi088_error_e VerifyGyroChipID(void);
bmi088_error_e VerifyAccSelfTest(void);
bmi088_error_e VerifyGyroSelfTest(void);
```

bmi088reg.h

```
/**
 * @Author         : Minghang Li
 * @Date           : 2022-11-25 23:01
 * @LastEditTime   : 2022-11-28 16:32
 * @Note           :
 * @Copyright(c)   : Minghang Li Copyright
 */
#pragma once

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define DEG2SEC 0.0174532925f
#define SEC2DEG 57.295779578f
#define PI 3.14159265f

/*-----bmi088的spi读取协议部分-----*/
#define BMI088_SPI_WRITE_CODE 0x7F
#define BMI088_SPI_READ_CODE 0x80

/*-----加速度计寄存器表-----*/
#define ACC_CHIP_ID_ADDR 0x00
#define ACC_CHIP_ID_VAL 0x1E

#define ACC_ERR_REG_ADDR 0x02

#define ACC_STATUS_ADDR 0x03

#define ACC_X_LSB_ADDR 0x12
#define ACC_X_MSB_ADDR 0x13
#define ACC_Y_LSB_ADDR 0x14
#define ACC_Y_MSB_ADDR 0x15
#define ACC_Z_LSB_ADDR 0x16
#define ACC_Z_MSB_ADDR 0x17
#define ACC_XYZ_LEN 6

#define SENSORTIME_0_ADDR 0x18
#define SENSORTIME_0_UNIT (39.0625f / 1000000.0f)
#define SENSORTIME_1_ADDR 0x19
#define SENSORTIME_1_UNIT (10.0 / 1000.0f)
#define SENSORTIME_2_ADDR 0x1A
#define SENSORTIME_2_UNIT (2.56f)
#define SENSORTIME_LEN 3

#define ACC_INT_STAT_1_ADDR 0x1D

#define TEMP_MSB_ADDR 0x22
#define TEMP_LSB_ADDR 0x23
#define TEMP_LEN 2
#define TEMP_UNIT 0.125f
#define TEMP_BIAS 23.0f

#define ACC_CONF_ADDR 0x40
#define ACC_CONF_RESERVED 0x01
#define ACC_CONF_BWP_OSR4 0x00
#define ACC_CONF_BWP_OSR2 0x01
#define ACC_CONF_BWP_NORM 0x02
#define ACC_CONF_ODR_12_5_Hz 0x05
#define ACC_CONF_ODR_25_Hz 0x06
#define ACC_CONF_ODR_50_Hz 0x07
#define ACC_CONF_ODR_100_Hz 0x08
#define ACC_CONF_ODR_200_Hz 0x09
#define ACC_CONF_ODR_400_Hz 0x0A
#define ACC_CONF_ODR_800_Hz 0x0B
#define ACC_CONF_ODR_1600_Hz 0x0C

#define ACC_RANGE_ADDR 0x41
#define ACC_RANGE_3G 0x00
#define ACC_RANGE_6G 0x01
#define ACC_RANGE_12G 0x02
#define ACC_RANGE_24G 0x03

#define INT1_IO_CTRL_ADDR 0x53

#define INT2_IO_CTRL_ADDR 0x54

#define INT_MAP_DATA_ADDR 0x58

#define ACC_SELF_TEST_ADDR 0x6D
#define ACC_SELF_TEST_OFF 0x00
#define ACC_SELF_TEST_POS 0x0D
#define ACC_SELF_TEST_NEG 0x09

#define ACC_PWR_CONF_ADDR 0x7C
#define ACC_PWR_CONF_SUS 0x03
#define ACC_PWR_CONF_ACT 0x00

#define ACC_PWR_CTRL_ADDR 0x7D
#define ACC_PWR_CTRL_ON 0x04
#define ACC_PWR_CTRL_OFF 0x00

#define ACC_SOFTRESET_ADDR 0x7E
#define ACC_SOFTRESET_VAL 0xB6

/*-----陀螺仪寄存器表-----*/
#define GYRO_CHIP_ID_ADDR 0x00
#define GYRO_CHIP_ID_VAL 0x0F

#define GYRO_RATE_X_LSB_ADDR 0x02
#define GYRO_RATE_X_MSB_ADDR 0x03
#define GYRO_RATE_Y_LSB_ADDR 0x04
#define GYRO_RATE_Y_MSB_ADDR 0x05
#define GYRO_RATE_Z_LSB_ADDR 0x06
#define GYRO_RATE_Z_MSB_ADDR 0x07
#define GYRO_XYZ_LEN 6

#define GYRO_INT_STAT_1_ADDR 0x0A

#define GYRO_RANGE_ADDR 0x0F
#define GYRO_RANGE_2000_DEG_S 0x00
#define GYRO_RANGE_1000_DEG_S 0x01
#define GYRO_RANGE_500_DEG_S 0x02
#define GYRO_RANGE_250_DEG_S 0x03
#define GYRO_RANGE_125_DEG_S 0x04

#define GYRO_BANDWIDTH_ADDR 0x10
#define GYRO_ODR_2000Hz_BANDWIDTH_532Hz 0x00
#define GYRO_ODR_2000Hz_BANDWIDTH_230Hz 0x01
#define GYRO_ODR_1000Hz_BANDWIDTH_116Hz 0x02
#define GYRO_ODR_400Hz_BANDWIDTH_47Hz 0x03
#define GYRO_ODR_200Hz_BANDWIDTH_23Hz 0x04
#define GYRO_ODR_100Hz_BANDWIDTH_12Hz 0x05
#define GYRO_ODR_200Hz_BANDWIDTH_64Hz 0x06
#define GYRO_ODR_100Hz_BANDWIDTH_32Hz 0x07

#define GYRO_LPM1_ADDR 0x11
#define GYRO_LPM1_NOR 0x00
#define GYRO_LPM1_SUS 0x80
#define GYRO_LPM1_DEEP_SUS 0x20

#define GYRO_SOFTRESET_ADDR 0x14
#define GYRO_SOFTRESET_VAL 0xB6

#define GYRO_INT_CTRL_ADDR 0x15

#define GYRO_INT3_INT4_IO_CONF_ADDR 0x16

#define GYRO_INT3_INT4_IO_MAP_ADDR 0x18

#define GYRO_SELF_TEST_ADDR 0x3C
#define GYRO_SELF_TEST_ON 0x01
```

效果如下

![image-20221202200647902](https://git.nrs-lab.com/lmh-group/picgo-pic/uploads/f16639bfd835d4d9f1ddb301e75f28db/202212022006952.png)