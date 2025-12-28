## CUBEMX配置

### 1.SPI外设

SPI外设配置，注意SPI1、波特率、时钟极性和相位
![](pic/SPI外设.png)
SPI引脚配置，注意MOSI非默认引脚
![](pic/SPI引脚.png)
SPI DMA配置，打开RX和TX两个请求，其他默认
![](pic/DMA配置.png)
片选和中断引脚配置，注意User_Label命名
![](pic/CS和INT.png)
NVIC配置，打开EXTI line5、[9:5]
![](pic/NVIC配置.png)
FreeRTOS任务配置参考
![](pic/FreeRTOS_Task配置.png)