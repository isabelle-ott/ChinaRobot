#include "QR_code.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "miniCMD_makeCmd.h"

/* 函数声明 */
void QR_Init(void);                     // 二维码模块初始化函数
void QR_USART_DATA_Processing(uint8_t); // 二维码串口数据处理函数
void Set_QR_Data_Zero(void);            // 清除二维码数据函数
int Get_QR_Data(void);                  // 获取二维码数据函数

/* 静态变量定义 */
static uint8_t Buf[20]; // 用于存储接收到的二维码数据的缓冲区

/* 二维码模块结构体初始化 */
QR_T My_QR_Handle = {
    .QR_huart = &huart1,                           // 绑定二维码模块使用的串口句柄
    .QR_data = 0,                                  // 二维码解析数据缓存，初始化为0
    .QR_Buf = Buf,                                 // 绑定数据接收缓冲区
    ._QR_Init = QR_Init,                           // 绑定初始化函数
    ._QR_USART_DATA_Processing = QR_USART_DATA_Processing, // 绑定串口数据处理函数
    ._Get_QR_Data = Get_QR_Data,                     // 绑定获取二维码数据函数
    ._Set_QR_Data_Zero = Set_QR_Data_Zero              // 绑定清除二维码数据函数
};

/**
 * @brief 获取二维码数据并进行场地匹配
 * @return 匹配结果 (1: 匹配成功, 0: 无数据, -1: 数据无效)
 * @note 根据机器人所在的场地颜色（红方/蓝方），校验收到的 'R' 或 'B' 字符
 */
int Get_QR_Data(void)
{
    if (My_QR_Handle.QR_data == 'R' && is_site(SITE_RED))
    {
       Set_QR_Data_Zero();
       return 1;
    }
    if (My_QR_Handle.QR_data == 'B' && is_site(SITE_BLUE))
    {
       Set_QR_Data_Zero();
       return 1;
    }
    if (My_QR_Handle.QR_data == 0)
    {
       return 0;
    }
    return -1; // 数据无效
}

void QR_text(void)
{
    while (1)
    {
       if (Get_QR_Data() == 1)
       {
          print("you\n");
       }
       else
       {
          print("NULL\n");
       }
       osDelay(500);
    }
}

MAKE_CMD(void, QR_text, void);

/**
 * @brief 清除二维码数据缓存
 * @note 将结构体中保存的二维码字符清零
 */
void Set_QR_Data_Zero(void)
{
    My_QR_Handle.QR_data = 0;
    return;
}

/**
 * @brief 二维码模块初始化函数
 * @note 开启串口接收中断，准备接收二维码模块发来的数据
 */
void QR_Init(void)
{
    /* 进入临界区，防止中断打断初始化过程 */
    taskENTER_CRITICAL();
    {
       HAL_UART_Receive_IT(My_QR_Handle.QR_huart, My_QR_Handle.QR_Buf, 1);
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 二维码串口数据处理函数
 * @param size 接收到的数据长度
 * @note 对接收到的二维码数据进行转存，并重新开启下一次串口接收中断
 */
void QR_USART_DATA_Processing(uint8_t size)
{
    /* 检查接收到的数据长度是否符合预期宏定义大小 */
    if (size == QR_DATA_SIZE)
    {
       My_QR_Handle.QR_data = My_QR_Handle.QR_Buf[0];
    }
    HAL_UART_Receive_IT(My_QR_Handle.QR_huart, My_QR_Handle.QR_Buf, 1);
}

void QR_USART_Error_Processing(void)
{
    HAL_UART_Abort_IT(My_QR_Handle.QR_huart);
    HAL_UART_Receive_IT(My_QR_Handle.QR_huart, My_QR_Handle.QR_Buf, 1);
}